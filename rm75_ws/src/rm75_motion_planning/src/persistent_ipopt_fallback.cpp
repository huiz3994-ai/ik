#include "rm75_motion_planning/persistent_ipopt_fallback.h"

#include <algorithm>
#include <cerrno>
#include <csignal>
#include <cstdlib>
#include <cstdio>
#include <cstring>
#include <iomanip>
#include <mutex>
#include <poll.h>
#include <sstream>
#include <stdexcept>
#include <sys/types.h>
#include <sys/wait.h>
#include <unistd.h>

namespace rm75_motion_planning {

namespace {
rm75_kinematics::IkStatus parseStatus(const std::string& text) {
  using rm75_kinematics::IkStatus;
  if (text == "SUCCESS") return IkStatus::SUCCESS;
  if (text == "MAX_ITERATIONS") return IkStatus::MAX_ITERATIONS;
  if (text == "TIMEOUT") return IkStatus::TIMEOUT;
  if (text == "JOINT_LIMIT") return IkStatus::JOINT_LIMIT;
  if (text == "SINGULARITY") return IkStatus::SINGULARITY;
  if (text == "UNREACHABLE_OR_STALLED") return IkStatus::UNREACHABLE_OR_STALLED;
  return IkStatus::NUMERICAL_ERROR;
}
}  // namespace

struct PersistentIpoptFallback::Impl {
  pid_t pid{-1};
  FILE* input{nullptr};
  FILE* output{nullptr};
  int timeout_ms{5000};
  bool ready{false};
  mutable std::mutex mutex;

  void shutdown() noexcept {
    if (input) {
      std::fputs("QUIT\n", input);
      std::fflush(input);
      std::fclose(input);
      input = nullptr;
    }
    if (output) {
      std::fclose(output);
      output = nullptr;
    }
    if (pid > 0) {
      int status = 0;
      if (waitpid(pid, &status, WNOHANG) == 0) {
        kill(pid, SIGTERM);
        waitpid(pid, &status, 0);
      }
      pid = -1;
    }
    ready = false;
  }

  bool readLine(std::string* line, int timeout) {
    pollfd descriptor{fileno(output), POLLIN, 0};
    const int polled = poll(&descriptor, 1, timeout);
    if (polled <= 0 || !(descriptor.revents & (POLLIN | POLLHUP))) return false;
    char* buffer = nullptr;
    std::size_t capacity = 0;
    const ssize_t count = getline(&buffer, &capacity, output);
    if (count <= 0) { std::free(buffer); return false; }
    line->assign(buffer, static_cast<std::size_t>(count));
    std::free(buffer);
    while (!line->empty() && (line->back() == '\n' || line->back() == '\r'))
      line->pop_back();
    return true;
  }

  Impl(const std::string& python, const std::string& server,
       const std::string& optimization, const std::string& urdf, int timeout)
      : timeout_ms(timeout) {
    if (timeout_ms <= 0) throw std::invalid_argument("response timeout must be positive");
    int parent_to_child[2], child_to_parent[2];
    if (pipe(parent_to_child) || pipe(child_to_parent))
      throw std::runtime_error(std::string("pipe failed: ") + std::strerror(errno));
    pid = fork();
    if (pid < 0) throw std::runtime_error("fork failed");
    if (pid == 0) {
      dup2(parent_to_child[0], STDIN_FILENO);
      dup2(child_to_parent[1], STDOUT_FILENO);
      close(parent_to_child[0]); close(parent_to_child[1]);
      close(child_to_parent[0]); close(child_to_parent[1]);
      execl(python.c_str(), python.c_str(), server.c_str(), "--urdf", urdf.c_str(),
            "--optimization-ik", optimization.c_str(), static_cast<char*>(nullptr));
      _exit(127);
    }
    close(parent_to_child[0]); close(child_to_parent[1]);
    input = fdopen(parent_to_child[1], "w");
    output = fdopen(child_to_parent[0], "r");
    if (!input || !output) throw std::runtime_error("fdopen failed");
    setvbuf(input, nullptr, _IOLBF, 0);
    // poll() observes the kernel fd, so prevent stdio from reading later
    // protocol lines into an invisible userspace buffer.
    setvbuf(output, nullptr, _IONBF, 0);
    // CasADi may print an Ipopt banner before the protocol marker.
    const int startup_timeout = std::max(30000, timeout_ms);
    for (int line_count = 0; line_count < 200; ++line_count) {
      std::string line;
      if (!readLine(&line, startup_timeout)) break;
      if (line == "READY") { ready = true; break; }
    }
    if (!ready) {
      shutdown();
      throw std::runtime_error("Ipopt fallback server did not become ready");
    }
  }

  ~Impl() { shutdown(); }
};

PersistentIpoptFallback::PersistentIpoptFallback(
    const std::string& python, const std::string& server,
    const std::string& optimization, const std::string& urdf, int timeout_ms)
    : impl_(new Impl(python, server, optimization, urdf, timeout_ms)) {}
PersistentIpoptFallback::~PersistentIpoptFallback() = default;

rm75_kinematics::IkResult PersistentIpoptFallback::solve(
    const rm75_kinematics::Pose& target, const Eigen::VectorXd& seed) {
  rm75_kinematics::IkResult result;
  result.q = seed;
  if (seed.size() != 7 || !seed.allFinite() || !target.position.allFinite() ||
      !target.rotation.allFinite()) {
    result.status = rm75_kinematics::IkStatus::NUMERICAL_ERROR;
    result.detail = "invalid persistent fallback input";
    return result;
  }
  std::lock_guard<std::mutex> lock(impl_->mutex);
  std::ostringstream request;
  request << std::setprecision(17);
  Eigen::Matrix4d matrix = Eigen::Matrix4d::Identity();
  matrix.topLeftCorner<3, 3>() = target.rotation;
  matrix.topRightCorner<3, 1>() = target.position;
  for (int row = 0; row < 4; ++row)
    for (int column = 0; column < 4; ++column)
      request << matrix(row, column) << ' ';
  for (int i = 0; i < 7; ++i) request << seed[i] << ' ';
  request << "1\n";
  if (std::fputs(request.str().c_str(), impl_->input) < 0 ||
      std::fflush(impl_->input) != 0) {
    result.status = rm75_kinematics::IkStatus::NUMERICAL_ERROR;
    result.detail = "failed to write to Ipopt fallback server";
    return result;
  }
  std::string response;
  if (!impl_->readLine(&response, impl_->timeout_ms)) {
    result.status = rm75_kinematics::IkStatus::TIMEOUT;
    result.detail = "persistent Ipopt fallback response timeout";
    return result;
  }
  std::istringstream parser(response);
  std::string status;
  parser >> status >> result.solve_time_ms >> result.iterations >>
      result.position_error >> result.orientation_error >> result.min_limit_margin;
  result.q.resize(7);
  for (int i = 0; i < 7; ++i) parser >> result.q[i];
  std::getline(parser >> std::ws, result.detail);
  result.status = parseStatus(status);
  if (!parser && result.detail.empty()) {
    result.status = rm75_kinematics::IkStatus::NUMERICAL_ERROR;
    result.detail = "malformed Ipopt fallback response: " + response;
  }
  return result;
}

ContinuousIkSolver::Fallback PersistentIpoptFallback::callback() {
  return [this](const rm75_kinematics::Pose& pose, const Eigen::VectorXd& seed) {
    return solve(pose, seed);
  };
}

bool PersistentIpoptFallback::healthy() const { return impl_ && impl_->ready; }

}  // namespace rm75_motion_planning
