#pragma once

#include <algorithm>
#include <chrono>
#include <cmath>
#include <limits>
#include <string>
#include <vector>

#include <Eigen/Core>
#include <Eigen/Cholesky>

namespace rm75_touch_teleop {

enum class BoxQpStatus {
  SUCCESS,
  INVALID_INPUT,
  INFEASIBLE_BOUNDS,
  NUMERICAL_ERROR,
  MAX_ITERATIONS,
  TIMEOUT
};

inline const char* boxQpStatusName(BoxQpStatus status) {
  switch (status) {
    case BoxQpStatus::SUCCESS: return "success";
    case BoxQpStatus::INVALID_INPUT: return "invalid_input";
    case BoxQpStatus::INFEASIBLE_BOUNDS: return "infeasible_bounds";
    case BoxQpStatus::NUMERICAL_ERROR: return "numerical_error";
    case BoxQpStatus::MAX_ITERATIONS: return "max_iterations";
    case BoxQpStatus::TIMEOUT: return "timeout";
  }
  return "unknown";
}

struct BoxQpOptions {
  int max_iterations = 64;
  double kkt_tolerance = 1e-8;
  double bound_tolerance = 1e-10;
  double time_limit_sec = 0.002;
};

struct BoxQpResult {
  BoxQpStatus status = BoxQpStatus::INVALID_INPUT;
  Eigen::VectorXd solution;
  int iterations = 0;
  double solve_time_sec = 0.0;
  double kkt_residual = std::numeric_limits<double>::infinity();
};

// Primal active-set solver for the strictly convex quadratic program
//   min 0.5*x' H x + g' x,  lower <= x <= upper.
// The online RM75 problem has seven variables and only joint-wise bounds, so
// this solver represents every constraint explicitly without converting the
// problem into an unconstrained least-squares/DLS step.
inline BoxQpResult solveBoxQp(const Eigen::MatrixXd& hessian,
                              const Eigen::VectorXd& gradient,
                              const Eigen::VectorXd& lower,
                              const Eigen::VectorXd& upper,
                              const Eigen::VectorXd& warm_start,
                              const BoxQpOptions& options = BoxQpOptions()) {
  using Clock = std::chrono::steady_clock;
  const auto started = Clock::now();
  BoxQpResult result;
  const Eigen::Index n = gradient.size();
  auto finish = [&](BoxQpStatus status) {
    result.status = status;
    result.solve_time_sec =
        std::chrono::duration<double>(Clock::now() - started).count();
    return result;
  };

  if (n <= 0 || hessian.rows() != n || hessian.cols() != n ||
      lower.size() != n || upper.size() != n || warm_start.size() != n ||
      options.max_iterations <= 0 || options.kkt_tolerance <= 0.0 ||
      options.bound_tolerance < 0.0 || options.time_limit_sec <= 0.0 ||
      !hessian.allFinite() || !gradient.allFinite() || !lower.allFinite() ||
      !upper.allFinite() || !warm_start.allFinite() ||
      !hessian.isApprox(hessian.transpose(), 1e-10)) {
    return finish(BoxQpStatus::INVALID_INPUT);
  }
  for (Eigen::Index i = 0; i < n; ++i) {
    if (lower[i] > upper[i] + options.bound_tolerance)
      return finish(BoxQpStatus::INFEASIBLE_BOUNDS);
  }
  Eigen::LDLT<Eigen::MatrixXd> full_ldlt(hessian);
  if (full_ldlt.info() != Eigen::Success || !full_ldlt.isPositive())
    return finish(BoxQpStatus::NUMERICAL_ERROR);

  Eigen::VectorXd x(n);
  std::vector<int> active(static_cast<std::size_t>(n), 0);  // -1 lower, +1 upper.
  for (Eigen::Index i = 0; i < n; ++i) {
    x[i] = std::max(lower[i], std::min(upper[i], warm_start[i]));
    if (std::abs(x[i] - lower[i]) <= options.bound_tolerance) active[i] = -1;
    else if (std::abs(x[i] - upper[i]) <= options.bound_tolerance) active[i] = 1;
  }

  for (int iteration = 0; iteration < options.max_iterations; ++iteration) {
    result.iterations = iteration + 1;
    const double elapsed =
        std::chrono::duration<double>(Clock::now() - started).count();
    if (elapsed > options.time_limit_sec) {
      result.solution = x;
      return finish(BoxQpStatus::TIMEOUT);
    }

    const Eigen::VectorXd grad = hessian * x + gradient;
    std::vector<Eigen::Index> free_indices;
    for (Eigen::Index i = 0; i < n; ++i)
      if (active[static_cast<std::size_t>(i)] == 0) free_indices.push_back(i);

    Eigen::VectorXd direction = Eigen::VectorXd::Zero(n);
    if (!free_indices.empty()) {
      const Eigen::Index nf = static_cast<Eigen::Index>(free_indices.size());
      Eigen::MatrixXd reduced(nf, nf);
      Eigen::VectorXd reduced_gradient(nf);
      for (Eigen::Index row = 0; row < nf; ++row) {
        reduced_gradient[row] = grad[free_indices[row]];
        for (Eigen::Index col = 0; col < nf; ++col)
          reduced(row, col) = hessian(free_indices[row], free_indices[col]);
      }
      Eigen::LDLT<Eigen::MatrixXd> ldlt(reduced);
      if (ldlt.info() != Eigen::Success || !ldlt.isPositive()) {
        result.solution = x;
        return finish(BoxQpStatus::NUMERICAL_ERROR);
      }
      const Eigen::VectorXd reduced_direction = ldlt.solve(-reduced_gradient);
      if (ldlt.info() != Eigen::Success || !reduced_direction.allFinite()) {
        result.solution = x;
        return finish(BoxQpStatus::NUMERICAL_ERROR);
      }
      for (Eigen::Index i = 0; i < nf; ++i)
        direction[free_indices[i]] = reduced_direction[i];
    }

    if (direction.lpNorm<Eigen::Infinity>() > options.kkt_tolerance) {
      double alpha = 1.0;
      for (Eigen::Index i : free_indices) {
        if (direction[i] > options.bound_tolerance)
          alpha = std::min(alpha, (upper[i] - x[i]) / direction[i]);
        else if (direction[i] < -options.bound_tolerance)
          alpha = std::min(alpha, (lower[i] - x[i]) / direction[i]);
      }
      alpha = std::max(0.0, std::min(1.0, alpha));
      x += alpha * direction;
      for (Eigen::Index i = 0; i < n; ++i) {
        x[i] = std::max(lower[i], std::min(upper[i], x[i]));
        if (active[i] == 0 && std::abs(x[i] - lower[i]) <= options.bound_tolerance)
          active[i] = -1;
        else if (active[i] == 0 &&
                 std::abs(x[i] - upper[i]) <= options.bound_tolerance)
          active[i] = 1;
      }
      continue;
    }

    // Free variables are stationary. Release the active bound with the worst
    // KKT sign violation, or accept the constrained optimum.
    double worst_violation = 0.0;
    Eigen::Index release = -1;
    double residual = 0.0;
    for (Eigen::Index i = 0; i < n; ++i) {
      double violation = 0.0;
      if (active[i] < 0) violation = std::max(0.0, -grad[i]);
      else if (active[i] > 0) violation = std::max(0.0, grad[i]);
      else violation = std::abs(grad[i]);
      residual = std::max(residual, violation);
      if (active[i] != 0 && violation > worst_violation) {
        worst_violation = violation;
        release = i;
      }
    }
    result.kkt_residual = residual;
    if (release < 0 || worst_violation <= options.kkt_tolerance) {
      result.solution = x;
      return finish(BoxQpStatus::SUCCESS);
    }
    active[static_cast<std::size_t>(release)] = 0;
  }

  result.solution = x;
  return finish(BoxQpStatus::MAX_ITERATIONS);
}

}  // namespace rm75_touch_teleop
