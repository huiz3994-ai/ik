#include "rm75_kinematics_optimization/adaptive_dls_ik.h"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <Eigen/Cholesky>
#include <Eigen/SVD>

namespace rm75_kinematics {

double AdaptiveDlsIk::adaptiveDamping(double sigma, const IkOptions& o) {
  if (!std::isfinite(sigma)) return o.damping_max;
  if (sigma >= o.singular_value_threshold) return o.damping_min;
  const double x = std::max(0.0, sigma) / o.singular_value_threshold;
  const double smooth = (1.0 - x * x);
  return o.damping_min + (o.damping_max - o.damping_min) * smooth * smooth;
}

IkResult AdaptiveDlsIk::solve(const Pose& target, const Eigen::VectorXd& seed,
                              const IkOptions& o) const {
  using Clock = std::chrono::steady_clock;
  const auto start = Clock::now();
  IkResult out; out.q = seed;
  const JointLimits limits = model_.limits();
  if (seed.size() != model_.dof() || !seed.allFinite() ||
      !target.position.allFinite() || !target.rotation.allFinite()) {
    out.status = IkStatus::NUMERICAL_ERROR; out.detail = "non-finite or wrong-sized input"; return out;
  }
  if ((seed.array() < limits.lower.array()).any() || (seed.array() > limits.upper.array()).any()) {
    out.status = IkStatus::JOINT_LIMIT; out.detail = "seed violates hard joint limits"; return out;
  }
  double previous_cost = std::numeric_limits<double>::infinity();
  int stalled = 0;
  for (int iter = 0; iter <= o.max_iterations; ++iter) {
    const double elapsed = std::chrono::duration<double>(Clock::now() - start).count();
    if (elapsed > o.timeout_sec) { out.status = IkStatus::TIMEOUT; out.detail = "wall-time budget exceeded"; break; }
    Pose current;
    Eigen::MatrixXd j;
    try { current = model_.forward(out.q); j = model_.jacobian(out.q); }
    catch (const std::exception& e) { out.status = IkStatus::NUMERICAL_ERROR; out.detail = e.what(); break; }
    const Vector6d raw_error = model_.poseError(current, target);
    out.position_error = raw_error.head<3>().norm();
    out.orientation_error = raw_error.tail<3>().norm();
    out.min_limit_margin = NullspaceOptimizer::minimumLimitMargin(out.q, limits);
    out.iterations = iter;
    if (out.position_error <= o.position_tolerance &&
        (!o.orientation_enabled || out.orientation_error <= o.orientation_tolerance)) {
      out.status = IkStatus::SUCCESS; out.detail = "converged"; break;
    }
    if (iter == o.max_iterations) { out.status = IkStatus::MAX_ITERATIONS; out.detail = "iteration budget exhausted"; break; }
    if (!j.allFinite() || !raw_error.allFinite()) { out.status = IkStatus::NUMERICAL_ERROR; out.detail = "non-finite model result"; break; }
    Eigen::JacobiSVD<Eigen::MatrixXd> svd(j);
    out.min_singular_value = svd.singularValues().minCoeff();
    out.damping = adaptiveDamping(out.min_singular_value, o);
    Eigen::Matrix<double, 6, 6> w = Eigen::Matrix<double, 6, 6>::Identity();
    w.diagonal().head<3>().setConstant(o.position_weight);
    w.diagonal().tail<3>().setConstant(o.orientation_enabled ? o.orientation_weight : 0.0);
    const Eigen::MatrixXd jw = w * j;
    const Vector6d ew = w * raw_error;
    Eigen::MatrixXd normal = jw * jw.transpose();
    normal.diagonal().array() += out.damping * out.damping;
    Eigen::LDLT<Eigen::MatrixXd> ldlt(normal);
    if (ldlt.info() != Eigen::Success) { out.status = IkStatus::NUMERICAL_ERROR; out.detail = "DLS factorization failed"; break; }
    const Eigen::MatrixXd pinv = jw.transpose() * ldlt.solve(Eigen::MatrixXd::Identity(6, 6));
    Eigen::VectorXd step = pinv * ew;
    if (o.enable_joint_centering || o.enable_joint_reference || o.enable_singularity_avoidance) {
      bool active = false;
      Eigen::VectorXd secondary = Eigen::VectorXd::Zero(out.q.size());
      if (o.enable_joint_centering)
        secondary += o.joint_centering_gain * NullspaceOptimizer::jointCenteringGradient(
            out.q, limits, o.soft_limit_margin, &active);
      // Negative gradient of 0.5*||q-seed||^2 discourages unnecessary branch motion.
      if (o.enable_joint_reference) secondary += o.joint_reference_gain * (seed - out.q);
      if (o.enable_singularity_avoidance) {
        Eigen::VectorXd gradient(out.q.size());
        for (Eigen::Index k = 0; k < out.q.size(); ++k) {
          Eigen::VectorXd qp = out.q, qm = out.q;
          qp[k] = std::min(limits.upper[k] - 1e-6, qp[k] + o.manipulability_fd_step);
          qm[k] = std::max(limits.lower[k] + 1e-6, qm[k] - o.manipulability_fd_step);
          auto score = [&](const Eigen::VectorXd& qx) {
            const Eigen::VectorXd s = Eigen::JacobiSVD<Eigen::MatrixXd>(
                model_.jacobian(qx)).singularValues();
            // Log-volume avoids product underflow and gives useful scale near singularities.
            return (s.array() + 1e-6).log().sum();
          };
          gradient[k] = (score(qp) - score(qm)) / (qp[k] - qm[k]);
        }
        secondary += o.manipulability_gain * gradient;
      }
      step += NullspaceOptimizer::project(jw, pinv, secondary);
    }
    if (!step.allFinite()) { out.status = IkStatus::NUMERICAL_ERROR; out.detail = "non-finite DLS step"; break; }
    const double max_abs = step.cwiseAbs().maxCoeff();
    if (max_abs > o.max_joint_step) step *= o.max_joint_step / max_abs;
    const double orientation_cost_weight = o.orientation_enabled ? o.orientation_weight : 0.0;
    const double cost = o.position_weight * out.position_error + orientation_cost_weight * out.orientation_error;
    bool accepted = false;
    for (double alpha = 1.0; alpha >= o.min_line_search_scale; alpha *= 0.5) {
      const Eigen::VectorXd candidate = out.q + alpha * step;
      if ((candidate.array() <= limits.lower.array()).any() || (candidate.array() >= limits.upper.array()).any()) continue;
      const Vector6d e2 = model_.poseError(model_.forward(candidate), target);
      const double candidate_cost = o.position_weight * e2.head<3>().norm() + orientation_cost_weight * e2.tail<3>().norm();
      if (std::isfinite(candidate_cost) && candidate_cost < cost - 1e-12) {
        out.q = candidate; accepted = true;
        stalled = (previous_cost - candidate_cost < 1e-10) ? stalled + 1 : 0;
        previous_cost = candidate_cost; break;
      }
    }
    if (!accepted || stalled >= o.stall_iterations) {
      out.status = out.min_singular_value < o.singularity_failure_threshold ?
          IkStatus::SINGULARITY : IkStatus::UNREACHABLE_OR_STALLED;
      out.detail = accepted ? "insufficient progress" : "line search found no descending feasible step";
      break;
    }
  }
  out.solve_time_ms = 1e3 * std::chrono::duration<double>(Clock::now() - start).count();
  return out;
}

}  // namespace rm75_kinematics
