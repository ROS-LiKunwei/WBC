#pragma once

#include <array>
#include <cstddef>
#include <string>

#include "min_snap/fa_joint_mapping.hpp"
#include "min_snap/min_snap_trajectory.hpp"

namespace min_snap
{

struct ArmTrajectorySample
{
  ArmArray position{};
  ArmArray velocity{};
  ArmArray acceleration{};
  ArmArray jerk{};
};

struct PlannerConfig
{
  double min_duration_s{0.01};
  double duration_scale_on_violation{1.1};
  double max_duration_on_violation_s{2.0};
  int max_duration_search_iterations{100};
  int constraint_sample_count{500};
  double snap_weight_lambda{0.85};
};

class MinSnapArmPlanner
{
public:
  explicit MinSnapArmPlanner(std::string name = "arm");

  bool plan(
    const ArmArray & start_pos,
    const ArmArray & start_vel,
    const ArmArray & start_acc,
    const ArmArray & goal_pos,
    double expected_duration,
    double max_velocity,
    double max_acceleration,
    const ArmArray & lower_limit,
    const ArmArray & upper_limit,
    const PlannerConfig & config,
    double * final_duration = nullptr,
    bool * duration_extended = nullptr);

  ArmTrajectorySample sample(double elapsed_time) const;
  bool active() const { return active_; }
  void deactivate() { active_ = false; }
  bool finished(double elapsed_time) const;
  double duration() const { return duration_s_; }
  const ArmArray & goal() const { return goal_pos_; }
  const std::string & last_failure_reason() const { return last_failure_reason_; }

private:
  double constrained_duration(
    const ArmArray & start_pos,
    const ArmArray & goal_pos,
    double expected_duration,
    double max_velocity,
    double max_acceleration,
    double min_duration) const;

  bool satisfies_constraints(
    double max_velocity,
    double max_acceleration,
    std::size_t sample_count) const;

  std::string constraint_failure_summary(
    double max_velocity,
    double max_acceleration,
    const ArmArray & lower_limit,
    const ArmArray & upper_limit,
    std::size_t sample_count,
    double duration) const;

  std::string name_;
  std::array<MinSnapTrajectory, kArmJointCount> trajectories_{};
  ArmArray goal_pos_{};
  std::string last_failure_reason_;
  double duration_s_{0.0};
  bool active_{false};
};

}  // namespace min_snap
