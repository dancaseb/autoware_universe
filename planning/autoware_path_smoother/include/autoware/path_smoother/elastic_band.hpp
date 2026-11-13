// Copyright 2023 TIER IV, Inc.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#ifndef AUTOWARE__PATH_SMOOTHER__ELASTIC_BAND_HPP_
#define AUTOWARE__PATH_SMOOTHER__ELASTIC_BAND_HPP_

#include "autoware/osqp_interface/osqp_interface.hpp"
#include "autoware/path_smoother/common_structs.hpp"
#include "autoware/path_smoother/type_alias.hpp"

#include <Eigen/Core>

#include <functional>
#include <memory>
#include <optional>
#include <string>
#include <tuple>
#include <utility>
#include <vector>

namespace autoware::agnocast_wrapper
{
class Node;
}  // namespace autoware::agnocast_wrapper

namespace autoware::path_smoother
{
class EBPathSmoother
{
public:
  EBPathSmoother(
    rclcpp::Node * node, const bool enable_debug_info, const EgoNearestParam ego_nearest_param,
    const CommonParam & common_param, const std::shared_ptr<TimeKeeper> time_keeper_ptr);
  EBPathSmoother(
    autoware::agnocast_wrapper::Node * node, const bool enable_debug_info,
    const EgoNearestParam ego_nearest_param, const CommonParam & common_param,
    const std::shared_ptr<TimeKeeper> time_keeper_ptr);

  std::vector<TrajectoryPoint> smoothTrajectory(
    const std::vector<TrajectoryPoint> & traj_points, const geometry_msgs::msg::Pose & ego_pose);

  void initialize(const bool enable_debug_info, const CommonParam & common_param);
  void resetPreviousData();
  void onParam(const std::vector<rclcpp::Parameter> & parameters);

private:
  struct EBParam
  {
    // qp
    struct QPParam
    {
      int max_iteration;
      double eps_abs;
      double eps_rel;
    };

    EBParam() = default;
    template <typename NodeT>
    explicit EBParam(NodeT * node);
    void onParam(const std::vector<rclcpp::Parameter> & parameters);

    // option
    bool enable_warm_start;
    bool enable_optimization_validation;

    // common
    double delta_arc_length;
    int num_points;

    // clearance
    int num_joint_points;
    double clearance_for_fix;
    double clearance_for_joint;
    double clearance_for_smooth;

    // weight
    double smooth_weight;
    double lat_error_weight;

    // qp
    QPParam qp_param;

    // validation
    double max_validation_error;
  };

  struct Constraint2d
  {
    struct Constraint
    {
      Eigen::Vector2d coef;
      double upper_bound;
      double lower_bound;
    };

    Constraint lon;
    Constraint lat;
  };

  // arguments
  bool enable_debug_info_;
  EgoNearestParam ego_nearest_param_;
  CommonParam common_param_;
  EBParam eb_param_;
  mutable std::shared_ptr<TimeKeeper> time_keeper_ptr_;
  rclcpp::Logger logger_;
  rclcpp::Clock clock_;

  // publisher
  std::function<void(const Trajectory &)> debug_eb_traj_pub_;
  std::function<void(const Trajectory &)> debug_eb_fixed_traj_pub_;

  template <typename NodeT>
  void setUpPublishers(NodeT * node);

  std::unique_ptr<autoware::osqp_interface::OSQPInterface> osqp_solver_ptr_;
  std::shared_ptr<std::vector<TrajectoryPoint>> prev_eb_traj_points_ptr_{nullptr};

  // QP dump for offline analysis. Enabled only when the environment variable
  // EB_QP_DUMP_DIR is set (EB_QP_DUMP_MAX limits the number of dumped problems).
  struct QPDump
  {
    std::string dir;
    size_t max_count{1000};
    size_t count{0};
    // last assembled problem: min 1/2 x'Px + q'x  s.t.  l <= x <= u  (A = I)
    Eigen::MatrixXd P;
    std::vector<double> q, l, u;
    std::vector<double> ref_x, ref_y, ref_yaw;  // reference points the offsets are applied to
    bool warm_start_update{false};
    double assemble_ms{0.0};
    double osqp_update_ms{0.0};
  };
  std::optional<QPDump> qp_dump_;

  void initQPDump();
  void dumpQP(
    const std::vector<double> & solution, const double solve_ms, const int status,
    const int64_t iterations, const double osqp_run_time_s, const double obj_val);

  std::vector<TrajectoryPoint> insertFixedPoint(
    const std::vector<TrajectoryPoint> & traj_point) const;

  std::tuple<std::vector<TrajectoryPoint>, size_t> getPaddedTrajectoryPoints(
    const std::vector<TrajectoryPoint> & traj_points) const;

  void updateConstraint(
    const std::vector<TrajectoryPoint> & traj_points, const bool is_goal_contained,
    const int pad_start_idx);

  std::optional<std::vector<double>> calcSmoothedTrajectory();

  std::optional<std::vector<TrajectoryPoint>> convertOptimizedPointsToTrajectory(
    const std::vector<double> & optimized_points, const std::vector<TrajectoryPoint> & traj_points,
    const int pad_start_idx) const;
};
}  // namespace autoware::path_smoother

#endif  // AUTOWARE__PATH_SMOOTHER__ELASTIC_BAND_HPP_
