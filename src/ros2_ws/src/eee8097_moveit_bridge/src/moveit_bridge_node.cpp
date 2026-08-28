#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <functional>
#include <memory>
#include <mutex>
#include <sstream>
#include <stdexcept>
#include <string>
#include <vector>

#include <eee8097_interfaces/srv/get_end_effector_pose.hpp>
#include <eee8097_interfaces/srv/move_cartesian.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit_msgs/msg/collision_object.hpp>
#include <rclcpp/rclcpp.hpp>
#include <shape_msgs/msg/solid_primitive.hpp>
#include <std_srvs/srv/trigger.hpp>

using namespace std::chrono_literals;

namespace eee8097_moveit_bridge
{

template<typename ParameterT>
ParameterT get_or_declare_parameter(
  const rclcpp::Node::SharedPtr & node, const std::string & name,
  const ParameterT & default_value)
{
  if (node->has_parameter(name)) {
    return node->get_parameter(name).get_value<ParameterT>();
  }
  return node->declare_parameter<ParameterT>(name, default_value);
}

class MoveItBridge
{
public:
  explicit MoveItBridge(const rclcpp::Node::SharedPtr & node)
  : node_(node),
    planning_group_(get_or_declare_parameter<std::string>(node_, "planning_group", "hand")),
    end_effector_link_(
      get_or_declare_parameter<std::string>(node_, "end_effector_link", "hand_tcp")),
    move_group_(node_, planning_group_)
  {
    allow_execution_ = get_or_declare_parameter<bool>(node_, "allow_execution", false);
    planning_time_s_ = get_or_declare_parameter<double>(node_, "planning_time_s", 5.0);
    default_velocity_scaling_ =
      get_or_declare_parameter<double>(node_, "default_velocity_scaling", 0.15);
    default_acceleration_scaling_ =
      get_or_declare_parameter<double>(node_, "default_acceleration_scaling", 0.15);
    workspace_min_ = get_or_declare_parameter<std::vector<double>>(
      node_,
      "workspace_min", {-0.45, -0.45, -0.15});
    workspace_max_ = get_or_declare_parameter<std::vector<double>>(
      node_,
      "workspace_max", {0.45, 0.45, 0.55});
    table_enabled_ = get_or_declare_parameter<bool>(node_, "table.enabled", true);
    table_dimensions_ = get_or_declare_parameter<std::vector<double>>(
      node_,
      "table.dimensions", {0.80, 0.80, 0.04});
    table_position_ = get_or_declare_parameter<std::vector<double>>(
      node_,
      "table.position", {0.20, 0.0, -0.04});

    require_three(workspace_min_, "workspace_min");
    require_three(workspace_max_, "workspace_max");
    require_three(table_dimensions_, "table.dimensions");
    require_three(table_position_, "table.position");
    validate_scaling(default_velocity_scaling_, "default_velocity_scaling");
    validate_scaling(default_acceleration_scaling_, "default_acceleration_scaling");

    move_group_.setPlanningTime(planning_time_s_);
    move_group_.setPoseReferenceFrame("base_link");
    move_group_.setEndEffectorLink(end_effector_link_);

    pose_publisher_ = node_->create_publisher<geometry_msgs::msg::PoseStamped>(
      "/roarm/end_effector_pose", 10);
    pose_service_ = node_->create_service<eee8097_interfaces::srv::GetEndEffectorPose>(
      "/roarm/get_end_effector_pose",
      std::bind(
        &MoveItBridge::handle_get_pose, this, std::placeholders::_1,
        std::placeholders::_2));
    cartesian_service_ = node_->create_service<eee8097_interfaces::srv::MoveCartesian>(
      "/roarm/move_cartesian",
      std::bind(
        &MoveItBridge::handle_move_cartesian, this, std::placeholders::_1,
        std::placeholders::_2));
    reload_scene_service_ = node_->create_service<std_srvs::srv::Trigger>(
      "/roarm/reload_collision_scene",
      std::bind(
        &MoveItBridge::handle_reload_scene, this, std::placeholders::_1,
        std::placeholders::_2));
    pose_timer_ = node_->create_wall_timer(500ms, [this]() {publish_pose();});
    scene_timer_ = node_->create_wall_timer(2s, [this]() {
      if (apply_collision_scene()) {
        scene_timer_->cancel();
      }
    });

    if (allow_execution_) {
      RCLCPP_WARN(
        node_->get_logger(),
        "MoveIt execution is enabled. The downstream action still accepts only J1-J3; "
        "the gripper joint is absent from this planning group and controller.");
    } else {
      RCLCPP_INFO(
        node_->get_logger(),
        "Plan-only mode active. Set execute=true requests will be refused after planning.");
    }
  }

private:
  static void require_three(const std::vector<double> & values, const std::string & name)
  {
    if (values.size() != 3U) {
      throw std::invalid_argument(name + " must contain exactly three values");
    }
    for (const auto value : values) {
      if (!std::isfinite(value)) {
        throw std::invalid_argument(name + " must contain finite values");
      }
    }
  }

  static void validate_scaling(double value, const std::string & name)
  {
    if (!std::isfinite(value) || value <= 0.0 || value > 1.0) {
      throw std::invalid_argument(name + " must be in (0, 1]");
    }
  }

  bool target_in_workspace(double x, double y, double z, std::string & reason) const
  {
    const std::vector<double> target{x, y, z};
    for (std::size_t index = 0; index < target.size(); ++index) {
      if (!std::isfinite(target[index])) {
        reason = "target contains a non-finite coordinate";
        return false;
      }
      if (target[index] < workspace_min_[index] || target[index] > workspace_max_[index]) {
        std::ostringstream stream;
        stream << "target axis " << index << "=" << target[index]
               << " is outside [" << workspace_min_[index] << ", "
               << workspace_max_[index] << "] m";
        reason = stream.str();
        return false;
      }
    }
    return true;
  }

  double request_scaling(double requested, double fallback, const std::string & name) const
  {
    if (requested == 0.0) {
      return fallback;
    }
    validate_scaling(requested, name);
    return requested;
  }

  bool apply_collision_scene()
  {
    if (!table_enabled_) {
      scene_ready_ = true;
      return true;
    }
    moveit_msgs::msg::CollisionObject table;
    table.header.frame_id = "base_link";
    table.id = "support_table";

    shape_msgs::msg::SolidPrimitive primitive;
    primitive.type = shape_msgs::msg::SolidPrimitive::BOX;
    primitive.dimensions = table_dimensions_;

    geometry_msgs::msg::Pose pose;
    pose.orientation.w = 1.0;
    pose.position.x = table_position_[0];
    pose.position.y = table_position_[1];
    pose.position.z = table_position_[2];
    table.primitives.push_back(primitive);
    table.primitive_poses.push_back(pose);
    table.operation = moveit_msgs::msg::CollisionObject::ADD;

    scene_ready_ = planning_scene_interface_.applyCollisionObject(table);
    if (scene_ready_) {
      RCLCPP_INFO(
        node_->get_logger(),
        "Applied support_table collision box in base_link: size=[%.3f, %.3f, %.3f] m",
        table_dimensions_[0], table_dimensions_[1], table_dimensions_[2]);
    } else {
      RCLCPP_WARN(node_->get_logger(), "Planning scene not ready; will retry");
    }
    return scene_ready_;
  }

  void publish_pose()
  {
    try {
      pose_publisher_->publish(move_group_.getCurrentPose(end_effector_link_));
    } catch (const std::exception & exception) {
      RCLCPP_WARN_THROTTLE(
        node_->get_logger(), *node_->get_clock(), 5000,
        "Unable to publish end-effector pose: %s", exception.what());
    }
  }

  void handle_get_pose(
    const std::shared_ptr<eee8097_interfaces::srv::GetEndEffectorPose::Request> request,
    std::shared_ptr<eee8097_interfaces::srv::GetEndEffectorPose::Response> response)
  {
    (void)request;
    std::lock_guard<std::mutex> lock(operation_mutex_);
    try {
      response->pose = move_group_.getCurrentPose(end_effector_link_);
      response->success = !response->pose.header.frame_id.empty();
      response->message = response->success ?
        "current hand_tcp pose from MoveIt robot state" :
        "current pose has no frame; check /joint_states and TF";
    } catch (const std::exception & exception) {
      response->success = false;
      response->message = exception.what();
    }
  }

  void handle_move_cartesian(
    const std::shared_ptr<eee8097_interfaces::srv::MoveCartesian::Request> request,
    std::shared_ptr<eee8097_interfaces::srv::MoveCartesian::Response> response)
  {
    std::lock_guard<std::mutex> lock(operation_mutex_);
    std::string workspace_reason;
    if (!target_in_workspace(request->x, request->y, request->z, workspace_reason)) {
      response->message = "request rejected before planning: " + workspace_reason;
      return;
    }

    try {
      const auto velocity = request_scaling(
        request->velocity_scaling, default_velocity_scaling_, "velocity_scaling");
      const auto acceleration = request_scaling(
        request->acceleration_scaling, default_acceleration_scaling_,
        "acceleration_scaling");
      move_group_.setMaxVelocityScalingFactor(velocity);
      move_group_.setMaxAccelerationScalingFactor(acceleration);
      move_group_.setStartStateToCurrentState();
      if (!move_group_.setPositionTarget(
          request->x, request->y, request->z, end_effector_link_))
      {
        response->message = "IK rejected the Cartesian target";
        move_group_.clearPoseTargets();
        return;
      }

      moveit::planning_interface::MoveGroupInterface::Plan plan;
      const auto planning_result = move_group_.plan(plan);
      response->planned = planning_result == moveit::core::MoveItErrorCode::SUCCESS;
      response->planning_time = plan.planning_time_;
      response->trajectory_points = static_cast<int32_t>(
        plan.trajectory_.joint_trajectory.points.size());
      if (!response->planned) {
        response->message =
          "MoveIt found no collision-free IK trajectory; inspect target, table, and current state";
        move_group_.clearPoseTargets();
        return;
      }

      if (!request->execute) {
        response->success = true;
        response->message = "collision-free arm-only plan generated; no motion requested";
        move_group_.clearPoseTargets();
        return;
      }
      if (!allow_execution_) {
        response->message =
          "plan generated but execution is locked; restart with allow_execution:=true";
        move_group_.clearPoseTargets();
        return;
      }

      const auto execution_result = move_group_.execute(plan);
      response->executed = execution_result == moveit::core::MoveItErrorCode::SUCCESS;
      response->success = response->executed;
      response->message = response->executed ?
        "arm-only plan executed; no gripper command was generated" :
        "trajectory execution failed or was rejected by the guarded driver";
      move_group_.clearPoseTargets();
    } catch (const std::exception & exception) {
      move_group_.clearPoseTargets();
      response->message = std::string("planning request failed: ") + exception.what();
    }
  }

  void handle_reload_scene(
    const std::shared_ptr<std_srvs::srv::Trigger::Request> request,
    std::shared_ptr<std_srvs::srv::Trigger::Response> response)
  {
    (void)request;
    std::lock_guard<std::mutex> lock(operation_mutex_);
    response->success = apply_collision_scene();
    response->message = response->success ?
      "collision scene applied" : "MoveIt planning scene service is not ready";
  }

  rclcpp::Node::SharedPtr node_;
  std::string planning_group_;
  std::string end_effector_link_;
  moveit::planning_interface::MoveGroupInterface move_group_;
  moveit::planning_interface::PlanningSceneInterface planning_scene_interface_;
  bool allow_execution_{false};
  bool table_enabled_{true};
  bool scene_ready_{false};
  double planning_time_s_{5.0};
  double default_velocity_scaling_{0.15};
  double default_acceleration_scaling_{0.15};
  std::vector<double> workspace_min_;
  std::vector<double> workspace_max_;
  std::vector<double> table_dimensions_;
  std::vector<double> table_position_;
  std::mutex operation_mutex_;
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr pose_publisher_;
  rclcpp::Service<eee8097_interfaces::srv::GetEndEffectorPose>::SharedPtr pose_service_;
  rclcpp::Service<eee8097_interfaces::srv::MoveCartesian>::SharedPtr cartesian_service_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr reload_scene_service_;
  rclcpp::TimerBase::SharedPtr pose_timer_;
  rclcpp::TimerBase::SharedPtr scene_timer_;
};

}  // namespace eee8097_moveit_bridge

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::NodeOptions options;
  options.automatically_declare_parameters_from_overrides(true);
  auto node = rclcpp::Node::make_shared("eee8097_moveit_bridge", options);
  auto bridge = std::make_shared<eee8097_moveit_bridge::MoveItBridge>(node);
  (void)bridge;
  rclcpp::executors::MultiThreadedExecutor executor;
  executor.add_node(node);
  executor.spin();
  rclcpp::shutdown();
  return 0;
}
