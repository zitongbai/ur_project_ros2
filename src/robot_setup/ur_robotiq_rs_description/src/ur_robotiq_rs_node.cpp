#include "ur_robotiq_rs_description/ur_robotiq_rs_node.h"

UrRobotiqRsNode::UrRobotiqRsNode(const std::string & node_name, const rclcpp::NodeOptions &options)
        : Node(node_name, options){
    // create action client
    gripper_action_client_ = rclcpp_action::create_client<GripperCommand>(this, gripper_action_name_);
    send_goal_options_.goal_response_callback = std::bind(&UrRobotiqRsNode::goal_response_callback, this, std::placeholders::_1);
    send_goal_options_.feedback_callback = std::bind(&UrRobotiqRsNode::feedback_callback, this, std::placeholders::_1, std::placeholders::_2);
    send_goal_options_.result_callback = std::bind(&UrRobotiqRsNode::result_callback, this, std::placeholders::_1);
    // wait for action server to come up
    RCLCPP_INFO(this->get_logger(), "Waiting for action server");
    gripper_action_client_->wait_for_action_server(std::chrono::seconds(10));
    if (!gripper_action_client_->action_server_is_ready()) {
        RCLCPP_ERROR(this->get_logger(), "Action server not available after waiting");
        throw std::runtime_error("Action server not available");
    }
}

void UrRobotiqRsNode::init(){
    // moveit
    move_group_ = std::make_shared<moveit::planning_interface::MoveGroupInterface>(shared_from_this(), PLANNING_GROUP);
    RCLCPP_INFO(this->get_logger(), "Reference frame: %s", move_group_->getPlanningFrame().c_str());

    // move_group_->setPlanningTime(5.0);
    // move_group_->setNumPlanningAttempts(5);
    // move_group_->setMaxVelocityScalingFactor(0.5);
    // move_group_->setMaxAccelerationScalingFactor(0.5);
    // move_group_->setGoalTolerance(0.01);
    // move_group_->setGoalJointTolerance(0.01);
    // move_group_->setGoalOrientationTolerance(0.01);
    // move_group_->setGoalPositionTolerance(0.01);
    // move_group_->setPlannerId("RRTConnectkConfigDefault");

    // move to ready configuration
    move_group_->setStartStateToCurrentState();
    move_group_->allowReplanning(true);
    move_group_->setNamedTarget("ready");
    move_group_->move();

}

void UrRobotiqRsNode::cartesian_target_2_joint_target(
        const geometry_msgs::msg::Pose & target_pose, 
        const std::string & reference_frame){
    geometry_msgs::msg::PoseStamped target_pose_stamped;
    target_pose_stamped.header.frame_id = reference_frame;
    target_pose_stamped.pose = target_pose;

    // Set the joint state goal for a particular joint by computing IK.
    move_group_->setJointValueTarget(target_pose_stamped);
}


bool UrRobotiqRsNode::plan_and_execute(const geometry_msgs::msg::Pose & target_pose){
    cartesian_target_2_joint_target(target_pose, "base_link");
    moveit::planning_interface::MoveGroupInterface::Plan my_plan;
    bool success = (move_group_->plan(my_plan) == moveit::core::MoveItErrorCode::SUCCESS);
    if(success){
        move_group_->execute(my_plan);
        RCLCPP_INFO(this->get_logger(), "Plan and execute successfully");
    }
    return success;
}


void UrRobotiqRsNode::grasp(double gripper_position){
    auto gripper_goal_msg = GripperCommand::Goal();
    gripper_goal_msg.command.position = gripper_position;
    gripper_goal_msg.command.max_effort = -1.0; // do not limit the effort

    RCLCPP_INFO(this->get_logger(), "Sending gripper goal");
    auto f = gripper_action_client_->async_send_goal(gripper_goal_msg, send_goal_options_);
    // TODO: find a way to check action
}

void UrRobotiqRsNode::goal_response_callback(const GoalHandleGripperCommand::SharedPtr & goal_handle){
  if(!goal_handle){
    RCLCPP_ERROR(this->get_logger(), "Goal was rejected by server");
  } else {
    RCLCPP_INFO(this->get_logger(), "Goal accepted by server, waiting for result");
  }
}
void UrRobotiqRsNode::feedback_callback(GoalHandleGripperCommand::SharedPtr, const std::shared_ptr<const GripperCommand::Feedback> feedback){
  RCLCPP_INFO(this->get_logger(), "Got Feedback: Current position is %f", feedback->position);
}
void UrRobotiqRsNode::result_callback(const GoalHandleGripperCommand::WrappedResult & result){
  switch (result.code)
  {
  case rclcpp_action::ResultCode::SUCCEEDED:
    break;
  case rclcpp_action::ResultCode::ABORTED:
    RCLCPP_ERROR(this->get_logger(), "Goal was aborted");
    return;
  case rclcpp_action::ResultCode::CANCELED:
    RCLCPP_ERROR(this->get_logger(), "Goal was canceled");
    return;
  default:
    RCLCPP_ERROR(this->get_logger(), "Unknown result code");
    return;
  }
  RCLCPP_INFO(this->get_logger(), "Goal is completed, current position is %f", result.result->position);
}