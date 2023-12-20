#include "dual_ur_robotiq_rs_description/dual_ur_robotiq_rs_node.h"

DualUrRobotiqRsNode::DualUrRobotiqRsNode(const std::string & node_name, const rclcpp::NodeOptions &options)
        : Node(node_name, options){
    // create action client
    left_gripper_action_client_ = 
        rclcpp_action::create_client<GripperCommand>(this, left_gripper_action_name_);
    right_gripper_action_client_ =
        rclcpp_action::create_client<GripperCommand>(this, right_gripper_action_name_);
    send_goal_options_.goal_response_callback = 
        std::bind(&DualUrRobotiqRsNode::goal_response_callback, this, std::placeholders::_1);
    send_goal_options_.feedback_callback =
        std::bind(&DualUrRobotiqRsNode::feedback_callback, this, std::placeholders::_1, std::placeholders::_2);
    send_goal_options_.result_callback =
        std::bind(&DualUrRobotiqRsNode::result_callback, this, std::placeholders::_1);
    // wait for action server to come up
    RCLCPP_INFO(this->get_logger(), "Waiting for action server");
    left_gripper_action_client_->wait_for_action_server(std::chrono::seconds(5));
    right_gripper_action_client_->wait_for_action_server(std::chrono::seconds(5));
    if (!left_gripper_action_client_->action_server_is_ready() || 
        !right_gripper_action_client_->action_server_is_ready()) {
        RCLCPP_ERROR(this->get_logger(), "Action server not available after waiting");
        throw std::runtime_error("Action server not available");
    }
}

void DualUrRobotiqRsNode::init(){
    // moveit 
    both_move_group_ = 
        std::make_shared<moveit::planning_interface::MoveGroupInterface>(
            shared_from_this(), both_planning_group_name_);
    left_move_group_ =
        std::make_shared<moveit::planning_interface::MoveGroupInterface>(
            shared_from_this(), left_planning_group_name_);
    right_move_group_ =
        std::make_shared<moveit::planning_interface::MoveGroupInterface>(
            shared_from_this(), right_planning_group_name_);

    // for fast demo
    both_move_group_->setMaxAccelerationScalingFactor(0.3);
    both_move_group_->setMaxVelocityScalingFactor(0.3);

    both_move_group_->setPlanningPipelineId("ompl"); // refer to the planning_pipeline_config.yaml
    both_move_group_->setPlannerId("RRTConnectkConfigDefault");
    left_move_group_->setPlanningPipelineId("ompl"); // refer to the planning_pipeline_config.yaml
    left_move_group_->setPlannerId("RRTConnectkConfigDefault");
    right_move_group_->setPlanningPipelineId("ompl"); // refer to the planning_pipeline_config.yaml
    right_move_group_->setPlannerId("RRTConnectkConfigDefault");

    // left_move_group_->setStartStateToCurrentState();
    // left_move_group_->setNamedTarget("left_ready");
    // left_move_group_->move();
    // right_move_group_->setStartStateToCurrentState();
    // right_move_group_->setNamedTarget("right_ready");
    // right_move_group_->move();
}


bool DualUrRobotiqRsNode::plan_and_execute(
        const geometry_msgs::msg::Pose & left_target_pose, 
        const geometry_msgs::msg::Pose & right_target_pose, 
        const std::string & left_reference_frame, 
        const std::string & right_reference_frame, 
        const std::string & left_end_effector_link,
        const std::string & right_end_effector_link){
    
    auto const left_target_pose_stamped = [&left_target_pose, &left_reference_frame]{
        geometry_msgs::msg::PoseStamped msg;
        msg.header.frame_id = left_reference_frame;
        msg.pose = left_target_pose;
        return msg;
    }();
    auto const right_target_pose_stamped = [&right_target_pose, &right_reference_frame]{
        geometry_msgs::msg::PoseStamped msg;
        msg.header.frame_id = right_reference_frame;
        msg.pose = right_target_pose;
        return msg;
    }();

    left_target_joint_.clear();
    right_target_joint_.clear();
    both_target_joint_.clear();

    left_move_group_->setStartStateToCurrentState();
    left_move_group_->setJointValueTarget(left_target_pose_stamped, left_end_effector_link);
    left_move_group_->getJointValueTarget(left_target_joint_); // use ik to get joint target from target pose
    right_move_group_->setStartStateToCurrentState();
    right_move_group_->setJointValueTarget(right_target_pose_stamped, right_end_effector_link);
    right_move_group_->getJointValueTarget(right_target_joint_); // use ik to get joint target from target pose

    // combine both target joint
    both_target_joint_.insert(both_target_joint_.end(), left_target_joint_.begin(), left_target_joint_.end());
    both_target_joint_.insert(both_target_joint_.end(), right_target_joint_.begin(), right_target_joint_.end());

    // set the target positions for both manipulators in joint space
    both_move_group_->setStartStateToCurrentState();
    both_move_group_->setJointValueTarget(both_target_joint_);
    moveit::planning_interface::MoveGroupInterface::Plan my_plan;
    bool success = (both_move_group_->plan(my_plan) == moveit::core::MoveItErrorCode::SUCCESS);
    if(success){
        both_move_group_->execute(my_plan);
        RCLCPP_INFO(this->get_logger(), "Plan and execute successfully");
    } else {
        RCLCPP_ERROR(this->get_logger(), "Plan and execute failed");
    }
    return success;
}



void DualUrRobotiqRsNode::goal_response_callback(const GoalHandleGripperCommand::SharedPtr & goal_handle){
  if(!goal_handle){
    RCLCPP_ERROR(this->get_logger(), "Goal was rejected by server");
  } else {
    RCLCPP_INFO(this->get_logger(), "Goal accepted by server, waiting for result");
  }
}
void DualUrRobotiqRsNode::feedback_callback(GoalHandleGripperCommand::SharedPtr, const std::shared_ptr<const GripperCommand::Feedback> feedback){
  RCLCPP_INFO(this->get_logger(), "Got Feedback: Current position is %f", feedback->position);
}
void DualUrRobotiqRsNode::result_callback(const GoalHandleGripperCommand::WrappedResult & result){
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