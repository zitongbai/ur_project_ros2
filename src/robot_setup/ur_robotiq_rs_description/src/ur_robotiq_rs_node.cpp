#include "ur_robotiq_rs_description/ur_robotiq_rs_node.h"

UrRobotiqRsNode::UrRobotiqRsNode(const std::string & node_name, const rclcpp::NodeOptions &options)
        : Node(node_name, options){
    // create action client
    gripper_action_client_ = rclcpp_action::create_client<GripperCommand>(this, gripper_action_name_);
    send_goal_options_.goal_response_callback = std::bind(&UrRobotiqRsNode::goal_response_callback, this, std::placeholders::_1);
    send_goal_options_.feedback_callback = std::bind(&UrRobotiqRsNode::feedback_callback, this, std::placeholders::_1, std::placeholders::_2);
    send_goal_options_.result_callback = std::bind(&UrRobotiqRsNode::result_callback, this, std::placeholders::_1);
    // wait for action server to come up
    while (!gripper_action_client_->wait_for_action_server(std::chrono::seconds(1))) {
        RCLCPP_INFO(this->get_logger(), "Waiting for action server to come up...");
    }
}

void UrRobotiqRsNode::init(){
    // moveit
    move_group_ = std::make_shared<moveit::planning_interface::MoveGroupInterface>(shared_from_this(), PLANNING_GROUP);
    RCLCPP_INFO(this->get_logger(), "Reference frame: %s", move_group_->getPlanningFrame().c_str());

    move_group_->setPlanningTime(5.0);
    move_group_->setNumPlanningAttempts(5);
    move_group_->setMaxVelocityScalingFactor(0.5);
    move_group_->setMaxAccelerationScalingFactor(0.5);
    move_group_->setGoalTolerance(0.01);
    move_group_->setGoalJointTolerance(0.01);
    move_group_->setGoalOrientationTolerance(0.01);
    move_group_->setGoalPositionTolerance(0.01);
    move_group_->setPlannerId("RRTConnectkConfigDefault");
    move_group_->setWorkspace(-1.0, -1.0, -1.0, 1.0, 1.0, 1.0);

    // move to ready configuration
    move_group_->setStartStateToCurrentState();
    move_group_->allowReplanning(true);
    move_group_->setNamedTarget("ready");
    move_group_->move();

    /* add collision objects */
    moveit_msgs::msg::CollisionObject collision_table;
    collision_table.header.frame_id = move_group_->getPlanningFrame(); // world
    collision_table.id = "table";
    // define box to add to the world
    shape_msgs::msg::SolidPrimitive primitive;
    primitive.type = primitive.BOX;
    primitive.dimensions.resize(3);
    primitive.dimensions[primitive.BOX_X] = 1.0;
    primitive.dimensions[primitive.BOX_Y] = 1.5;
    primitive.dimensions[primitive.BOX_Z] = 1.0;
    geometry_msgs::msg::Pose box_pose;
    box_pose.orientation.w = 1.0;
    box_pose.position.x = 0.5;
    box_pose.position.y = 0.0;
    box_pose.position.z = 0.5;
    collision_table.primitives.push_back(primitive);
    collision_table.primitive_poses.push_back(box_pose);
    collision_table.operation = collision_table.ADD;
    std::vector<moveit_msgs::msg::CollisionObject> collision_objects;
    collision_objects.push_back(collision_table);
    RCLCPP_INFO(this->get_logger(), "Add an object into the world");
    planning_scene_interface_.addCollisionObjects(collision_objects);

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