#include "ur_ft_description/ur_ft_node.h"

UrFTNode::UrFTNode(const std::string & node_name, const rclcpp::NodeOptions &options)
        : Node(node_name, options){
    move_group_ = nullptr;

    // wrench subscription
    wrench_sub_ = this->create_subscription<geometry_msgs::msg::Wrench>(
        wrench_topic_name_, 10, std::bind(UrFTNode::wrench_topic_callback, this, std::placeholders::_1)
    );
}

void UrFTNode::init(){
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

void UrFTNode::cartesian_target_2_joint_target(
        const geometry_msgs::msg::Pose & target_pose, 
        const std::string & reference_frame){
    geometry_msgs::msg::PoseStamped target_pose_stamped;
    target_pose_stamped.header.frame_id = reference_frame;
    target_pose_stamped.pose = target_pose;

    // Set the joint state goal for a particular joint by computing IK.
    move_group_->setJointValueTarget(target_pose_stamped);
}


bool UrFTNode::plan_and_execute(const geometry_msgs::msg::Pose & target_pose){
    cartesian_target_2_joint_target(target_pose, "base_link");
    moveit::planning_interface::MoveGroupInterface::Plan my_plan;
    bool success = (move_group_->plan(my_plan) == moveit::core::MoveItErrorCode::SUCCESS);
    if(success){
        move_group_->execute(my_plan);
        RCLCPP_INFO(this->get_logger(), "Plan and execute successfully");
    }
    return success;
}

void UrFTNode::add_collision_objects(std::vector<moveit_msgs::msg::CollisionObject> & collision_objects){
    planning_scene_interface_.addCollisionObjects(collision_objects);
}