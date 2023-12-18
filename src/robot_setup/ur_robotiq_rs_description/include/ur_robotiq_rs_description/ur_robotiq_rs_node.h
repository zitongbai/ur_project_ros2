#ifndef UR_ROBOTIQ_RS_DESCRIPTION_INCLUDE_UR_ROBOTIQ_RS_NODE_H_
#define UR_ROBOTIQ_RS_DESCRIPTION_INCLUDE_UR_ROBOTIQ_RS_NODE_H_

#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <thread>
#include <chrono>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit_msgs/msg/display_robot_state.hpp>
#include <moveit_msgs/msg/display_trajectory.hpp>
#include <moveit_msgs/msg/attached_collision_object.hpp>
#include <moveit_msgs/msg/collision_object.hpp>
#include <control_msgs/action/gripper_command.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

class UrRobotiqRsNode : public rclcpp::Node{
public:
    using GripperCommand = control_msgs::action::GripperCommand;
    using GoalHandleGripperCommand = rclcpp_action::ClientGoalHandle<GripperCommand>;

    /**
     * @brief Construct a new Ur Robotiq Rs Node object
     * 
     * @param options 
     */
    explicit UrRobotiqRsNode(const std::string & node_name,
        const rclcpp::NodeOptions &options = rclcpp::NodeOptions());

    /**
     * @brief initialize move group in init function
     * 
    *  MoveGroupInterface needs the shared point of the Node, but shared_ptr won't be created until after the constructor returns
    *  so we need to create the move group interface in a separate function
    * @ref https://robotics.stackexchange.com/questions/96027/getting-a-nodesharedptr-from-this
     */
    void init();

    /**
     * @brief Plan the path to the target pose and execute it
     * 
     * @param target_pose 
     * @return true 
     * @return false 
     */
    bool plan_and_execute(const geometry_msgs::msg::Pose & target_pose);
    
    /**
     * @brief grasp the object
     * 
     * @param gripper_position target position for the robotiq gripper
     */
    void grasp(double gripper_position);

private:
    // moveit
    const std::string PLANNING_GROUP = "manipulator";
    moveit::planning_interface::MoveGroupInterfacePtr move_group_;
    moveit::planning_interface::PlanningSceneInterface planning_scene_interface_;
    /**
     * @brief convert cartesian target to joint target
     * 
     * @param target_pose cartesian target pose
     * @param joint_target joint target
     * @param reference_frame reference frame
     */
    void cartesian_target_2_joint_target(const geometry_msgs::msg::Pose & target_pose, 
                                        const std::string & reference_frame);

    // robotiq gripper
    const std::string gripper_action_name_ = "gripper_controller/gripper_cmd";
    rclcpp_action::Client<GripperCommand>::SharedPtr gripper_action_client_;
    void goal_response_callback(const GoalHandleGripperCommand::SharedPtr & goal_handle);
    void feedback_callback(GoalHandleGripperCommand::SharedPtr, const std::shared_ptr<const GripperCommand::Feedback> feedback);
    void result_callback(const GoalHandleGripperCommand::WrappedResult & result);
    rclcpp_action::Client<GripperCommand>::SendGoalOptions send_goal_options_;

    

};



#endif 