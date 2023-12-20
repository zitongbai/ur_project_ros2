#ifndef UR_FT_DESCRIPTION_INCLUDE_UR_FT_NODE_H_
#define UR_FT_DESCRIPTION_INCLUDE_UR_FT_NODE_H_

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
#include <geometry_msgs/msg/wrench.hpp>

class UrFTNode : public rclcpp::Node{
public:
    /**
     * @brief Construct a new Ur Robotiq Rs Node object
     * 
     * @param options 
     */
    explicit UrFTNode(const std::string & node_name,
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
     * @brief 链式构造
     */
    UrFTNode & set_planning_group_name(const std::string & planning_group_name){
        PLANNING_GROUP = planning_group_name;
        return *this;
    }

    UrFTNode & set_wrench_topic_name(const std::string & wrench_topic_name){
        wrench_topic_name_ = wrench_topic_name;
        return *this;
    }

    /**
     * @brief Plan the path to the target pose and execute it
     * 
     * @param target_pose 
     * @return true 
     * @return false 
     */
    bool plan_and_execute(const geometry_msgs::msg::Pose & target_pose);
    
    void add_collision_objects(std::vector<moveit_msgs::msg::CollisionObject> & collision_objects);

    moveit::planning_interface::MoveGroupInterfacePtr & get_move_group(){
        return move_group_;
    }

private:
    // moveit
    std::string PLANNING_GROUP = "ur_manipulator";
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


    // robotiq ft sensor wrench msg subscribe
    std::string wrench_topic_name_ = "/robotiq_ft300/wrench";
    rclcpp::Subscription<geometry_msgs::msg::Wrench>::SharedPtr wrench_sub_;
    geometry_msgs::msg::Wrench current_wrench_;
    void wrench_topic_callback(const geometry_msgs::msg::Wrench & msg){
        current_wrench_ = msg;
    }
};



#endif 