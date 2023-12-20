#include <rclcpp/rclcpp.hpp>
#include "dual_ur_robotiq_rs_description/dual_ur_robotiq_rs_node.h"

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::NodeOptions node_options;
    node_options.automatically_declare_parameters_from_overrides(true);
    auto node = std::make_shared<DualUrRobotiqRsNode>("dual_plan", node_options);
    node->init();
    node->set_both_planning_group_name("both_manipulators")
        .set_left_planning_group_name("left_ur_manipulator")
        .set_right_planning_group_name("right_ur_manipulator");
    
    rclcpp::executors::SingleThreadedExecutor executor;
    executor.add_node(node);
    std::thread([&executor]() { executor.spin(); }).detach();

    geometry_msgs::msg::Pose left_target_pose = 
        node->get_left_move_group()->getCurrentPose().pose;
    geometry_msgs::msg::Pose right_target_pose =
        node->get_right_move_group()->getCurrentPose().pose;

    left_target_pose.position.x += 0.2;
    right_target_pose.position.x += 0.2;

    tf2::Quaternion left_quat, right_quat;
    left_quat.setRPY(0, M_PI_2, -M_PI_2);
    right_quat.setRPY(0, -M_PI_2, M_PI_2);
    left_quat.normalize();
    right_quat.normalize();
    left_target_pose.orientation.x = left_quat.x();
    left_target_pose.orientation.y = left_quat.y();
    left_target_pose.orientation.z = left_quat.z();
    left_target_pose.orientation.w = left_quat.w();
    right_target_pose.orientation.x = right_quat.x();
    right_target_pose.orientation.y = right_quat.y();
    right_target_pose.orientation.z = right_quat.z();
    right_target_pose.orientation.w = right_quat.w();

    node->plan_and_execute(left_target_pose, right_target_pose);
    



    rclcpp::shutdown();
    return 0;
}