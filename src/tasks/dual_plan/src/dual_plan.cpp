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

    // add collision 
    moveit_msgs::msg::CollisionObject collision_table;
    collision_table.header.frame_id = "world";
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
    RCLCPP_INFO(node->get_logger(), "Add an object into the world");
    node->add_collision_objects(collision_objects);


    geometry_msgs::msg::Pose left_target_pose;
    geometry_msgs::msg::Pose right_target_pose;

    left_target_pose.position.x = 0.4;
    left_target_pose.position.y = -0.2;
    left_target_pose.position.z = 0.3;
    right_target_pose.position.x = 0.4;
    right_target_pose.position.y = 0.2;
    right_target_pose.position.z = 0.3;

    // node->get_left_move_group()->getCurrentPose();

    tf2::Quaternion left_quat, right_quat;
    left_quat.setRPY(M_PI_2, -M_PI_2, 0.0);
    right_quat.setRPY(-M_PI_2, M_PI_2, 0.0);
    left_target_pose.orientation.x = left_quat.x();
    left_target_pose.orientation.y = left_quat.y();
    left_target_pose.orientation.z = left_quat.z();
    left_target_pose.orientation.w = left_quat.w();
    right_target_pose.orientation.x = right_quat.x();
    right_target_pose.orientation.y = right_quat.y();
    right_target_pose.orientation.z = right_quat.z();
    right_target_pose.orientation.w = right_quat.w();

    node->plan_and_execute(left_target_pose, right_target_pose, "left_base_link", "right_base_link", 
        "left_grasp_point", "right_grasp_point");
    



    rclcpp::shutdown();
    return 0;
}