#include <rclcpp/rclcpp.hpp>
#include "ur_ft_description/ur_ft_node.h"


int main(int argc, char ** argv){
    rclcpp::init(argc, argv);
    rclcpp::NodeOptions node_options;
    node_options.automatically_declare_parameters_from_overrides(true);
    auto node = std::make_shared<UrFTNode>("find_hole", node_options);
    node->init();
    node->set_planning_group_name("ur_manipulator").set_wrench_topic_name("/robotiq_ft300/wrench");

    rclcpp::executors::SingleThreadedExecutor executor;
    executor.add_node(node);
    std::thread([&executor]() { executor.spin(); }).detach();

    // // add collision object
    // moveit_msgs::msg::CollisionObject collision_table;
    // collision_table.header.frame_id = "world";
    // collision_table.id = "table";
    // shape_msgs::msg::SolidPrimitive primitive;
    // primitive.type = primitive.BOX;
    // primitive.dimensions.resize(3);
    // primitive.dimensions[primitive.BOX_X] = 0.3;
    // primitive.dimensions[primitive.BOX_Y] = 0.3;
    // primitive.dimensions[primitive.BOX_Z] = 0.058; // it is slightly lower than real height
    // // the pose of the table should be the same as that in `sim_env.world`
    // geometry_msgs::msg::Pose box_pose;
    // box_pose.orientation.w = 1.0;
    // box_pose.position.x = 0.5;
    // box_pose.position.y = 0.0;
    // box_pose.position.z = primitive.dimensions[primitive.BOX_Z]/2.0;
    // collision_table.primitives.push_back(primitive);
    // collision_table.primitive_poses.push_back(box_pose);
    // collision_table.operation = collision_table.ADD;
    // // create collision_objects for planning_scene_interface
    // std::vector<moveit_msgs::msg::CollisionObject> collision_objects;
    // collision_objects.push_back(collision_table);
    // node->add_collision_objects(collision_objects);
    // RCLCPP_INFO(node->get_logger(), "Add collision objects successfully");


    auto & move_group = node->get_move_group();
    std::string planning_frame = "base_link";

    // first make sure the tip link is in horizontal plane
    geometry_msgs::msg::Pose target_pose = move_group->getCurrentPose().pose;
    tf2::Quaternion quat;
    quat.setRPY(0, M_PI, 0);
    quat.normalize();
    target_pose.orientation.w = quat.w();
    target_pose.orientation.x = quat.x();
    target_pose.orientation.y = quat.y();
    target_pose.orientation.z = quat.z();
    target_pose.position.x = 0.46;
    target_pose.position.y = 0.0;
    target_pose.position.z = 0.0588;
    node->plan_and_execute(target_pose, planning_frame);

    target_pose.position.x = 0.5;
    target_pose.position.y = 0.0;
    target_pose.position.z = 0.058;
    node->plan_and_execute_cartesian(target_pose, planning_frame);

    target_pose.position.x = 0.5;
    target_pose.position.y = 0.0;
    target_pose.position.z = 0.03;
    node->plan_and_execute_cartesian(target_pose, planning_frame);

    target_pose.position.x = 0.5;
    target_pose.position.y = 0.0;
    target_pose.position.z = 0.07;
    node->plan_and_execute_cartesian(target_pose, planning_frame);

    
    
    


    rclcpp::shutdown();
    return 0;
}