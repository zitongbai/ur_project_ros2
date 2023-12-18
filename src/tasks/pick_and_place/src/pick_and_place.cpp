#include <rclcpp/rclcpp.hpp>
#include "ur_robotiq_rs_description/ur_robotiq_rs_node.h"


void str_list_2_double_list(const std::vector<std::string> & str_list, 
                            std::vector<std::vector<double>> & double_list){
    double_list.clear();
    // parse the string
    // each element in the list is a string
    // each string is a list of doubles, with ',' as delimiter
    for (auto & pose_str : str_list){
        std::vector<double> pose;
        std::stringstream ss(pose_str);
        std::string token;
        while (std::getline(ss, token, ',')){
            pose.push_back(std::stod(token));
        }
        double_list.push_back(pose);
    }
}


int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::NodeOptions node_options;
    node_options.automatically_declare_parameters_from_overrides(true);
    auto node = std::make_shared<UrRobotiqRsNode>("pick_and_place", node_options);
    node->init();
    node->set_planning_group_name("ur_manipulator").set_gripper_action_name("robotiq_gripper_controller/gripper_cmd");

    rclcpp::executors::SingleThreadedExecutor executor;
    executor.add_node(node);
    std::thread([&executor]() { executor.spin(); }).detach();

    // get target pose list from parameter
    const std::vector<std::string> target_pose_str_list = node->get_parameter("target_pose_list").as_string_array();
    // make sure it is not empty
    assert (target_pose_str_list.size() > 0);
    const std::vector<std::vector<double>> target_pose_list = [&target_pose_str_list](){
        std::vector<std::vector<double>> target_pose_list;
        str_list_2_double_list(target_pose_str_list, target_pose_list);
        return target_pose_list;
    }();
    const std::vector<geometry_msgs::msg::Pose> target_pose_msg_list = [&target_pose_list](){
        std::vector<geometry_msgs::msg::Pose> target_pose_msg_list;
        for (auto & pose : target_pose_list){
            geometry_msgs::msg::Pose pose_msg;
            pose_msg.position.x = pose[0];
            pose_msg.position.y = pose[1];
            pose_msg.position.z = pose[2];
            tf2::Quaternion quat;
            quat.setRPY(pose[3], pose[4], pose[5]);
            quat.normalize();
            pose_msg.orientation.x = quat.x();
            pose_msg.orientation.y = quat.y();
            pose_msg.orientation.z = quat.z();
            pose_msg.orientation.w = quat.w();
            target_pose_msg_list.push_back(pose_msg);
        }
        return target_pose_msg_list;
    }();
    const std::vector<double> gripper_position_list = [&target_pose_list](){
        std::vector<double> gripper_position_list;
        for (auto & pose : target_pose_list){
            gripper_position_list.push_back(pose[6]);
        }
        return gripper_position_list;
    }();

    for(size_t i=0; i<target_pose_msg_list.size(); i++){
        node->plan_and_execute(target_pose_msg_list[i]);
        node->grasp(gripper_position_list[i]);
        rclcpp::sleep_for(std::chrono::seconds(1));
    }

    rclcpp::shutdown();
    return 0;
}
