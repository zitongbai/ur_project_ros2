#include <rclcpp/rclcpp.hpp>
#include "dual_ur_robotiq_rs_description/dual_ur_robotiq_rs_node.h"
#include <moveit_visual_tools/moveit_visual_tools.h>


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

    std::string left_ee_link = "left_grasp_point";
    std::string right_ee_link = "right_grasp_point";
    std::string left_planning_frame = "left_base_link";
    std::string right_planning_frame = "right_base_link";

    geometry_msgs::msg::Pose left_target_pose;
    geometry_msgs::msg::Pose right_target_pose;

    // // TEST
    // auto test_pose_stamped = node->get_left_move_group()->getCurrentPose();
    // RCLCPP_INFO(node->get_logger(), "The frame is: %s", test_pose_stamped.header.frame_id.c_str());
    // node->get_left_move_group()->setPoseReferenceFrame("left_planning_frame");
    // test_pose_stamped = node->get_left_move_group()->getCurrentPose();
    // RCLCPP_INFO(node->get_logger(), "The frame is: %s", test_pose_stamped.header.frame_id.c_str());

    // -------------------------------------------------------------
    // visualization in rviz
    // -------------------------------------------------------------
    namespace rvt = rviz_visual_tools;
    moveit_visual_tools::MoveItVisualTools visual_tools(
        node, "world", rviz_visual_tools::RVIZ_MARKER_TOPIC,
        node->get_both_move_group()->getRobotModel()
    );
    visual_tools.deleteAllMarkers();
    visual_tools.loadRemoteControl();
    // create some closures (function objects that have access to variables 
    // in our current scope) that we can use later in our program to help render 
    // visualizations in RViz
    auto const draw_title = [&visual_tools](auto text) {
        auto const text_pose = [] {
            auto msg = Eigen::Isometry3d::Identity();
            msg.translation().z() = 2.0;  // Place text 1m above the base link
            return msg;
        }();
        visual_tools.publishText(text_pose, text, rviz_visual_tools::WHITE,
                                        rviz_visual_tools::XXXLARGE);
    };
    auto const prompt = [&visual_tools](auto text) {
        visual_tools.prompt(text);
    };
    // auto const draw_trajectory_tool_path =
    //     [&visual_tools,
    //     jmg = node->get_both_move_group()->getRobotModel()->getJointModelGroup(
    //         "both_manipulators")](auto const trajectory) 
    // {
    //     visual_tools.publishTrajectoryLine(trajectory, jmg);
    // };

    draw_title("Safety_Demonstration");
    visual_tools.trigger();

    // --------------------------------------------------
    // 0. move to home position
    // --------------------------------------------------
    node->go_to_ready_position();
    
    // --------------------------------------------------
    // 1. do a path planning to show the ability.
    // --------------------------------------------------

    left_target_pose.position.x = 0.4;
    left_target_pose.position.y = -0.2;
    left_target_pose.position.z = 0.3;
    right_target_pose.position.x = 0.4;
    right_target_pose.position.y = 0.2;
    right_target_pose.position.z = 0.3;

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

    // draw marker that shows the target pose

    prompt("Press 'Next' in the RvizVisualToolsGui window to execute");
    visual_tools.setBaseFrame(left_planning_frame);
    visual_tools.publishAxisLabeled(left_target_pose, "left_target_pose", rvt::LARGE);
    visual_tools.setBaseFrame(right_planning_frame);
    visual_tools.publishAxisLabeled(right_target_pose, "right_target_pose", rvt::LARGE);
    visual_tools.trigger();
    rclcpp::sleep_for(std::chrono::seconds(1));

    node->plan_and_execute(left_target_pose, right_target_pose, 
        left_planning_frame, right_planning_frame, 
        left_ee_link, right_ee_link);

    visual_tools.deleteAllMarkers();
    visual_tools.setBaseFrame("world");
    draw_title("Trajectory_successfully_executed");
    visual_tools.trigger();
    // --------------------------------------------------
    // 2. try to make a plan that two arms would colliside
    //  display the capability of safety.
    // --------------------------------------------------

    left_target_pose.position.x = 0.4;
    left_target_pose.position.y = -0.6;
    left_target_pose.position.z = 0.3;
    right_target_pose.position.x = 0.4;
    right_target_pose.position.y = 0.6;
    right_target_pose.position.z = 0.3;

    prompt("Press 'Next' in the RvizVisualToolsGui window to plan");
    visual_tools.deleteAllMarkers();
    visual_tools.setBaseFrame("world");
    draw_title("Unsafe_target_pose");


    auto left_joint_model_group = node->get_left_move_group()
        ->getRobotModel()->getJointModelGroup("left_ur_manipulator");
    auto right_joint_model_group = node->get_right_move_group()
        ->getRobotModel()->getJointModelGroup("right_ur_manipulator");
    moveit::planning_interface::MoveGroupInterface::Plan left_plan, right_plan;
    bool single_success;
    bool use_left = true;
    bool use_right = false;

    single_success = node->plan_single_arm(
        use_left, left_target_pose, left_planning_frame, "", left_plan);

    if(single_success){
        visual_tools.setBaseFrame(left_planning_frame);
        visual_tools.publishAxisLabeled(left_target_pose, "left_target_pose", rvt::LARGE);
        visual_tools.setBaseFrame("world");
        visual_tools.publishTrajectoryLine(left_plan.trajectory_, left_joint_model_group);
    }
    single_success = node->plan_single_arm(
        use_right, right_target_pose, right_planning_frame, "", right_plan);
    if(single_success){
        visual_tools.setBaseFrame(right_planning_frame);
        visual_tools.publishAxisLabeled(right_target_pose, "right_target_pose", rvt::LARGE);
        visual_tools.setBaseFrame("world");
        visual_tools.publishTrajectoryLine(right_plan.trajectory_, right_joint_model_group);
    }

    bool dual_success = node->plan_and_execute(left_target_pose, right_target_pose, 
        left_planning_frame, right_planning_frame, 
        "", "");
    if(!dual_success){
        visual_tools.setBaseFrame("world");
        draw_title("Two_arms_would_collide");
    }
    visual_tools.trigger();

    // --------------------------------------------------
    // 3. go back to ready position
    // --------------------------------------------------
    prompt("Press 'Next' in the RvizVisualToolsGui window to plan"); 
    visual_tools.deleteAllMarkers();
    visual_tools.setBaseFrame("world");
    draw_title("Back_to_safe_trajectory");
    visual_tools.trigger();
    node->go_to_ready_position();


    rclcpp::shutdown();
    return 0;
}