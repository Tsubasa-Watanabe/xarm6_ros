#include <ros/ros.h>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit/planning_interface/planning_interface.h>
#include <moveit_msgs/Constraints.h>
#include <moveit_msgs/JointConstraint.h>


int main(int argc, char **argv)
{
    ros::init(argc, argv, "xArm6");
    ros::NodeHandle node_handle;

    // ムーブグループの名前を指定
    moveit::planning_interface::MoveGroupInterface move_group("xarm6");

    // MotionPlanRequestメッセージを作成
    planning_interface::MotionPlanRequest req;

    // グループの名前を設定
    req.group_name = "xarm6";

    // 関節空間の目標を設定
    req.goal_constraints.push_back(moveit_msgs::Constraints());
    moveit_msgs::JointConstraint joint_constraint;
    joint_constraint.joint_name = "joint1";  // 任意の関節名
    joint_constraint.position = 0.9;          // 関節の目標位置
    joint_constraint.tolerance_above = 0.1;   // 上側の許容範囲
    joint_constraint.tolerance_below = 0.1;   // 下側の許容範囲
    joint_constraint.weight = 1.0;            // 重み
    req.goal_constraints[0].joint_constraints.push_back(joint_constraint);

    moveit_msgs::RobotState start_state_msg;
    // ここで適切な初期関節の状態を指定
    start_state_msg.joint_state.name = {"joint1", "joint2", "joint3", "joint4", "joint5", "joint6"};
    start_state_msg.joint_state.position = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
    req.start_state = start_state_msg;

    // MotionPlanRequestを出力
    ROS_INFO_STREAM("MotionPlanRequest:\n" << req);

    // プランニングを実行
    moveit::planning_interface::MoveGroupInterface::Plan my_plan;
    move_group.setPlanningTime(5.0); // プランニングにかかる時間の設定
    bool success = (move_group.plan(my_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);

    if (success)
    {
        ROS_INFO("Planning succeeded");
        // move_group.execute(my_plan);
    }
    else
    {
        ROS_ERROR("Planning failed");
    }

    ros::shutdown();
    return 0;
}
