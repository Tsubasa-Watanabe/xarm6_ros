#include <ros/ros.h>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit/planning_interface/planning_interface.h>
#include <moveit_msgs/Constraints.h>
#include <moveit_msgs/JointConstraint.h>
#include <moveit/robot_state/conversions.h>

// MoveIt
#include <moveit/planning_request_adapter/planning_request_adapter.h>

// STOMP
#include <stomp_moveit/stomp_planner.h>

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

    // 目標値のリスト
    std::vector<double> joint_values = {-0.24606383099885676, -0.5382532206499215, -1.5948111215233824, -0.00033154586141304065, 2.134209144222833, -0.24605545821852814};

    // 目標値を設定
    for (size_t i = 0; i < joint_values.size(); ++i)
    {
        moveit_msgs::Constraints goal_constraint;
        moveit_msgs::JointConstraint joint_goal;
        joint_goal.joint_name = "joint" + std::to_string(i + 1);  // 関節名
        joint_goal.position = joint_values[i];                     // 目標位置
        joint_goal.tolerance_above = 0.01;                           // 上側の許容範囲
        joint_goal.tolerance_below = 0.01;                           // 下側の許容範囲
        joint_goal.weight = 1.0;                                    // 重み
        goal_constraint.joint_constraints.push_back(joint_goal);
        req.goal_constraints.push_back(goal_constraint);
    }

    // パス制約のリスト (waypoints)
    std::vector<std::vector<double>> waypoints = {
        {4.361691394816347e-05, -4.549517718466234e-05, -7.679402761073817e-05, 9.749420298188483e-05, -9.865705155043258e-06, -7.692414441429207e-06},
        {0.027371501845528834, 0.022613741838399603, -0.09573733451761833, -0.04120260599950464, 0.08482237291496367, -0.035482117029908986},
        {0.054699386777109504, 0.04527297885398387, -0.19139787500762592, -0.08250270620199117, 0.16965461153508238, -0.07095654164537654},
        {0.08202727170869017, 0.06793221586956813, -0.2870584154976335, -0.1238028064044777, 0.25448685015520106, -0.1064309662608441},
        {0.10935515664027085, 0.0905914528851524, -0.3827189559876411, -0.16510290660696422, 0.3393190887753198, -0.14190539087631165}
    };

    // パス制約を設定
    moveit_msgs::Constraints path_constraints;
    for (size_t i = 0; i < waypoints[0].size(); ++i)
    {
        moveit_msgs::JointConstraint joint_constraint;
        joint_constraint.joint_name = "joint" + std::to_string(i + 1);
        joint_constraint.position = waypoints[0][i];  // パス制約は最初のwaypointの値を使用
        joint_constraint.tolerance_above = 0.1;       // 上側の許容範囲
        joint_constraint.tolerance_below = 0.1;       // 下側の許容範囲
        joint_constraint.weight = 1.0;                // 重み

        // JointConstraintをパス制約に追加
        path_constraints.joint_constraints.push_back(joint_constraint);
    }
    for (size_t i = 0; i < waypoints[0].size(); ++i)
    {
        moveit_msgs::JointConstraint joint_constraint;
        joint_constraint.joint_name = "joint" + std::to_string(i + 1);
        joint_constraint.position = waypoints[1][i];  // パス制約は最初のwaypointの値を使用
        joint_constraint.tolerance_above = 0.1;       // 上側の許容範囲
        joint_constraint.tolerance_below = 0.1;       // 下側の許容範囲
        joint_constraint.weight = 1.0;                // 重み

        // JointConstraintをパス制約に追加
        path_constraints.joint_constraints.push_back(joint_constraint);
    }

    // MotionPlanRequestにパス制約を追加
    //req.path_constraints = path_constraints;

    // ワークスペースの設定
    moveit_msgs::WorkspaceParameters workspace;
    workspace.min_corner.x = -1.0;
    workspace.min_corner.y = -1.0;
    workspace.min_corner.z = -1.0;
    workspace.max_corner.x = 1.0;
    workspace.max_corner.y = 1.0;
    workspace.max_corner.z = 1.0;
    req.workspace_parameters = workspace;

    // // 現在のロボットの状態を取得
    // robot_state::RobotStatePtr current_state = move_group.getCurrentState();

    // // 開始状態を設定
    // moveit_msgs::RobotState start_state_msg;
    // moveit::core::robotStateToRobotStateMsg(*current_state, start_state_msg);
    // req.start_state = start_state_msg;

    // MotionPlanRequestを出力
    ROS_INFO_STREAM("MotionPlanRequest:\n" << req);

    //     // プランニングを実行
    //     moveit::planning_interface::MoveGroupInterface::Plan my_plan;
    //     move_group.setPlanningTime(5.0); // プランニングにかかる時間の設定
    //     bool success = (move_group.plan(my_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);

    //     if (success)
    //     {
    //         ROS_INFO("Planning succeeded");
    //         move_group.execute(my_plan);
    //     }
    //     else
    //     {
    //         ROS_ERROR("Planning failed");
    //     }

    //     ros::shutdown();
    //     return 0;
    // }

}