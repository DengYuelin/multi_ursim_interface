#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>

#include <moveit_msgs/DisplayRobotState.h>
#include <moveit_msgs/DisplayTrajectory.h>

#include <moveit_msgs/AttachedCollisionObject.h>
#include <moveit_msgs/CollisionObject.h>

#include <moveit_visual_tools/moveit_visual_tools.h>

class InterfaceNode
{

public:
    InterfaceNode(std::string PLANNING_GROUP) : move_group(PLANNING_GROUP)
    {
        // Setup moveit
        const robot_state::JointModelGroup *joint_model_group =
            move_group.getCurrentState()->getJointModelGroup(PLANNING_GROUP);
        // max joint velocity
        move_group.setMaxVelocityScalingFactor(0.5);

        // subscribe to the topic of which the target pose is published
        tar_pos_sub = nh_.subscribe<geometry_msgs::Pose>("robot_cmd/pose", 1, &InterfaceNode::tar_pose_callback, this);

        // Goto initial position
        target_pose.orientation.w = 1.0;
        target_pose.position.x = 0.28;
        target_pose.position.y = -0.2;
        target_pose.position.z = 0.8;

        moveit::core::RobotStatePtr current_state = move_group.getCurrentState();
        std::vector<double> joint_group_positions;
        current_state->copyJointGroupPositions(joint_model_group, joint_group_positions);
        move_group.setJointValueTarget(joint_group_positions);

        move_to_joint_pos(joint_group_positions);
    }

    // subscribe to a target pose and move to it
    void tar_pose_callback(const geometry_msgs::Pose::ConstPtr &msg)
    {
        target_pose.orientation = msg->orientation;
        target_pose.position = msg->position;
        // use cartesian planning
        move_to_pose_cartesian(target_pose);
    }

    // move to a target pose using moveit
    void move_to_pose(geometry_msgs::Pose pose, bool verbose = false)
    {
        move_group.setPoseTarget(pose);

        if (verbose)
        {
            ROS_INFO("Replanning");
            success = (move_group.plan(new_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
            ROS_INFO("Replan %s", success ? "SUCCEED" : "FAILED");
            if (success)
            {
                move_group.move();
                ROS_INFO("Reached Goal");
            }
        }
        else
        {
            move_group.plan(new_plan);
            move_group.move();
        }
    }

    void move_to_pose_cartesian(geometry_msgs::Pose pose, bool verbose = false)
    {
        waypoints.clear();
        waypoints.push_back(pose);
        move_group.computeCartesianPath(waypoints, eef_step, jump_threshold, trajectory);
        move_group.execute(trajectory);
        if (verbose)
        {
            ROS_INFO("Reached Goal");
        }
    }

    void move_to_joint_pos(std::vector<double> joint_group_positions, bool verbose = false)
    {
        move_group.setJointValueTarget(joint_group_positions);

        if (verbose)
        {
            ROS_INFO("Replanning");
            success = (move_group.plan(new_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
            ROS_INFO("Replan %s", success ? "SUCCEED" : "FAILED");
            if (success)
            {
                move_group.move();
                ROS_INFO("Reached Goal");
            }
        }
        else
        {
            move_group.plan(new_plan);
            move_group.move();
        }
    }

private:
    ros::NodeHandle nh_;
    ros::Subscriber tar_pos_sub;
    geometry_msgs::Pose target_pose;
    sensor_msgs::JointState initial_joint_state;
    std::vector<geometry_msgs::Pose> waypoints;
    moveit_msgs::RobotTrajectory trajectory;
    const double jump_threshold = 0.0;
    const double eef_step = 0.01;
    moveit::planning_interface::MoveGroupInterface move_group;
    moveit::planning_interface::MoveGroupInterface::Plan new_plan;
    bool success;
};

int main(int argc, char **argv)
{
    ros::init(argc, argv, "moveit_interface");
    ros::AsyncSpinner spinner(1);
    spinner.start();
    ros::Rate rate(1); // 1 hz
    InterfaceNode InterfaceNode("manipulator");
    ROS_INFO("Simulation Hardware interface Node ready");
    while (ros::ok())
    {
        ros::spinOnce();
        rate.sleep();
    }
}