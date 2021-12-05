/**
 * Copyright (c) Aalto  - All Rights Reserved
 * Created on: 11/23/18
 *     Author: Vladimir Petrik <vladimir.petrik@aalto.fi>
 *     This node subsribe to Gripper command action and forward all commands to the JointTrajectoryAction as required
 *     by mujoco simulation. It mimic individual fingers.
 */


#include <ros/ros.h>
#include <actionlib/server/simple_action_server.h>
#include <actionlib/client/simple_action_client.h>
#include <control_msgs/GripperCommandAction.h>
#include <control_msgs/FollowJointTrajectoryAction.h>

#include <iostream>



using namespace control_msgs;
using actionlib::SimpleActionServer;
using actionlib::SimpleActionClient;

class ForwardAction {
public:
    ForwardAction() : node("~"),
                      ac(node, "/kinova_mujoco/trajectory_controller_fingers/follow_joint_trajectory"),
                      as(node, "/robotiq_2f_85_gripper_controller/gripper_cmd", boost::bind(&ForwardAction::execute, this, _1), false) {
        as.start();
    }

    void execute(const GripperCommandGoalConstPtr &goal) {
        trajectory_msgs::JointTrajectoryPoint point;
        point.positions.resize(6, goal->command.position);
        point.positions[2] *= -1;
        point.positions[5] *= -1;

//        for (auto i = point.positions.begin(); i != point.positions.end(); ++i)
//            std::cout << *i << ' ';
//        std::cout << point.positions;

        point.effort.resize(6, goal->command.max_effort);
        point.time_from_start = ros::Duration(2.0);
        point.accelerations.resize(6, 0.00);
        point.velocities.resize(6, 0.00);

        FollowJointTrajectoryGoal t;
        t.trajectory.joint_names.push_back("finger_joint");
        t.trajectory.joint_names.push_back("left_inner_knuckle_joint");     // 1
        t.trajectory.joint_names.push_back("left_inner_finger_joint");      // -1
        t.trajectory.joint_names.push_back("right_outer_knuckle_joint");    // 1
        t.trajectory.joint_names.push_back("right_inner_knuckle_joint");    // 1
        t.trajectory.joint_names.push_back("right_inner_finger_joint");     // -1

        t.trajectory.points.push_back(point);

        const auto state = ac.sendGoalAndWait(t, ros::Duration(60.0), ros::Duration(60.0));
        if (state == state.SUCCEEDED) {
            as.setSucceeded();
        } else {
            as.setAborted();
        }
    }

private:
    ros::NodeHandle node;
    SimpleActionServer<GripperCommandAction> as;
    SimpleActionClient<FollowJointTrajectoryAction> ac;
};

int main(int argc, char **argv) {
    ros::init(argc, argv, "gripper_action_to_trajectory_action");

    ros::AsyncSpinner spinner(2);
    spinner.start();
    ForwardAction forward_action;
    ros::waitForShutdown();

    return 0;
}
