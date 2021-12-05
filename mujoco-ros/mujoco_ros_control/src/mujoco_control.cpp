/**
 * Copyright (c) Aalto  - All Rights Reserved
 * Created on: 8/2/18
 *     Author: Vladimir Petrik <vladimir.petrik@aalto.fi>
 *     Adapted 2020: Jan Behrens <jan.kristof.behrens@cvut.cz>
 */

#include <ros/ros.h>
#include <ros/package.h>
#include <std_msgs/String.h>
#include <std_srvs/Empty.h>
#include <mujoco_interface_msgs/GetObjectPose.h>
#include <mujoco_interface_msgs/GetAllObjectPoses.h>
#include <mujoco_interface_msgs/SetObjectPoses.h>

#include <mujoco.h>
#include <mujoco_ros_control/RobotHWMujoco.h>
#include <mujoco_ros_control/RenderImage.h>
#include <image_transport/image_transport.h>
#include <controller_manager/controller_manager.h>
#include <cv_bridge/cv_bridge.h>
#include <mutex>
#include <pcl/point_cloud.h>
#include <pcl_ros/point_cloud.h>

#include <pcl/conversions.h>
#include <rosgraph_msgs/Clock.h>
std::unique_ptr<RobotHWMujoco> hw;
std::unique_ptr<controller_manager::ControllerManager> cm;
typedef pcl::PointCloud<pcl::PointXYZ> PointCloud;

/** @brief Update controller manager and current time. If curent time is not initialized than call reset. */
void cb_controller(const mjModel *m, mjData *d, bool reset_controller = false) {
    hw->read(*d);
    cm->update(ros::Time::now(), ros::Duration(m->opt.timestep), reset_controller);
    hw->write(*d);
}

void reset_joints(ros::NodeHandle &node, const mjModel &m, mjData &d) {
    const auto n = (size_t) m.njnt;
    for (size_t i = 0; i < n; ++i) {
        const auto joint_type = m.jnt_type[i];
        if (joint_type == mjJNT_FREE || joint_type == mjJNT_BALL) {
            continue;
        }

        const auto joint_name = mj_id2name(&m, mjOBJ_JOINT, i);

        //double value = ros::param::param<std::double>("/" + std::string(joint_name) + "_init", 0);
        double value;
        ros::param::get("/" + std::string(joint_name) + "_init", value);
        std::cout << "/" + std::string(joint_name) + "_init " << value << std::endl;
        d.qpos[(size_t) m.jnt_qposadr[i]] = value;
    }
}

void set_joint(ros::NodeHandle &node, const mjModel &m, mjData &d) {
//    int bodyid = mujoco::mj_name2id(m, mjOBJ_BODY, "cube1");
    int bodyid = mj_name2id(&m, mjOBJ_BODY, "cube1");
    int qposadr = -1, qveladr = -1;

// make sure we have a floating body: it has a single free joint
    if( bodyid>=0 && m.body_jntnum[bodyid]==1 &&
        m.jnt_type[m.body_jntadr[bodyid]]==mjJNT_FREE )
    {
        // extract the addresses from the joint specification
        qposadr = m.jnt_qposadr[m.body_jntadr[bodyid]];
        qveladr = m.jnt_dofadr[m.body_jntadr[bodyid]];

        for(size_t k = 0; k < 7; k++){
            std::cout << bodyid << k << ": pos: " << d.qpos[qposadr+k] << std::endl;
        }
        size_t k = 2;
        if(d.qpos[qposadr+k] < 1.1){
//            d.qpos[qposadr+k] = 2.0;
            k = 0;
            d.qpos[qposadr + k++] = 0.2;
            d.qpos[qposadr + k++] = 0.0;
            d.qpos[qposadr + k++] = 2.0;

            k = 0;
            d.qvel[qveladr+k++] = 0.0;
            d.qvel[qveladr+k++] = 0.0;
            d.qvel[qveladr+k++] = 10.0;
        }
    }
}

std::vector<std::string> get_free_body_names(ros::NodeHandle &node, const mjModel &m, const mjData &d) {

    std::vector<std::string> names;  // names of all free obj

    const auto n = (size_t) m.njnt;
    auto k = (size_t) 0;
    // count relevant joints
    for (size_t i = 0; i < n; ++i) {
        const auto joint_type = m.jnt_type[i];
        if (joint_type == 0  || joint_type == mjJNT_BALL) {
            names.emplace_back(mj_id2name(&m, mjOBJ_BODY, m.jnt_bodyid[i]));
//            std::cout << i << ": name:" << names.back() << std::endl;

        }
    }
    return names;
}



std::vector<double> get_free_body_pose(ros::NodeHandle &node, const mjModel &m, const mjData &d, const char &body_name) {
//    int bodyid = mujoco::mj_name2id(m, mjOBJ_BODY, "cube1");
//    int bodyid = mj_name2id(&m, mjOBJ_BODY, body_name);
    int bodyid = mj_name2id(&m, mjOBJ_BODY, &body_name);

    int qposadr = -1, qveladr = -1;
    std::vector<double> pose;  //storing the pose (trans: x y z rot: w x y z success: suc)

// make sure we have a floating body: it has a single free joint
    if( bodyid>=0 && m.body_jntnum[bodyid]==1 &&
        m.jnt_type[m.body_jntadr[bodyid]]==mjJNT_FREE )
    {
        // extract the addresses from the joint specification
        qposadr = m.jnt_qposadr[m.body_jntadr[bodyid]];
        qveladr = m.jnt_dofadr[m.body_jntadr[bodyid]];

        for(size_t k = 0; k < 7; k++){
//            std::cout << bodyid << k << ": pos: " << d.qpos[qposadr+k] << std::endl;
            pose.push_back(d.qpos[qposadr+k]);
        }
        pose.push_back(1.0);
        return pose;
    }else{
        for(int k = 0; k < 8; k++) {
            pose.push_back(0.0);
        }
        return pose;
    }
}

bool set_free_body_pose(ros::NodeHandle &node, const mjModel &m, const mjData &d, const char &body_name, const std::vector<double> pose) {
//    int bodyid = mujoco::mj_name2id(m, mjOBJ_BODY, "cube1");
//    int bodyid = mj_name2id(&m, mjOBJ_BODY, body_name);
    int bodyid = mj_name2id(&m, mjOBJ_BODY, &body_name);

    int qposadr = -1, qveladr = -1;
//    std::vector<double> pose;  //storing the pose (trans: x y z rot: w x y z success: suc)

// make sure we have a floating body: it has a single free joint
    if( bodyid>=0 && m.body_jntnum[bodyid]==1 &&
        m.jnt_type[m.body_jntadr[bodyid]]==mjJNT_FREE )
    {
        // extract the addresses from the joint specification
        qposadr = m.jnt_qposadr[m.body_jntadr[bodyid]];
        qveladr = m.jnt_dofadr[m.body_jntadr[bodyid]];

        int t = 0;
        for(size_t k = 0; k < 7; k++){
//            std::cout << bodyid << k << ": pos: " << d.qpos[qposadr+k] << std::endl;
            d.qpos[qposadr+k] = pose[t];
//            d.qvel[qveladr]
//            pose.push_back(d.qpos[qposadr+k]);
            t++;
        }
        for(size_t k = 0; k < 6; k++){
//            std::cout << bodyid << k << ": pos: " << d.qpos[qposadr+k] << std::endl;
            d.qvel[qveladr+k] = 0.0;
//            pose.push_back(d.qpos[qposadr+k]);
        }
        return true;
    }else{
        return false;
    }
}

std::vector<double> weight_obj(ros::NodeHandle &node, const mjModel &m, const mjData &d, const char &body_name) {
//    int bodyid = mujoco::mj_name2id(m, mjOBJ_BODY, "cube1");
//    int bodyid = mj_name2id(&m, mjOBJ_BODY, body_name);
    int bodyid = mj_name2id(&m, mjOBJ_BODY, &body_name);

    int qposadr = -1, qveladr = -1;
    std::vector<double> pose;  //storing the pose (trans: x y z rot: w x y z success: suc)

// make sure we have a floating body: it has a single free joint
    if( bodyid>=0 && m.body_jntnum[bodyid]==1 &&
        m.jnt_type[m.body_jntadr[bodyid]]==mjJNT_FREE )
    {
        // extract the addresses from the joint specification
        qposadr = m.jnt_qposadr[m.body_jntadr[bodyid]];
        qveladr = m.jnt_dofadr[m.body_jntadr[bodyid]];

        for(size_t k = 0; k < 7; k++){
//            std::cout << bodyid << k << ": pos: " << d.qpos[qposadr+k] << std::endl;
            pose.push_back(d.qpos[qposadr+k]);
        }
        pose.push_back(1.0);
        return pose;
    }else{
        for(int k = 0; k < 8; k++) {
            pose.push_back(0.0);
        }
        return pose;
    }
}

int main(int argc, char **argv) {
    ros::init(argc, argv, "mujoco_control");
    ros::NodeHandle node("~");
    ros::AsyncSpinner spinner(2);
    spinner.start();

    const auto default_model_path = ros::package::getPath("mujoco_ros_control") + "/model/simple_robot.urdf";
    const auto model_path = node.param("model", default_model_path);

    // The camera look at vector defines direction of the camera which it points to.
    const float default_camera_lookat_at[3] = {0, 0, 1.5};

    float camera_look_at[3];
    camera_look_at[0] = node.param<float>("look_at_x", default_camera_lookat_at[0]);
    camera_look_at[1] = node.param<float>("look_at_y", default_camera_lookat_at[1]);
    camera_look_at[2] = node.param<float>("look_at_z", default_camera_lookat_at[2]);

    const auto key_path = std::string(getenv("HOME")) + "/.mujoco/mujoco200/bin/mjkey.txt";

    if (!mj_activate(key_path.c_str())) {
        ROS_ERROR_STREAM("Cannot activate mujoco with key: " << key_path);
        return -1;
    }

    char error[1000] = "";
    auto m = mj_loadXML(model_path.c_str(), nullptr, error, 1000);
    if (!m) {
        ROS_ERROR_STREAM("Cannot load model: " << model_path);
        ROS_ERROR_STREAM(error);
        return -1;
    }

    auto d = mj_makeData(m);
    if (!d) {
        ROS_ERROR_STREAM("Cannot make data structure for model.");
        return -1;
    }

    hw.reset(new RobotHWMujoco(*m));
    cm.reset(new controller_manager::ControllerManager(hw.get(), node));
    mjcb_control = [](const mjModel *m, mjData *d) { cb_controller(m, d); };

    hw->compensate_bias = node.param("/compensate_bias", false);
    hw->show_full_torques = node.param("/show_full_torques", false);
    hw->bias_error = node.param("bias_error", 1.0);

    reset_joints(node, *m, *d);
    std::mutex mutex_data;

    const auto timer = node.createTimer(ros::Duration(m->opt.timestep), [&](const ros::TimerEvent &e) {
        std::lock_guard<std::mutex> l(mutex_data);
        mj_step(m, d);
    });


    boost::function<bool(std_srvs::Empty::Request &, std_srvs::Empty::Response &)> reset =
            [&](std_srvs::Empty::Request &request, std_srvs::Empty::Response &response) {
                std::lock_guard<std::mutex> l(mutex_data);
                mj_resetData(m, d);
                reset_joints(node, *m, *d);
                cb_controller(m, d, true);
                return true;
            };
    const auto reset_server = node.advertiseService("reset", reset);

    boost::function<bool(mujoco_interface_msgs::GetObjectPose::Request &, mujoco_interface_msgs::GetObjectPose::Response &)> getObjPose =
            [&](mujoco_interface_msgs::GetObjectPose::Request &request, mujoco_interface_msgs::GetObjectPose::Response &response) {
                auto pose = get_free_body_pose(node, *m, *d, *request.name.c_str());
                response.pose.header.stamp = ros::Time::now();
                response.pose.header.frame_id = "world";
                response.pose.pose.position.x = pose[0];
                response.pose.pose.position.y = pose[1];
                response.pose.pose.position.z = pose[2];
                response.pose.pose.orientation.w = pose[3];
                response.pose.pose.orientation.x = pose[4];
                response.pose.pose.orientation.y = pose[5];
                response.pose.pose.orientation.z = pose[6];

                response.success = 1;

                if(pose[7] <= 0.5){
                    response.success = 0;
                }

                return true;
            };

    const auto get_obj_pose_server = node.advertiseService("getObjectPose", getObjPose);


    boost::function<bool(mujoco_interface_msgs::GetAllObjectPoses::Request &, mujoco_interface_msgs::GetAllObjectPoses::Response &)> getAllObjPoses =
            [&](mujoco_interface_msgs::GetAllObjectPoses::Request &request, mujoco_interface_msgs::GetAllObjectPoses::Response &response) {

                auto names = get_free_body_names(node, *m, *d);
                for (const auto &i : names){
                    //std::cout << i << std::endl;
                    auto pose = get_free_body_pose(node, *m, *d, *i.c_str());
                    geometry_msgs::PoseStamped poseStamped = geometry_msgs::PoseStamped();
                    poseStamped.header.stamp = ros::Time::now();
                    poseStamped.header.frame_id = "world";
                    poseStamped.pose.position.x = pose[0];
                    poseStamped.pose.position.y = pose[1];
                    poseStamped.pose.position.z = pose[2];
                    poseStamped.pose.orientation.w = pose[3];
                    poseStamped.pose.orientation.x = pose[4];
                    poseStamped.pose.orientation.y = pose[5];
                    poseStamped.pose.orientation.z = pose[6];

                    response.poses.push_back(poseStamped);
                    response.names.push_back(i);
                }

                response.success = 1;

                if(response.poses.size() < 1){
                    response.success = 0;
                }

                return true;
            };

    const auto get_all_obj_pose_server = node.advertiseService("getAllObjectPoses", getAllObjPoses);

    boost::function<bool(mujoco_interface_msgs::SetObjectPoses::Request &, mujoco_interface_msgs::SetObjectPoses::Response &)> setObjPoses =
            [&](mujoco_interface_msgs::SetObjectPoses::Request &request, mujoco_interface_msgs::SetObjectPoses::Response &response) {
                std::lock_guard<std::mutex> l(mutex_data);

                std::vector<double> pose;

                auto names = get_free_body_names(node, *m, *d);
                for (std::size_t i = 0; i < request.names.size(); ++i){
                    pose.clear();
                    pose.push_back(request.poses[i].pose.position.x);
                    pose.push_back(request.poses[i].pose.position.y);
                    pose.push_back(request.poses[i].pose.position.z);
                    pose.push_back(request.poses[i].pose.orientation.w);
                    pose.push_back(request.poses[i].pose.orientation.x);
                    pose.push_back(request.poses[i].pose.orientation.y);
                    pose.push_back(request.poses[i].pose.orientation.z);
                    auto name = request.names[i];
                    set_free_body_pose(node, *m, *d, *name.c_str(), pose);
                }

                response.success = 1;
                return true;
            };

    const auto set_obj_poses_server = node.advertiseService("setObjectPoses", setObjPoses);

    const auto init_reset_delay = node.param("mujoco_initial_reset_delay", 1.0);
    const auto initial_reset = node.createTimer(ros::Duration(init_reset_delay), [&](const ros::TimerEvent &e) {
        std_srvs::Empty srv;
        reset(srv.request, srv.response);
    }, true);

    image_transport::ImageTransport it(node);
    image_transport::Publisher pub_rgb = it.advertise("rgb", 1);
    image_transport::Publisher pub_depth = it.advertise("depth", 1);
    ros::Publisher pub_pc = node.advertise<PointCloud>("points2", 1);

    float camera_orient[2];
    float camera_dist[1];

    const auto timer_rendered = node.createTimer(ros::Duration(0.1), [&](const ros::TimerEvent &e) {
        camera_look_at[0] = node.param<float>("look_at_x", default_camera_lookat_at[0]);
        camera_look_at[1] = node.param<float>("look_at_y", default_camera_lookat_at[1]);
        camera_look_at[2] = node.param<float>("look_at_z", default_camera_lookat_at[2]);


        camera_orient[0] = node.param<float>("look_azimuth", 0.0);
        camera_orient[1] = node.param<float>("look_elevation", 1.0);
        camera_dist[0] = node.param<float>("look_dist", 1.5);

        thread_local RenderImage renderer(m, camera_look_at, camera_orient, camera_dist);

        if ((pub_rgb.getNumSubscribers() == 0) && (pub_depth.getNumSubscribers() == 0)) {
            return;
        }

        {
            renderer.setCamPos(camera_look_at, camera_orient, camera_dist);
            std::lock_guard<std::mutex> l(mutex_data);
            renderer.updateScene(m, d);
        }
        cv::Mat rgb, depth;
        std::tie(rgb, depth) = renderer.render();


        std_msgs::Header header;
        header.stamp = ros::Time::now();

        if (pub_rgb.getNumSubscribers() != 0) {
            header.frame_id = "rgb";
            pub_rgb.publish(cv_bridge::CvImage(header, "rgb8", rgb).toImageMsg());
        }

        if (pub_depth.getNumSubscribers() != 0) {
            header.frame_id = "depth";
            pub_depth.publish(cv_bridge::CvImage(header, "depth", depth).toImageMsg());
        }
        if (pub_pc.getNumSubscribers() != 0) {
            pcl::PointCloud<pcl::PointXYZRGB>::Ptr pc = renderer.RGBDtoPCL(depth, rgb, camera_look_at, camera_orient, camera_dist);
            sensor_msgs::PointCloud2 pc_msg;
            pcl::toROSMsg(*pc.get(), pc_msg);
            pc_msg.header.frame_id = "virtual_camera";
            pc_msg.header.stamp = header.stamp;
            pub_pc.publish(pc_msg);
        }

    });

    /*ros::Publisher pub_clock_ = node.advertise<rosgraph_msgs::Clock>("/clock", 10);
    while(ros::ok())
    {
        mjtNum sim_start = d->time;
        //while(ros::ok() && d->time - sim_start < 1/500){
        ros::Time current_time = (ros::Time)d->time;
        rosgraph_msgs::Clock ros_time_;
        ros_time_.clock.fromSec(current_time.toSec());
        pub_clock_.publish(ros_time_);
        std::lock_guard<std::mutex> l(mutex_data);
        mj_step(m, d);
        //}


    }*/

    ros::waitForShutdown();
    mj_deleteModel(m);
    mj_deleteData(d);
    mj_deactivate();

    return 0;
}
