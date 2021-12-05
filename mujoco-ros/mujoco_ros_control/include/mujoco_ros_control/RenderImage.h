//
// Created by aleksi on 29.10.2018.
//

#ifndef RENDER_IMAGE_H
#define RENDER_IMAGE_H

#include <mujoco.h>
#include <opencv2/highgui/highgui.hpp>
#include <glfw3.h>
#include <sensor_msgs/Image.h>
#include <pcl/point_cloud.h>
#include <pcl_ros/point_cloud.h>

class RenderImage {
public:
    explicit RenderImage(const mjModel* m);
    explicit RenderImage(const mjModel* m, const float camera_look_at[3]);
    explicit RenderImage(const mjModel* m, const float camera_look_at[3], const float camera_orient[2], const float camera_dist[1]);
    void setCamPos(const float camera_look_at[3], const float camera_orient[2], const float camera_dist[1]);


        ~RenderImage();

    /** @brief Update the scene info stored in internal variables */
    void updateScene(const mjModel* m, mjData* d);

    /** @brief Convert the depth image to a point cloud */
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr RGBDtoPCL(cv::Mat depth_image, cv::Mat rgb, const float camera_look_at[3], const float camera_orient[2], const float camera_dist[1]);

    /** @brief Render tuple of RGB and Depth image from the current scene.
     * Be sure to call updateScene before rendering. */
    std::tuple<cv::Mat, cv::Mat> render();


    /** @brief Create OpenGL context/window which is invisible and used for rendering. */
    static void initOpenGL();

    /** @brief Close OpenGL window */
    static void closeOpenGL();

private:
    mjvScene scn;
    mjvCamera cam;
    mjvOption opt;
    mjrContext con;
    mjVisual vis;
    mjrRect viewport;
    float lookatx;
    float lookaty;
    float lookatz;
    float znear;
    float zfar;
    float fovy;
    float azimuth;
    float elevation;
    float distance;
};


#endif //RENDER_IMAGE_H
