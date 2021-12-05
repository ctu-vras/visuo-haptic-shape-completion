//
// Created by aleksi on 31.10.2018.
//


#include <mujoco_ros_control/RenderImage.h>
#include <opencv2/highgui/highgui.hpp>
#include <mujoco.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_cloud.h>
#include <pcl/common/transforms.h>
#include <iostream>
#include <Eigen/Geometry>
#define _USE_MATH_DEFINES
#include <cmath>

// select EGL, OSMESA or GLFW
#if defined(MJ_EGL)
#include <EGL/egl.h>
#elif defined(MJ_OSMESA)
#include <GL/osmesa.h>
OSMesaContext ctx;
unsigned char buffer[10000000];
#else

#include "glfw3.h"

#endif


RenderImage::RenderImage(const mjModel *m) {

    initOpenGL();
    // initialize visualization data structures
    mjv_defaultCamera(&cam);
    mjv_defaultOption(&opt);
    mjv_defaultScene(&scn);
    mjr_defaultContext(&con);
    //
    // create scene and context
    mjv_makeScene(m, &scn, 2000);
    mjr_makeContext(m, &con, 200);
    znear = m->vis.map.znear;
    zfar = m->vis.map.zfar;
    fovy = m->vis.global.fovy;
    // center and scale view
    cam.lookat[0] = m->stat.center[0];
    cam.lookat[1] = m->stat.center[1];
    cam.lookat[2] = m->stat.center[2];
    cam.distance = 0.7 * m->stat.extent;

    lookatx = cam.lookat[0];
    lookaty = cam.lookat[1];
    lookatz = cam.lookat[2];

    azimuth = cam.azimuth;
    elevation = cam.elevation;
    distance = cam.distance;

    // set rendering to offscreen buffer
    mjr_setBuffer(mjFB_OFFSCREEN, &con);
    if (con.currentBuffer != mjFB_OFFSCREEN) {
        printf("Warning: offscreen rendering not supported, using default/window framebuffer\n");
    }
    // get size of active renderbuffer
    viewport = mjr_maxViewport(&con);
}

RenderImage::RenderImage(const mjModel *m, const float camera_look_at[3]) {

    initOpenGL();
    // initialize visualization data structures

    mjv_defaultCamera(&cam);
    mjv_defaultOption(&opt);
    mjv_defaultScene(&scn);
    mjr_defaultContext(&con);
    mj_defaultVisual(&vis);
    // create scene and context
    mjv_makeScene(m, &scn, 2000);
    mjr_makeContext(m, &con, 200);

    vis.map.zfar = 1;
    // center and scale view
    znear = m->vis.map.znear;
    zfar = m->vis.map.zfar;
    fovy = m->vis.global.fovy;

    // Transforming the camera look at vector to a unit vector (may not be necessary)

    cam.lookat[0] = camera_look_at[0];
    cam.lookat[1] = camera_look_at[1];
    cam.lookat[2] = camera_look_at[2];
    cam.distance = 1.0*m->stat.extent; // distance to lookat point or tracked body (may not be necessary)
    lookatx = cam.lookat[0];
    lookaty = cam.lookat[1];
    lookatz = cam.lookat[2];

    azimuth = cam.azimuth;
    elevation = cam.elevation;
    distance = cam.distance;
    // set rendering to offscreen buffer
    mjr_setBuffer(mjFB_OFFSCREEN, &con);
    if (con.currentBuffer != mjFB_OFFSCREEN) {
        printf("Warning: offscreen rendering not supported, using default/window framebuffer\n");
    }

    // get size of active renderbuffer
    viewport = mjr_maxViewport(&con);
}

RenderImage::RenderImage(const mjModel *m, const float camera_look_at[3], const float camera_orient[2], const float camera_dist[1]) {

    initOpenGL();
    // initialize visualization data structures

    mjv_defaultCamera(&cam);
    mjv_defaultOption(&opt);
    mjv_defaultScene(&scn);
    mjr_defaultContext(&con);
    mj_defaultVisual(&vis);
    // create scene and context
    mjv_makeScene(m, &scn, 2000);
    mjr_makeContext(m, &con, 200);

    vis.map.zfar = 1;
    // center and scale view
    znear = m->vis.map.znear;
    zfar = m->vis.map.zfar;
    fovy = m->vis.global.fovy;

    // Transforming the camera look at vector to a unit vector (may not be necessary)

    cam.lookat[0] = camera_look_at[0];
    cam.lookat[1] = camera_look_at[1];
    cam.lookat[2] = camera_look_at[2];
    cam.distance = camera_dist[0]; // distance to lookat point or tracked body (may not be necessary)
    cam.azimuth = camera_orient[0];
    cam.elevation = camera_orient[1];

//    lookatx = cam.lookat[0];
//    lookaty = cam.lookat[1];
//    lookatz = cam.lookat[2];
//
//    azimuth = cam.azimuth;
//    elevation = cam.elevation;
//    distance = cam.distance;
    // set rendering to offscreen buffer
    mjr_setBuffer(mjFB_OFFSCREEN, &con);
    if (con.currentBuffer != mjFB_OFFSCREEN) {
        printf("Warning: offscreen rendering not supported, using default/window framebuffer\n");
    }

    // get size of active renderbuffer
    viewport = mjr_maxViewport(&con);
}


void RenderImage::setCamPos(const float camera_look_at[3], const float camera_orient[2], const float camera_dist[1]){

    this->cam.lookat[0] = camera_look_at[0];
    this->cam.lookat[1] = camera_look_at[1];
    this->cam.lookat[2] = camera_look_at[2];
    this->cam.distance = camera_dist[0]; // distance to lookat point or tracked body (may not be necessary)
//    this->cam.distance = 10.0; // distance to lookat point or tracked body (may not be necessary)

//    std::cout << "dist: " << camera_dist[0] << std::endl;
    this->cam.azimuth = camera_orient[0];
    this->cam.elevation = camera_orient[1];
}



void RenderImage::updateScene(const mjModel *m, mjData *d) {
    mjv_updateScene(m, d, &opt, nullptr, &cam, mjCAT_ALL, &scn);
}

pcl::PointCloud<pcl::PointXYZRGB>::Ptr RenderImage::RGBDtoPCL(cv::Mat depth_image, cv::Mat rgb, const float camera_look_at[3], const float camera_orient[2], const float camera_dist[1])
{
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr pointcloud(new pcl::PointCloud<pcl::PointXYZRGB>);
    float width = depth_image.cols;
    float height = depth_image.rows;

    float fovx = 2*atan(tan(fovy * M_PI/ 180 / 2)*(width/height))*180/M_PI;
    float fx = (width / 2) * (1/(tan(fovx * (M_PI / 180.0) / 2)));
    float fy = (height / 2) *(1/(tan(fovy * (M_PI / 180.0) / 2)));
    float cx = width / 2;
    float cy = height /2;
    float factor = 1;// 1.0;
    //cv::FileStorage file("ast.xml", cv::FileStorage::WRITE);
    //file << "ast" << depth_image;
    depth_image.convertTo(depth_image, CV_32F); // convert the image data to float type

    if (!depth_image.data) {
        std::cerr << "No depth data!!!" << std::endl;
        exit(EXIT_FAILURE);
    }

    pointcloud->width = width; //Dimensions must be initialized to use 2-D indexing
    pointcloud->height = height;
    pointcloud->points.resize(pointcloud->width*pointcloud->height);
    int k = 0;
#pragma omp parallel for
    for (int v = 0; v < depth_image.rows; v++)
    {
        for (int u = 0; u < depth_image.cols; u++)
        {
            float Z = depth_image.at<float>(v, u) / factor;
            pcl::PointXYZRGB p;
            p.z = Z;
            p.x = (u - cx) * Z / fx;
            p.y = (v - cy) * Z / fy;
            //p.z *=2.8302;//1000;
            //p.x *=2.8302;//1000;
            //p.y *=2.8302;//1000;
            p.z *=(2.8302*0.95);//1000;
            p.x *=(2.8302*0.95);//1000;
            p.y *=(2.8302*0.95);//1000;
            p.r = rgb.at<cv::Vec3b>(v, u)[0];
            p.g = rgb.at<cv::Vec3b>(v, u)[1];
            p.b = rgb.at<cv::Vec3b>(v, u)[2];

            pointcloud->points[k] = p;
            k++;
        }
    }
    //Eigen::Quaternionf quat = Eigen::Quaternionf(0.7071068 , 0, 0, 0.7071068);
    //pointcloud->sensor_orientation_ = quat;

    /*
    Eigen::Affine3f transform = Eigen::Affine3f::Identity();
    transform.rotate (Eigen::AngleAxisf (camera_orient[0]*M_PI/180-M_PI/2, Eigen::Vector3f::UnitZ()));
    transform.rotate (Eigen::AngleAxisf (-M_PI/2, Eigen::Vector3f::UnitX()));
    transform.rotate (Eigen::AngleAxisf (camera_orient[1]*M_PI/180, Eigen::Vector3f::UnitX()));
    float r = camera_dist[0];
    float theta = camera_orient[1]*M_PI/180;
    float phi = camera_orient[0]*M_PI/180;
    float x_ = r*cos(phi)*sin(theta);
    float y = r*sin(phi)*sin(theta);
    float z_ = r*cos(theta);
    float x = copysign(z_, z_);
    float z = copysign(x_, x_);
    transform.translation() << camera_look_at[0]+0.015+x, camera_look_at[1]+y, camera_look_at[2]+z;
    pcl::transformPointCloud (*pointcloud, *pointcloud, transform);
    */
    //pcl::io::savePCDFileASCII ("/home/rustli/test_pcd.pcd", *pointcloud);
    return pointcloud;

}

std::tuple<cv::Mat, cv::Mat> RenderImage::render() {
    // render scene in offscreen buffer
    mjr_render(viewport, &scn, &con);

    // get size of active renderbuffer
    int W = viewport.width;
    int H = viewport.height;

    cv::Mat mat_rgb(H, W, CV_8UC3);
    cv::Mat mat_depth(H, W, CV_32F);

    mjr_readPixels(mat_rgb.data, (float *) mat_depth.data, viewport, &con);
    mat_depth = mat_depth*2-1;
    mat_depth = (2*znear*zfar)/(zfar+znear-mat_depth*(zfar-znear));
    cv::Mat flipped_rgb, flipped_depth;

    cv::flip(mat_rgb, flipped_rgb, cv::ROTATE_90_CLOCKWISE);
    cv::flip(mat_depth, flipped_depth, cv::ROTATE_90_CLOCKWISE);

    return std::make_tuple(flipped_rgb, flipped_depth);

}

RenderImage::~RenderImage() {
    mjr_freeContext(&con);
    mjv_freeScene(&scn);
    closeOpenGL();
}

void RenderImage::initOpenGL() {
    //------------------------ EGL
#if defined(MJ_EGL)
    // desired config
    const EGLint configAttribs[] ={
        EGL_RED_SIZE,           8,
        EGL_GREEN_SIZE,         8,
        EGL_BLUE_SIZE,          8,
        EGL_ALPHA_SIZE,         8,
        EGL_DEPTH_SIZE,         24,
        EGL_STENCIL_SIZE,       8,
        EGL_COLOR_BUFFER_TYPE,  EGL_RGB_BUFFER,
        EGL_SURFACE_TYPE,       EGL_PBUFFER_BIT,
        EGL_RENDERABLE_TYPE,    EGL_OPENGL_BIT,
        EGL_NONE
    };

    // get default display
    EGLDisplay eglDpy = eglGetDisplay(EGL_DEFAULT_DISPLAY);
    if( eglDpy==EGL_NO_DISPLAY )
        mju_error_i("Could not get EGL display, error 0x%x\n", eglGetError());

    // initialize
    EGLint major, minor;
    if( eglInitialize(eglDpy, &major, &minor)!=EGL_TRUE )
        mju_error_i("Could not initialize EGL, error 0x%x\n", eglGetError());

    // choose config
    EGLint numConfigs;
    EGLConfig eglCfg;
    if( eglChooseConfig(eglDpy, configAttribs, &eglCfg, 1, &numConfigs)!=EGL_TRUE )
        mju_error_i("Could not choose EGL config, error 0x%x\n", eglGetError());

    // bind OpenGL API
    if( eglBindAPI(EGL_OPENGL_API)!=EGL_TRUE )
        mju_error_i("Could not bind EGL OpenGL API, error 0x%x\n", eglGetError());

    // create context
    EGLContext eglCtx = eglCreateContext(eglDpy, eglCfg, EGL_NO_CONTEXT, NULL);
    if( eglCtx==EGL_NO_CONTEXT )
        mju_error_i("Could not create EGL context, error 0x%x\n", eglGetError());

    // make context current, no surface (let OpenGL handle FBO)
    if( eglMakeCurrent(eglDpy, EGL_NO_SURFACE, EGL_NO_SURFACE, eglCtx)!=EGL_TRUE )
        mju_error_i("Could not make EGL context current, error 0x%x\n", eglGetError());

    //------------------------ OSMESA
#elif defined(MJ_OSMESA)
    // create context
    ctx = OSMesaCreateContextExt(GL_RGBA, 24, 8, 8, 0);
    if( !ctx )
        mju_error("OSMesa context creation failed");

    // make current
    if( !OSMesaMakeCurrent(ctx, buffer, GL_UNSIGNED_BYTE, 800, 800) )
        mju_error("OSMesa make current failed");

    //------------------------ GLFW
#else
    // init GLFW
    if (!glfwInit())
        mju_error("Could not initialize GLFW");

    // create invisible window, single-buffered
    glfwWindowHint(GLFW_VISIBLE, 0);
    glfwWindowHint(GLFW_DOUBLEBUFFER, GLFW_FALSE);
    GLFWwindow *window = glfwCreateWindow(800, 800, "Invisible window", nullptr, nullptr);
    if (!window)
        mju_error("Could not create GLFW window");

    // make context current
    glfwMakeContextCurrent(window);
#endif
}

// close OpenGL context/window
void RenderImage::closeOpenGL() {
    //------------------------ EGL
#if defined(MJ_EGL)
    // get current display
    EGLDisplay eglDpy = eglGetCurrentDisplay();
    if( eglDpy==EGL_NO_DISPLAY )
        return;

    // get current context
    EGLContext eglCtx = eglGetCurrentContext();

    // release context
    eglMakeCurrent(eglDpy, EGL_NO_SURFACE, EGL_NO_SURFACE, EGL_NO_CONTEXT);

    // destroy context if valid
    if( eglCtx!=EGL_NO_CONTEXT )
        eglDestroyContext(eglDpy, eglCtx);

    // terminate display
    eglTerminate(eglDpy);

    //------------------------ OSMESA
#elif defined(MJ_OSMESA)
    OSMesaDestroyContext(ctx);

    //------------------------ GLFW
#else
    // terminate GLFW (crashes with Linux NVidia drivers)
#if defined(__APPLE__) || defined(_WIN32)
    glfwTerminate();
#endif
#endif
}
