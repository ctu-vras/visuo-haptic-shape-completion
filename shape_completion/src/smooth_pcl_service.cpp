#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/surface/mls.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/filters/voxel_grid.h>
#include <iostream>
#include "shape_completion/smooth_pcl.h"
#include "ros/ros.h"

bool smooth(shape_completion::smooth_pcl::Request &req,
                 shape_completion::smooth_pcl::Response &res)
{

    pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZRGBA>);
    pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloud_filtered (new pcl::PointCloud<pcl::PointXYZRGBA>);
    pcl::PCDReader reader;
    reader.read<pcl::PointXYZRGBA> (req.path, *cloud);
    //std::cerr << "Cloud before filtering: " << std::endl;
    //std::cerr << *cloud << std::endl;
    /*
    pcl::StatisticalOutlierRemoval<pcl::PointXYZRGBA> sor;

    sor.setInputCloud (cloud);
    sor.setMeanK (50);
    sor.setStddevMulThresh (1.0);
    sor.filter (*cloud_filtered);

    //std::cerr << "Cloud after SOR: " << std::endl;
    //std::cerr << *cloud_filtered << std::endl;
    */
    pcl::VoxelGrid<pcl::PointXYZRGBA> avg;
    double leaf_size = req.leaf_size;

    avg.setInputCloud(cloud);
    avg.setLeafSize(leaf_size, leaf_size, leaf_size);
    avg.filter(*cloud_filtered);

    //std::cerr << "Cloud after voxel grid: " << std::endl;
    //std::cerr << *cloud_filtered << std::endl;


    pcl::PCDWriter writer;
    writer.write<pcl::PointXYZRGBA> ("/home/robot3/filtered.pcd", *cloud_filtered, false);

    //sor.setNegative (true);
    //sor.filter (*cloud_filtered);
    //writer.write<pcl::PointXYZRGBA> ("/home/robot3/outs.pcd", *cloud_filtered, false);

    pcl::search::KdTree<pcl::PointXYZRGBA>::Ptr search(new pcl::search::KdTree<pcl::PointXYZRGBA>);
    pcl::MovingLeastSquares<pcl::PointXYZRGBA, pcl::PointXYZRGBA> mls;
    pcl::PointCloud<pcl::PointXYZRGBA>::Ptr result (new pcl::PointCloud<pcl::PointXYZRGBA>);
    search->setInputCloud(cloud_filtered);

    mls.setInputCloud(cloud_filtered);
    mls.setPolynomialFit(true);
    mls.setPolynomialOrder(2);//General 2-5
    mls.setSearchMethod(search);
    mls.setSearchRadius(req.smooth_factor);//The bigger the smoother the greater
    mls.process(*result);

    pcl::io::savePCDFile("/home/robot3/smooth.pcd", *result);

  return true;
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "smooth_pcl_server");

  ros::NodeHandle node_handle;
  ros::ServiceServer service = node_handle.advertiseService("smooth_pcl", smooth);
  ros::AsyncSpinner spinner(2);
  spinner.start();
  ros::waitForShutdown();

  return 0;
}