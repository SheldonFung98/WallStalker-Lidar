#ifndef STICKWALL_COMMON
#define STICKWALL_COMMON

#include <ros/ros.h>

// PCL specific includes
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/LaserScan.h>
#include <nav_msgs/Path.h>

#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/ModelCoefficients.h>
#include <pcl/segmentation/sac_segmentation.h>

#include <pcl/features/normal_3d_omp.h>
#include <pcl/segmentation/region_growing.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/segmentation/progressive_morphological_filter.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/filters/passthrough.h>
#include <pcl/common/impl/angles.hpp>

#include "laser_geometry/laser_geometry.h"
#include "tf/transform_listener.h"

#include <iostream>
#include <unordered_set>
#include <chrono>



typedef sensor_msgs::PointCloud2 PcROS;
typedef sensor_msgs::PointCloud2ConstPtr PcROS_Ptr;
typedef pcl::PointCloud<pcl::PointXYZ> PcXYZ;
typedef pcl::PointCloud<pcl::PointXYZ>::Ptr PcXYZ_Ptr;

#endif