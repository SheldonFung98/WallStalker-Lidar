#ifndef WALL_DETECT_H
#define WALL_DETECT_H

#include "common.h"
#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.h>

#define POLYNUM 1
#define WALL_VALID_THRESHOLD        5
#define WALL_VALID_SUM_THRESHOLD    20
#define WALL_VALID_POINT_NUM_THRESHOLD    20

// Laser Crop ROI
#define LROI_X_MIN  -1.0
#define LROI_X_MAX  3.0
#define LROI_Y_MIN  -1.5
#define LROI_Y_MAX  1.5

// Linear regression Region
#define LRR_X_MIN  -0.2
#define LRR_X_MAX  2.0

// Coordinate of Original point
#define ORIG_X 150
#define ORIG_Y 300
#define ORIGINAL ORIG_X, ORIG_Y

// Size of the laser image 
#define LASER_IMAGE_H   400
#define LASER_IMAGE_W   300
#define LASER_IMAGE_SIZE LASER_IMAGE_H, LASER_IMAGE_W
using namespace std;

class WallDetect{

private:
    ros::NodeHandle nh;
    ros::Subscriber laserSub;
    ros::Publisher wallPathPub;
    ros::Publisher imagePub;
   
private:
    string laser_sub_path;
    string wall_pub_path;
    string image_pub_path;
    string transform_coord;
    
private:
    laser_geometry::LaserProjection projector_;
    tf::TransformListener listener_;
    tf::TransformListener coord_transform_listener;
    tf::StampedTransform trans;

    bool shouldFindWall = true;
    bool echoNoWallWarning = false;
    cv::Mat lineFitCoeff;
    cv::Point ori;
    const double scale_x = LASER_IMAGE_H/(LROI_X_MAX-LROI_X_MIN);
    const double scale_y = LASER_IMAGE_W/(LROI_Y_MAX-LROI_Y_MIN);

public:
    WallDetect();
    ~WallDetect();
    // void laser_callback(const PcROS_Ptr& cloud_msg);
    void laser_callback(const sensor_msgs::LaserScan::ConstPtr& scan_in);

private:
    void getParams();
    void init_transform();
    void laser2PcXYZ(sensor_msgs::LaserScan::ConstPtr scan, PcXYZ_Ptr pc);
    cv::Point pcPoint2imPoint(pcl::PointXYZ p);
    bool wallValidation(std::vector<cv::Point> linePoint);
    // Calculate distance between each point and the line.
    // Draw expected wall point
    std::vector<double> point2lineDist(
        std::vector<cv::Point> linePoint, 
        cv::Mat lineCoeff);
};


#endif

