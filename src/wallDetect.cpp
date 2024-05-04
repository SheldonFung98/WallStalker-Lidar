#include "wallDetect.h"

#include <opencv2/opencv.hpp>

WallDetect::WallDetect(): ori(ORIGINAL){
    getParams();
    // Create a ROS subscriber for the input point cloud
    laserSub = nh.subscribe(laser_sub_path, 1, 
                            &WallDetect::laser_callback, 
                            this);
    // Create a ROS publisher for the output point cloud
    wallPathPub = nh.advertise<nav_msgs::Path>(wall_pub_path, 1);
    imagePub = nh.advertise<sensor_msgs::Image>(image_pub_path, 1);
    init_transform();
}

WallDetect::~WallDetect(){

}

tf::StampedTransform FindTransform(const std::string& target_frame, 
                                const std::string source_frame, 
                                tf::TransformListener& transform_listener){
    tf::StampedTransform transform;
    //velodyne_map_tf_ok = false;
    try {
        // ros::Time(0)指定了时间为0，即获得最新有效的变换。
        // 改变获取当前时间的变换，即改为ros::Time::now(),不过now的话因为监听器有缓存区的原因。一般会出错
        // 参考：https://www.ncnynl.com/archives/201702/1313.html
        transform_listener.waitForTransform(target_frame, 
                                            source_frame, 
                                            ros::Time(0), 
                                            ros::Duration(3.0));
        transform_listener.lookupTransform(target_frame, 
                                            source_frame, 
                                            ros::Time(0), 
                                            transform);
        //velodyne_map_tf_ok = true;
        // ROS_INFO("[local_planner_trajectories_generator] : velodyne-map-tf obtained");
    } catch (tf::TransformException ex) {
        ROS_INFO(" find tf failed");
        ROS_INFO(ex.what());
        
    }
    return transform;
}

geometry_msgs::PoseStamped TransformPoint(const geometry_msgs::PoseStamped& in_point_pos,
                                        const tf::StampedTransform& in_transform){
    tf::Vector3 tf_point(in_point_pos.pose.position.x, in_point_pos.pose.position.y, in_point_pos.pose.position.z);
    tf::Vector3 tf_point_transformed = in_transform * tf_point;
    geometry_msgs::PoseStamped out_point_pos;
    out_point_pos.pose.position.x = tf_point_transformed.x();
    out_point_pos.pose.position.y = tf_point_transformed.y();
    out_point_pos.pose.position.z = tf_point_transformed.z();
    return out_point_pos;
    //return geometry_msgs::Point(tf_point_transformed.x(), tf_point_transformed.y(), tf_point_transformed.z());
}

void WallDetect::init_transform(){
    if(transform_coord.empty()) {
        ROS_INFO("Navigation Path frame is /laser");
    } else {
        ROS_INFO("Navigation Path frame transforms to frame: %s", transform_coord.c_str());
        trans = FindTransform(transform_coord, "/laser", coord_transform_listener);
    }
}


void WallDetect::getParams(){
    ros::param::get(ros::this_node::getName()+"/laser_sub_path", laser_sub_path);
    ros::param::get(ros::this_node::getName()+"/wall_pub_path", wall_pub_path);
    ros::param::get(ros::this_node::getName()+"/transform_coord", transform_coord);
    ros::param::get(ros::this_node::getName()+"/image_pub_path", image_pub_path);
}


cv::Mat polyfit(std::vector<cv::Point>& in_point, int n){
    // polyfit with repect to y. 
	int size = in_point.size();
    if(size > 0){
        // number of variable
        int x_num = n + 1;
        cv::Mat mat_u(size, x_num, CV_64F);
        cv::Mat mat_y(size, 1, CV_64F);
        for (int i = 0; i < mat_u.rows; ++i)
            for (int j = 0; j < mat_u.cols; ++j)
                mat_u.at<double>(i, j) = pow(in_point[i].y, j);
        for (int i = 0; i < mat_y.rows; ++i)
            mat_y.at<double>(i, 0) = in_point[i].x;
        // Get coefficient matrix k
        cv::Mat mat_k(x_num, 1, CV_64F);
        mat_k = (mat_u.t()*mat_u).inv()*mat_u.t()*mat_y;
        return mat_k;
    } else {
        cv::Mat mat;
        return mat;
    }
}

void WallDetect::laser2PcXYZ(sensor_msgs::LaserScan::ConstPtr scan, PcXYZ_Ptr pc){
    PcROS cloud;
    try{
        projector_.transformLaserScanToPointCloud(
          "laser",*scan, cloud, listener_);
    }catch (tf::TransformException& e){
        std::cout << e.what();
        return;
    }
    if ((cloud.width * cloud.height) == 0)
      return; //return if the cloud is not dense!
    else {
        pcl::fromROSMsg(cloud, *pc);
    }
}

cv::Point WallDetect::pcPoint2imPoint(pcl::PointXYZ p){
        cv::Point point(scale_x*p.y , scale_y*p.x);
        return ori-point;
}


void WallDetect::laser_callback(const sensor_msgs::LaserScan::ConstPtr& scan_in){
    PcXYZ_Ptr pcCloud(new PcXYZ);
    PcXYZ_Ptr pcCloud_roi(new PcXYZ);
    laser2PcXYZ(scan_in, pcCloud);

    cv::Mat laser_image(LASER_IMAGE_SIZE, CV_8UC3, cv::Scalar(0, 0, 0));
    // Draw original point
    cv::circle(laser_image, ori, 8, cv::Scalar(255, 255, 255), -1);
    pcl::PassThrough<pcl::PointXYZ> ptfilter (true); // Initializing with true will allow us to extract the removed indices
    // Crop out the ROI, namely the region we 

    ptfilter.setInputCloud (pcCloud);
    ptfilter.setFilterFieldName ("x");
    ptfilter.setFilterLimits (LROI_X_MIN, LROI_X_MAX);
    ptfilter.filter (*pcCloud);
    ptfilter.setFilterFieldName ("y");
    ptfilter.setFilterLimits (LROI_Y_MIN, LROI_Y_MAX);
    ptfilter.filter (*pcCloud);
    // Draw laser point on the image
    for (size_t i = 0; i < pcCloud->size(); i++){
        laser_image.at<cv::Vec3b>(pcPoint2imPoint(pcCloud->points[i])) = cv::Vec3b(255, 255, 255);
    }
    // Crop out the regression reference region. 
    // This region is used to perform linear regression
    PcXYZ_Ptr pcCloud_reg(new PcXYZ);
    ptfilter.setInputCloud (pcCloud);
    ptfilter.setFilterFieldName ("x");
    ptfilter.setFilterLimits(LRR_X_MIN, LRR_X_MAX);
    ptfilter.filter (*pcCloud_reg);

    // Draw points for regression on the image
    std::vector<cv::Point> linePoints;
    for (size_t i = 0; i < pcCloud_reg->size(); i++){
        cv::Point point = pcPoint2imPoint(pcCloud_reg->points[i]);
        circle(laser_image, point, 3, cv::Scalar(250, 0, 0), -1);
        linePoints.push_back(point);
    }

    std::vector<cv::Point> validLinePoints;
    if(shouldFindWall){
        // initialize the wall position and regression
        if(wallValidation(linePoints)){
            shouldFindWall = false;
        } else return;
    }

    // Get the distance between reference wall points and the previous regression line
    std::vector<double> dists = point2lineDist(linePoints, lineFitCoeff);
    // As the robot move forward, invalid wall points may appear at front.
    // Therefore, we validate each point from front to back and we empty 
    // the point list we encounter invalid points during this process.
    for(int i=dists.size()-1; i>0; i--){ 
        if(dists[i] < WALL_VALID_THRESHOLD){
            validLinePoints.push_back(linePoints[i]);
        } else {
            validLinePoints.clear();
        }
    }

    // Check if enough valid points exist.
    // Look for the wall again if the number of valid points
    // do not meet the requirement.
    if(validLinePoints.size() < WALL_VALID_POINT_NUM_THRESHOLD) {
        if(echoNoWallWarning){
            ROS_WARN("NO VALID POINTS DETECTED.");
            echoNoWallWarning = false;
        }
        shouldFindWall = true;
        return;
    } else {
        echoNoWallWarning = true;
        // Draw validated points for regression on the image
        for (size_t i = 0; i < validLinePoints.size(); i++){
            circle(laser_image, validLinePoints[i], 3, cv::Scalar(0, 0, 250), -1);
        }
    }

    // Perform linear regression on the valid points.
    lineFitCoeff = polyfit(validLinePoints, POLYNUM);

    nav_msgs::Path path;
    path.header.frame_id = "laser";
    path.header.stamp = ros::Time();
    // Draw regression line
    // Create navigation path
    int vlpStartY = validLinePoints.front().y;
    int vlpEndY = validLinePoints.back().y;
    // Swap the start and end index if start index is greater.
    if(vlpStartY > vlpEndY){
        int swapTemp;
        swapTemp = vlpEndY;
        vlpEndY = vlpStartY;
        vlpStartY = swapTemp;
    }
    for (int i = vlpStartY; i < vlpEndY; i++){
        cv::Point2d ipt;
        ipt.x = 0;
		ipt.y = i;
		for (int j = 0; j <= POLYNUM; j++){
			ipt.x += lineFitCoeff.at<double>(j, 0)*pow(i,j);
		}
        
		circle(laser_image, ipt, 1, 
            cv::Scalar(0, 200, 0), -1);
        geometry_msgs::PoseStamped pose;

        pose.header.stamp = ros::Time();
        pose.header.frame_id = "wall_path_point";
        pose.pose.position.x = (ori.y-ipt.y)/scale_x;
        pose.pose.position.y = (ori.x-ipt.x)/scale_y;

        geometry_msgs::PoseStamped transformed_pose;
        if(transform_coord.empty()) transformed_pose = pose;
        else transformed_pose = TransformPoint(pose, trans);

        path.poses.push_back(transformed_pose);
	}

    sensor_msgs::ImagePtr msg = cv_bridge::CvImage(
                                    std_msgs::Header(), 
                                    "rgb8", laser_image).toImageMsg();
    if(imagePub.getNumSubscribers() > 0) imagePub.publish(msg);
    if(wallPathPub.getNumSubscribers() > 0) wallPathPub.publish(path);
}

bool WallDetect::wallValidation(std::vector<cv::Point> linePoints){
    lineFitCoeff = polyfit(linePoints, POLYNUM);
    if(lineFitCoeff.empty()) return false;
    std::vector<double> dists = point2lineDist(linePoints, lineFitCoeff);

    int sum_error = 0;
    for(double dist : dists){
        if(dist > WALL_VALID_THRESHOLD){
            sum_error +=1;
        }
    }
    if(sum_error>WALL_VALID_SUM_THRESHOLD){
        ROS_WARN_ONCE("NO WALL DETECTED.");
        return false;
    } else {
        return true;
    }
}

std::vector<double> WallDetect::point2lineDist(std::vector<cv::Point> linePoints, cv::Mat lineCoeff){
    std::vector<double> dists;
    for(size_t i = 0; i < linePoints.size(); i++){
        cv::Point wallpoint(linePoints[i].y * lineCoeff.at<double>(1, 0) + lineCoeff.at<double>(0, 0), linePoints[i].y);
        dists.push_back(cv::norm(linePoints[i]-wallpoint));
    }
    return dists;
}