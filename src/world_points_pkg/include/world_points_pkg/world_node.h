#include "ros/ros.h"
#include <iostream>
#include <opencv2/opencv.hpp>
#include <cv_bridge/cv_bridge.h>
#include<geometry_msgs/Point.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/PointCloud2.h>
#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
#include "msgs_pkg/yolo_points.h"
#include "msgs_pkg/yolo_point.h"
#include "get_world.h"

/* 世界坐标类 */
class WorldNode
{
public:
    WorldNode();
    ~WorldNode();

    void depthImageCallback(const sensor_msgs::ImageConstPtr& msg);
    void numberMsgCallback(const msgs_pkg::yolo_points::ConstPtr& input);

    ros::NodeHandle m_nh;
    ros::Subscriber m_subDepthImg;
    ros::Subscriber m_subNumber;

};

