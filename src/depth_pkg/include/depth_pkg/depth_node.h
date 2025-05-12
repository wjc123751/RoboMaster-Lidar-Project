#include <iostream>
#include <ros/ros.h>
#include <pcl/point_cloud.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_types.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/segmentation/extract_clusters.h>
#include <opencv2/opencv.hpp>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/PointCloud2.h>
#include<geometry_msgs/Point.h>
#include "depth_image.h"

/* 深度节点类 */
class DepthNode
{
public:
    DepthNode();                                                                              // 构造函数
    ~DepthNode();                                                                           // 析构函数


private:
    ros::NodeHandle m_nh;                                                       // ROS节点句柄
    ros::Subscriber m_subLidar;                                             // 订阅雷达节点的点云消息
    ros::Subscriber m_subNumber;                                       // 订阅号码牌检测框信息


};