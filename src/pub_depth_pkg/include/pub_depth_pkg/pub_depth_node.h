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
#include <sensor_msgs/Image.h>
#include <sensor_msgs/fill_image.h>
#include <image_transport/image_transport.h>
#include "pub_depth_image.h"

/* 深度节点类 */
class PubDepthNode
{
public:
    PubDepthNode();                                                                                         // 构造函数
    ~PubDepthNode();                                                                                      // 析构函数

    bool sendImage();
    bool spin();

private:
    ros::NodeHandle m_nh;                                                                           // ROS节点句柄
    ros::Subscriber m_subLidar;                                                                 // 订阅雷达节点的点云消息
    image_transport::CameraPublisher m_depthImgPub;             // 图像发布者对象
    sensor_msgs::Image m_depthImgMsg;                                           // 图像消息对象
};