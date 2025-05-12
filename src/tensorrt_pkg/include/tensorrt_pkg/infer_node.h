#include <iostream>
#include <ros/ros.h>
#include <opencv2/opencv.hpp>
#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/image_encodings.h>
#include "mycuda.h"                                                                 // CUDA相关功能头文件
#include "msgs_pkg/yolo_points.h"
#include "msgs_pkg/yolo_point.h"

class InferNode
{
public:
    InferNode();                                                                                // 构造函数
    ~InferNode();                                                                              // 析构函数

    void imageCallback(const sensor_msgs::ImageConstPtr& msg);              // 图像回调函数

private:
    ros::NodeHandle m_nh;                                        // ROS节点句柄
    ros::Subscriber m_ImageSub;                                            // ROS图像消息订阅者
    ros::Publisher m_rectMsgPub;

    msgs_pkg::yolo_points m_rectMsg;
};

