#include <stdio.h>
#include <string.h>
#include <unistd.h>
#include <stdlib.h>
#include <sstream>
#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/fill_image.h>
#include <image_transport/image_transport.h>
#include "MvCameraControl.h"
#include "hk.h"

/* 海康相机节点类 */
class HkCamNode
{
public:
    HkCamNode();                                                                              // 构造函数
    ~HkCamNode();                                                                           // 析构函数

    HkCam m_cam;                                                                            // 海康相机对象
    bool sendImage();                                                                       // 发送图像函数
    bool spin();                                                                                     // 运行节点函数

private:
    ros::NodeHandle m_csHkNode;                                             // ROS节点句柄
    sensor_msgs::Image m_stImgMsg;                                       // 图像消息对象
    image_transport::CameraPublisher m_csImgPub;        // 图像发布者对象
};
