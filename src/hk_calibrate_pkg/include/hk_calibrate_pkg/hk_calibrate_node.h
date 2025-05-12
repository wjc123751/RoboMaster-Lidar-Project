#include <yaml-cpp/yaml.h>
#include <iostream>
#include <fstream>
#include <vector>
#include <ros/ros.h>
#include <opencv2/opencv.hpp>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/Image.h>
#include <Eigen/Geometry>
#include <Eigen/Dense>

/* 海康相机节点类 */
class CalibrateNode
{
public:
    CalibrateNode();                                                                              // 构造函数
    ~CalibrateNode();                                                                           // 析构函数

    void imageCallback(const sensor_msgs::ImageConstPtr& msg);
    void pointInit();
    void calculate();
    void writeYaml(const cv::Mat& rvec, const cv::Mat& tvec, const std::string& yaml_name);



private:
    ros::NodeHandle m_nh;                                             // ROS节点句柄
    ros::Subscriber m_hk;
    sensor_msgs::Image m_imgMsg;                                       // 图像消息对象
    std::vector<cv::Point3d> m_cloudPoints;    // 点云的3D点集
    std::vector<cv::Point2f> m_imgPoints;      // 图片的2D点集
    cv::Mat m_cameraMatrix = cv::Mat_<double>(3, 3);              // 相机内参矩阵
    cv::Mat m_distortionCoefficient = cv::Mat_<double>(5, 1);     // 相机畸变系数
    cv::Mat m_rvec;
    cv::Mat m_tvec;

};

