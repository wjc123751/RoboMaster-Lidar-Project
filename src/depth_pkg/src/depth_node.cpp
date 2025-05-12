#include "depth_node.h"

/*构造函数*/
DepthNode::DepthNode()
{
    // 订阅雷达节点
    m_subLidar = m_nh.subscribe<sensor_msgs::PointCloud2>("/livox/lidar", 1, &depth_image::pointCloudCallback);

    // 订阅号码牌检测框信息
    m_subNumber = m_nh.subscribe<msgs_pkg::yolo_points>("number_msg", 1, &depth_image::numberMsgCallback);
}

DepthNode::~DepthNode()
{
    // 析构函数
}


/*主函数*/
int main(int argc, char **argv) 
{
    // 初始化ROS节点
    ros::init(argc, argv, "depth_node");

    // 加载点云和相机参数
    parameters::loadPointCloudParameters();
    parameters::loadCameraParameters();

    // 创建深度节点对象
    DepthNode dep;

    // 进入ROS循环，处理订阅的消息
    ros::spin();

    return 0;
}