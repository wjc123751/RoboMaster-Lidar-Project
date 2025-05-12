#include "pub_depth_node.h"

/*构造函数*/
PubDepthNode::PubDepthNode() :  m_nh("~")
{
    // 订阅雷达节点
    m_subLidar = m_nh.subscribe<sensor_msgs::PointCloud2>("/livox/lidar", 1, &pub_depth_image::pointCloudCallback);

    // 初始化图像传输对象
    image_transport::ImageTransport p_imgTopic(m_nh);
    // 创建图像发布者
    m_depthImgPub = p_imgTopic.advertiseCamera("depth_raw", 1);
    // 从参数服务器获取相机的frame_id，默认为"depth_frame"
    m_nh.param("depth_frame_id", m_depthImgMsg.header.frame_id, std::string("depth_frame"));
}

/*析构函数*/
PubDepthNode::~PubDepthNode()
{
    // 析构函数
}

/*发送深度图像函数*/
bool PubDepthNode::sendImage()
{
    // 使用互斥量保护共享数据
    std::lock_guard<std::mutex> lock(pub_depth_image::bufferMutex);
    // 如果成功获取深度图像
    if (pub_depth_image::depthImageBuffer.size() > 0)
    {
        cv::Mat p_depthImage;
        p_depthImage = pub_depth_image::depthImageBuffer.back();
        // 更新图像消息的时间戳
        m_depthImgMsg.header.stamp = ros::Time::now();
        // 填充图像消息
        sensor_msgs::fillImage(m_depthImgMsg, sensor_msgs::image_encodings::TYPE_32FC1, p_depthImage.rows, p_depthImage.cols,
            p_depthImage.step, p_depthImage.data);
        // 创建相机信息对象
        sensor_msgs::CameraInfo p_depthCamInfo;
        // 设置相机信息对象的帧ID和时间戳
        p_depthCamInfo.header.frame_id = m_depthImgMsg.header.frame_id;
        p_depthCamInfo.header.stamp = m_depthImgMsg.header.stamp;
        // 发布图像消息和相机信息
        m_depthImgPub.publish(m_depthImgMsg, p_depthCamInfo);

        return true;
    }

    return false;
}

/*运行节点函数*/
bool PubDepthNode::spin()
{
    // 设置循环频率
    ros::Rate p_csRate(50);

    // 循环运行节点
    while (m_nh.ok())
    {
        // 发送图像，如果超时则发出警告
        if (!PubDepthNode::sendImage())
        {
            ROS_WARN("Depth image did not respond in time.");
        }

        // 处理回调函数
        ros::spinOnce();
        // 按照循环频率休眠
        p_csRate.sleep();
    }

    return true;
}

/*主函数*/
int main(int argc, char **argv) 
{
    // 初始化ROS节点
    ros::init(argc, argv, "pub_depth_node");

    // 加载点云和相机参数
    parameters::loadPointCloudParameters();
    parameters::loadCameraParameters();

    // 创建深度节点对象
    PubDepthNode pub_dep;

    // 进入ROS循环，处理订阅的消息
    pub_dep.spin();

    return 0;
}