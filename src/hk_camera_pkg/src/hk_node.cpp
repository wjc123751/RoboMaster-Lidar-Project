#include "hk_node.h"

/*构造函数*/
HkCamNode::HkCamNode() : m_csHkNode("~")
{
    // 初始化图像传输对象
    image_transport::ImageTransport p_imgTopic(m_csHkNode);
    // 创建图像发布者
    m_csImgPub = p_imgTopic.advertiseCamera("image_raw", 1);
    // 从参数服务器获取相机的frame_id，默认为"hk_camera"
    m_csHkNode.param("camera_frame_id", m_stImgMsg.header.frame_id, std::string("hk_camera"));
    // 输出ROS节点启动信息
    ROS_INFO("Starting ");

    // 启动海康相机
    m_cam.start();
}

/*析构函数*/
HkCamNode::~HkCamNode()
{
    // 析构函数为空
}

/*发送图像函数*/
bool HkCamNode::sendImage()
{
    // 如果成功获取RGB图像
    if (m_cam.grabRGBImage())
    {
        // 更新图像消息的时间戳
        m_stImgMsg.header.stamp = ros::Time::now();
        // 填充图像消息
        sensor_msgs::fillImage(m_stImgMsg, "rgb8", m_cam.m_stConvertParam.nHeight, m_cam.m_stConvertParam.nWidth,
            3 * m_cam.m_stConvertParam.nWidth, m_cam.m_stConvertParam.pDstBuffer);
        // 创建相机信息对象
        sensor_msgs::CameraInfo p_stCamInfo;
        // 设置相机信息对象的帧ID和时间戳
        p_stCamInfo.header.frame_id = m_stImgMsg.header.frame_id;
        p_stCamInfo.header.stamp = m_stImgMsg.header.stamp;
        // 发布图像消息和相机信息
        m_csImgPub.publish(m_stImgMsg, p_stCamInfo);
        return true;
    }

    return false;
}

/*运行节点函数*/
bool HkCamNode::spin()
{
    // 设置循环频率
    ros::Rate p_csRate(50);

    // 循环运行节点
    while (m_csHkNode.ok())
    {
        // 发送图像，如果超时则发出警告
        if (!HkCamNode::sendImage())
        {
            ROS_WARN("HIKROBOT camera did not respond in time.");
        }

        // 处理回调函数
        ros::spinOnce();
        // 按照循环频率休眠
        p_csRate.sleep();
    }

    return true;
}

/*主函数*/
int main(int argc, char** argv)
{
    // 初始化ROS节点
    ros::init(argc, argv, "hk_node");

    // 创建海康相机节点对象
    HkCamNode hk;

    // 运行节点
    hk.spin();

    return 0;
}
