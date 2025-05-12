#include "world_node.h"

void WorldNode::depthImageCallback(const sensor_msgs::ImageConstPtr& msg)
{
    // 将ROS图像消息转换为OpenCV格式
    cv_bridge::CvImagePtr p_cvptrImage = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::TYPE_32FC1);
    cv::Mat p_Image = p_cvptrImage->image;

    if(!p_Image.empty())
    {
        // 显示深度图像
        cv::imshow("p_Image", p_Image);
        cv::waitKey(1);
    }
}

void WorldNode::numberMsgCallback(const msgs_pkg::yolo_points::ConstPtr& input)
{
    msgs_pkg::yolo_points p_numberMsg;
    p_numberMsg = *input;

    for(int i = 0; i < p_numberMsg.data.size(); i++)
    {
        msgs_pkg::yolo_point p_rectMsg;
        p_rectMsg = p_numberMsg.data[i];                                // 取出检测框信息缓冲队列最新元素
        
        std::cout<<"p_rectMsg.color : "<<(int)p_rectMsg.color<<std::endl;
        std::cout<<"p_rectMsg.id : "<<(int)p_rectMsg.id<<std::endl;
        std::cout<<"p_rectMsg.x : "<<p_rectMsg.x<<std::endl;
        std::cout<<"p_rectMsg.y : "<<p_rectMsg.y<<std::endl;
        std::cout<<"p_rectMsg.width : "<<p_rectMsg.width<<std::endl;
        std::cout<<"p_rectMsg.height : "<<p_rectMsg.height<<std::endl;
    }
}

WorldNode::WorldNode()
{
    // // 订阅图像话题，设置回调函数为imageCallback，并将this指针传递作为回调函数的对象
    // m_subDepthImg = m_nh.subscribe<sensor_msgs::Image>("/pub_depth_node/depth_raw", 1, &WorldNode::depthImageCallback, this);

    // // 订阅号码牌检测框信息
    // m_subNumber = m_nh.subscribe<msgs_pkg::yolo_points>("number_msg", 1, &WorldNode::numberMsgCallback, this);
}

WorldNode::~WorldNode()
{

}

/*同步数据回调函数*/
void syncCallback(const sensor_msgs::ImageConstPtr& depth_msg, const msgs_pkg::yolo_points::ConstPtr& number_msg)
{
    // 将ROS图像消息转换为OpenCV格式
    cv_bridge::CvImagePtr p_ptrDepthImage = cv_bridge::toCvCopy(depth_msg, sensor_msgs::image_encodings::TYPE_32FC1);
    cv::Mat p_depthImage = p_ptrDepthImage->image;

    msgs_pkg::yolo_points p_numberMsg;
    p_numberMsg = *number_msg;

    if(!p_depthImage.empty())
    {
        std::cout<<"p_numberMsg.data.size() "<<p_numberMsg.data.size()<<std::endl;
        for(int i = 0; i < p_numberMsg.data.size(); i++)
        {
            msgs_pkg::yolo_point p_rectMsg;
            p_rectMsg = p_numberMsg.data[i];

            // 获取检测框深度信息
            float p_numberDepth = world_space::getNumberDepth(p_rectMsg.x, p_rectMsg.y, p_rectMsg.width, p_rectMsg.height, p_depthImage);
        
            // 检查相邻两次深度信息之差是否小于 1 米
            if (std::abs(p_numberDepth - world_space::prevNumberDepth) < 1.0f) 
            {
                // 计算目标点的世界坐标
                geometry_msgs::Point p_worldPoint;
                cv::Point2f p_center(p_rectMsg.x + p_rectMsg.width/2, p_rectMsg.y + p_rectMsg.height/2);
                p_worldPoint = world_space::getWorldPosition(p_numberDepth, p_center.x, p_center.y);

                // 地图点消息
                world_space::mapMsg.id = p_rectMsg.id;
                world_space::mapMsg.x = p_worldPoint.x;
                world_space::mapMsg.y = p_worldPoint.y;

                // 发消息
                world_space::mapMsgPub.publish(world_space::mapMsg);
            }

            // 更新上一次的深度信息值
            world_space::prevNumberDepth = p_numberDepth;

        }
    }
}

/*主函数*/
int main(int argc, char** argv)
{
    ros::init(argc, argv, "world_node");

    // WorldNode wh;

    ros::NodeHandle p_nh;
    // 发布号码牌检测框话题
    world_space::mapMsgPub = p_nh.advertise<msgs_pkg::world_points>("map_msg", 10);
    message_filters::Subscriber<sensor_msgs::Image> p_subDepthImg(p_nh, "/pub_depth_node/depth_raw", 1);
    message_filters::Subscriber<msgs_pkg::yolo_points> p_subNumerMsg(p_nh, "number_msg", 1);

    typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::Image, msgs_pkg::yolo_points> MySyncPolicy;
    message_filters::Synchronizer<MySyncPolicy> sync(MySyncPolicy(20), p_subDepthImg, p_subNumerMsg);

    sync.registerCallback(boost::bind(&syncCallback,  _1, _2));

    ros::spin();

    return 0;
}
