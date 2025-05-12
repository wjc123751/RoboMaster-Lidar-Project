#include "infer_node.h" 

/*推理节点类的构造函数*/ 
InferNode::InferNode()
{
    // 订阅图像话题，设置回调函数为imageCallback，并将this指针传递作为回调函数的对象
    m_ImageSub = m_nh.subscribe("/hk_node/image_raw", 1, &InferNode::imageCallback, this);

    // 发布号码牌检测框话题
    m_rectMsgPub = m_nh.advertise<msgs_pkg::yolo_points>("number_msg", 10);
}

/*推理节点类的析构函数*/ 
InferNode::~InferNode()
{

}

/*图像回调函数，处理接收到的图像消息*/
void InferNode::imageCallback(const sensor_msgs::ImageConstPtr& msg)
{
    m_rectMsg.data.clear();

    // 将ROS图像消息转换为OpenCV格式
    cv_bridge::CvImagePtr p_cvptrImage = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
    cv::Mat p_Image = p_cvptrImage->image;

    // 判断图像是否为空，如果为空，输出错误消息并断言
    if(p_Image.empty())
    {
        std::cerr<<"p_Image empty!!!!!!"<<std::endl;
        assert(!p_Image.empty());
    }

    // 拷贝原始图像用于显示
    cv::Mat p_ImgShow;
    p_Image.copyTo(p_ImgShow);
    cv::Mat p_ImgRaw;
    p_Image.copyTo(p_ImgRaw);

    float *p_BufferIdxCar = (float *)g_buffersCar[g_inputIndexCar];

    // 计算图像大小和目标大小
    size_t p_ImageSize = p_Image.cols * p_Image.rows * 3;
    size_t p_ImageDstSize = INPUT_W_CAR * INPUT_H_CAR * 3;

    // 将图像数据从主机内存复制到设备内存
    memcpy(g_imgHostCar, p_Image.data, p_ImageSize);
    CUDA_CHECK(cudaMemcpyAsync(g_imgDeviceCar, g_imgHostCar, p_ImageSize, cudaMemcpyHostToDevice));

    // 对输入图像进行预处理操作
    preprocess_kernel_img(g_imgDeviceCar, p_Image.cols, p_Image.rows, p_BufferIdxCar, INPUT_W_CAR, INPUT_H_CAR, g_streamCar);

    p_BufferIdxCar += p_ImageDstSize;

    // 执行车辆检测推理
    inferspace::doInference(*g_contextCar, g_streamCar, (void **) g_buffersCar, g_probCar, BATCH_SIZE_CAR);
    
    // 非极大值抑制得到车辆检测结果
    std::vector<Yolo::Detection> p_resCar;
    nms(p_resCar, &g_probCar[0], CONF_THRESH, NMS_THRESH);
    
    // 绘制车辆检测框
    // std::cout<<"p_resCar.size :"<<(int)p_resCar.size()<<std::endl;

    if(p_resCar.size() > 0)
        for (size_t j = 0; j < p_resCar.size(); j++) 
        {
            cv::Rect r = get_rect_car(p_Image, p_resCar[j].bbox);
            cv::rectangle(p_ImgShow, r, cv::Scalar(0x27, 0xC1, 0x36), 2);
            // cv::putText(p_ImgShow, std::to_string((int)p_resCar[j].class_id), cv::Point(r.x, r.y - 1), cv::FONT_HERSHEY_PLAIN, 1.2, cv::Scalar(0xFF, 0xFF, 0xFF), 2);
        }

    // 在车辆检测框内进行车牌号识别
    int fcount = 0;
    std::vector<cv::Mat> p_imgsBuffer(BATCH_SIZE_NUMBER);
    for (size_t j = 0; j < p_resCar.size(); j++) 
    {
        fcount++;
        if (fcount < BATCH_SIZE_NUMBER && j + 1 != p_resCar.size()) 
        {
            continue;
        }

        // 从车辆检测框中提取车辆图像ROI
        float *p_BufferIdxNumber = (float *) g_buffersNumber[g_inputIndexNumber];
        for (int b = 0; b < fcount; b++) 
        {
            cv::Rect p_rectCar = get_rect_car(p_Image, p_resCar[j - fcount + 1 + b].bbox);
            cv::Mat p_carRoi;
            // 裁剪ROI区域
            if (p_rectCar.x < 0) 
            {
                p_rectCar.x = 0;
            }
            if (p_rectCar.y < 0) 
            {
                p_rectCar.y = 0;
            }
            if ((p_rectCar.x + p_rectCar.width) > p_Image.cols) 
            {
                p_rectCar.width = p_Image.cols - p_rectCar.x;
            }
            if ((p_rectCar.y + p_rectCar.height) > p_Image.rows) 
            {
                p_rectCar.height = p_Image.rows - p_rectCar.y;
            }
            p_ImgRaw(p_rectCar).copyTo(p_carRoi);
            if (p_carRoi.empty())
            {
                continue;
            }

            // 将车辆ROI图像复制到缓冲区
            p_imgsBuffer[b] = p_carRoi;
            size_t p_carRoiSize = p_carRoi.cols * p_carRoi.rows * 3;
            size_t p_carRoiDstSize = INPUT_H_NUMBER * INPUT_W_NUMBER * 3;

            memcpy(g_imgHostNumber, p_carRoi.data, p_carRoiSize);
            CUDA_CHECK(cudaMemcpyAsync(g_imgDeviceNumber, g_imgHostNumber, p_carRoiSize, cudaMemcpyHostToDevice));

            // 对车牌号识别输入图像进行预处理
            preprocess_kernel_img(g_imgDeviceNumber, p_carRoi.cols, p_carRoi.rows, p_BufferIdxNumber, INPUT_W_NUMBER, INPUT_H_NUMBER, g_streamNumber);

            p_BufferIdxNumber += p_carRoiDstSize;
        }
        // 执行车牌号识别推理
        inferspace::doInference(*g_contextNumber, g_streamNumber, (void **) g_buffersNumber, g_probNumber, BATCH_SIZE_NUMBER);
        std::vector<std::vector<Yolo::Detection>> p_resNumber(fcount);
        for (int b = 0; b < fcount; b++) 
        {
            auto &res = p_resNumber[b];
            nms(res, &g_probNumber[b * OUTPUT_SIZE], NUMBER_CONF_THRESH, NUMBER_NMS_THRESH);
        }

        // 绘制车牌号检测框
        for (int b = 0; b < fcount; b++)
        {
            auto &res = p_resNumber[b];
            cv::Rect p_getRectCar = get_rect_car(p_ImgRaw, p_resCar[j - fcount + 1 + b].bbox);
            Yolo::Detection p_maxConfRes;
            p_maxConfRes.conf = 0;
            p_maxConfRes.class_id = 14;
            cv::Rect real_rect(0, 0, 0, 0);
            
            for (size_t m = 0; m < res.size(); m++)
            {
                std::cout<<"res.size :"<<(int)res.size()<<std::endl;

                cv::Rect p_numberInCarRoi = get_rect_number(p_imgsBuffer[b], res[m].bbox);
                cv::Rect p_numberInImage = p_numberInCarRoi;
                p_numberInImage.x += p_getRectCar.x;
                p_numberInImage.y += p_getRectCar.y;
                if (real_rect.area() < p_numberInImage.area()) 
                {
                    real_rect = p_numberInImage;
                }

                cv::rectangle(p_ImgShow, p_numberInImage, cv::Scalar(0x27, 0xC1, 0x36), 3);

                if (res[m].conf > p_maxConfRes.conf) 
                {
                    p_maxConfRes = res[m];
                }
            }

            // 根据装甲版Rect修改车的Rect
            if(real_rect.area() > 0)
            {
                msgs_pkg::yolo_point p_pointMsg;
                p_pointMsg.id = p_maxConfRes.class_id;
                
                // if(p_pointMsg.id == 0)
                // {
                //     p_pointMsg.id = 3;
                //     cv::putText(p_ImgShow, std::to_string(3), cv::Point(real_rect.x, real_rect.y - 1), cv::FONT_HERSHEY_PLAIN, 1.2, cv::Scalar(0, 0, 255), 2);
                // }
                // else if(p_pointMsg.id == 1)
                // {
                //     p_pointMsg.id = 8;
                //     cv::putText(p_ImgShow, std::to_string(8), cv::Point(real_rect.x, real_rect.y - 1), cv::FONT_HERSHEY_PLAIN, 1.2, cv::Scalar(0, 0, 255), 2);
                // }

                cv::putText(p_ImgShow, std::to_string(p_pointMsg.id), cv::Point(real_rect.x, real_rect.y - 1), cv::FONT_HERSHEY_PLAIN, 1.2, cv::Scalar(0, 0, 255), 2);
                
                if(p_resCar[j - fcount + 1 + b].class_id == 0)
                {
                    p_pointMsg.color = false;
                    cv::putText(p_ImgShow, "car", cv::Point(p_getRectCar.x+20, p_getRectCar.y), cv::FONT_HERSHEY_PLAIN, 1.2, cv::Scalar(0, 0, 255), 2);
                }
                else if(p_resCar[j - fcount + 1 + b].class_id == 1)
                {
                    p_pointMsg.color = true;
                    cv::putText(p_ImgShow, "car", cv::Point(p_getRectCar.x+20, p_getRectCar.y), cv::FONT_HERSHEY_PLAIN, 1.2, cv::Scalar(0, 0, 255), 2);
                }
                p_pointMsg.x = (int)real_rect.x;
                p_pointMsg.y = (int)real_rect.y;
                p_pointMsg.width = (int)real_rect.width;
                p_pointMsg.height = (int)real_rect.height;

                m_rectMsg.data.push_back(p_pointMsg);

                // std::cout<<"p_pointMsg.id :"<<(int)p_pointMsg.id<<std::endl
                //                 <<"p_pointMsg.color  :"<<(int)p_pointMsg.color<<std::endl
                //                 <<"p_pointMsg.x :"<<p_pointMsg.x<<std::endl
                //                 <<"p_pointMsg.y :"<<p_pointMsg.y<<std::endl
                //                 <<"p_pointMsg.width :"<<p_pointMsg.width<<std::endl
                //                 <<"p_pointMsg.height :"<<p_pointMsg.height<<std::endl;

                // 发送ROS消息
                if(m_rectMsg.data.size() != 0)
                {
                    m_rectMsg.text = "some";
                    m_rectMsg.header.stamp = ros::Time::now();
                    m_rectMsg.header.frame_id = "number";

                    m_rectMsgPub.publish(m_rectMsg);
                }
                else
                {
                    msgs_pkg::yolo_points p_rectMsg;
                    p_rectMsg.text = "none";
                    // m_rectMsgPub.publish(p_rectMsg);
                }
            }
            else
            {
                // int p_change = p_getRectCar.width * 0.2;
                // p_getRectCar.x += p_change;
                // p_getRectCar.width -= p_change;
                // p_change = p_getRectCar.height * 0.2;
                // p_getRectCar.y += p_change;
                // p_getRectCar.height -= p_change;
            }

            // cv::rectangle(p_ImgShow, p_getRectCar, cv::Scalar(0x27, 0xC1, 0x36), 3);

        }
        fcount = 0;
    }
    // 显示结果图像
    cv::imshow("image_show",p_ImgShow);
    cv::waitKey(1);
}

/*主函数*/
int main(int argc, char** argv)
{
    // 初始化CUDA
    inferspace::cudaInit();

    // 初始化ROS节点
    ros::init(argc, argv, "infer_node");

    // 创建推理节点对象
    InferNode nn;

    // ROS主循环
    ros::spin();

    // 销毁CUDA资源
    inferspace::cudaDestroy();

    return 0;
}

