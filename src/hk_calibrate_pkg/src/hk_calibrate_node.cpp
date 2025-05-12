#include "hk_calibrate_node.h"

/**/
CalibrateNode::CalibrateNode()
{

    m_hk = m_nh.subscribe("/hk_node/image_raw", 1, &CalibrateNode::imageCallback, this);
}

/**/
CalibrateNode::~CalibrateNode()
{
    
}

/**/
cv::Mat g_mouseImg;
std::vector<cv::Point2f> g_yamlpoints;
int g_imgPointCount = 0;
bool g_selectPoints = false;
int g_calculateCount = 0;
bool g_calculateParam = false;

/**/
void CalibrateNode::pointInit()
{
    // 相机内参矩阵
    m_cameraMatrix.at<double>(0, 0) = 1785.12025;
    m_cameraMatrix.at<double>(0, 1) = 0.000000;
    m_cameraMatrix.at<double>(0, 2) = 769.9007;
    m_cameraMatrix.at<double>(1, 0) = 0.000000;
    m_cameraMatrix.at<double>(1, 1) = 1782.83017;
    m_cameraMatrix.at<double>(1, 2) = 545.44113;
    m_cameraMatrix.at<double>(2, 0) = 0.000000;
    m_cameraMatrix.at<double>(2, 1) = 0.000000;
    m_cameraMatrix.at<double>(2, 2) = 1.000000;

    // 畸变系数
    m_distortionCoefficient.at<double>(0) = -0.009408;
    m_distortionCoefficient.at<double>(1) = 0.112613;
    m_distortionCoefficient.at<double>(2) = -0.001735;
    m_distortionCoefficient.at<double>(3) = 0.010169;
    m_distortionCoefficient.at<double>(4) = 0.000000;

    m_imgPoints.push_back(g_yamlpoints[0]);
    m_imgPoints.push_back(g_yamlpoints[1]);
    m_imgPoints.push_back(g_yamlpoints[2]);
    m_imgPoints.push_back(g_yamlpoints[3]);

    m_cloudPoints.push_back(cv::Point3d(6230,14640,2520));
    m_cloudPoints.push_back(cv::Point3d(9070,11373,700));
    m_cloudPoints.push_back(cv::Point3d(1800,6900,230));
    m_cloudPoints.push_back(cv::Point3d(900,6900,230));
}

/**/
void CalibrateNode::calculate()
{
    /*旋转向量和平移向量*/
    static cv::Mat rvec;
    static cv::Mat tvec;

    CalibrateNode::pointInit();

    solvePnPRansac(m_cloudPoints, m_imgPoints, m_cameraMatrix, m_distortionCoefficient, rvec, tvec, cv::SOLVEPNP_AP3P);   /*pnp解算外参矩阵*/

    std::cout << "旋转向量: " << std::endl << rvec << std::endl;
    std::cout << "平移向量：" << std::endl << tvec << std::endl;

    m_rvec = rvec;
    m_tvec = tvec;
}

/*选点(角点)事件回调函数*/
void mouse(int event, int x, int y, int flags, void*)
{
    cv::Point2f imgPoint(x, y);
    if (event == cv::EVENT_LBUTTONDOWN) //单击左键，输出坐标
    {
        if(g_imgPointCount < 4 )
        {
            cv::circle(g_mouseImg, imgPoint, 5, cv::Scalar(0, 0, 255), 2);
            std::cout << " x = " << imgPoint.x << " y = " << imgPoint.y << std::endl;
            g_yamlpoints.push_back(imgPoint);
            g_imgPointCount++;
        }
    }
    imshow("g_mouseImg", g_mouseImg);
}

void CalibrateNode::writeYaml(const cv::Mat& rvec, const cv::Mat& tvec, const std::string& yaml_name)
{
    cv::FileStorage fs(yaml_name, cv::FileStorage::WRITE);
    if (!fs.isOpened()) 
    {
        std::cerr << "Error: Failed to open file " << yaml_name << std::endl;
        return;
    }

    // 写入旋转向量和平移向量到文件中
    fs << "rvec0" << rvec.at<double>(0,0);
    fs << "rvec1" << rvec.at<double>(1,0);
    fs << "rvec2" << rvec.at<double>(2,0);
    fs << "tvec0" << tvec.at<double>(0,0);
    fs << "tvec1" << tvec.at<double>(1,0);
    fs << "tvec2" << tvec.at<double>(2,0);

    fs.release();

    std::cout<<"保存成功！"<<std::endl;
}

/**/
void CalibrateNode::imageCallback(const sensor_msgs::ImageConstPtr& msg)
{
    cv_bridge::CvImagePtr cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
    cv::Mat img = cv_ptr->image;

    if(g_selectPoints == false)
    {
        img.copyTo(g_mouseImg);
        cv::imshow("g_mouseImg", g_mouseImg);
        cv::setMouseCallback("g_mouseImg", mouse, 0); //鼠标影响
        std::cout<<"选点完成后，按键继续"<<std::endl;
        cv::waitKey(0);
        g_selectPoints = true;
        cv::destroyWindow("g_mouseImg");
    }
    else
    {
        if(g_calculateParam == false)
        {
            /*pnp*/
            CalibrateNode::calculate();
            g_calculateCount++;

            if(g_calculateCount > 100)
            {
                g_calculateParam = true;
                g_calculateCount = 0;
            }
        }
        else
        {
            char p_response;
            std::cout << "是否保存外参矩阵？(y/n): ";
            std::cin >> p_response;
            if (p_response == 'y' || p_response == 'Y') 
            {
                writeYaml(m_rvec, m_tvec, "/home/climber/RM2024Climber/Yaml/hk_calibrate.yaml");

                ros::shutdown();
            }
            else
            {
                g_imgPointCount = 0;
                g_selectPoints = false;
                g_calculateParam = false;
            }

        }
    }
}


/**/
int main(int argc, char **argv)
{
    //执行 ros 节点初始化
    ros::init(argc, argv, "hk_calibrate_node");
    
    CalibrateNode cn;

    ros::spin();

    return 0;
}

