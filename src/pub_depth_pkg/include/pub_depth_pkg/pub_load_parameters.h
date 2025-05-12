#include <iostream>
#include <ros/ros.h>
#include <opencv2/opencv.hpp>

namespace parameters
{
    int length_of_cloud_queue_;                                                                 // 点云队列长度
    
    int imgcols_,imgrows_;                                                                              // 相机分辨率
    cv::Size imageSize;                                                                                       // 图片大小
    cv::Mat camera_matrix_ = cv::Mat_<float>(3, 3);                            // 相机内参矩阵
    cv::Mat distortion_coefficient_ = cv::Mat_<float>(5, 1);               // 相机畸变系数
    cv::Mat uni_matrix_ = cv::Mat_<float>(3, 4);                                     // 相机和雷达的变换矩阵
    cv::Mat tran_vec_ = cv::Mat_<float>(3, 1);                                          // 平移向量

    /*加载点云参数*/
    void loadPointCloudParameters()
    {
        ros::param::get("/length_of_cloud_queue", length_of_cloud_queue_);          //点云队列长度

        std::cout<<"点云队列长度："<<length_of_cloud_queue_<< std::endl;               // 输出点云队列长度
    }

    /*加载相机参数*/
    void loadCameraParameters()
    {
        // 获取相机分辨率参数
        ros::param::get("/image_dpi/image_width", imgcols_);
        ros::param::get("/image_dpi/image_height", imgrows_);
        imageSize.width = imgcols_;
        imageSize.height = imgrows_;
        std::cout<<"相机分辨率"<<imgcols_<<" * "<< imgrows_<<std::endl;

        // 获取相机内参参数
        ros::param::get("/camera_matrix/zerozero", camera_matrix_.at<float>(0, 0));
        camera_matrix_.at<float>(0, 1) = 0;
        ros::param::get("/camera_matrix/zerotwo", camera_matrix_.at<float>(0, 2));
        camera_matrix_.at<float>(1, 0) = 0;
        ros::param::get("/camera_matrix/oneone", camera_matrix_.at<float>(1, 1));
        ros::param::get("/camera_matrix/onetwo", camera_matrix_.at<float>(1, 2));
        camera_matrix_.at<float>(2, 0) = 0;
        camera_matrix_.at<float>(2, 1) = 0;
        ros::param::get("/camera_matrix/twotwo", camera_matrix_.at<float>(2, 2));

        // 获取相机畸变参数
        ros::param::get("/distortion_coefficient/zero", distortion_coefficient_.at<float>(0, 0));
        ros::param::get("/distortion_coefficient/one", distortion_coefficient_.at<float>(1, 0));
        ros::param::get("/distortion_coefficient/two", distortion_coefficient_.at<float>(2, 0));
        ros::param::get("/distortion_coefficient/three", distortion_coefficient_.at<float>(3, 0));
        ros::param::get("/distortion_coefficient/four", distortion_coefficient_.at<float>(4, 0));

        std::cout << "相机内参：" << camera_matrix_ << std::endl;                               // 输出相机内参
        std::cout << "相机畸变：" << distortion_coefficient_ << std::endl;                 // 输出相机畸变

        // 获取相机和雷达的变换矩阵参数
        ros::param::get("/uni_matrix/zerozero", uni_matrix_.at<float>(0, 0));
        ros::param::get("/uni_matrix/zeroone", uni_matrix_.at<float>(0, 1));
        ros::param::get("/uni_matrix/zerotwo", uni_matrix_.at<float>(0, 2));
        ros::param::get("/uni_matrix/zerothree", uni_matrix_.at<float>(0, 3));
        ros::param::get("/uni_matrix/onezero", uni_matrix_.at<float>(1, 0));
        ros::param::get("/uni_matrix/oneone", uni_matrix_.at<float>(1, 1));
        ros::param::get("/uni_matrix/onetwo", uni_matrix_.at<float>(1, 2));
        ros::param::get("/uni_matrix/onethree", uni_matrix_.at<float>(1, 3));
        ros::param::get("/uni_matrix/twozero", uni_matrix_.at<float>(2, 0));
        ros::param::get("/uni_matrix/twoone", uni_matrix_.at<float>(2, 1));
        ros::param::get("/uni_matrix/twotwo", uni_matrix_.at<float>(2, 2));
        ros::param::get("/uni_matrix/twothree", uni_matrix_.at<float>(2, 3));

        std::cout << "相机和雷达之间外参：" << uni_matrix_ << std::endl;                // 输出相机和雷达之间外参

        // 计算平移向量
        tran_vec_.at<float>(0, 0) =  uni_matrix_.at<float>(0, 3);
        tran_vec_.at<float>(1, 0) =  uni_matrix_.at<float>(1, 3);
        tran_vec_.at<float>(2, 0) =  uni_matrix_.at<float>(2, 3);

        std::cout << "相机和雷达之间平移向量：" << tran_vec_ << std::endl;           // 输出相机和雷达之间平移向量
    }

}
