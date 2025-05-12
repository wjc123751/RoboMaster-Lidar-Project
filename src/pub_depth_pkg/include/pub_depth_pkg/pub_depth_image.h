#include <iostream>
#include <mutex>
#include <vector>
#include <numeric>
#include <cmath>
#include <ros/ros.h>
#include <pcl/point_cloud.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_types.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/segmentation/extract_clusters.h>
#include <opencv2/opencv.hpp>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/PointCloud2.h>
#include<geometry_msgs/Point.h>
#include "pub_load_parameters.h"

namespace pub_depth_image
{
    pcl::PointCloud<pcl::PointXYZ>::Ptr pointCloud;                            // 点云指针

    std::mutex bufferMutex;                                                                             // 互斥量，用于保护共享数据

    std::queue<cv::Mat> depthImageBuffer;                                           // 深度图像缓冲队列

    std::vector<cv::Point3f> pointVec;                                                       // 存储点云数据的三维坐标点

    /*将 PCL 点云数据转换为 OpenCV 的 Mat 矩阵数据*/
    cv::Mat Cloud2Mat(const pcl::PointCloud<pcl::PointXYZ>::Ptr &input)
    {
        pointVec.clear();       // 清空点云数据存储容器
        for (int k = 0; k < (int) input->size(); k++) 
        {
            cv::Point3f p_point(input->points[k].x, input->points[k].y, input->points[k].z);
            
            pointVec.push_back(p_point);        // 将点云数据转换为 OpenCV 三维坐标点并存储
        }

        cv::Mat res(pointVec);          // 创建 Mat 矩阵并初始化为 pointVec 中的数据
        
        return res;         // 返回转换后的 Mat 矩阵
    }

    /*将点云数据投影到相机平面*/
    cv::Mat projectToImage(const cv::Mat& pointCloud, const cv::Mat& uni_matrix, const cv::Mat& tran_vec, const cv::Mat& cameraMatrix, const cv::Mat& distortion_coefficient) 
    {
        // 分解相机和雷达的齐次变换矩阵
        cv::Mat camera_matrix, rot_mat, trans_vec;
        cv::decomposeProjectionMatrix(uni_matrix, camera_matrix, rot_mat, trans_vec);

        // 计算旋转矩阵的转置
        cv::Mat T_rot_mat;
        T_rot_mat = rot_mat.t();

        // 将旋转矩阵转换为旋转向量
        cv::Mat rot_vec;
        cv::Rodrigues(T_rot_mat, rot_vec);

        // 使用相机内参矩阵进行点云到像素坐标的投影
        cv::Mat projectedPoints;
        cv::projectPoints(pointCloud, rot_vec, tran_vec, cameraMatrix, distortion_coefficient, projectedPoints);

        // 将投影得到的像素坐标值转为整数
        for (int i = 0; i < projectedPoints.rows; ++i) 
        {
            projectedPoints.at<cv::Point2f>(i).x = static_cast<int>(std::round(projectedPoints.at<cv::Point2f>(i).x));
            projectedPoints.at<cv::Point2f>(i).y = static_cast<int>(std::round(projectedPoints.at<cv::Point2f>(i).y));
        }

        return projectedPoints;         // 返回投影得到的像素坐标
    }

    /*生成深度图像*/
    cv::Mat generateDepthImage(const cv::Mat& projectedPoints, const cv::Size& imageSize, const cv::Mat& pointCloud) 
    {
        // 创建一个与图像大小相同的深度图像
        cv::Mat depthImage = cv::Mat::zeros(parameters::imgrows_, parameters::imgcols_, CV_32FC1);

        // 定义一个点云半径
        const int radius = 2;
        
        // 将投影得到的点云深度信息填充到深度图中
        for (int i = 0; i < projectedPoints.rows; ++i) 
        {
             if(projectedPoints.at<float>(i,0) > 0 && projectedPoints.at<float>(i,0) < imageSize.width)
            {
                if(projectedPoints.at<float>(i,1) > 0 && projectedPoints.at<float>(i,1) < imageSize.height)
                {
                    int x = projectedPoints.at<float>(i,0);
                    int y = projectedPoints.at<float>(i,1);

                    // 将点云深度值填充到指定区域内
                    for (int dy = -radius; dy <= radius; ++dy) 
                    {
                        for (int dx = -radius; dx <= radius; ++dx) 
                        {
                            int nx = x + dx;
                            int ny = y + dy;

                            // 确保在图像范围内
                            if (nx >= 0 && nx < depthImage.cols && ny >= 0 && ny < depthImage.rows) 
                            {
                                depthImage.at<float>(ny, nx) = pointCloud.at<float>(i, 0);
                            }
                        }
                    }
                }
            }
        }
        
        return depthImage;          // 返回生成的深度图像
    }

    /*获取深度图像并处理*/
    void getDepthImage()
    {
        // 如果接收到了点云数据
        if (pointCloud)
        {
            cv::Mat cloud2Mat = Cloud2Mat(pointCloud);    // 将 PCL 点云数据转换为 Mat 格式

            // 投影点云到相机平面
            cv::Mat projectedPoints = projectToImage(cloud2Mat, parameters::uni_matrix_, parameters::tran_vec_
                                                                                                    , parameters::camera_matrix_, parameters::distortion_coefficient_);

            // 生成深度图像
            cv::Mat depthImage = cv::Mat::zeros(parameters::imgrows_, parameters::imgcols_, CV_32FC1);
            depthImage = generateDepthImage(projectedPoints, parameters::imageSize, cloud2Mat);

            // 使用互斥量保护共享数据
            std::lock_guard<std::mutex> lock(bufferMutex);

            // 如果深度图像缓冲队列大小超过阈值，则移除最旧的元素
            if(depthImageBuffer.size() >= 20)
            {
                depthImageBuffer.pop();
            }

            // 将处理后的深度图像加入到缓冲队列中
            depthImageBuffer.push(depthImage.clone());
            
            // 显示深度图像
            cv::imshow("depthImage", depthImage);
            cv::waitKey(1);
        }
    }

    /*回调函数，用于接收点云数据*/
    void pointCloudCallback(const sensor_msgs::PointCloud2ConstPtr& input)
    {
        pointCloud.reset(new pcl::PointCloud<pcl::PointXYZ>);
        pcl::fromROSMsg(*input, *pointCloud);

        // 处理接收到的点云数据，生成深度图像
        getDepthImage();
    }


}
