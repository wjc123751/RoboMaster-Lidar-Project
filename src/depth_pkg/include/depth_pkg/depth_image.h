#include <iostream>
#include <mutex>
#include <vector>
#include <numeric>
#include <algorithm>
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
#include <geometry_msgs/Point.h>
#include "load_parameters.h"
#include "msgs_pkg/yolo_points.h"
#include "msgs_pkg/yolo_point.h"

namespace depth_image
{
    pcl::PointCloud<pcl::PointXYZ>::Ptr pointCloud;                            // 点云指针

    std::mutex bufferMutex;                                                                             // 互斥量，用于保护共享数据

    std::queue<cv::Mat> depthImageBuffer;                                           // 深度图像缓冲队列

    std::vector<cv::Point3f> pointVec;                                                       // 存储点云数据的三维坐标点

    std::queue<msgs_pkg::yolo_point> yoloPointsBuffer;              // 检测框信息缓冲队列

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

        // 定义一个半径
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
            cv::imshow("depthImageShow", depthImage);
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

    /*函数原型：对一组数据进行掐头去尾的滤波*/
    std::vector<float> trimFilter(const std::vector<float>& data, float headTrimPercent, float tailTrimPercent) 
    {
        // 计算要去除的头部和尾部数据数量
        int headTrim = static_cast<int>(data.size() * headTrimPercent);
        int tailTrim = static_cast<int>(data.size() * tailTrimPercent);

        // 检查数据长度是否足够
        if (data.size() <= (headTrim + tailTrim)) {
            cout << "Error: Data length is too short for trimming." << endl;
            return std::vector<float>();
        }

        // 创建结果数组
        std::vector<float> filteredData;

        // 遍历数据并进行滤波
        for (int i = headTrim; i < data.size() - tailTrim; ++i) {
            filteredData.push_back(data[i]);
        }

        return filteredData;
    }

    /*正态分布滤波，用于滤除异常值*/
    std::vector<float> normalFilterData(const std::vector<float>& data, float threshold) 
    {
        // 计算均值
        float mean = 0.0;
        for (float value : data) 
        {
            mean += value;
        }
        mean /= data.size();

        // 计算标准差
        float sum_squared_diff = 0.0;
        for (float value : data) 
        {
            sum_squared_diff += (value - mean) * (value - mean);
        }
        float std_dev = sqrt(sum_squared_diff / data.size());

        // 滤除极端值
        std::vector<float> filtered_data;
        for (float value : data) 
        {
            if (value >= mean - threshold * std_dev && value <= mean + threshold * std_dev) 
            {
                filtered_data.push_back(value);
            }
        }

        return filtered_data;           // 返回滤波后的数据
    }

    /*均值滤波函数*/
    std::vector<float> blurFilterData(const std::vector<float> &data, float threshold) 
    {
        // 计算整体数据的平均值
        float overallAverage = std::accumulate(data.begin(), data.end(), 0.0f) / data.size();

        // 创建存储过滤后数据的容器
        std::vector<float> filteredData(data.size());

        // 遍历数据并应用滤波逻辑
        for (size_t i = 0; i < data.size(); ++i) 
        {
            // std::cout << data[i] - overallAverage << std::endl;
            // 如果当前元素与整体平均值的差异大于阈值，则用整体平均值替换当前元素
            if (std::abs(data[i] - overallAverage) > threshold) 
            {
                filteredData[i] = overallAverage;
            } 
            else 
            {
                filteredData[i] = data[i];
            }
        }

        return filteredData;            // 返回滤波后的数据
    }

    /*获取号码牌深度值*/
    float getNumberDepth(int rectX, int rectY, int rectWidth, int rectHeight, cv::Mat depthImage)
    {
        std::vector<float> p_zValueVec;         // 存储号码牌区域内的深度值
        p_zValueVec.clear();
        int p_pixelCount = 0;

        // 遍历号码牌区域内的深度值
        for (int i = rectY; i <= (rectY + rectHeight); i++) 
        {
            for (int j = rectX; j <= (rectX + rectWidth); j++) 
            {
                float p_zValue = depthImage.at<float>(i, j);

                // 如果深度值大于 0，则将其存入 p_zValueVec 中
                if (p_zValue > 0)
                {
                    p_zValueVec.push_back(p_zValue);

                    p_pixelCount++;
                }
            }
        }

        // std::cout << "滤波前 :" << p_zValueVec.size() << std::endl;
        std::sort(p_zValueVec.begin(), p_zValueVec.end());
        if(p_zValueVec.size() > 0)
        {
            // std::cout << "滤波前：" << (p_zValueVec.back() - p_zValueVec.front()) * 100<< std::endl;
        }
        
        // 正态滤波数据
        float p_threshold = 0.7;
        std::vector<float> p_filteredZValue = normalFilterData(p_zValueVec, p_threshold);

        // std::cout << "正态滤波后 :" << p_filteredZValue.size() << std::endl;
        std::sort(p_filteredZValue.begin(), p_filteredZValue.end());
        if(p_filteredZValue.size() > 0)
        {
            // std::cout << "正态滤波后差值：" << (p_filteredZValue.back() - p_filteredZValue.front()) * 100 << std::endl;
        }

        // p_threshold = 0.02;
        // std::vector<float> p_blurFilteredZValue = blurFilterData(p_filteredZValue, p_threshold);

        // std::cout << "均值滤波后 :" << p_blurFilteredZValue.size() << std::endl;
        // std::sort(p_blurFilteredZValue.begin(), p_blurFilteredZValue.end());
        // if(p_blurFilteredZValue.size() > 0)
        // {
        //     // std::cout << "均值滤波后差值：" << (p_blurFilteredZValue.back() - p_blurFilteredZValue.front()) * 100 << std::endl;
        // }

        float p_zSum = 0.0;
        float p_distance = 0.0;
        if (p_pixelCount > 0) 
        {
            // 计算滤波前平均深度值
            for (int i = 0; i < p_zValueVec.size(); i++) 
            {
                p_zSum += p_zValueVec[i];
            }

            if(p_zValueVec.size() > 0)
            {
                p_distance = p_zSum / p_zValueVec.size();
                // std::cout << "p_zValueVec: " << p_distance * 100 << std::endl;
            }

            // 计算正态滤波后平均深度值
            p_zSum = 0.0;
            p_distance = 0.0;

            for (int i = 0; i < p_filteredZValue.size(); i++) 
            {
                p_zSum += p_filteredZValue[i];
            }

            if(p_filteredZValue.size() > 0)
            {
                p_distance = p_zSum / p_filteredZValue.size();
                if(p_distance > 13)
                {
                    for(int i = 0; i < p_filteredZValue.size();i++)
                    {
                        std::cout << "p_filteredZValue: " << p_filteredZValue[i] * 100 << std::endl;
                    }
                }
                //std::cout << "p_filteredZValue: " << p_distance * 100 << std::endl;
            }

            // 计算均值滤波后平均深度值
            // p_zSum = 0.0;
            // p_distance = 0.0;

            // for (int i = 0; i < p_blurFilteredZValue.size(); i++) 
            // {
            //     p_zSum += p_blurFilteredZValue[i];

            // }

            // if(p_blurFilteredZValue.size() > 0)
            // {
            //     p_distance = p_zSum / p_blurFilteredZValue.size();
            //     std::cout << "p_blurFilteredZValue: " << p_distance * 100 << std::endl;
            // }
        }

        return p_distance;
    }

    /*回调函数，用于接收号码牌检测框信息*/
    void numberMsgCallback(const msgs_pkg::yolo_points::ConstPtr& input)
    {
        msgs_pkg::yolo_points p_numberMsg;
        p_numberMsg = *input;

        // 如果检测框信息缓冲队列大小超过阈值，则移除最旧的元素
        if(yoloPointsBuffer.size() >= 20)
        {
            yoloPointsBuffer.pop();
        }

        yoloPointsBuffer.push(p_numberMsg.data.back());         // 将最新的检测框信息加入到缓冲队列中

        cv::Mat p_depthImage;

        std::lock_guard<std::mutex> lock(bufferMutex);              // 使用互斥量保护共享数据

        if(!depthImageBuffer.empty())
        {
            p_depthImage = depthImageBuffer.back();                    // 取出深度图像缓冲队列最新元素

            msgs_pkg::yolo_point p_rectMsg;
            p_rectMsg = yoloPointsBuffer.back();                                // 取出检测框信息缓冲队列最新元素
            
            // std::cout<<"p_rectMsg.color : "<<(int)p_rectMsg.color<<std::endl;
            // std::cout<<"p_rectMsg.id : "<<(int)p_rectMsg.id<<std::endl;
            // std::cout<<"p_rectMsg.x : "<<p_rectMsg.x<<std::endl;
            // std::cout<<"p_rectMsg.y : "<<p_rectMsg.y<<std::endl;
            // std::cout<<"p_rectMsg.width : "<<p_rectMsg.width<<std::endl;
            // std::cout<<"p_rectMsg.height : "<<p_rectMsg.height<<std::endl;

            // 获取检测框深度信息
            float p_numberDepth = getNumberDepth(p_rectMsg.x, p_rectMsg.y, p_rectMsg.width, p_rectMsg.height, p_depthImage);

        }
    }
    
}
