#include <iostream>
#include <mutex>
#include <vector>
#include <numeric>
#include <algorithm>
#include <cmath>
#include <ros/ros.h>
#include <opencv2/opencv.hpp>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/PointCloud2.h>
#include <geometry_msgs/Point.h>
#include "msgs_pkg/yolo_points.h"
#include "msgs_pkg/yolo_point.h"
#include "msgs_pkg/world_points.h"

namespace world_space
{
    ros::Publisher mapMsgPub;

    float prevNumberDepth = 0.0f;                               // 存储上一次的深度值
    geometry_msgs::Point worldPoint_;                    //世界坐标点
    msgs_pkg::world_points mapMsg;

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
                std::cout << "p_distance: " << p_distance * 100 << std::endl;
            }
        }

        return p_distance;
    }

    /*获取世界坐标*/
    geometry_msgs::Point getWorldPosition(float distance,int input_u,int input_v)
    {
        cv::Mat invR;                                                                                       // 旋转矩阵逆矩阵
        cv::Mat invM;                                                                                      // 相机内参矩阵逆矩阵
        cv::Mat rotM=cv::Mat::eye(3,3,CV_64FC1);                            // 旋转矩阵
        cv::Mat tanT = cv::Mat::zeros(3,1,CV_64FC1);                      // 平移矩阵
        
        cv::Mat rvec =cv::Mat_<double>(3,1);  // 旋转向量
        cv::Mat tvec=cv::Mat_<double>(3,1);    // 平移向量

        // 相机与世界的外参
        ros::param::get("/rvec0", rvec.at<double>(0, 0));
        ros::param::get("/rvec1", rvec.at<double>(1, 0));
        ros::param::get("/rvec2", rvec.at<double>(2, 0));

        ros::param::get("/tvec0", tvec.at<double>(0, 0));
        ros::param::get("/tvec1", tvec.at<double>(1, 0));
        ros::param::get("/tvec2", tvec.at<double>(2, 0));

        // std::cout << "rvec:" << rvec<< std::endl;
        // std::cout << "tvec:" << tvec<< std::endl;

        tanT = tvec;                         // 平移向量转换为平移矩阵
        Rodrigues(rvec, rotM);   // 旋转向量转换为旋转矩阵

        cv::Mat camera_matrix_ = cv::Mat_<float>(3, 3);
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

        //矩阵求逆
        invert(rotM,invR);
        invert(camera_matrix_,invM);

        invM.convertTo(invM,CV_64FC1);
        invR.convertTo(invR,CV_64FC1);

        if(distance > 0)
        {
            cv::Mat uvMatrix = (cv::Mat_<double>(3,1) << (double) input_u , (double) input_v ,  1 );
            uvMatrix *=(1000 * (double)distance) ;

            cv::Mat calcWorld = invR * (invM *uvMatrix - tanT);          // 获取世界坐标

            calcWorld /= 1000;

            worldPoint_.x = calcWorld.at<double>(0,0);
            worldPoint_.y = calcWorld.at<double>(1,0);
            worldPoint_.z = calcWorld.at<double>(2,0);

            std::cout<<"worldPoint_ : "<<worldPoint_<<std::endl;

            return worldPoint_;
        }
    }
}