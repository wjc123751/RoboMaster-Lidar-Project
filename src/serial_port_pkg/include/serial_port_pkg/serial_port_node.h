#include "ros/ros.h"
#include "msgs_pkg/world_points.h"
#include <serial/serial.h>
#include "judge_analysis_app.h"
#include <vector>
#include <sys/stat.h>
#include <stdlib.h>

std::vector<msgs_pkg::world_points> g_worldPoints;

/*接受裁判系统数据*/
uint8_t judge_re[512];

/*标记进度*/
uint8_t mark_hero_progress;
uint8_t mark_engineer_progress;
uint8_t mark_standard_3_progress;
uint8_t mark_standard_4_progress;
uint8_t mark_standard_5_progress;
uint8_t mark_sentry_progress;

class serial_port
{
private:
    ros::NodeHandle m_nh;
    serial::Serial ser;
    bool is_enemy_red;                                       //敌方颜色为红色
    points_map_t mapMsg;
    ros::Subscriber worldPointsSub;

public:
    
    serial_port(/* args */);
    ~serial_port();

    void serialPortInit();
    void worldPointsCallback(const msgs_pkg::world_points &msg);
    bool sendMessage(uint16_t id, float x, float y);
    bool receiveMessage();
};

serial_port::serial_port(/* args */)
{
    serialPortInit();

    if(!ser.isOpen())
    {
        std::cout<<"USB打不开!"<<std::endl;
    }
    else
    {
        std::cout<<"串口打开!"<<std::endl;
    }

    //读取我方阵容参数
    std::string exchange;
    ros::param::get("/team_color",exchange);
    if(exchange == "red")
    {
        is_enemy_red = false;
    }
    else
    {
        is_enemy_red = true;
    }
    
    worldPointsSub = m_nh.subscribe("/map_msg",1 ,&serial_port::worldPointsCallback, this);
}

serial_port::~serial_port()
{
}

void serial_port::serialPortInit()
{
    std::string port;
    ros::param::get("/serial_port_CH343",port);

    mode_t mode = S_IRUSR | S_IWUSR | S_IRGRP | S_IWGRP | S_IROTH | S_IWOTH;
    const char* serialPort = port.data();
    char command[100];
    snprintf(command, sizeof(command), "sudo chmod %o %s", mode, serialPort);
    int result = system(command);
    std::cout<<"给串口权限!"<<std::endl;

    ser.setPort(port);
    ser.setBaudrate(115200);
    serial::Timeout to = serial::Timeout::simpleTimeout(1000);
    ser.setTimeout(to);
    ser.open();
}

void serial_port::worldPointsCallback(const msgs_pkg::world_points &msg)
{
    // std::cout<<"g_worldPoints = "<<g_worldPoints.size()<<std::endl;
    if(is_enemy_red)
    {
        if(msg.id >= 6 && msg.id <= 10)
        {
            msgs_pkg::world_points p_carPoint;
            p_carPoint.id = msg.id - 5;
            p_carPoint.x = msg.x;
            p_carPoint.y = msg.y;
            g_worldPoints.insert(g_worldPoints.begin(), p_carPoint);
        }
        else if(msg.id == 11)
        {
            msgs_pkg::world_points p_carPoint;
            p_carPoint.id = 7;
            p_carPoint.x = msg.x;
            p_carPoint.y = msg.y;
            g_worldPoints.insert(g_worldPoints.begin(), p_carPoint);
        }
    }
    else
    {
        if(msg.id <= 4)
        {
            msgs_pkg::world_points p_carPoint;
            p_carPoint.id = msg.id + 101;
            p_carPoint.x = msg.x;
            p_carPoint.y = msg.y;
            g_worldPoints.insert(g_worldPoints.begin(), p_carPoint);
        }
        else if(msg.id == 5)
        {
            msgs_pkg::world_points p_carPoint;
            p_carPoint.id = 107;
            p_carPoint.x = msg.x;
            p_carPoint.y = msg.y;
            g_worldPoints.insert(g_worldPoints.begin(), p_carPoint);
        }

    }
}

bool serial_port::sendMessage(uint16_t id, float x, float y)
{
    mapMsg.Header.SOF = 0xA5;
    mapMsg.Header.DataLenth = 10;
    mapMsg.Header.Seq = 1;
    mapMsg.Header.CRC8 = Get_CRC8_Check_Sum((uint8_t *) &mapMsg, (sizeof(mapMsg.Header) - sizeof(mapMsg.Header.CRC8)), 0xff);

    mapMsg.cmd_id = 0x0305;
    mapMsg.mapdata.target_robot_id = id;
    mapMsg.mapdata.target_position_x = x;
    mapMsg.mapdata.target_position_y = y;
    mapMsg.crc = Get_CRC16_Check_Sum((uint8_t *) &mapMsg, (sizeof(mapMsg) - sizeof(mapMsg.crc)), 0xffff);

    ser.write((uint8_t *) &mapMsg, sizeof(points_map_t));

    // std::cout<<"target_robot_id : "<<mapMsg.mapdata.target_robot_id<<std::endl;
    // std::cout<<"target_position_x : "<<mapMsg.mapdata.target_position_x<<std::endl;
    // std::cout<<"target_position_y : "<<mapMsg.mapdata.target_position_y<<std::endl;

    return true;
}

bool serial_port::receiveMessage()
{
    if(ser.available())
    {
        ser.read(judge_re, ser.available());
        judgeCalculate(judge_re);

        uint8_t progess = JudgementData.radar_mark_data_t.mark_standard_4_progress;
        // std::cout<<"progess = "<<(int)progess<<std::endl;

        memset(judge_re, 0, 400);
    }

    return true;
}

