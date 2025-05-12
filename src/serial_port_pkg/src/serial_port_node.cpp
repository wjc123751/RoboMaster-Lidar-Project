#include "serial_port_node.h"


/*主函数*/
int main(int argc, char *argv[])
{
    //执行 ros 节点初始化
    ros::init(argc,argv,"serial_port_node");
    
    serial_port sp;

    ros::Rate loop(100);
    int count = 0;
    int max_size = 2000;
    while(ros::ok())
    {
        count++;
        if(count >= 10)
        {
            if(!g_worldPoints.empty())
            {
                if(g_worldPoints.size() > max_size)
                {
                    g_worldPoints.erase(g_worldPoints.begin() + max_size, g_worldPoints.end());
                }

                msgs_pkg::world_points p_carWorldPoint = *g_worldPoints.begin();
                std::cout<<p_carWorldPoint.x<<", "<<p_carWorldPoint.y<<std::endl;
                sp.sendMessage(p_carWorldPoint.id, p_carWorldPoint.x, p_carWorldPoint.y);
                g_worldPoints.erase(g_worldPoints.begin());
            }
            else
            {
                ros::spinOnce();
            }

            count = 0;
        }

        sp.receiveMessage();
        ros::spinOnce();
        loop.sleep();
    }

    return 0;
}
