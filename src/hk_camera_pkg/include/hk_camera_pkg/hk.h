#include <stdio.h>
#include <string.h>
#include <unistd.h>
#include <stdlib.h>
#include <memory>
#include "MvCameraControl.h"

/* 海康相机类 */
class HkCam
{
public:
    MV_CC_PIXEL_CONVERT_PARAM m_stConvertParam;         // 像素转换参数
    
    HkCam();                                                                                                    // 构造函数
    ~HkCam();                                                                                                 // 析构函数

    void start();                                                                                               // 启动相机
    void shutDown();                                                                                   // 关闭相机
    bool grabRGBImage();                                                                         // 获取RGB图像

private:
    MV_CC_DEVICE_INFO_LIST m_stDeviceList;                              // 设备列表
    void* m_pHandle;                                                                                  // 相机句柄
    unsigned char* m_pData;                                                                   // 数据缓冲区
    unsigned char* m_pDataForRGB;                                                   // RGB数据缓冲区
    int m_nRet;                                                                                               // 错误码
    MVCC_INTVALUE m_stParam;                                                           // 参数值
    
    bool printDeviceInfo(MV_CC_DEVICE_INFO* pstMVDevInfo);          // 打印设备信息
};
