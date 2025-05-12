#include "hk.h"

/*构造函数，初始化成员变量*/
HkCam::HkCam() : m_pHandle(NULL), m_nRet(MV_OK), m_pData(NULL), m_pDataForRGB(NULL), m_stConvertParam{ 0 }
{

}

/*析构函数，关闭相机*/
HkCam::~HkCam()
{
    HkCam::shutDown();
}

/*关闭相机*/
void HkCam::shutDown()
{
    if (m_pHandle != NULL)
    {
        // 销毁相机句柄
        MV_CC_DestroyHandle(m_pHandle);
        m_pHandle = NULL;
    }

    printf("exit\n");
}

/*启动相机*/
void HkCam::start()
{
    // 初始化设备列表
    memset(&m_stDeviceList, 0, sizeof(MV_CC_DEVICE_INFO_LIST));
    // 枚举设备
    m_nRet = MV_CC_EnumDevices(MV_GIGE_DEVICE | MV_USB_DEVICE, &m_stDeviceList);
    if (MV_OK != m_nRet)
    {
        printf("MV_CC_EnumDevices fail! nRet [%x]\n", m_nRet);
        HkCam::shutDown();
    }
    // 如果找到设备
    if (m_stDeviceList.nDeviceNum > 0)
    {
        for (int i = 0; i < m_stDeviceList.nDeviceNum; i++)
        {
            printf("[device %d]:\n", i);
            MV_CC_DEVICE_INFO* p_pDeviceInfo = m_stDeviceList.pDeviceInfo[i];
            if (NULL == p_pDeviceInfo)
            {
                HkCam::shutDown();
            }
            // 打印设备信息
            HkCam::printDeviceInfo(p_pDeviceInfo);
        }
    }
    else
    {
        printf("Find No Devices!\n");
        HkCam::shutDown();
    }

    size_t p_unIndex = 0;
    // 创建相机句柄
    m_nRet = MV_CC_CreateHandle(&m_pHandle, m_stDeviceList.pDeviceInfo[p_unIndex]);
    if (MV_OK != m_nRet)
    {
        printf("MV_CC_CreateHandle fail! nRet [%x]\n", m_nRet);
        HkCam::shutDown();
    }
    // 打开相机
    m_nRet = MV_CC_OpenDevice(m_pHandle);
    if (MV_OK != m_nRet)
    {
        printf("MV_CC_OpenDevice fail! nRet [%x]\n", m_nRet);
        HkCam::shutDown();
    }
    // 配置网络相机数据包大小
    if (m_stDeviceList.pDeviceInfo[p_unIndex]->nTLayerType == MV_GIGE_DEVICE)
    {
        int p_nPacketSize = MV_CC_GetOptimalPacketSize(m_pHandle);
        if (p_nPacketSize > 0)
        {
            m_nRet = MV_CC_SetIntValue(m_pHandle, "GevSCPSPacketSize", p_nPacketSize);
            if (m_nRet != MV_OK)
            {
                printf("Warning: Set Packet Size fail nRet [0x%x]!\n", m_nRet);
            }
        }
        else
        {
            printf("Warning: Get Packet Size fail nRet [0x%x]!\n", p_nPacketSize);
        }
    }
    // 设置触发模式
    m_nRet = MV_CC_SetEnumValue(m_pHandle, "TriggerMode", 0);
    if (MV_OK != m_nRet)
    {
        printf("MV_CC_SetTriggerMode fail! nRet [%x]\n", m_nRet);
        HkCam::shutDown();
    }
    // 获取负载大小
    memset(&m_stParam, 0, sizeof(MVCC_INTVALUE));
    m_nRet = MV_CC_GetIntValue(m_pHandle, "PayloadSize", &m_stParam);
    if (MV_OK != m_nRet)
    {
        printf("Get PayloadSize fail! nRet [0x%x]\n", m_nRet);
        HkCam::shutDown();
    }
    // 开始抓取图像
    m_nRet = MV_CC_StartGrabbing(m_pHandle);
    if (MV_OK != m_nRet)
    {
        printf("MV_CC_StartGrabbing fail! nRet [%x]\n", m_nRet);
        HkCam::shutDown();
    }
}

/*获取RGB图像*/
bool HkCam::grabRGBImage()
{
    MV_FRAME_OUT_INFO_EX p_stImageInfo = { 0 };
    memset(&p_stImageInfo, 0, sizeof(MV_FRAME_OUT_INFO_EX));

    unsigned int p_unDataSize = m_stParam.nCurValue;
    unsigned char data[p_unDataSize];           // 使用栈上的缓冲区存储图像数据
    m_pData = data;

    // 获取一帧图像数据
    m_nRet = MV_CC_GetOneFrameTimeout(m_pHandle, m_pData, p_unDataSize, &p_stImageInfo, 1000);
    if (MV_OK != m_nRet)
    {
        printf("MV_CC_GetOneFrameTimeout fail! nRet [%x]\n", m_nRet);
        HkCam::shutDown();
    }

    printf("Frame, Width[%d], Height[%d], nFrameNum[%d]\n\n",
        p_stImageInfo.nWidth, p_stImageInfo.nHeight, p_stImageInfo.nFrameNum);

    unsigned char p_ucDataForRGB[p_stImageInfo.nWidth * p_stImageInfo.nHeight * 4 + 2048];      // 使用栈上的缓冲区存储RGB数据
    m_pDataForRGB = p_ucDataForRGB;

    // 设置像素转换参数
    m_stConvertParam.nWidth = p_stImageInfo.nWidth;
    m_stConvertParam.nHeight = p_stImageInfo.nHeight;
    m_stConvertParam.pSrcData = m_pData;
    m_stConvertParam.nSrcDataLen = p_stImageInfo.nFrameLen;
    m_stConvertParam.enSrcPixelType = p_stImageInfo.enPixelType;
    m_stConvertParam.enDstPixelType = PixelType_Gvsp_RGB8_Packed;
    m_stConvertParam.pDstBuffer = m_pDataForRGB;
    m_stConvertParam.nDstBufferSize = p_stImageInfo.nWidth * p_stImageInfo.nHeight * 4 + 2048;
    // 转换像素类型
    m_nRet = MV_CC_ConvertPixelType(m_pHandle, &m_stConvertParam);
    if (MV_OK != m_nRet)
    {
        printf("MV_CC_ConvertPixelType fail! nRet [%x]\n", m_nRet);
        HkCam::shutDown();
        return false;
    }

    return true;
}

/*打印设备信息*/
bool HkCam::printDeviceInfo(MV_CC_DEVICE_INFO* pstMVDevInfo)
{
    if (NULL == pstMVDevInfo)
    {
        printf("The Pointer of pstMVDevInfo is NULL!\n");
        return false;
    }
    if (pstMVDevInfo->nTLayerType == MV_GIGE_DEVICE)
    {
        // 获取IP地址的各个部分
        int nIp1 = ((pstMVDevInfo->SpecialInfo.stGigEInfo.nCurrentIp & 0xff000000) >> 24);
        int nIp2 = ((pstMVDevInfo->SpecialInfo.stGigEInfo.nCurrentIp & 0x00ff0000) >> 16);
        int nIp3 = ((pstMVDevInfo->SpecialInfo.stGigEInfo.nCurrentIp & 0x0000ff00) >> 8);
        int nIp4 = (pstMVDevInfo->SpecialInfo.stGigEInfo.nCurrentIp & 0x000000ff);
        // 打印设备信息
        printf("Device Model Name: %s\n", pstMVDevInfo->SpecialInfo.stGigEInfo.chModelName);
        printf("CurrentIp: %d.%d.%d.%d\n", nIp1, nIp2, nIp3, nIp4);
        printf("UserDefinedName: %s\n\n", pstMVDevInfo->SpecialInfo.stGigEInfo.chUserDefinedName);
    }
    else if (pstMVDevInfo->nTLayerType == MV_USB_DEVICE)
    {
        // 打印设备信息
        printf("Device Model Name: %s\n", pstMVDevInfo->SpecialInfo.stUsb3VInfo.chModelName);
        printf("UserDefinedName: %s\n\n", pstMVDevInfo->SpecialInfo.stUsb3VInfo.chUserDefinedName);
    }
    else
    {
        printf("Not support.\n");
    }
    return true;
}
