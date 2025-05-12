#include <string>
#include <chrono>
#include <cmath>
#include "cuda_utils.h"
#include "logging.h"
#include "common.hpp"
#include "utils.h"
#include "calibrator.h"
#include "preprocess.h"

#define DEVICE 0                                                                                               // 设备编号
#define NMS_THRESH 0.4                                                                             // 非极大值抑制阈值
#define CONF_THRESH 0.5                                                                           // 置信度阈值
#define NUMBER_NMS_THRESH 0.4                                                        // 非极大值抑制阈值
#define NUMBER_CONF_THRESH 0.5                                                      // 置信度阈值
#define BATCH_SIZE_NUMBER 1                                                               // 号码牌批处理大小
#define BATCH_SIZE_CAR 1                                                                         // 车辆批处理大小
#define MAX_IMAGE_INPUT_SIZE_THRESH 3000 * 3000                 // 图像最大输入尺寸阈值

static const int INPUT_H_NUMBER = Yolo::INPUT_H_NUMBER;     // 号码牌输入高度
static const int INPUT_W_NUMBER = Yolo::INPUT_W_NUMBER;    // 号码牌输入宽度

static const int INPUT_H_CAR= Yolo::INPUT_H_CAR;                           // 车辆输入高度
static const int INPUT_W_CAR = Yolo::INPUT_W_CAR;                         // 车辆输入宽度

static const int CLASS_NUM_NUMBER = Yolo::CLASS_NUM_NUMBER;     // 号码牌类别数
static const int CLASS_NUM_CAR = Yolo::CLASS_NUM_CAR;                         // 车辆类别数

// 假设Yolo层输出的框数不超过MAX_OUTPUT_BBOX_COUNT且置信度 >= 0.1
static const int OUTPUT_SIZE = Yolo::MAX_OUTPUT_BBOX_COUNT * sizeof(Yolo::Detection) / sizeof(float) + 1;  

const char* INPUT_BLOB_NAME = "data";                                                   // 输入Blob名称
const char* OUTPUT_BLOB_NAME = "prob";                                              // 输出Blob名称

IRuntime* g_runtime;                                                                                            // TensorRT运行时对象

static Logger g_logger;                                                                                          // 日志记录器

int g_inputIndexCar;                                                                                                // 车辆模型输入索引
int g_outputIndexCar;                                                                                             // 车辆模型输出索引
int g_inputIndexNumber;                                                                                      // 号码牌模型输入索引
int g_outputIndexNumber;                                                                                   // 号码牌模型输出索引

uint8_t* g_imgHostCar = nullptr;                                                                         // 车辆模型主机图像数据指针
uint8_t* g_imgDeviceCar = nullptr;                                                                     // 车辆模型设备图像数据指针
uint8_t* g_imgHostNumber = nullptr;                                                               // 号码牌模型主机图像数据指针
uint8_t* g_imgDeviceNumber = nullptr;                                                           // 号码牌模型设备图像数据指针

cudaStream_t g_streamCar;                                                                                  // 车辆模型CUDA流
cudaStream_t g_streamNumber;                                                                        // 号码牌模型CUDA流

ICudaEngine* g_engineCar;                                                                                     // 车辆模型CUDA引擎
IExecutionContext* g_contextCar;                                                                        // 车辆模型执行上下文
ICudaEngine* g_engineNumber;                                                                           // 号码牌模型CUDA引擎
IExecutionContext* g_contextNumber;                                                              // 号码牌模型执行上下文

float* g_buffersCar[2];                                                                                                 // 车辆模型输入输出缓冲区
float* g_buffersNumber[2];                                                                                       // 号码牌模型输入输出缓冲区

static float g_probCar[BATCH_SIZE_CAR * OUTPUT_SIZE];                         // 车辆模型推理输出
static float g_probNumber[BATCH_SIZE_NUMBER * OUTPUT_SIZE];     // 号码牌模型推理输出

namespace inferspace
{
    /*CUDA初始化函数*/
    int cudaInit()
    {
        // 设置当前CUDA设备
        cudaSetDevice(DEVICE);

        /*车辆*/
        std::string p_engineNameCar = "/home/climber/RM2024Climber/src/tensorrt_pkg/include/weight/car_0509.engine"; // 车辆模型引擎文件路径

        if (p_engineNameCar.empty()) 
        {
            std::cerr << "p_engineNameCar arguments not right!" << std::endl;
            return -1;
        }

        // 从文件中读取车辆模型引擎
        std::ifstream p_fileCar(p_engineNameCar, std::ios::binary);
        if (!p_fileCar.good()) 
        {
            std::cerr << "read " << p_engineNameCar << " error!" << std::endl;
            return -1;
        }

        char* p_trtModelStream = nullptr;
        size_t p_sizeCar = 0;
        p_fileCar.seekg(0, p_fileCar.end);
        p_sizeCar = p_fileCar.tellg();
        p_fileCar.seekg(0, p_fileCar.beg);
        p_trtModelStream = new char[p_sizeCar];
        assert(p_trtModelStream);
        p_fileCar.read(p_trtModelStream, p_sizeCar);
        p_fileCar.close();

        // 反序列化车辆模型引擎
        g_runtime = createInferRuntime(g_logger);
        assert(g_runtime != nullptr);
        g_engineCar = g_runtime->deserializeCudaEngine(p_trtModelStream, p_sizeCar);
        assert(g_engineCar != nullptr);
        g_contextCar = g_engineCar->createExecutionContext();
        assert(g_contextCar != nullptr);
        delete[] p_trtModelStream;
        assert(g_engineCar->getNbBindings() == 2);
        
        g_inputIndexCar = g_engineCar->getBindingIndex(INPUT_BLOB_NAME);
        g_outputIndexCar = g_engineCar->getBindingIndex(OUTPUT_BLOB_NAME);
        assert(g_inputIndexCar == 0);
        assert(g_outputIndexCar == 1);
        
        // 分配车辆模型输入输出缓冲区和CUDA流
        CUDA_CHECK(cudaMalloc((void**)&g_buffersCar[g_inputIndexCar], BATCH_SIZE_CAR * 3 * INPUT_H_CAR * INPUT_W_CAR * sizeof(float)));
        CUDA_CHECK(cudaMalloc((void**)&g_buffersCar[g_outputIndexCar], BATCH_SIZE_CAR * OUTPUT_SIZE * sizeof(float)));
        CUDA_CHECK(cudaStreamCreate(&g_streamCar));
        
        CUDA_CHECK(cudaMallocHost((void**)&g_imgHostCar, MAX_IMAGE_INPUT_SIZE_THRESH * 3));
        CUDA_CHECK(cudaMalloc((void**)&g_imgDeviceCar, MAX_IMAGE_INPUT_SIZE_THRESH * 3));

        /*号码牌*/
        std::string p_engineNameNumber = "/home/climber/RM2024Climber/src/tensorrt_pkg/include/weight/number_0506.engine"; // 号码牌模型引擎文件路径
        
        if (p_engineNameNumber.empty()) 
        {
            std::cerr << "p_engineNameNumber arguments not right!" << std::endl;
            return -1;
        }

        // 从文件中读取号码牌模型引擎
        std::ifstream p_fileNumber(p_engineNameNumber, std::ios::binary);
        if (!p_fileNumber.good()) 
        {
            std::cerr << "read " << p_engineNameNumber << " error!" << std::endl;
            return -1;
        }

        size_t p_sizeNumber = 0;
        p_fileNumber.seekg(0, p_fileNumber.end);
        p_sizeNumber = p_fileNumber.tellg();
        p_fileNumber.seekg(0, p_fileNumber.beg);
        p_trtModelStream = new char[p_sizeNumber];
        assert(p_trtModelStream);
        p_fileNumber.read(p_trtModelStream, p_sizeNumber);
        p_fileNumber.close();

        // 反序列化号码牌模型引擎
        g_runtime = createInferRuntime(g_logger);
        assert(g_runtime != nullptr);
        g_engineNumber = g_runtime->deserializeCudaEngine(p_trtModelStream, p_sizeNumber);
        assert(g_engineNumber != nullptr);
        g_contextNumber = g_engineNumber->createExecutionContext();
        assert(g_contextNumber != nullptr);
        delete[] p_trtModelStream;
        assert(g_engineNumber->getNbBindings() == 2);

        g_inputIndexNumber = g_engineNumber->getBindingIndex(INPUT_BLOB_NAME);
        g_outputIndexNumber = g_engineNumber->getBindingIndex(OUTPUT_BLOB_NAME);
        assert(g_inputIndexNumber == 0);
        assert(g_outputIndexNumber == 1);

        // 分配号码牌模型输入输出缓冲区和CUDA流
        CUDA_CHECK(cudaMalloc((void**)&g_buffersNumber[g_inputIndexNumber], BATCH_SIZE_NUMBER * 3 * INPUT_H_NUMBER * INPUT_W_NUMBER * sizeof(float)));
        CUDA_CHECK(cudaMalloc((void**)&g_buffersNumber[g_outputIndexNumber], BATCH_SIZE_NUMBER * OUTPUT_SIZE * sizeof(float)));
        CUDA_CHECK(cudaStreamCreate(&g_streamNumber));

        CUDA_CHECK(cudaMallocHost((void**)&g_imgHostNumber, MAX_IMAGE_INPUT_SIZE_THRESH * 3));
        CUDA_CHECK(cudaMalloc((void**)&g_imgDeviceNumber, MAX_IMAGE_INPUT_SIZE_THRESH * 3));

        return 0;
    }

    /*销毁CUDA资源函数*/
    int cudaDestroy()
    {
        // 销毁车辆模型CUDA流和内存
        cudaStreamDestroy(g_streamCar);
        CUDA_CHECK(cudaFreeHost(g_imgHostCar));
        CUDA_CHECK(cudaFree(g_imgDeviceCar));
        CUDA_CHECK(cudaFree(g_buffersCar[g_inputIndexCar]));
        CUDA_CHECK(cudaFree(g_buffersCar[g_outputIndexCar]));

        // 销毁号码牌模型CUDA流和内存
        cudaStreamDestroy(g_streamNumber);
        CUDA_CHECK(cudaFreeHost(g_imgHostNumber));
        CUDA_CHECK(cudaFree(g_imgDeviceNumber));
        CUDA_CHECK(cudaFree(g_buffersNumber[g_inputIndexNumber]));
        CUDA_CHECK(cudaFree(g_buffersNumber[g_outputIndexNumber]));

        /*顺序不能换*/
        g_contextCar->destroy();
        g_engineCar->destroy();

        g_contextNumber->destroy();
        g_engineNumber->destroy();

        g_runtime->destroy();

        return 0;
    }

    /*执行推理函数*/
    void doInference(IExecutionContext& context, cudaStream_t& stream, void** buffers, float* output, int batchSize)
    {
        context.enqueue(batchSize, buffers, stream, nullptr);
        CUDA_CHECK(cudaMemcpyAsync(output, buffers[1], batchSize * OUTPUT_SIZE * sizeof(float), cudaMemcpyDeviceToHost, stream));
        cudaStreamSynchronize(stream);
    }

} // namespace inferspace

