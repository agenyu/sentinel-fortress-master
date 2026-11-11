#include <ad_rscl/ad_rscl.h>
#include <ad_msg_idl/ad_localization/localization.capnp.h>

#include "rscl_access/rscl_access.h"


#include <cuda_runtime.h>
#include <cuda.h>
#include <cuda_runtime_api.h>
#include <device_launch_parameters.h>
#include <chrono>
#include <iostream>
#include <mutex>
#include <random>
#include <thread>
#include <vector>

#include <ad_rscl/ad_rscl.h>
#include <ad_rscl/comm/cuda/cuda_publisher.h>
#include <ad_rscl/comm/cuda/cuda_subscriber.h>
#include <ad_rscl/comm/cuda/helper_cuda.h>

#include "ad_common/data_type/base.hpp"


#include <opencv2/opencv.hpp>
#include "rscl_access.h"


//camera_name  size
//std::unordered_map<std::string, std::tuple<int, int>> camera_input_resolutation{
//  { "center_camera_fov30", {3840, 2160}},
//  { "center_camera_fov120", {3840, 2160}},
//  { "rear_camera", {1920, 1280}},
//  { "left_front_camera", {1920, 1280}},
//  { "left_rear_camera", {1920, 1280}},
//  { "right_front_camera", {1920, 1280}},
//  { "right_rear_camera", {1920, 1280}},
//  { "front_camera_fov195", {1920, 1536}},
//  { "left_camera_fov195", {1920, 1536}},
//  { "rear_camera_fov195", {1920, 1536}},
//  { "right_camera_fov195", {1920, 1536}},
//};

//to do: 发出0和1 两个相机 按照20Hz还是10Hz?
std::vector<std::pair<std::string, std::pair<int, int>>> camera_input_resolution{

    { "center_camera_fov30", {3840, 2160}},
    { "center_camera_fov120", {3840, 2160}},
    { "rear_camera", {1920, 1280}},
    { "left_front_camera", {1920, 1280}},
    { "left_rear_camera", {1920, 1280}},
    { "right_front_camera", {1920, 1280}},
    { "right_rear_camera", {1920, 1280}},
    { "front_camera_fov195", {1920, 1536}},
    { "left_camera_fov195", {1920, 1536}},
    { "rear_camera_fov195", {1920, 1536}},
    { "right_camera_fov195", {1920, 1536}},
};



int main(int argc, char* argv[])
{


//    const std::string topic = "center_camera_fov120";
//    const int dice_roll = 1024;

    cudaSetDevice(0);
    int gpucount;
    cudaGetDeviceCount(&gpucount);
    std::cout << "gpucount: " << gpucount << std::endl;

    auto rt = senseAD::rscl::GetCurRuntime();
    rt->Init(argc, argv);

    std::this_thread::sleep_for(std::chrono::milliseconds(1000));

    // Construct publisher
    auto pub_node = rt->CreateNode("camera");


    // 设置CUDA发布器配置

    int camera_idx = 1;
    std::cout << "=== 访问索引0的分辨率 ===" << std::endl;
    std::string camera_name_ = camera_input_resolution[camera_idx].first; // 相机名称
    int width_ = camera_input_resolution[0].second.first;        // 分辨率宽（pair的first）
    int height_ = camera_input_resolution[0].second.second;      // 分辨率高（pair的second）
    std::cout << "相机：" << camera_name_ << std::endl;
    std::cout << "分辨率：" << width_ << " x " << height_ << std::endl;


    //    camera_->SetCameraNewHeight(camera_idx, height_);
    //    camera_->SetCameraNewWidth(camera_idx, width_);
    // 计算图像数据大小（根据格式计算）
    auto data_size = width_ *
            height_ * 3 +
            sizeof(senseAD::ImageHeader_t);
    senseAD::rscl::comm::CudaPublisherConf cuda_pconf;
//    std::shared_ptr<senseAD::rscl::comm::CudaPublisher> cuda_ipc_image_pub_;

    cuda_pconf.set_buffer_size(data_size);
    cuda_pconf.set_depth(9);
    cuda_pconf.set_device_id(0);
    cuda_pconf.set_channel_name(camera_name_);
    std::cout << "camera " << camera_idx << " name: " << camera_name_
              << " width: " << width_
              << " height:" << height_
              << " set_buffer_size: " << data_size << " set_depth: " << 9;
    // 创建CUDA发布器实例
    auto cuda_publisher_ = std::make_shared<senseAD::rscl::comm::CudaPublisher>(pub_node.get(), cuda_pconf);
    //    cuda_ipc_image_pubs_[camera_idx] = publisher;





    while (true) {





        // 计算缓冲区大小
        uint32_t img_size = width_ * height_ * 3;
        auto size = img_size + sizeof(senseAD::ImageHeader_t);
        // 创建CUDA内存块
        auto pre_mem_block = cuda_publisher_->NewMemblockInplace(size);
        if (pre_mem_block == nullptr)
        {
            std::cerr << "Failed to create CUDA memory block" << std::endl;
        }

        auto mem_block = pre_mem_block->GetMemBlock();
        if (mem_block == nullptr)
        {
            std::cerr << "Failed to get CUDA memory block" << std::endl;
            //        return false;
        }


        // 创建并设置图像头
        // 获取当前时间戳
        auto now = std::chrono::high_resolution_clock::now();
        uint64_t camera_timestamp_ns = std::chrono::duration_cast<std::chrono::nanoseconds>(now.time_since_epoch()).count();


        senseAD::ImageHeader_t tmp;
        tmp.format = senseAD::ImageFormat_t::BGR;
        tmp.timestamp = camera_timestamp_ns;
        tmp.width = width_;
        tmp.height = height_;


        // 将图像头复制到CUDA内存
        // pub timestamp and image
        cudaMemcpy(mem_block->ptr, &tmp, sizeof(senseAD::ImageHeader_t), cudaMemcpyHostToDevice);




        // 将图像数据复制到设备内存（跳过图像头部分）
        // 1. 加载或创建cv::Mat图像（BGR格式，与发布器配置一致）
        cv::Mat image;
        std::string image_path = "./image/demo.jpg"; // 替换为实际图像路径

        // 加载图像
        image = cv::imread(image_path);
        if (image.empty()) {
            std::cerr << "Failed to load image: " << image_path << std::endl;
        }
        //    cv::imshow("src",image);
        //    cv::waitKey(0);

        // 4. 获取cv::Mat的原始数据指针（BGR格式，3通道）
        uint8_t* cpu_image_data = image.data;
        size_t cpu_image_size = image.total() * image.channels();

        // 5. 图像数据指针（跳过图像头，指向CUDA内存中的图像数据部分）
        uint8_t *img_data_ptr = reinterpret_cast<uint8_t*>(mem_block->ptr) + sizeof(senseAD::ImageHeader_t);

        // 6. 将cv::Mat图像数据从CPU内存复制到CUDA设备内存
        cudaError_t cuda_ret = cudaMemcpy(img_data_ptr, cpu_image_data, cpu_image_size, cudaMemcpyHostToDevice);
        if (cuda_ret != cudaSuccess) {
            std::cerr << "Failed to copy image data from CPU to CUDA memory: "
                      << cudaGetErrorString(cuda_ret) << std::endl;
        }


        // 发布数据 几Hz?s
        // 发布CUDA内存块
        cuda_publisher_->Publish(mem_block);

        std::cout << "Published CUDA image: timestamp=" << camera_timestamp_ns
                  << ", width=" << width_ << ", height=" << height_ << std::endl;
        std::this_thread::sleep_for(std::chrono::milliseconds(500));
        //    }


    }



    return 0;
}

