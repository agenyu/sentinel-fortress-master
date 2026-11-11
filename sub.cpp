#include <ad_rscl/ad_rscl.h>
//#include <ad_msg_idl/ad_localization/localization.capnp.h>
//#include <ad_msg_idl/ad_gac/camera.capnp.h>
#include "rscl_access/rscl_access.h"
#include "ad_msg_idl/ad_sensor/sensor.capnp.h"

#include <opencv2/opencv.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/highgui.hpp>


#include "ad_rscl/comm/buf_subscriber.h"
#include <ad_log/ad_log.hpp>


////new
#include "ad_msg_idl/std_msgs/raw_data.capnp.h"
using CameraEncodedMsg = senseAD::msg::std_msgs::CameraEncodedStruct;

//
#include "ad_msg_idl/ad_gac/camera.capnp.h"
using CameraEncodedMsg_wkw = gac::rscl::CameraDecodedStruct;


#define YUV_SCALE_W 2
#define YUV_SCALE_H 3



#include <opencv2/opencv.hpp>
#include <opencv2/videoio.hpp>
#include <iostream>
#include <fstream>
#include <vector>
#include <memory>
#include <chrono>
#include <thread>
#include <queue>
#include <mutex>
#include <condition_variable>
#include <atomic>









#include "ad_common/data_type/base.hpp"
#include "ad_rscl/ad_rscl.h"
#include "ad_buf/ad_buf.h"
//#include "ad_buf/impl_nvscibuf/ad_buf_nvscibuf.h"

#include "ad_rscl/comm/cuda/cuda_subscriber.h"

#include "ad_rscl/comm/buf_subscriber.h"


//#include <opencv2/opencv.hpp>


#include <ad_log/ad_log.hpp>
#include <ad_pal/data_type/video_frame.hpp>


#include <cuda_runtime.h>
// senseAD::rscl::comm::Memblock aa;
using Memblock = std::shared_ptr<senseAD::rscl::comm::Memblock>;
std::shared_ptr<senseAD::rscl::comm::CudaSubscriber> subscriber_;
uint64_t frame_count_{0};
static char cap_name[] = "VideoFrame";



// 可视化和保存配置
bool enable_save_images_ = true;      // 是否保存图像
bool enable_display_ = true;         // 是否显示图像（默认关闭，避免影响性能）
bool show_timestamp_ = false;          // 是否显示时间戳

// 保存配置
int save_interval_ = 10;             // 保存间隔（每100帧保存一次）
std::string save_directory_ = "./bgr_images";  // 保存目录

// 统计信息
uint64_t total_processed_frames_ = 0;  // 总共处理的帧数
uint64_t total_saved_images_ = 0;      // 总共保存的图像数
bool enable_opencv_processing_ = true;

// 在文件顶部添加
#if __cplusplus >= 201703L
#include <filesystem>
namespace fs = std::filesystem;
#else
#include <experimental/filesystem>
namespace fs = std::experimental::filesystem;
#endif

//#include "buffer.h"
//using CudaBuffer = senseAD::pal::cuda::CudaBuffer;
#include <ad_pal/buffer.hpp>
#include <ad_pal/data_type/image.hpp>




// 极简宏配置（一键修改核心参数，无冗余）
#define TARGET_FRAME_TYPE 2    // 目标处理帧类型
#define MAX_DEBUG_FRAME   10   // 最多处理前10帧调试
#define IMG_SHOW_TIME     1000 // 图像显示时长(ms)




// 添加YAML配置支持
#include <yaml-cpp/yaml.h>
#include <fstream>
#include <string>

// std::string config_file = "./config.yaml";



class SubTest : public senseAD::swcfl::Workshop<senseAD::swcfl::RsclWorkshopOption>{
public:

    SubTest() { this->SetName("PalVideoFrameReceiverWorkshop"); }


    void Test_rscl(int argc, char* argv[]) {
        std::cout << "start init" << std::endl;
        RsclAccess::Instance()->init(argc, argv, "test_sub_rscl");
        std::cout << "start subscribe rscl" << std::endl;
        // 从配置文件加载话题名称
        std::cout << "当前工作目录: " << fs::current_path() << std::endl;

        std::string config_file = "./config.yaml";

        //   // 组合成绝对路径
        // fs::path config_absolute_path = fs::current_path() / config_file;
        // std::cout << "配置文件绝对路径: " << config_absolute_path << std::endl;

        YAML::Node config = YAML::LoadFile(config_file);
        std::string camera_topic = config["camera_topic"].as<std::string>();
        std::cout << "订阅话题: " << camera_topic << std::endl;
        //        RsclAccess::Instance()->CudaSubscriber("center_camera_fov120", &SubTest::OnMsg20260109, this);
        // RsclAccess::Instance()->CudaSubscriber("center_camera_fov120", &SubTest::CallbackWWZ, this);
        RsclAccess::Instance()->CudaSubscriber(camera_topic, &SubTest::CallbackWWZ, this);

        std::cout << "end subscribe rscl" << std::endl;

    }


    void OnMsg20260109(const Memblock &mem_block)
    {
        std::cout << "终于进入OnMsg了！Memblock是否为空：" << (mem_block ? "否" : "是") << std::endl;

        if (!mem_block) {
            std::cout<< " received an empty Memblock message" << std::endl;
            return;
        }

        // 1. 解析Header
        std::cout << "start cudaMemcpy mem_block->ptr to header" << std::endl;
        senseAD::ImageHeader_t header;
//        cudaMemcpy(&header, mem_block->ptr, sizeof(senseAD::ImageHeader_t), cudaMemcpyDeviceToHost);
        cudaError_t cuda_ret = cudaMemcpy(&header, mem_block->ptr,
                                                 sizeof(senseAD::ImageHeader_t),
                                                 cudaMemcpyDeviceToHost);

        if (cuda_ret != cudaSuccess)
        {
            std::cerr << "Failed to copy image header from CUDA memory: "
                      << cudaGetErrorString(cuda_ret) << std::endl;
            return;
        }

        std::cout << "header.width: " << header.width  << std::endl;
        std::cout << "header.height: " << header.height  << std::endl;

//        // 图像数据指针（跳过图像头）
//        uint8_t* img_data_ptr = reinterpret_cast<uint8_t*>(mem_block->ptr) +
//                sizeof(senseAD::ImageHeader_t);

        std::cout << "Received image: " << std::endl;
        std::cout << "  Timestamp: " << header.timestamp << " ns" << std::endl;
        std::cout << "  Resolution: " << header.width << "x" << header.height << std::endl;
        std::cout << "  Format: " << (header.format == senseAD::ImageFormat_t::BGR ? "BGR" : "NV12") << std::endl;

        // TODO: 这里可以处理图像数据
        // 例如：将CUDA图像复制到主机内存进行处理
        // 或者直接在CUDA设备上进行处理

        // 示例：计算图像数据大小
        size_t img_data_size = 0;
        if (header.format == senseAD::ImageFormat_t::BGR) {
            img_data_size = header.width * header.height * 3;
        } else if (header.format == senseAD::ImageFormat_t::NV12) {
            img_data_size = header.width * header.height * 3 / 2;
        }

        std::cout << "  Image data size: " << img_data_size << " bytes" << std::endl;

        // 直接进行OpenCV转换（BGR格式）
        //        if (enable_opencv_processing_ && header.format == senseAD::ImageFormat_t::BGR) {
        if (enable_opencv_processing_) {

            // 检查图像尺寸是否有效
            if (header.width <= 0 || header.height <= 0)
            {
                std::cout << "Invalid image dimensions: " << header.width << "x" << header.height << std::endl;
            }
            else
            {

                // 直接从Memblock转换，最简单高效
                void *img_data_ptr = static_cast<uint8_t *>(mem_block->ptr) + sizeof(header);
                if (img_data_ptr == nullptr) {
                    std::cout << "image_data buffer data is nullptr " << std::endl;
                }
                size_t bgr_data_size = header.width * header.height * 3;
                //            size_t bgr_data_size = 3840 * 2160 * 3;

                std::cout << "bgr_data_size: " << header.width << "  " <<header.height<<  " " << bgr_data_size << std::endl;


                // **关键修复：安全分配主机内存**
                std::vector<uint8_t> host_buffer;
                // 方法1：直接resize并检查
                host_buffer.resize(bgr_data_size);
                // 检查分配是否成功
                if (host_buffer.empty()) {
                    std::cout << "Failed to allocate host buffer: vector is empty after resize"
                              << ", size: " << bgr_data_size << std::endl;

                }
                cudaMemcpy(host_buffer.data(), img_data_ptr, bgr_data_size, cudaMemcpyDeviceToHost);
                if (host_buffer.data() == nullptr) {
                    std::cout << "Host buffer data is nullptr " << std::endl;
                }
                cv::Mat cv_mat(static_cast<int>(header.height), static_cast<int>(header.width), CV_8UC3, host_buffer.data());
                //            cv::Mat cv_mat(2160, 3840, CV_8UC3, host_buffer.data());

                std::cout << "cv_mat.empty(): " << cv_mat.empty() << " " << cv_mat.size() << std::endl;

                if (!cv_mat.empty()) {
                    ProcessBGRWithOpenCV(cv_mat, frame_count_, header.timestamp);
                }
            }
        }

      frame_count_++;


    }




    void CallbackWWZ(const std::shared_ptr<senseAD::rscl::comm::Memblock>& _msg) {
        std::lock_guard<std::mutex> lock(buffer_mutex_);

        // 检查消息大小是否足够包含头部信息
        if (_msg->size < sizeof(senseAD::ImageHeader_t)) {
            AD_FERROR() << "✗ 消息大小不足，无法读取图像头部信息";
            return;
        }

        senseAD::ImageHeader_t header;
        cudaError_t err =
            cudaMemcpy(&header, _msg->ptr, sizeof(senseAD::ImageHeader_t),
                       cudaMemcpyDeviceToHost);
        if (err != cudaSuccess) {
            AD_FERROR() << "✗ 从设备到主机拷贝图像头部失败: "
                        << cudaGetErrorString(err);

            return;
        }
        AD_FINFO() << "图像宽度: " << header.width;
        AD_FINFO() << "图像高度: " << header.height;
        AD_FINFO() << "时间戳: " << header.timestamp;
        AD_FINFO() << "图像格式: "
                   << (header.format == senseAD::BGR    ? "BGR"
                       : header.format == senseAD::NV12 ? "NV12"
                                                        : "未知");

//        void* buf_data =
//            static_cast<uint8_t*>(_msg->ptr + sizeof(senseAD::ImageHeader_t));

        void* buf_data = static_cast<uint8_t *>(_msg->ptr) + sizeof(senseAD::ImageHeader_t);


        auto buf_size = _msg->size - sizeof(senseAD::ImageHeader_t);

        if (header.format == senseAD::ImageFormat_t::BGR) {
            if (buf_size !=
                static_cast<size_t>(header.height * header.width * 3)) {
                AD_FWARN() << "⚠️ BGR图像数据大小不匹配，预期: "
                           << (header.height * header.width * 3)
                           << ", 实际: " << buf_size;
                return;
            }

            cv ::Mat bgr_mat(header.height, header.width, CV_8UC3);

            err = cudaMemcpy(bgr_mat.data, buf_data, buf_size,
                             cudaMemcpyDeviceToHost);
            if (err != cudaSuccess) {
                AD_FERROR()
                    << "✗ CUDA内存拷贝失败: " << cudaGetErrorString(err);
            }
            if (bgr_mat.empty()) {
                AD_FERROR() << "✗ 创建的BGR矩阵为空";
                return;
            }


            // cv::imshow("dst",bgr_mat);
            // cv::waitKey(0);

            if (!bgr_mat.empty()) {
                ProcessBGRWithOpenCV(bgr_mat, frame_count_, header.timestamp);
            }
//            if (cv::imwrite(filename, bgr_mat)) {
//                AD_FWARN() << "✅ BGR图像保存成功: " << filename;
//                AD_FINFO() << "   - 图像尺寸: " << bgr_mat.cols << "x"
//                           << bgr_mat.rows;
//                AD_FINFO() << "   - 数据类型: " << bgr_mat.type();
//                AD_FINFO() << "   - 总像素数: " << bgr_mat.total();
//            } else {
//                AD_FERROR() << "❌ BGR图像保存失败: " << filename;
//            }
        } else if (header.format == senseAD::ImageFormat_t::NV12) {
            AD_FWARN() << "TODO: 暂不支持保存NV12格式的图像";
        }
//        AD_FWARN() << "===============================================";
//        AD_FWARN() << "话题处理完成: " << _topic_name;
//        AD_FWARN() << "===============================================";
    }



    senseAD::pal::PixelFormat ImageFormat2PixelFormat(senseAD::ImageFormat_t format) {
        senseAD::pal::PixelFormat pixel_format;
        switch(format) {
        case senseAD::ImageFormat_t::NV12:
            std::cout << "header.format: " << format  << " is NV12" << std::endl;
            pixel_format = senseAD::pal::PixelFormat::YUV_NV12;
            break;
        case senseAD::ImageFormat_t::BGR:
            std::cout << "header.format: " << format  << " is BGR" << std::endl;
            pixel_format = senseAD::pal::PixelFormat::BGR;
            break;
        default:
            std::cout << "header.format: " << format  << " is NONE" << std::endl;
            pixel_format = senseAD::pal::PixelFormat::NONE;
        }
        return pixel_format;
    }



    /**
     * @brief 处理BGR格式的OpenCV图像 - 只包含可视化和保存操作
     * @param cv_mat BGR格式的OpenCV图像
     * @param frame_count 帧编号
     * @param timestamp 时间戳
     */
    void ProcessBGRWithOpenCV(
        cv::Mat cv_mat,  // 传值，确保线程安全
        uint64_t frame_count,
        uint64_t timestamp) {

        // 1. 基础检查
        if (cv_mat.empty()) {
            AD_SWCFL_WARN(PalVideoFrameReceiverWorkshop::ProcessBGRWithOpenCV)
                << "Received empty BGR image, frame: " << frame_count;
            return;
        }

        int width = cv_mat.cols;
        int height = cv_mat.rows;

        AD_SWCFL_DEBUG(PalVideoFrameReceiverWorkshop::ProcessBGRWithOpenCV)
            << "Processing BGR frame " << frame_count
            << ", size: " << width << "x" << height;

       // 2. 保存图像（如果启用）
       if (enable_save_images_) {
           SaveBGRImage(cv_mat, frame_count, timestamp);
       }

        // 3. 显示图像（如果启用）
        if (enable_display_) {
            DisplayBGRImage(cv_mat, frame_count, timestamp);
        }


    }


    /**
     * @brief 显示BGR图像
     */
    void DisplayBGRImage(
        const cv::Mat& cv_mat,
        uint64_t frame_count,
        uint64_t timestamp) {

        try {
            static bool window_created = false;

            // 1. 创建窗口（如果需要）
            if (!window_created) {
                cv::namedWindow("BGR Image Viewer", cv::WINDOW_NORMAL);
                cv::moveWindow("BGR Image Viewer", 100, 100);
                window_created = true;

                AD_SWCFL_INFO(PalVideoFrameReceiverWorkshop::DisplayBGRImage)
                    << "Created OpenCV display window";
            }

            // 2. 准备显示图像（添加信息叠加）
            cv::Mat display_mat = cv_mat.clone();

            // 添加信息叠加
            std::string frame_text = "Frame: " + std::to_string(frame_count);
            cv::putText(display_mat, frame_text,
                       cv::Point(10, 30),
                       cv::FONT_HERSHEY_SIMPLEX, 1.0,
                       cv::Scalar(0, 0, 255), 2);  // 红色文字

            std::string size_text = "Size: " + std::to_string(cv_mat.cols) +
                                   "x" + std::to_string(cv_mat.rows);
            cv::putText(display_mat, size_text,
                       cv::Point(10, 60),
                       cv::FONT_HERSHEY_SIMPLEX, 0.7,
                       cv::Scalar(0, 255, 0), 1);  // 绿色文字

            // 添加时间信息（可选）
            if (show_timestamp_) {
                std::string time_text = "Time: " + std::to_string(timestamp / 1000000) + "ms";
                cv::putText(display_mat, time_text,
                           cv::Point(10, 90),
                           cv::FONT_HERSHEY_SIMPLEX, 0.7,
                           cv::Scalar(255, 255, 0), 1);  // 黄色文字
            }

            // 3. 显示图像
            cv::imshow("BGR Image Viewer", display_mat);

            // 4. 处理键盘输入（非阻塞）
            int key = cv::waitKey(1);


            // 6. 更新显示统计
            if (frame_count % 100 == 0) {
                AD_SWCFL_DEBUG(PalVideoFrameReceiverWorkshop::DisplayBGRImage)
                    << "Displayed frame " << frame_count;
            }

        } catch (const cv::Exception& e) {
            AD_SWCFL_ERROR(PalVideoFrameReceiverWorkshop::DisplayBGRImage)
                << "OpenCV error displaying image: " << e.what();
        }
    }

    /**
     * @brief 保存BGR图像到文件
     */
    void SaveBGRImage(
        const cv::Mat& cv_mat,
        uint64_t frame_count,
        uint64_t timestamp) {

        try {
            // 1. 确定保存间隔
            if (frame_count % save_interval_ != 0) {
                return;
            }

            // 2. 创建保存目录
            // std::string save_dir = "bgr_images";
            if (!fs::exists(save_directory_)) {
                fs::create_directories(save_directory_);
            }
            // 3. 生成文件名
            std::string filename = save_directory_ + "/frame_" +
                                  std::to_string(frame_count) +
                                  "_" + std::to_string(timestamp) + ".jpg";

            // 4. 添加帧号和时间戳水印（可选）
            cv::Mat image_to_save = cv_mat.clone();

            // 5. 保存图像
            std::vector<int> compression_params;
            compression_params.push_back(cv::IMWRITE_JPEG_QUALITY);
            compression_params.push_back(90);  // JPEG质量

            bool saved = cv::imwrite(filename, image_to_save, compression_params);

            if (saved) {
               std::cout << "Saved BGR image to: " << filename
                    << ", frame: " << frame_count << std::endl;

                // 更新保存统计
                total_saved_images_++;

                // 定期输出保存统计
                if (total_saved_images_ % 10 == 0) {
                   std::cout << "Total images saved: " << total_saved_images_ << std::endl;
                }
            } else {
                std::cout  << "Failed to save image: " << filename << std::endl;
            }

        } catch (const cv::Exception& e) {
            std::cout << "OpenCV error saving image: " << e.what() << std::endl;
        } catch (const std::exception& e) {
            std::cout<< "Error saving image: " << e.what() << std::endl;
        }
    }



//private:
//   bool CreateDirectories(const std::string& pat, const int& camera_id) {
//       try {
//           // 使用提供的参数创建目录
//           std::filesystem::create_directories(pat);
//           std::string camera_dir =
//               pat + "/camera_" + std::to_string(camera_id);
//           std::filesystem::create_directories(camera_dir);

//           AD_FINFO() << "图像保存目录创建成功: " << pat << std::endl;
//           return true;
//       } catch (const std::exception& e) {
//           AD_FINFO() << "创建目录失败: " << e.what() << std::endl;
//           return false;
//       }
//   }
   int ExtractCameraIdBySplit(const std::string& topic_name) {
       std::vector<std::string> parts;
       std::string delimiter = "/";
       size_t start = 0;
       size_t end = topic_name.find(delimiter);

       while (end != std::string::npos) {
           parts.push_back(topic_name.substr(start, end - start));
           start = end + delimiter.length();
           end = topic_name.find(delimiter, start);
       }
       parts.push_back(topic_name.substr(start));

       // 寻找"camera"后面的数字
       for (size_t i = 0; i < parts.size(); ++i) {
           if (parts[i] == "camera" && i + 1 < parts.size()) {
               try {
                   return std::stoi(parts[i + 1]);
               } catch (...) {
                   continue;  // 如果无法转换为整数，继续寻找下一个
               }
           }
       }

       return 0;  // 默认返回0
   }

private:
   std::mutex buffer_mutex_;
   std::map<std::string, int> topic_frame_counts_;  // 每个话题的帧计数器
   std::map<std::string, std::shared_ptr<senseAD::rscl::comm::Memblock>>
       mem_map_;



};

// ====================== 主函数示例 ======================

int main(int argc, char* argv[]) {
    SubTest test;
//    test.Test_ros(argc, argv);
     test.Test_rscl(argc, argv);

    while(true) {
        sleep(20);
    }

    return 0;
}
