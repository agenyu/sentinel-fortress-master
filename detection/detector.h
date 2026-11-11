#pragma once

#include <opencv2/opencv.hpp>
#include <memory>
#include <vector>
#include <string>

namespace sentinel {

// 单个检测目标
struct Detection {
    int      class_id{-1};
    float    confidence{0.f};
    cv::Rect bbox;           // 像素坐标
    std::string label;       // "person" / "animal" / "vehicle" 等
};

/**
 * @brief 目标检测器（TensorRT）
 *
 * 支持 TensorRT engine 直接加载推理。通过配置选择 RF-DETR 或 YOLO
 * 后处理，解码后继续返回稳定的 Detection 列表，供 ByteTrack / 融合 /
 * ORSM 复用。
 *
 * 若当前编译环境缺少 TensorRT/CUDA 头文件，会自动退化为不可初始化状态，
 * 便于非车端环境做文档/静态检查；在车端 SDK 环境下会启用真实推理。
 */
class Detector {
public:
    Detector();
    ~Detector();

    Detector(const Detector&) = delete;
    Detector& operator=(const Detector&) = delete;

    /**
     * @brief 加载 TensorRT engine 文件
     * @param engine_path .engine 文件路径
     * @param conf_threshold 置信度阈值（默认 0.5）
     * @param nms_threshold  YOLO 使用的 NMS 阈值；RF-DETR 默认不做传统 NMS
     * @param model_type     "rfdetr" 或 "yolo"
     */
    bool init(const std::string& engine_path,
              float conf_threshold = 0.5f,
              float nms_threshold  = 0.45f,
              const std::string& model_type = "rfdetr");

    /**
     * @brief 对单帧图像执行目标检测
     * @param image BGR 格式输入图像
     * @return 检测结果列表
     */
    std::vector<Detection> detect(const cv::Mat& image);

    bool isInitialized() const { return initialized_; }

private:
    struct Impl;

    bool  initialized_{false};
    float conf_threshold_{0.5f};
    float nms_threshold_{0.45f};
    std::string model_type_{"rfdetr"};
    std::string engine_path_;
    std::unique_ptr<Impl> impl_;
};

}  // namespace sentinel
