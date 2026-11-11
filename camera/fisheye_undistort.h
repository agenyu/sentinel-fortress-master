#pragma once
// 鱼眼去畸变模块
// 使用 OpenCV cv::fisheye 等距投影模型（4 参数 k1/k2/k3/k4）
// 标定参数来源：param/calib/<camera_name>/<camera_name>-intrinsic.json

#include <opencv2/opencv.hpp>
#include <string>
#include <unordered_map>
#include <mutex>

namespace sentinel {

// 单个鱼眼相机的标定参数
struct FisheyeCalib {
    std::string name;           // e.g. "front_camera_fov195"
    int img_w{1920};            // 原始图像宽
    int img_h{1536};            // 原始图像高
    cv::Mat K;                  // 3x3 内参矩阵 (CV_64F)
    cv::Mat D;                  // 1x4 畸变系数 (CV_64F) [k1,k2,k3,k4]
    cv::Mat K_new;              // 3x3 去畸变后内参 (CV_64F)
    cv::Mat extrinsic;          // 4x4 camera→car_center (CV_64F)
};

// 单个相机的去畸变上下文（预计算 remap 表）
struct UndistortContext {
    FisheyeCalib calib;
    cv::Mat map1;               // remap 查找表 X
    cv::Mat map2;               // remap 查找表 Y
    bool ready{false};
};

/**
 * @brief 鱼眼去畸变管理器
 *
 * 初始化时从 JSON 标定文件加载 4 路鱼眼参数，预计算 remap 表。
 * 每帧调用 undistort() 做查表映射，开销约 2-4ms（1920x1536）。
 *
 * 用法：
 *   FisheyeUndistortManager mgr;
 *   mgr.loadCalib("front_camera_fov195", "param/calib/front_camera_fov195/");
 *   cv::Mat rectified = mgr.undistort("front_camera_fov195", raw_image);
 */
class FisheyeUndistortManager {
public:
    FisheyeUndistortManager() = default;

    // 从标定目录加载单个相机参数
    // calib_dir 应包含 <name>-intrinsic.json 和 <name>-to-car_center-extrinsic.json
    bool loadCalib(const std::string& camera_name, const std::string& calib_dir);

    // 从 config.yaml 中的 calib_base_dir 批量加载 4 路鱼眼
    bool loadAll(const std::string& calib_base_dir,
                 const std::vector<std::string>& camera_names);

    // 去畸变（线程安全，remap 表只读）
    cv::Mat undistort(const std::string& camera_name, const cv::Mat& distorted) const;

    // 获取标定参数（用于距离估算等）
    const FisheyeCalib* getCalib(const std::string& camera_name) const;

    // 是否已加载
    bool hasCamera(const std::string& camera_name) const;

    // 已加载的相机列表
    std::vector<std::string> cameraNames() const;

private:
    bool parseIntrinsic(const std::string& json_path, FisheyeCalib& calib);
    bool parseExtrinsic(const std::string& json_path, FisheyeCalib& calib);
    void buildRemapTables(UndistortContext& ctx);

    mutable std::mutex mutex_;
    std::unordered_map<std::string, UndistortContext> cameras_;
};

}  // namespace sentinel
