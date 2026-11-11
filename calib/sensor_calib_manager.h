#pragma once
// 传感器标定参数加载器
// 从 JSON 标定文件加载 LiDAR、IMU、车辆等参数
// 与 fisheye_undistort.h 共用 JSON 解析风格，不引入第三方 JSON 库

#include <opencv2/opencv.hpp>
#include <string>
#include <vector>
#include <unordered_map>
#include <optional>

namespace sentinel {

// ==================== LiDAR 标定 ====================

struct LidarCalib {
    std::string name;               // e.g. "front_lidar"
    std::string lidar_type;         // e.g. "ATX", "Pandar128"
    bool is_open{true};
    bool use_undist{true};
    float max_range_m{300.0f};
    float min_range_m{0.5f};
    float view_width_rad{7.0f};
    float view_direction_rad{0.0f};
    std::string ip_addr;
    int port{0};
    cv::Mat extrinsic;              // 4x4 lidar→car_center (CV_64F)
    // design info (optional)
    float designed_height_m{0.0f};
    float designed_vertical_offset_m{0.0f};
    float designed_lateral_offset_m{0.0f};
};

// ==================== IMU 标定 ====================

struct ImuCalib {
    struct Vec3 { double x{0}, y{0}, z{0}; };
    Vec3 accel_bias;
    Vec3 gyro_bias;
    Vec3 accel_scale;
    Vec3 gyro_scale;
};

struct OdometerCalib {
    double speed_scale{1.0};
};

struct DualAntCalib {
    double heading_offset{0.0};
};

// ==================== 车辆参数 ====================

struct VehicleParam {
    double car_length{0};
    double car_width{0};
    double car_height{0};
    double wheel_base{0};
    double front_axis_to_front_bumper{0};
    double rear_axis_to_rear_bumper{0};
    double wheel_track{0};
};

// ==================== 统一加载器 ====================

/**
 * @brief 传感器标定参数管理器
 *
 * 从 calib_base_dir 加载各传感器标定 JSON，提供统一查询接口。
 *
 * 用法：
 *   SensorCalibManager mgr;
 *   mgr.loadAll("param/calib");
 *   auto* lidar = mgr.getLidar("front_lidar");
 *   auto* imu = mgr.getImu();
 */
class SensorCalibManager {
public:
    SensorCalibManager() = default;

    // 从 calib_base_dir 批量加载所有传感器标定
    bool loadAll(const std::string& calib_base_dir);

    // 加载单个 LiDAR
    bool loadLidar(const std::string& name, const std::string& calib_dir);

    // 加载 IMU
    bool loadImu(const std::string& calib_dir);

    // 加载车辆参数
    bool loadVehicle(const std::string& json_path);

    // 查询接口
    const LidarCalib* getLidar(const std::string& name) const;
    std::vector<std::string> lidarNames() const;
    const ImuCalib* getImu() const;
    const OdometerCalib* getOdometer() const;
    const DualAntCalib* getDualAnt() const;
    const VehicleParam* getVehicle() const;

private:
    std::unordered_map<std::string, LidarCalib> lidars_;
    std::optional<ImuCalib> imu_;
    std::optional<OdometerCalib> odometer_;
    std::optional<DualAntCalib> dual_ant_;
    std::optional<VehicleParam> vehicle_;
};

}  // namespace sentinel
