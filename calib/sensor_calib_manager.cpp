// 传感器标定参数加载器实现
// 从 JSON 标定文件解析 LiDAR / IMU / 车辆参数

#include "sensor_calib_manager.h"
#include <fstream>
#include <iostream>
#include <sstream>
#include <filesystem>

namespace fs = std::filesystem;

namespace {

std::string readFile(const std::string& path) {
    std::ifstream f(path);
    if (!f.is_open()) return {};
    return std::string((std::istreambuf_iterator<char>(f)),
                        std::istreambuf_iterator<char>());
}

// 提取 "key": number_value（支持整数和浮点）
bool extractDouble(const std::string& json, const std::string& key, double& out) {
    std::string needle = "\"" + key + "\"";
    auto pos = json.find(needle);
    if (pos == std::string::npos) return false;
    auto colon = json.find(':', pos + needle.size());
    if (colon == std::string::npos) return false;
    // skip whitespace
    size_t start = colon + 1;
    while (start < json.size() && (json[start] == ' ' || json[start] == '\t' || json[start] == '\n' || json[start] == '\r'))
        ++start;
    std::istringstream iss(json.substr(start));
    return bool(iss >> out);
}

bool extractInt(const std::string& json, const std::string& key, int& out) {
    double d;
    if (!extractDouble(json, key, d)) return false;
    out = static_cast<int>(d);
    return true;
}

bool extractBool(const std::string& json, const std::string& key, bool& out) {
    std::string needle = "\"" + key + "\"";
    auto pos = json.find(needle);
    if (pos == std::string::npos) return false;
    auto colon = json.find(':', pos + needle.size());
    if (colon == std::string::npos) return false;
    auto rest = json.substr(colon + 1, 20);
    if (rest.find("true") != std::string::npos) { out = true; return true; }
    if (rest.find("false") != std::string::npos) { out = false; return true; }
    return false;
}

bool extractString(const std::string& json, const std::string& key, std::string& out) {
    std::string needle = "\"" + key + "\"";
    auto pos = json.find(needle);
    if (pos == std::string::npos) return false;
    auto colon = json.find(':', pos + needle.size());
    if (colon == std::string::npos) return false;
    auto q1 = json.find('"', colon + 1);
    if (q1 == std::string::npos) return false;
    auto q2 = json.find('"', q1 + 1);
    if (q2 == std::string::npos) return false;
    out = json.substr(q1 + 1, q2 - q1 - 1);
    return true;
}

// 提取 4x4 外参矩阵 "sensor_calib": { "data": [[...]] }
bool extractExtrinsic(const std::string& json, cv::Mat& out) {
    auto data_pos = json.find("\"sensor_calib\"");
    if (data_pos == std::string::npos) return false;
    auto d2 = json.find("\"data\"", data_pos);
    if (d2 == std::string::npos) return false;
    auto bracket = json.find('[', d2 + 6);
    if (bracket == std::string::npos) return false;

    out = cv::Mat::zeros(4, 4, CV_64F);
    size_t p = bracket + 1;
    for (int r = 0; r < 4; ++r) {
        auto row_start = json.find('[', p);
        if (row_start == std::string::npos) return false;
        auto row_end = json.find(']', row_start);
        if (row_end == std::string::npos) return false;
        std::string row_str = json.substr(row_start + 1, row_end - row_start - 1);
        std::istringstream iss(row_str);
        for (int c = 0; c < 4; ++c) {
            char comma;
            if (!(iss >> out.at<double>(r, c))) return false;
            iss >> comma;
        }
        p = row_end + 1;
    }
    return true;
}

// 提取 Vec3 子对象 "key": { "x": ..., "y": ..., "z": ... }
bool extractVec3(const std::string& json, const std::string& key,
                 sentinel::ImuCalib::Vec3& out) {
    std::string needle = "\"" + key + "\"";
    auto pos = json.find(needle);
    if (pos == std::string::npos) return false;
    auto brace = json.find('{', pos);
    if (brace == std::string::npos) return false;
    auto end_brace = json.find('}', brace);
    if (end_brace == std::string::npos) return false;
    std::string sub = json.substr(brace, end_brace - brace + 1);
    return extractDouble(sub, "x", out.x) &&
           extractDouble(sub, "y", out.y) &&
           extractDouble(sub, "z", out.z);
}

}  // namespace

namespace sentinel {

bool SensorCalibManager::loadLidar(const std::string& name,
                                    const std::string& calib_dir) {
    LidarCalib calib;
    calib.name = name;

    std::string dir = calib_dir;
    if (!dir.empty() && dir.back() != '/') dir += '/';

    // intrinsic
    std::string intrinsic_path = dir + name + "-intrinsic.json";
    std::string json = readFile(intrinsic_path);
    if (!json.empty()) {
        extractString(json, "lidar_type", calib.lidar_type);
        extractBool(json, "is_open", calib.is_open);
        extractBool(json, "use_undist", calib.use_undist);
        double v;
        if (extractDouble(json, "max_range_m", v)) calib.max_range_m = static_cast<float>(v);
        if (extractDouble(json, "min_range_m", v)) calib.min_range_m = static_cast<float>(v);
        if (extractDouble(json, "view_width_rad", v)) calib.view_width_rad = static_cast<float>(v);
        if (extractDouble(json, "view_direction_rad", v)) calib.view_direction_rad = static_cast<float>(v);
        extractString(json, "ip_addr", calib.ip_addr);
        extractInt(json, "port", calib.port);

        std::cout << "[SensorCalib] Loaded LiDAR intrinsic: " << name
                  << " type=" << calib.lidar_type
                  << " range=[" << calib.min_range_m << "," << calib.max_range_m << "]m"
                  << std::endl;
    } else {
        std::cerr << "[SensorCalib] Cannot read " << intrinsic_path << std::endl;
    }

    // extrinsic
    std::string extrinsic_path = dir + name + "-to-car_center-extrinsic.json";
    json = readFile(extrinsic_path);
    if (!json.empty()) {
        if (extractExtrinsic(json, calib.extrinsic)) {
            std::cout << "[SensorCalib] Loaded LiDAR extrinsic: " << name
                      << " tx=" << calib.extrinsic.at<double>(0, 3)
                      << " ty=" << calib.extrinsic.at<double>(1, 3)
                      << " tz=" << calib.extrinsic.at<double>(2, 3)
                      << std::endl;
        }
    }

    lidars_[name] = std::move(calib);
    return true;
}

bool SensorCalibManager::loadImu(const std::string& calib_dir) {
    std::string dir = calib_dir;
    if (!dir.empty() && dir.back() != '/') dir += '/';

    // imu-intrinsic.json
    std::string imu_path = dir + "imu-intrinsic.json";
    std::string json = readFile(imu_path);
    if (!json.empty()) {
        ImuCalib calib;
        extractVec3(json, "accel_bias", calib.accel_bias);
        extractVec3(json, "gyro_bias", calib.gyro_bias);
        extractVec3(json, "accel_scale", calib.accel_scale);
        extractVec3(json, "gyro_scale", calib.gyro_scale);
        imu_ = calib;

        std::cout << "[SensorCalib] Loaded IMU intrinsic:"
                  << " accel_bias=(" << calib.accel_bias.x << "," << calib.accel_bias.y << "," << calib.accel_bias.z << ")"
                  << " gyro_bias=(" << calib.gyro_bias.x << "," << calib.gyro_bias.y << "," << calib.gyro_bias.z << ")"
                  << std::endl;
    } else {
        std::cerr << "[SensorCalib] Cannot read " << imu_path << std::endl;
    }

    // odometer-intrinsic.json
    std::string odo_path = dir + "odometer-intrinsic.json";
    json = readFile(odo_path);
    if (!json.empty()) {
        OdometerCalib odo;
        extractDouble(json, "speed_scale", odo.speed_scale);
        odometer_ = odo;
        std::cout << "[SensorCalib] Loaded odometer: speed_scale=" << odo.speed_scale << std::endl;
    }

    // dual_ant-intrinsic.json
    std::string ant_path = dir + "dual_ant-intrinsic.json";
    json = readFile(ant_path);
    if (!json.empty()) {
        DualAntCalib ant;
        extractDouble(json, "heading_offset", ant.heading_offset);
        dual_ant_ = ant;
        std::cout << "[SensorCalib] Loaded dual_ant: heading_offset=" << ant.heading_offset << std::endl;
    }

    return imu_.has_value();
}

bool SensorCalibManager::loadVehicle(const std::string& json_path) {
    std::string json = readFile(json_path);
    if (json.empty()) {
        std::cerr << "[SensorCalib] Cannot read " << json_path << std::endl;
        return false;
    }

    VehicleParam vp;
    extractDouble(json, "car_length", vp.car_length);
    extractDouble(json, "car_width", vp.car_width);
    extractDouble(json, "car_height", vp.car_height);
    extractDouble(json, "wheel_base", vp.wheel_base);
    extractDouble(json, "front_axis_to_front_bumper", vp.front_axis_to_front_bumper);
    extractDouble(json, "rear_axis_to_rear_bumper", vp.rear_axis_to_rear_bumper);
    extractDouble(json, "wheel_track", vp.wheel_track);
    vehicle_ = vp;

    std::cout << "[SensorCalib] Loaded vehicle: "
              << vp.car_length << "x" << vp.car_width << "x" << vp.car_height << "m"
              << " wheelbase=" << vp.wheel_base << "m"
              << std::endl;
    return true;
}

bool SensorCalibManager::loadAll(const std::string& calib_base_dir) {
    std::string base = calib_base_dir;
    if (!base.empty() && base.back() != '/') base += '/';

    bool ok = true;

    // 自动发现 LiDAR 目录（包含 *-intrinsic.json 且 lidar_type 字段存在）
    for (const auto& entry : fs::directory_iterator(base)) {
        if (!entry.is_directory()) continue;
        std::string dirname = entry.path().filename().string();
        // 跳过已知的非 LiDAR 目录
        if (dirname.find("camera") != std::string::npos) continue;
        if (dirname == "imu" || dirname == "gnss" || dirname == "vehicle" ||
            dirname == "bev" || dirname == "actuator" || dirname == "hardware" ||
            dirname == "sensor_design_info" || dirname == "car_center" || dirname == "ipm") continue;

        std::string intrinsic = base + dirname + "/" + dirname + "-intrinsic.json";
        if (fs::exists(intrinsic)) {
            std::string content = readFile(intrinsic);
            if (content.find("\"lidar_type\"") != std::string::npos) {
                if (!loadLidar(dirname, base + dirname + "/")) ok = false;
            }
        }
    }

    // IMU
    std::string imu_dir = base + "imu/";
    if (fs::exists(imu_dir)) {
        if (!loadImu(imu_dir)) ok = false;
    }

    // Vehicle
    std::string vehicle_path = base + "vehicle/vehicle.json";
    if (fs::exists(vehicle_path)) {
        if (!loadVehicle(vehicle_path)) ok = false;
    }

    return ok;
}

const LidarCalib* SensorCalibManager::getLidar(const std::string& name) const {
    auto it = lidars_.find(name);
    return it != lidars_.end() ? &it->second : nullptr;
}

std::vector<std::string> SensorCalibManager::lidarNames() const {
    std::vector<std::string> names;
    names.reserve(lidars_.size());
    for (const auto& [k, _] : lidars_) names.push_back(k);
    return names;
}

const ImuCalib* SensorCalibManager::getImu() const {
    return imu_.has_value() ? &imu_.value() : nullptr;
}

const OdometerCalib* SensorCalibManager::getOdometer() const {
    return odometer_.has_value() ? &odometer_.value() : nullptr;
}

const DualAntCalib* SensorCalibManager::getDualAnt() const {
    return dual_ant_.has_value() ? &dual_ant_.value() : nullptr;
}

const VehicleParam* SensorCalibManager::getVehicle() const {
    return vehicle_.has_value() ? &vehicle_.value() : nullptr;
}

}  // namespace sentinel
