// 鱼眼去畸变模块实现
// 使用 cv::fisheye::initUndistortRectifyMap + cv::remap
// 预计算 remap 表，每帧只做一次查表映射

#include "fisheye_undistort.h"
#include <fstream>
#include <iostream>
#include <sstream>

// 简易 JSON 解析（仅提取标定文件中的数值矩阵，不引入第三方 JSON 库）
namespace {

// 从 JSON 字符串中提取 "data": [[...],[...],[...]] 形式的二维数组
bool extractMatrix(const std::string& json, const std::string& key,
                   int expected_rows, int expected_cols, cv::Mat& out) {
    // 找到 "key": { ... "data": [
    auto pos = json.find("\"" + key + "\"");
    if (pos == std::string::npos) return false;

    auto data_pos = json.find("\"data\"", pos);
    if (data_pos == std::string::npos) return false;

    auto bracket = json.find('[', data_pos + 6);
    if (bracket == std::string::npos) return false;

    // 解析嵌套数组
    out = cv::Mat::zeros(expected_rows, expected_cols, CV_64F);
    size_t p = bracket + 1;  // skip outer '['
    for (int r = 0; r < expected_rows; ++r) {
        auto row_start = json.find('[', p);
        if (row_start == std::string::npos) return false;
        auto row_end = json.find(']', row_start);
        if (row_end == std::string::npos) return false;

        std::string row_str = json.substr(row_start + 1, row_end - row_start - 1);
        std::istringstream iss(row_str);
        for (int c = 0; c < expected_cols; ++c) {
            char comma;
            if (!(iss >> out.at<double>(r, c))) return false;
            iss >> comma;  // skip comma
        }
        p = row_end + 1;
    }
    return true;
}

// 提取 "key": value (integer)
bool extractInt(const std::string& json, const std::string& key, int& out) {
    auto pos = json.find("\"" + key + "\"");
    if (pos == std::string::npos) return false;
    auto colon = json.find(':', pos);
    if (colon == std::string::npos) return false;
    std::istringstream iss(json.substr(colon + 1));
    return bool(iss >> out);
}

std::string readFile(const std::string& path) {
    std::ifstream f(path);
    if (!f.is_open()) return {};
    return std::string((std::istreambuf_iterator<char>(f)),
                        std::istreambuf_iterator<char>());
}

}  // namespace

namespace sentinel {

bool FisheyeUndistortManager::parseIntrinsic(const std::string& json_path,
                                              FisheyeCalib& calib) {
    std::string json = readFile(json_path);
    if (json.empty()) {
        std::cerr << "[FisheyeUndistort] Cannot read " << json_path << std::endl;
        return false;
    }

    int w = 0, h = 0;
    extractInt(json, "img_dist_w", w);
    extractInt(json, "img_dist_h", h);
    if (w > 0 && h > 0) {
        calib.img_w = w;
        calib.img_h = h;
    }

    if (!extractMatrix(json, "cam_K", 3, 3, calib.K)) {
        std::cerr << "[FisheyeUndistort] Failed to parse cam_K from " << json_path << std::endl;
        return false;
    }

    if (!extractMatrix(json, "cam_dist", 1, 4, calib.D)) {
        std::cerr << "[FisheyeUndistort] Failed to parse cam_dist from " << json_path << std::endl;
        return false;
    }

    // cam_K_new：如果存在则使用，否则用 cam_K
    if (!extractMatrix(json, "cam_K_new", 3, 3, calib.K_new)) {
        calib.K_new = calib.K.clone();
    }

    std::cout << "[FisheyeUndistort] Loaded intrinsic: " << calib.name
              << " " << calib.img_w << "x" << calib.img_h
              << " fx=" << calib.K.at<double>(0, 0)
              << " fy=" << calib.K.at<double>(1, 1)
              << " D=[" << calib.D.at<double>(0, 0) << ","
              << calib.D.at<double>(0, 1) << ","
              << calib.D.at<double>(0, 2) << ","
              << calib.D.at<double>(0, 3) << "]"
              << std::endl;
    return true;
}

bool FisheyeUndistortManager::parseExtrinsic(const std::string& json_path,
                                              FisheyeCalib& calib) {
    std::string json = readFile(json_path);
    if (json.empty()) {
        std::cerr << "[FisheyeUndistort] Cannot read " << json_path << std::endl;
        return false;
    }

    if (!extractMatrix(json, "sensor_calib", 4, 4, calib.extrinsic)) {
        std::cerr << "[FisheyeUndistort] Failed to parse extrinsic from " << json_path << std::endl;
        return false;
    }

    std::cout << "[FisheyeUndistort] Loaded extrinsic: " << calib.name
              << " tx=" << calib.extrinsic.at<double>(0, 3)
              << " ty=" << calib.extrinsic.at<double>(1, 3)
              << " tz=" << calib.extrinsic.at<double>(2, 3)
              << std::endl;
    return true;
}

void FisheyeUndistortManager::buildRemapTables(UndistortContext& ctx) {
    // cv::fisheye::initUndistortRectifyMap
    // 输入：K (3x3), D (1x4), R=Identity, P=K_new (3x3), size, type
    // 输出：map1, map2
    cv::Mat R = cv::Mat::eye(3, 3, CV_64F);
    cv::fisheye::initUndistortRectifyMap(
        ctx.calib.K,
        ctx.calib.D,
        R,
        ctx.calib.K_new,
        cv::Size(ctx.calib.img_w, ctx.calib.img_h),
        CV_16SC2,  // 使用 short 格式，remap 更快
        ctx.map1,
        ctx.map2
    );
    ctx.ready = true;

    std::cout << "[FisheyeUndistort] Remap tables built for " << ctx.calib.name
              << " (" << ctx.calib.img_w << "x" << ctx.calib.img_h << ")"
              << std::endl;
}

bool FisheyeUndistortManager::loadCalib(const std::string& camera_name,
                                         const std::string& calib_dir) {
    UndistortContext ctx;
    ctx.calib.name = camera_name;

    std::string dir = calib_dir;
    if (!dir.empty() && dir.back() != '/') dir += '/';

    std::string intrinsic_path = dir + camera_name + "-intrinsic.json";
    std::string extrinsic_path = dir + camera_name + "-to-car_center-extrinsic.json";

    if (!parseIntrinsic(intrinsic_path, ctx.calib)) return false;
    if (!parseExtrinsic(extrinsic_path, ctx.calib)) {
        // 外参可选，不影响去畸变
        std::cerr << "[FisheyeUndistort] Warning: no extrinsic for " << camera_name << std::endl;
    }

    buildRemapTables(ctx);

    std::lock_guard<std::mutex> lk(mutex_);
    cameras_[camera_name] = std::move(ctx);
    return true;
}

bool FisheyeUndistortManager::loadAll(const std::string& calib_base_dir,
                                       const std::vector<std::string>& camera_names) {
    bool all_ok = true;
    for (const auto& name : camera_names) {
        std::string dir = calib_base_dir;
        if (!dir.empty() && dir.back() != '/') dir += '/';
        dir += name + "/";
        if (!loadCalib(name, dir)) {
            std::cerr << "[FisheyeUndistort] Failed to load " << name << std::endl;
            all_ok = false;
        }
    }
    return all_ok;
}

cv::Mat FisheyeUndistortManager::undistort(const std::string& camera_name,
                                            const cv::Mat& distorted) const {
    std::lock_guard<std::mutex> lk(mutex_);
    auto it = cameras_.find(camera_name);
    if (it == cameras_.end() || !it->second.ready) {
        return distorted;  // 未加载标定，返回原图
    }

    cv::Mat rectified;
    cv::remap(distorted, rectified, it->second.map1, it->second.map2,
              cv::INTER_LINEAR, cv::BORDER_CONSTANT);
    return rectified;
}

const FisheyeCalib* FisheyeUndistortManager::getCalib(const std::string& camera_name) const {
    std::lock_guard<std::mutex> lk(mutex_);
    auto it = cameras_.find(camera_name);
    if (it == cameras_.end()) return nullptr;
    return &it->second.calib;
}

bool FisheyeUndistortManager::hasCamera(const std::string& camera_name) const {
    std::lock_guard<std::mutex> lk(mutex_);
    return cameras_.count(camera_name) > 0;
}

std::vector<std::string> FisheyeUndistortManager::cameraNames() const {
    std::lock_guard<std::mutex> lk(mutex_);
    std::vector<std::string> names;
    names.reserve(cameras_.size());
    for (const auto& [k, _] : cameras_) names.push_back(k);
    return names;
}

}  // namespace sentinel
