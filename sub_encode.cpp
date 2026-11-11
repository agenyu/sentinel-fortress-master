#include <ad_rscl/ad_rscl.h>
#include "rscl_access/rscl_access.h"
#include "ad_msg_idl/ad_sensor/sensor.capnp.h"
#include "ad_msg_idl/std_msgs/raw_data.capnp.h"

#include <opencv2/opencv.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/highgui.hpp>

#include "ad_rscl/comm/buf_subscriber.h"
#include <ad_log/ad_log.hpp>
#include <ad_pal/data_type/video_frame.hpp>

#include "ad_rscl/comm/cuda/cuda_subscriber.h"
#include <cuda_runtime.h>

#include "nv_hevc_decoder.h"

// IMU 消息类型
#include <ad_msg_idl/ad_localization/localization.capnp.h>

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
#include <cmath>

#if __cplusplus >= 201703L
#include <filesystem>
namespace fs = std::filesystem;
#else
#include <experimental/filesystem>
namespace fs = std::experimental::filesystem;
#endif

#include <yaml-cpp/yaml.h>

using CameraEncodedMsg = senseAD::msg::std_msgs::CameraEncodedStruct;
using Memblock = std::shared_ptr<senseAD::rscl::comm::Memblock>;

// ====================== 配置参数 ======================
static constexpr bool kEnableSaveImages = true;
static constexpr bool kEnableDisplay   = true;
static constexpr int  kSaveInterval    = 10;
static constexpr char kSaveDirectory[] = "./bgr_images";

// IMU 振动检测阈值（m/s²），超过此值认为有碰撞/振动事件
static constexpr float kVibrationThresholdAccel = 2.0f;
// IMU 角速度阈值（rad/s），超过此值认为车辆被推动/倾斜
static constexpr float kVibrationThresholdGyro  = 0.5f;

// ====================== IMU 数据快照（原子存储，无锁读写） ======================
struct ImuSnapshot {
    float accel_x{0.f}, accel_y{0.f}, accel_z{0.f};  // m/s²
    float gyro_x{0.f},  gyro_y{0.f},  gyro_z{0.f};   // rad/s
    uint64_t timestamp_ns{0};
    bool vibration_alert{false};  // 是否触发振动报警
};

// ====================== 异步解码任务结构体 ======================
struct DecodeTask {
    uint64_t             seq;
    uint64_t             timestamp;
    int                  frame_type;
    int                  target_idx;
    std::vector<uint8_t> encoded_data;
    bool                 is_keyframe;
    ImuSnapshot          imu_snapshot;  // 入队时的 IMU 快照
};


class SubTest : public senseAD::swcfl::Workshop<senseAD::swcfl::RsclWorkshopOption> {
public:

    SubTest() {
        this->SetName("SentinelPerceptionWorkshop");

        // 初始化 NVDEC 硬件解码器
        decoder_ = std::make_unique<NVHevcDecoder>();
        if (!decoder_->init(3840, 2160, 0, true)) {
            std::cerr << "Failed to initialize NVDEC decoder!" << std::endl;
        }

        // 启动异步解码线程
        decode_thread_ = std::thread(&SubTest::DecoderWorker, this);
    }

    ~SubTest() {
        {
            std::lock_guard<std::mutex> lk(queue_mutex_);
            running_ = false;
        }
        queue_cv_.notify_all();
        if (decode_thread_.joinable()) decode_thread_.join();
    }

    void Test_rscl(int argc, char* argv[]) {
        std::cout << "[Sentinel] Initializing RSCL..." << std::endl;
        RsclAccess::Instance()->init(argc, argv, "sentinel_perception");
        std::cout << "[Sentinel] Current dir: " << fs::current_path() << std::endl;

        // 读取配置
        YAML::Node config = YAML::LoadFile("./config.yaml");
        std::string camera_topic = config["camera_topic"].as<std::string>();
        std::cout << "[Sentinel] Camera topic: " << camera_topic << std::endl;

        // 订阅摄像头编码流
        RsclAccess::Instance()->subscribe(camera_topic, &SubTest::CallbackCamera, this);

        // 订阅 IMU
        RsclAccess::Instance()->subscribe("/sensor/imu", &SubTest::CallbackImu, this);
        std::cout << "[Sentinel] IMU topic: /sensor/imu" << std::endl;

        std::cout << "[Sentinel] All subscriptions active." << std::endl;
    }

// ====================== 成员变量 ======================
private:
    // --- NVDEC 解码器 ---
    std::unique_ptr<NVHevcDecoder> decoder_;

    // --- GOP 解码状态（仅回调线程访问） ---
    bool     has_keyframe_           = false;
    int      gop_actual_frame_count_ = 0;
    uint64_t last_seq_               = 0;

    // --- 异步解码队列 ---
    std::queue<DecodeTask>  decode_queue_;
    std::mutex              queue_mutex_;
    std::condition_variable queue_cv_;
    std::thread             decode_thread_;
    std::atomic<bool>       running_{true};
    static constexpr size_t MAX_QUEUE_SIZE = 10;

    // --- 性能统计 ---
    std::atomic<uint64_t> decode_success_count_{0};
    std::atomic<uint64_t> decode_fail_count_{0};

    // --- IMU 最新数据（原子写入，供解码线程无锁读取） ---
    std::mutex   imu_mutex_;
    ImuSnapshot  latest_imu_;

    // --- 振动事件统计 ---
    std::atomic<uint64_t> vibration_event_count_{0};


// ====================== IMU 回调（高频，必须极快返回） ======================

    void CallbackImu(const std::shared_ptr<ReceivedMsg<senseAD::msg::sensor::Imu>>& msg) {
        auto m = msg->Msg();

        ImuSnapshot snap;
        snap.accel_x      = m.getAccelX();
        snap.accel_y      = m.getAccelY();
        snap.accel_z      = m.getAccelZ();
        snap.gyro_x       = m.getGyroX();
        snap.gyro_y       = m.getGyroY();
        snap.gyro_z       = m.getGyroZ();
        snap.timestamp_ns = m.getActualTimeNs();

        // 振动检测：加速度合力（去除重力分量后的动态加速度）
        // 简化处理：检测 X/Y 轴（水平方向）加速度突变
        float accel_mag = std::sqrt(snap.accel_x * snap.accel_x +
                                    snap.accel_y * snap.accel_y);
        float gyro_mag  = std::sqrt(snap.gyro_x * snap.gyro_x +
                                    snap.gyro_y * snap.gyro_y +
                                    snap.gyro_z * snap.gyro_z);

        snap.vibration_alert = (accel_mag > kVibrationThresholdAccel ||
                                gyro_mag  > kVibrationThresholdGyro);

        if (snap.vibration_alert) {
            vibration_event_count_++;
            AD_LINFO(Sentinel) << "[Sentinel][IMU] VIBRATION ALERT! "
                       << "accel_mag=" << accel_mag
                       << " gyro_mag=" << gyro_mag;
        }

        {
            std::lock_guard<std::mutex> lk(imu_mutex_);
            latest_imu_ = snap;
        }
    }


// ====================== 摄像头回调（必须极快返回） ======================

    void CallbackCamera(const std::shared_ptr<ReceivedMsg<CameraEncodedMsg>>& msg) {
        auto msg_content = msg->Msg();
        const uint64_t seq        = msg_content.getHeader().getSeq();
        const uint64_t timestamp  = msg_content.getHeader().getTime().getNanoSec();
        const uint32_t data_size  = msg_content.getData().size();
        const auto     frame_type = msg_content.getFrameType();

        if (frame_type == 2) {
            std::cout << "\n=== [I-Frame] seq=" << seq
                      << " ts=" << (timestamp / 1000000) << "ms"
                      << " size=" << data_size << " bytes ===" << std::endl;
        }

        if (data_size == 0) return;

        const uint8_t* encoded_data = msg_content.getData().begin();

        // 采集当前 IMU 快照，随帧一起入队
        ImuSnapshot imu_snap;
        {
            std::lock_guard<std::mutex> lk(imu_mutex_);
            imu_snap = latest_imu_;
        }

        if (frame_type == 2) {
            has_keyframe_ = true;
            last_seq_ = seq;
            gop_actual_frame_count_ = 0;
            SubmitDecodeTask(seq, timestamp, static_cast<int>(frame_type), 0,
                             encoded_data, data_size, true, imu_snap);
            return;
        }

        if (!has_keyframe_) return;

        const uint64_t seq_diff = seq - last_seq_;
        last_seq_ = seq;

        constexpr uint64_t MAX_SEQ_GAP = 10;
        if (seq_diff > MAX_SEQ_GAP) {
            std::cout << "[Camera] Large seq gap " << seq_diff
                      << ", waiting for next I-frame" << std::endl;
            has_keyframe_ = false;
            gop_actual_frame_count_ = 0;
            return;
        }

        gop_actual_frame_count_++;
        SubmitDecodeTask(seq, timestamp, static_cast<int>(frame_type),
                         gop_actual_frame_count_, encoded_data, data_size, false, imu_snap);
    }


// ====================== 异步解码队列 ======================

    void SubmitDecodeTask(uint64_t seq, uint64_t timestamp,
                          int frame_type, int target_idx,
                          const uint8_t* data, size_t size,
                          bool is_keyframe, const ImuSnapshot& imu_snap) {
        std::lock_guard<std::mutex> lk(queue_mutex_);

        if (decode_queue_.size() >= MAX_QUEUE_SIZE) {
            std::cout << "[Queue] Full, dropping oldest task." << std::endl;
            decode_queue_.pop();
        }

        DecodeTask task;
        task.seq          = seq;
        task.timestamp    = timestamp;
        task.frame_type   = frame_type;
        task.target_idx   = target_idx;
        task.is_keyframe  = is_keyframe;
        task.imu_snapshot = imu_snap;
        task.encoded_data.assign(data, data + size);

        decode_queue_.push(std::move(task));
        queue_cv_.notify_one();
    }

    void DecoderWorker() {
        while (true) {
            DecodeTask task;
            {
                std::unique_lock<std::mutex> lk(queue_mutex_);
                queue_cv_.wait(lk, [this] {
                    return !decode_queue_.empty() || !running_;
                });
                if (!running_ && decode_queue_.empty()) break;
                task = std::move(decode_queue_.front());
                decode_queue_.pop();
            }
            DecodeAndProcess(task);
        }
        std::cout << "[DecoderWorker] Exiting." << std::endl;
    }


// ====================== 解码核心（只在解码线程调用） ======================

    void DecodeAndProcess(const DecodeTask& task) {
        const uint64_t seq        = task.seq;
        const uint64_t timestamp  = task.timestamp;
        const int      frame_type = task.frame_type;
        const ImuSnapshot& imu    = task.imu_snapshot;

        auto start = std::chrono::high_resolution_clock::now();

        cv::Mat decoded;
        bool success = false;
        if (decoder_ && decoder_->isInitialized()) {
            success = decoder_->decode(task.encoded_data, decoded);
        }

        auto end      = std::chrono::high_resolution_clock::now();
        auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(end - start);

        if (success && !decoded.empty()) {
            decode_success_count_++;

            // --- 叠加摄像头帧信息 ---
            char cam_buf[128];
            std::snprintf(cam_buf, sizeof(cam_buf),
                          "seq=%lu ts=%lums %s decode=%ldms",
                          seq, timestamp / 1000000,
                          (frame_type == 2 ? "[I]" : "[P]"),
                          duration.count());
            cv::putText(decoded, cam_buf, cv::Point(10, 40),
                        cv::FONT_HERSHEY_SIMPLEX, 0.8, cv::Scalar(0, 255, 0), 2);

            // --- 叠加 IMU 数据 ---
            char imu_buf[192];
            std::snprintf(imu_buf, sizeof(imu_buf),
                          "IMU accel=(%.2f,%.2f,%.2f)m/s2  gyro=(%.2f,%.2f,%.2f)rad/s",
                          imu.accel_x, imu.accel_y, imu.accel_z,
                          imu.gyro_x,  imu.gyro_y,  imu.gyro_z);
            cv::putText(decoded, imu_buf, cv::Point(10, 80),
                        cv::FONT_HERSHEY_SIMPLEX, 0.6, cv::Scalar(255, 200, 0), 2);

            // --- 振动报警叠加 ---
            if (imu.vibration_alert) {
                cv::putText(decoded, "!! VIBRATION ALERT !!",
                            cv::Point(10, 130),
                            cv::FONT_HERSHEY_SIMPLEX, 1.2,
                            cv::Scalar(0, 0, 255), 3);
                // 红色边框提示
                cv::rectangle(decoded,
                              cv::Point(0, 0),
                              cv::Point(decoded.cols - 1, decoded.rows - 1),
                              cv::Scalar(0, 0, 255), 8);
            }

            // --- 控制台日志 ---
            std::cout << "SUCCESS: seq=" << seq
                      << " ts=" << (timestamp / 1000000) << "ms"
                      << " " << decoded.cols << "x" << decoded.rows
                      << (frame_type == 2 ? " [I]" : " [P]")
                      << " decode=" << duration.count() << "ms"
                      << " | IMU accel=(" << imu.accel_x << "," << imu.accel_y << "," << imu.accel_z << ")"
                      << " gyro=(" << imu.gyro_x << "," << imu.gyro_y << "," << imu.gyro_z << ")"
                      << (imu.vibration_alert ? " [VIBRATION!]" : "")
                      << " (ok=" << decode_success_count_
                      << " fail=" << decode_fail_count_ << ")"
                      << std::endl;

            if (kEnableDisplay) {
                cv::namedWindow("Sentinel Perception", cv::WINDOW_NORMAL);
                cv::resizeWindow("Sentinel Perception", 1920, 1080);
                cv::imshow("Sentinel Perception", decoded);
                cv::waitKey(1);
            }

            if (kEnableSaveImages && seq % kSaveInterval == 0) {
                if (!fs::exists(kSaveDirectory)) {
                    fs::create_directories(kSaveDirectory);
                }
                char save_path[256];
                std::snprintf(save_path, sizeof(save_path),
                              "%s/seq_%lu_ts_%lu%s.jpg",
                              kSaveDirectory, seq, timestamp,
                              imu.vibration_alert ? "_ALERT" : "");
                cv::imwrite(save_path, decoded, {cv::IMWRITE_JPEG_QUALITY, 90});
                std::cout << "Saved: " << save_path << std::endl;
            }

        } else {
            decode_fail_count_++;

            if (task.is_keyframe) {
                std::cout << "WARNING: I-frame decode failed! seq=" << seq
                          << " This will cause P-frames to fail too." << std::endl;
            }

            std::cout << "FAILED to decode: seq=" << seq
                      << " type=" << frame_type
                      << " size=" << task.encoded_data.size()
                      << " (ok=" << decode_success_count_
                      << " fail=" << decode_fail_count_ << ")"
                      << std::endl;
        }
    }
};


// ====================== 主函数 ======================
int main(int argc, char* argv[]) {
    SubTest test;
    test.Test_rscl(argc, argv);

    while (true) {
        sleep(20);
    }

    return 0;
}
