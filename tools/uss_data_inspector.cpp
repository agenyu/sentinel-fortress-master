#include "rscl_access/rscl_access.h"

#include <ad_msg_idl/avp_uss/uss_perception.capnp.h>

#include <algorithm>
#include <atomic>
#include <chrono>
#include <cmath>
#include <csignal>
#include <cstdint>
#include <iomanip>
#include <iostream>
#include <limits>
#include <map>
#include <mutex>
#include <sstream>
#include <string>
#include <thread>
#include <utility>
#include <vector>

namespace {

std::atomic<bool> g_running{true};

void signalHandler(int) {
    g_running = false;
}

struct PointFields {
    uint64_t timestamp{0};
    uint16_t age{0};
    int16_t x{0};
    int16_t y{0};
    uint8_t low_prob{0};
    uint8_t high_prob{0};
    uint8_t exist_prob{0};
    uint8_t free_prob{0};
    uint8_t type{0};
};

struct MetadataRun {
    uint64_t first_frame{0};
    uint64_t last_frame{0};
    uint64_t first_map_ts{0};
    uint64_t last_map_ts{0};
    uint64_t frame_count{0};
};

bool allFieldsZero(const PointFields& p) {
    return p.timestamp == 0 && p.age == 0 && p.x == 0 && p.y == 0 &&
           p.low_prob == 0 && p.high_prob == 0 && p.exist_prob == 0 &&
           p.free_prob == 0 && p.type == 0;
}

bool allMetadataZero(const PointFields& p) {
    return p.timestamp == 0 && p.age == 0 && p.low_prob == 0 &&
           p.high_prob == 0 && p.exist_prob == 0 && p.free_prob == 0 &&
           p.type == 0;
}

bool isSentinel(const PointFields& p) {
    return p.x == -2000 && p.y == -2000 && allMetadataZero(p);
}

bool hasMetadata(const PointFields& p) {
    return !allMetadataZero(p);
}

std::string pointKey(const PointFields& p) {
    std::ostringstream os;
    os << p.x << ',' << p.y
       << " age=" << p.age
       << " low=" << static_cast<unsigned>(p.low_prob)
       << " high=" << static_cast<unsigned>(p.high_prob)
       << " exist=" << static_cast<unsigned>(p.exist_prob)
       << " free=" << static_cast<unsigned>(p.free_prob)
       << " type=" << static_cast<unsigned>(p.type)
       << " pts=" << (p.timestamp != 0 ? 1 : 0);
    return os.str();
}

class UssInspector {
public:
    void subscribe(const std::string& topic) {
        RsclAccess::Instance()->subscribe<
            senseAD::msg::avp_perception::UssFusedStaticMap>(
                topic, &UssInspector::onMessage, this);
    }

    void printSummary() const {
        std::lock_guard<std::mutex> lock(mutex_);
        std::cout << "\n=== USS DATA SUMMARY ===\n"
                  << "frames=" << frames_
                  << " first_map_ts=" << first_map_ts_
                  << " last_map_ts=" << last_map_ts_ << '\n'
                  << "declared_size_zero_frames=" << declared_zero_frames_
                  << " declared_size_nonzero_frames=" << declared_nonzero_frames_
                  << " seq_zero_frames=" << seq_zero_frames_ << '\n'
                  << "total_slots=" << total_slots_
                  << " all_zero_slots=" << all_zero_slots_
                  << " sentinel_slots=" << sentinel_slots_
                  << " non_filler_slots=" << non_filler_slots_
                  << " metadata_slots=" << metadata_slots_
                  << " probability_slots=" << probability_slots_ << '\n'
                  << "frames_with_non_filler=" << frames_with_non_filler_
                  << " frames_with_metadata=" << frames_with_metadata_
                  << " frames_with_probability=" << frames_with_probability_ << '\n'
                  << "first_metadata_frame=" << first_metadata_frame_
                  << " last_metadata_frame=" << last_metadata_frame_
                  << " first_metadata_map_ts=" << first_metadata_map_ts_
                  << " last_metadata_map_ts=" << last_metadata_map_ts_ << '\n'
                  << "point_timestamp_nonzero=" << point_timestamp_nonzero_
                  << " age_nonzero=" << age_nonzero_
                  << " low_nonzero=" << low_nonzero_
                  << " high_nonzero=" << high_nonzero_
                  << " exist_nonzero=" << exist_nonzero_
                  << " free_nonzero=" << free_nonzero_
                  << " type_nonzero=" << type_nonzero_ << '\n';

        printDistribution("declared_sizes", declared_sizes_);
        printDistribution("list_sizes", list_sizes_);
        printDistribution("non_filler_per_frame", non_filler_per_frame_);
        printDistribution("metadata_per_frame", metadata_per_frame_);
        printDistribution("probability_per_frame", probability_per_frame_);
        printDistribution("exist_prob_values", exist_prob_values_);
        printDistribution("type_values", type_values_);

        std::cout << "metadata_raw_radius_min=" << metadata_radius_min_
                  << " metadata_raw_radius_max=" << metadata_radius_max_ << '\n'
                  << "non_filler_raw_radius_min=" << non_filler_radius_min_
                  << " non_filler_raw_radius_max=" << non_filler_radius_max_ << '\n';

        std::cout << "radius_counts_for_metadata"
                  << " <=130=" << metadata_radius_le_130_
                  << " <=500=" << metadata_radius_le_500_
                  << " <=1300=" << metadata_radius_le_1300_
                  << " <=2000=" << metadata_radius_le_2000_
                  << " <=5000=" << metadata_radius_le_5000_ << '\n';

        printTopPoints("top_non_filler_points", non_filler_points_);
        printTopPoints("top_metadata_points", metadata_points_);
        std::cout << "metadata_runs:\n";
        for (const auto& run : metadata_runs_) {
            std::cout << "  frames=" << run.first_frame << '-' << run.last_frame
                      << " count=" << run.frame_count
                      << " map_ts=" << run.first_map_ts << '-' << run.last_map_ts
                      << '\n';
        }
        std::cout << "=== END USS DATA SUMMARY ===" << std::endl;
    }

private:
    template <typename K>
    static void printDistribution(const char* name,
                                  const std::map<K, uint64_t>& values) {
        std::cout << name << ':';
        for (const auto& [key, count] : values) {
            std::cout << ' ' << key << '=' << count;
        }
        std::cout << '\n';
    }

    static void printTopPoints(const char* name,
                               const std::map<std::string, uint64_t>& values) {
        std::vector<std::pair<std::string, uint64_t>> sorted(values.begin(), values.end());
        std::sort(sorted.begin(), sorted.end(), [](const auto& lhs, const auto& rhs) {
            if (lhs.second != rhs.second) return lhs.second > rhs.second;
            return lhs.first < rhs.first;
        });
        std::cout << name << ":\n";
        const size_t limit = std::min<size_t>(30, sorted.size());
        for (size_t i = 0; i < limit; ++i) {
            std::cout << "  count=" << sorted[i].second << ' ' << sorted[i].first << '\n';
        }
    }

    void onMessage(const std::shared_ptr<ReceivedMsg<
                       senseAD::msg::avp_perception::UssFusedStaticMap>>& msg) {
        if (!msg) return;
        const auto map = msg->Msg();
        const auto points = map.getFusedPoints();

        std::lock_guard<std::mutex> lock(mutex_);
        frames_++;
        if (first_map_ts_ == 0) first_map_ts_ = map.getTimestamp();
        last_map_ts_ = map.getTimestamp();
        declared_sizes_[map.getFusedPointsSize()]++;
        list_sizes_[points.size()]++;
        if (map.getFusedPointsSize() == 0) declared_zero_frames_++;
        else declared_nonzero_frames_++;
        if (map.getSeq() == 0) seq_zero_frames_++;

        uint64_t frame_non_filler = 0;
        uint64_t frame_metadata = 0;
        uint64_t frame_probability = 0;
        for (const auto& raw : points) {
            PointFields p;
            p.timestamp = raw.getTimestamp();
            p.age = raw.getAge();
            p.x = raw.getX();
            p.y = raw.getY();
            p.low_prob = raw.getLowProb();
            p.high_prob = raw.getHighProb();
            p.exist_prob = raw.getExistProb();
            p.free_prob = raw.getFreeProb();
            p.type = raw.getType();

            total_slots_++;
            if (allFieldsZero(p)) {
                all_zero_slots_++;
                continue;
            }
            if (isSentinel(p)) {
                sentinel_slots_++;
                continue;
            }

            non_filler_slots_++;
            frame_non_filler++;
            non_filler_points_[pointKey(p)]++;
            const double radius = std::hypot(static_cast<double>(p.x),
                                             static_cast<double>(p.y));
            non_filler_radius_min_ = std::min(non_filler_radius_min_, radius);
            non_filler_radius_max_ = std::max(non_filler_radius_max_, radius);

            if (p.timestamp != 0) point_timestamp_nonzero_++;
            if (p.age != 0) age_nonzero_++;
            if (p.low_prob != 0) low_nonzero_++;
            if (p.high_prob != 0) high_nonzero_++;
            if (p.exist_prob != 0) {
                exist_nonzero_++;
                exist_prob_values_[static_cast<unsigned>(p.exist_prob)]++;
            }
            if (p.free_prob != 0) free_nonzero_++;
            if (p.type != 0) {
                type_nonzero_++;
                type_values_[static_cast<unsigned>(p.type)]++;
            }

            const bool probability = p.low_prob != 0 || p.high_prob != 0 ||
                                     p.exist_prob != 0 || p.free_prob != 0;
            if (probability) {
                probability_slots_++;
                frame_probability++;
            }
            if (!hasMetadata(p)) continue;

            metadata_slots_++;
            frame_metadata++;
            metadata_points_[pointKey(p)]++;
            metadata_radius_min_ = std::min(metadata_radius_min_, radius);
            metadata_radius_max_ = std::max(metadata_radius_max_, radius);
            if (radius <= 130.0) metadata_radius_le_130_++;
            if (radius <= 500.0) metadata_radius_le_500_++;
            if (radius <= 1300.0) metadata_radius_le_1300_++;
            if (radius <= 2000.0) metadata_radius_le_2000_++;
            if (radius <= 5000.0) metadata_radius_le_5000_++;
        }

        non_filler_per_frame_[frame_non_filler]++;
        metadata_per_frame_[frame_metadata]++;
        probability_per_frame_[frame_probability]++;
        if (frame_non_filler > 0) frames_with_non_filler_++;
        if (frame_metadata > 0) {
            frames_with_metadata_++;
            if (first_metadata_frame_ == 0) {
                first_metadata_frame_ = frames_;
                first_metadata_map_ts_ = map.getTimestamp();
            }
            last_metadata_frame_ = frames_;
            last_metadata_map_ts_ = map.getTimestamp();
            if (metadata_runs_.empty() ||
                metadata_runs_.back().last_frame + 1 != frames_) {
                metadata_runs_.push_back(MetadataRun{
                    frames_, frames_, map.getTimestamp(), map.getTimestamp(), 1});
            } else {
                auto& run = metadata_runs_.back();
                run.last_frame = frames_;
                run.last_map_ts = map.getTimestamp();
                run.frame_count++;
            }
        }
        if (frame_probability > 0) frames_with_probability_++;

        if (frames_ <= 10 || frames_ % 500 == 0) {
            std::cout << "[USSInspector] frame=" << frames_
                      << " map_ts=" << map.getTimestamp()
                      << " seq=" << map.getSeq()
                      << " declared=" << map.getFusedPointsSize()
                      << " list=" << points.size()
                      << " non_filler=" << frame_non_filler
                      << " metadata=" << frame_metadata
                      << " probability=" << frame_probability
                      << std::endl;
        }
    }

    mutable std::mutex mutex_;
    uint64_t frames_{0};
    uint64_t first_map_ts_{0};
    uint64_t last_map_ts_{0};
    uint64_t declared_zero_frames_{0};
    uint64_t declared_nonzero_frames_{0};
    uint64_t seq_zero_frames_{0};
    uint64_t total_slots_{0};
    uint64_t all_zero_slots_{0};
    uint64_t sentinel_slots_{0};
    uint64_t non_filler_slots_{0};
    uint64_t metadata_slots_{0};
    uint64_t probability_slots_{0};
    uint64_t frames_with_non_filler_{0};
    uint64_t frames_with_metadata_{0};
    uint64_t frames_with_probability_{0};
    uint64_t first_metadata_frame_{0};
    uint64_t last_metadata_frame_{0};
    uint64_t first_metadata_map_ts_{0};
    uint64_t last_metadata_map_ts_{0};
    uint64_t point_timestamp_nonzero_{0};
    uint64_t age_nonzero_{0};
    uint64_t low_nonzero_{0};
    uint64_t high_nonzero_{0};
    uint64_t exist_nonzero_{0};
    uint64_t free_nonzero_{0};
    uint64_t type_nonzero_{0};
    double metadata_radius_min_{std::numeric_limits<double>::max()};
    double metadata_radius_max_{0.0};
    double non_filler_radius_min_{std::numeric_limits<double>::max()};
    double non_filler_radius_max_{0.0};
    uint64_t metadata_radius_le_130_{0};
    uint64_t metadata_radius_le_500_{0};
    uint64_t metadata_radius_le_1300_{0};
    uint64_t metadata_radius_le_2000_{0};
    uint64_t metadata_radius_le_5000_{0};
    std::map<unsigned, uint64_t> declared_sizes_;
    std::map<unsigned, uint64_t> list_sizes_;
    std::map<uint64_t, uint64_t> non_filler_per_frame_;
    std::map<uint64_t, uint64_t> metadata_per_frame_;
    std::map<uint64_t, uint64_t> probability_per_frame_;
    std::map<unsigned, uint64_t> exist_prob_values_;
    std::map<unsigned, uint64_t> type_values_;
    std::map<std::string, uint64_t> non_filler_points_;
    std::map<std::string, uint64_t> metadata_points_;
    std::vector<MetadataRun> metadata_runs_;
};

}  // namespace

int main(int argc, char** argv) {
    const std::string topic = argc > 1 ? argv[1] : "/sensor/uss/fused_points";
    std::signal(SIGINT, signalHandler);
    std::signal(SIGTERM, signalHandler);

    RsclAccess::Instance()->init(argc, argv, "uss_data_inspector");
    UssInspector inspector;
    inspector.subscribe(topic);
    std::cout << "[USSInspector] subscribed topic=" << topic
              << "; press Ctrl+C after replay" << std::endl;

    while (g_running.load()) {
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(250));
    inspector.printSummary();
    RsclAccess::Instance()->release();
    return 0;
}
