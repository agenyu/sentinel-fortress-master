#include <ad_bag/bag_reader.h>
#include <ad_msg_idl/ad_sensor/sensor.capnp.h>
#include <capnp/serialize.h>
#include <opencv2/imgproc.hpp>
#include <opencv2/videoio.hpp>

#include <array>
#include <algorithm>
#include <cmath>
#include <cstdint>
#include <cstring>
#include <deque>
#include <fstream>
#include <iostream>
#include <limits>
#include <map>
#include <regex>
#include <set>
#include <string>
#include <unordered_set>
#include <vector>

namespace {
struct Cloud { uint64_t ts{0}; std::vector<cv::Point3f> points; };
struct Candidate {
    float x{0.f};
    float y{0.f};
    float radius{0.f};
    int points{0};
    bool semantic{false};
    int temporal_id{-1};
    std::string source;
};
struct ClusterTrack {
    int id{0};
    float x{0.f};
    float y{0.f};
    uint64_t last_ts{0};
    int observations{0};
    std::deque<cv::Point2f> history;
};
struct Sensor { const char* topic; const char* name; cv::Scalar colour; };

struct LeaseEvent {
    uint64_t ts{0};
    uint64_t expires{0};
    int temporal_id{-1};
    int streak{0};
};

struct PublishedTarget {
    int id{-1};
    float x{0.f};
    float y{0.f};
};

struct LifecycleLog {
    std::vector<LeaseEvent> leases;
    std::map<uint64_t, std::vector<PublishedTarget>> published;
    size_t inbox_matches{0};
};

const std::array<Sensor, 4> kSensors{{
    {"/sensor/lidar/front_lidar", "FRONT", {80, 210, 255}},
    {"/sensor/lidar/left_lidar",  "LEFT",  {90, 255, 90}},
    {"/sensor/lidar/rear_lidar",  "REAR",  {255, 110, 80}},
    {"/sensor/lidar/right_lidar", "RIGHT", {255, 90, 255}},
}};

int16_t ReadI16(const uint8_t* p) { int16_t v; std::memcpy(&v, p, sizeof(v)); return v; }

Cloud ParseCloud(const senseAD::msg::sensor::AdsfiLidarPointCloud::Reader& msg) {
    Cloud cloud;
    cloud.ts = msg.getHeader().getTime().getNanoSec();
    const auto data = msg.getData();
    const uint32_t step = msg.getPointStep();
    if (step < 16) return cloud;
    const auto* bytes = reinterpret_cast<const uint8_t*>(data.begin());
    const size_t count = data.size() / step;
    cloud.points.reserve(count);
    for (size_t i = 0; i < count; ++i) {
        const uint8_t* p = bytes + i * step;
        const float x = ReadI16(p) * 0.01f;
        const float y = ReadI16(p + 2) * 0.01f;
        const float z = ReadI16(p + 4) * 0.01f;
        const float r = std::hypot(x, y);
        if (r >= 0.5f && r <= 40.f && z >= -2.5f && z <= 3.f) cloud.points.emplace_back(x, y, z);
    }
    return cloud;
}

cv::Scalar TrackColour(int id) {
    // Bright, deterministic BGR colours so a cluster retains its colour across frames.
    static const std::array<cv::Scalar, 10> colours{{
        {70, 220, 255}, {255, 160, 70}, {120, 255, 100}, {255, 90, 220}, {230, 230, 80},
        {80, 130, 255}, {255, 255, 90}, {190, 90, 255}, {100, 255, 210}, {120, 180, 255},
    }};
    return colours[static_cast<size_t>(id) % colours.size()];
}

void UpdateGlobalTracks(std::vector<ClusterTrack>* tracks, int* next_id,
                        const std::vector<Candidate>& candidates, uint64_t ts) {
    constexpr uint64_t kTrackTimeoutNs = 1500000000ULL;
    constexpr float kAssociationGateM = 1.8f;
    tracks->erase(std::remove_if(tracks->begin(), tracks->end(), [ts](const ClusterTrack& track) {
        return ts > track.last_ts && ts - track.last_ts > kTrackTimeoutNs;
    }), tracks->end());
    std::unordered_set<int> used_tracks;
    for (const auto& candidate : candidates) {
        int best = -1;
        float best_distance = kAssociationGateM;
        for (size_t i = 0; i < tracks->size(); ++i) {
            if (used_tracks.count(static_cast<int>(i))) continue;
            const float dx = candidate.x - (*tracks)[i].x;
            const float dy = candidate.y - (*tracks)[i].y;
            const float distance = std::hypot(dx, dy);
            if (distance < best_distance) { best_distance = distance; best = static_cast<int>(i); }
        }
        if (best < 0) {
            const int id = candidate.temporal_id > 0 ? candidate.temporal_id : (*next_id)++;
            tracks->push_back(ClusterTrack{id, candidate.x, candidate.y, ts, 0, {}});
            best = static_cast<int>(tracks->size()) - 1;
        }
        auto& track = (*tracks)[best];
        track.x = candidate.x; track.y = candidate.y; track.last_ts = ts; ++track.observations;
        track.history.emplace_back(candidate.x, candidate.y);
        while (track.history.size() > 80) track.history.pop_front();
        used_tracks.insert(best);
    }
}

cv::Mat Render(const std::map<std::string, Cloud>& latest, uint64_t ts, uint64_t start,
               const std::vector<Candidate>* candidates, const std::vector<ClusterTrack>* tracks) {
    constexpr int kSize = 1024; constexpr float kRange = 40.f; constexpr float kScale = kSize / (2.f * kRange);
    cv::Mat image(kSize, kSize, CV_8UC3, cv::Scalar(18, 18, 18));
    const cv::Point center(kSize / 2, kSize / 2);
    for (int r : {5, 10, 20, 30, 40}) cv::circle(image, center, static_cast<int>(r * kScale), {58,58,58}, 1, cv::LINE_AA);
    cv::line(image, {center.x, 0}, {center.x, kSize}, {65,65,65}); cv::line(image, {0, center.y}, {kSize, center.y}, {65,65,65});
    std::string labels;
    for (const auto& sensor : kSensors) {
        const auto it = latest.find(sensor.topic);
        if (it == latest.end() || ts - it->second.ts > 150000000ULL) continue;
        labels += std::string(sensor.name) + " ";
        for (const auto& p : it->second.points) {
            const int u = static_cast<int>(std::lround(center.x - p.y * kScale));
            const int v = static_cast<int>(std::lround(center.y - p.x * kScale));
            if (u >= 0 && u < kSize && v >= 0 && v < kSize) image.at<cv::Vec3b>(v, u) = cv::Vec3b(sensor.colour[0], sensor.colour[1], sensor.colour[2]);
        }
    }
    if (candidates != nullptr) {
        for (const auto& candidate : *candidates) {
            const int u = static_cast<int>(std::lround(center.x - candidate.y * kScale));
            const int v = static_cast<int>(std::lround(center.y - candidate.x * kScale));
            const int radius_px = std::max(5, static_cast<int>(std::lround(candidate.radius * kScale)));
            const cv::Scalar colour = candidate.semantic ? cv::Scalar(40, 230, 40)
                                                         : cv::Scalar(0, 190, 255);
            cv::circle(image, {u, v}, radius_px, colour, 2, cv::LINE_AA);
            cv::circle(image, {u, v}, 3, colour, -1, cv::LINE_AA);
            if (candidate.semantic) {
                cv::putText(image, "semantic " + std::to_string(candidate.points), {u + 7, v - 7},
                            cv::FONT_HERSHEY_SIMPLEX, .42, colour, 1, cv::LINE_AA);
            }
        }
    }
    if (tracks != nullptr) {
        for (const auto& track : *tracks) {
            // Suppress short-lived clutter while showing a track after three associated scans.
            if (track.observations < 3 || track.history.size() < 3) continue;
            const cv::Scalar colour = TrackColour(track.id);
            std::vector<cv::Point> line;
            line.reserve(track.history.size());
            for (const auto& point : track.history) {
                line.emplace_back(static_cast<int>(std::lround(center.x - point.y * kScale)),
                                  static_cast<int>(std::lround(center.y - point.x * kScale)));
            }
            cv::polylines(image, line, false, colour, 2, cv::LINE_AA);
            cv::circle(image, line.back(), 5, colour, -1, cv::LINE_AA);
            cv::putText(image, "L" + std::to_string(track.id), line.back() + cv::Point(7, -7),
                        cv::FONT_HERSHEY_SIMPLEX, .42, colour, 1, cv::LINE_AA);
        }
    }
    std::vector<cv::Point> car{{center.x,center.y-25},{center.x-13,center.y+22},{center.x+13,center.y+22}};
    cv::fillConvexPoly(image, car, {230,230,230});
    const std::string title = tracks ? "LiDAR cluster global tracks | coloured=3+ scan association"
                                     : (candidates ? "LiDAR clusters: orange=unlabeled green=visual semantic"
                                                   : "RAW LiDAR point cloud | x forward, y left");
    cv::putText(image, title, {18,30}, cv::FONT_HERSHEY_SIMPLEX, .58, {240,240,240}, 2, cv::LINE_AA);
    cv::putText(image, labels, {18,kSize-20}, cv::FONT_HERSHEY_SIMPLEX, .55, {220,220,220}, 1, cv::LINE_AA);
    char t[64]; std::snprintf(t, sizeof(t), "t=%05.2fs", (ts-start)/1e9); cv::putText(image, t, {850,30}, cv::FONT_HERSHEY_SIMPLEX, .55, {240,240,240}, 1, cv::LINE_AA);
    return image;
}

const LeaseEvent* FindActiveLease(const std::vector<LeaseEvent>& leases,
                                  int temporal_id, uint64_t ts) {
    const LeaseEvent* active = nullptr;
    for (const auto& lease : leases) {
        if (lease.temporal_id != temporal_id || lease.ts > ts || ts > lease.expires) continue;
        if (active == nullptr || lease.ts > active->ts) active = &lease;
    }
    return active;
}

bool HasSource(const Candidate& candidate, const char* source) {
    return candidate.source == source;
}

cv::Scalar LifecycleCandidateColour(const Candidate& candidate, bool lease_active) {
    if (lease_active || HasSource(candidate, "orsm_far_image_motion_lidar_latched")) {
        return cv::Scalar(80, 230, 80);       // green: 20 s lease
    }
    if (HasSource(candidate, "orsm_far_image_motion_lidar_confirmed")) {
        return cv::Scalar(255, 220, 40);      // cyan: current Inbox match
    }
    if (HasSource(candidate, "orsm_far_image_motion_lidar_pending")) {
        return cv::Scalar(0, 210, 255);       // orange: first/pending Inbox hit
    }
    if (candidate.semantic) return cv::Scalar(40, 230, 40);
    return cv::Scalar(160, 170, 190);         // grey: ordinary LiDAR candidate
}

cv::Mat RenderLifecycle(
    const std::map<std::string, Cloud>& latest,
    uint64_t ts,
    uint64_t start,
    const std::vector<Candidate>* candidates,
    const std::map<int, std::deque<cv::Point2f>>& temporal_history,
    const std::vector<LeaseEvent>& leases,
    const std::vector<PublishedTarget>& published,
    size_t inbox_matches,
    size_t lease_count) {
    cv::Mat image = Render(latest, ts, start, nullptr, nullptr);
    constexpr int kSize = 1024;
    constexpr float kRange = 40.f;
    constexpr float kScale = kSize / (2.f * kRange);
    const cv::Point center(kSize / 2, kSize / 2);

    std::set<int> interesting_ids;
    for (const auto& lease : leases) {
        if (FindActiveLease(leases, lease.temporal_id, ts) != nullptr) {
            interesting_ids.insert(lease.temporal_id);
        }
    }
    if (candidates != nullptr) {
        for (const auto& candidate : *candidates) {
            if (candidate.temporal_id > 0 &&
                (candidate.source.find("far_image_motion") != std::string::npos ||
                 FindActiveLease(leases, candidate.temporal_id, ts) != nullptr)) {
                interesting_ids.insert(candidate.temporal_id);
            }
        }
    }

    // Draw every temporal LiDAR history, matching the legacy global-track
    // view.  Lifecycle-related tracks keep their semantic colours while
    // ordinary LiDAR candidates are shown in grey instead of being hidden.
    for (const auto& [id, history] : temporal_history) {
        if (history.size() < 2) continue;
        std::vector<cv::Point> line;
        line.reserve(history.size());
        for (const auto& point : history) {
            line.emplace_back(static_cast<int>(std::lround(center.x - point.y * kScale)),
                              static_cast<int>(std::lround(center.y - point.x * kScale)));
        }
        const bool active = FindActiveLease(leases, id, ts) != nullptr;
        const bool inbox_track = interesting_ids.count(id) > 0;
        const cv::Scalar colour = active
            ? cv::Scalar(70, 220, 70)      // 20s lease
            : (inbox_track
                ? cv::Scalar(255, 200, 40) // Inbox/pending
                : cv::Scalar(160, 170, 190)); // ordinary LiDAR candidate
        cv::polylines(image, line, false,
                      colour, active ? 3 : 2, cv::LINE_AA);
        cv::putText(image, "L" + std::to_string(id), line.back() + cv::Point(7, -7),
                    cv::FONT_HERSHEY_SIMPLEX, .45, colour, 1, cv::LINE_AA);
    }

    if (candidates != nullptr) {
        for (const auto& candidate : *candidates) {
            const int u = static_cast<int>(std::lround(center.x - candidate.y * kScale));
            const int v = static_cast<int>(std::lround(center.y - candidate.x * kScale));
            if (u < 0 || u >= kSize || v < 0 || v >= kSize) continue;
            const bool lease_active = candidate.temporal_id > 0 &&
                                      FindActiveLease(leases, candidate.temporal_id, ts) != nullptr;
            const cv::Scalar colour = LifecycleCandidateColour(candidate, lease_active);
            const int radius_px = std::max(4, static_cast<int>(std::lround(candidate.radius * kScale)));
            const bool inbox = candidate.source.find("far_image_motion_lidar") != std::string::npos;
            cv::circle(image, {u, v}, inbox ? radius_px : 4, colour,
                       inbox ? 2 : -1, cv::LINE_AA);
            if (lease_active) {
                cv::circle(image, {u, v}, radius_px + 5, cv::Scalar(70, 240, 70), 2, cv::LINE_AA);
                cv::putText(image, "LEASE L" + std::to_string(candidate.temporal_id),
                            {u + 8, v - 8}, cv::FONT_HERSHEY_SIMPLEX, .42,
                            cv::Scalar(80, 240, 80), 1, cv::LINE_AA);
            } else if (HasSource(candidate, "orsm_far_image_motion_lidar_confirmed")) {
                cv::putText(image, "INBOX L" + std::to_string(candidate.temporal_id),
                            {u + 8, v - 8}, cv::FONT_HERSHEY_SIMPLEX, .42,
                            cv::Scalar(255, 220, 40), 1, cv::LINE_AA);
            } else if (HasSource(candidate, "orsm_far_image_motion_lidar_pending")) {
                cv::putText(image, "INBOX pending", {u + 8, v - 8},
                            cv::FONT_HERSHEY_SIMPLEX, .42, colour, 1, cv::LINE_AA);
            }
        }
    }

    // Red is reserved for the final ORSM output, which is parsed from the
    // StatusOutput log rather than inferred from a LiDAR cluster.
    for (const auto& target : published) {
        const int u = static_cast<int>(std::lround(center.x - target.y * kScale));
        const int v = static_cast<int>(std::lround(center.y - target.x * kScale));
        if (u < 0 || u >= kSize || v < 0 || v >= kSize) continue;
        cv::circle(image, {u, v}, 12, cv::Scalar(0, 0, 255), 3, cv::LINE_AA);
        cv::circle(image, {u, v}, 4, cv::Scalar(0, 0, 255), -1, cv::LINE_AA);
        cv::putText(image, "PUBLISHED #" + std::to_string(target.id),
                    {u + 8, v + 18}, cv::FONT_HERSHEY_SIMPLEX, .44,
                    cv::Scalar(40, 80, 255), 1, cv::LINE_AA);
    }

    // Cover the generic renderer title/time so this mode has one unambiguous
    // lifecycle header instead of showing two different time labels.
    cv::rectangle(image, cv::Rect(0, 0, 1024, 100), cv::Scalar(18, 18, 18), -1, cv::LINE_AA);
    cv::putText(image, "ORSM lifecycle BEV | candidate -> Inbox -> lease -> published",
                {18, 32}, cv::FONT_HERSHEY_SIMPLEX, .52, cv::Scalar(245, 245, 245), 1, cv::LINE_AA);
    cv::putText(image, "grey=LiDAR candidate  orange=Inbox pending  cyan=Inbox hit  green=20s lease  red=published",
                {18, 57}, cv::FONT_HERSHEY_SIMPLEX, .38, cv::Scalar(225, 225, 225), 1, cv::LINE_AA);
    cv::putText(image, "Inbox hits=" + std::to_string(inbox_matches) +
                    "  active leases=" + std::to_string(lease_count) +
                    "  published=" + std::to_string(published.size()),
                {18, 80}, cv::FONT_HERSHEY_SIMPLEX, .42, cv::Scalar(245, 245, 245), 1, cv::LINE_AA);
    char time_text[64];
    std::snprintf(time_text, sizeof(time_text), "t=%05.2fs", (ts - start) / 1e9);
    cv::putText(image, time_text, {875, 32}, cv::FONT_HERSHEY_SIMPLEX, .55,
                cv::Scalar(245, 245, 245), 1, cv::LINE_AA);
    return image;
}
}

int main(int argc, char** argv) {
    if (argc < 3 || argc > 6) {
        std::cerr << "usage: lidar_bev_video BAG OUTPUT.mp4 [sentinel.log] [--global-tracks] [--lifecycle]\n";
        return 2;
    }
    bool global_tracks = false;
    bool lifecycle_mode = false;
    for (int i = 4; i < argc; ++i) {
        const std::string option(argv[i]);
        if (option == "--global-tracks") global_tracks = true;
        else if (option == "--lifecycle") lifecycle_mode = true;
        else { std::cerr << "unknown option: " << option << "\n"; return 2; }
    }
    std::map<uint64_t, std::vector<Candidate>> clusters;
    std::map<uint64_t, std::vector<cv::Point3f>> temporal;
    LifecycleLog lifecycle_log;
    if (argc >= 4) {
        std::ifstream log(argv[3]);
        const auto parseU64 = [](const std::string& value) -> uint64_t {
            try { return std::stoull(value); }
            catch (const std::exception&) { return 0; }
        };
        // Accept both cluster diagnostics with and without the optional source field.
        // Older/current replay logs omit source=..., but their positions are still
        // valid LiDAR candidates and must remain available for trajectory rendering.
        const std::regex pattern_with_source(R"(\[ORSMLidarClusterDiag\] ts=([0-9]+) cluster=[0-9]+ center=\((-?[0-9.]+),(-?[0-9.]+),-?[0-9.]+\) radius=([0-9.]+) points=([0-9]+) semantic=([01]).*source=([A-Za-z0-9_]+))");
        const std::regex pattern_without_source(R"(\[ORSMLidarClusterDiag\] ts=([0-9]+) cluster=[0-9]+ center=\((-?[0-9.]+),(-?[0-9.]+),-?[0-9.]+\) radius=([0-9.]+) points=([0-9]+) semantic=([01]))");
        const std::regex temporal_pattern(R"(\[ORSMLidarTemporalMap\] ts=([0-9]+) L=([0-9]+) center=\((-?[0-9.]+),(-?[0-9.]+)\))");
        // Tolerate extra diagnostic fields (min_streak/classed/label) between
        // streak and expires so added logging cannot silently drop all leases.
        const std::regex lease_pattern(R"(\[ORSMFarMotionLease\] ts=([0-9]+) L=([0-9]+) streak=([0-9]+).*?expires=([0-9]+))");
        const std::regex matched_pattern(R"(\[ORSMFarMotionInBoxMatched\])");
        const std::regex status_pattern(R"(\[SentinelNode\]\[StatusOutput\]\[ORSM\].*output_ts_ns=([0-9]+).*intruders=([0-9]+).*intruder_details=\[(.*)\])");
        const std::regex published_pattern(R"(#[0-9]+ type=[^ ]+ zone=[^ ]+ pos=\((-?[0-9.]+),(-?[0-9.]+)\))");
        std::string line;
        while (std::getline(log, line)) {
            std::smatch match;
            if (std::regex_search(line, match, temporal_pattern)) {
                const uint64_t ts = parseU64(match[1].str());
                if (ts == 0) continue;
                temporal[ts].emplace_back(
                    std::stof(match[3].str()), std::stof(match[4].str()), std::stof(match[2].str()));
                continue;
            }
            if (std::regex_search(line, match, lease_pattern)) {
                const uint64_t ts = parseU64(match[1].str());
                const uint64_t expires = parseU64(match[4].str());
                if (ts == 0 || expires == 0) continue;
                lifecycle_log.leases.push_back(LeaseEvent{
                    ts, expires,
                    std::stoi(match[2].str()), std::stoi(match[3].str())});
                continue;
            }
            if (std::regex_search(line, matched_pattern)) {
                ++lifecycle_log.inbox_matches;
            }
            if (std::regex_search(line, match, status_pattern)) {
                const uint64_t status_ts = parseU64(match[1].str());
                if (status_ts > 0 && std::stoi(match[2].str()) > 0) {
                    auto& frame = lifecycle_log.published[status_ts];
                    for (std::sregex_iterator it(match[3].first, match[3].second, published_pattern), end;
                         it != end; ++it) {
                        const int id = std::stoi((*it)[0].str().substr(1, (*it)[0].str().find(' ')));
                        const float x = std::stof((*it)[1].str());
                        const float y = std::stof((*it)[2].str());
                        const bool exists = std::any_of(frame.begin(), frame.end(),
                            [id](const PublishedTarget& target) { return target.id == id; });
                        if (!exists) frame.push_back(PublishedTarget{id, x, y});
                    }
                }
            }
            const bool has_source = std::regex_search(line, match, pattern_with_source);
            if (!has_source && !std::regex_search(line, match, pattern_without_source)) continue;
            const uint64_t cluster_ts = parseU64(match[1].str());
            const uint64_t point_count = parseU64(match[5].str());
            // Concurrent diagnostics can occasionally interleave two log lines.
            // Ignore the resulting malformed point count instead of aborting the
            // entire lifecycle render with std::stoi(out_of_range).
            if (cluster_ts == 0 ||
                point_count > static_cast<uint64_t>(std::numeric_limits<int>::max())) {
                continue;
            }
            const std::string source = has_source ? match[7].str() : "orsm_lidar_primary";
            clusters[cluster_ts].push_back(Candidate{
                std::stof(match[2].str()), std::stof(match[3].str()), std::stof(match[4].str()),
                static_cast<int>(point_count), match[6].str() == "1", -1, source});
        }
        for (auto& [ts, candidates] : clusters) for (auto& candidate : candidates) {
            const auto it = temporal.find(ts); if (it == temporal.end()) continue;
            float best = .05f;
            for (const auto& mapped : it->second) {
                const float d = std::hypot(candidate.x - mapped.x, candidate.y - mapped.y);
                if (d < best) { best = d; candidate.temporal_id = static_cast<int>(mapped.z); }
            }
        }
        std::cout << "cluster_frames=" << clusters.size() << "\n";
        if (lifecycle_mode) {
            std::cout << "lifecycle_inbox_matches=" << lifecycle_log.inbox_matches
                      << " leases=" << lifecycle_log.leases.size()
                      << " published_frames=" << lifecycle_log.published.size() << "\n";
        }
    }
    senseAD::bag::BagReaderAttribute attr;
    for (const auto& sensor : kSensors) attr.included_channels.insert(sensor.topic);
    attr.iterator_type = senseAD::bag::BagIteratorType::READ_NEXT_BY_TIMESTAMP_ORDER;
    senseAD::bag::BagReader reader(argv[1], attr);
    if (!reader.IsValid()) { std::cerr << "invalid bag\n"; return 3; }
    const std::string output_path(argv[2]);
    const bool avi_output = output_path.size() >= 4 &&
        output_path.substr(output_path.size() - 4) == ".avi";
    const int output_fourcc = avi_output
        ? cv::VideoWriter::fourcc('M','J','P','G')
        : cv::VideoWriter::fourcc('m','p','4','v');
    cv::VideoWriter writer(output_path, output_fourcc, 10.0, {1024,1024});
    if (!writer.isOpened()) { std::cerr << "cannot open output\n"; return 4; }
    std::map<std::string, Cloud> latest; uint64_t start = 0; size_t frames = 0;
    std::vector<ClusterTrack> tracks; int next_track_id = 1; uint64_t last_cluster_ts = 0;
    std::map<int, std::deque<cv::Point2f>> temporal_history;
    uint64_t last_lifecycle_cluster_ts = 0;
    senseAD::bag::ReadedMessage item;
    while (reader.ReadNextMessage(&item)) {
        try {
            if (item.message_buffer.size() % sizeof(capnp::word)) continue;
            auto words = kj::ArrayPtr<const capnp::word>(reinterpret_cast<const capnp::word*>(item.message_buffer.data()), item.message_buffer.size()/sizeof(capnp::word));
            capnp::FlatArrayMessageReader message(words);
            auto cloud = ParseCloud(message.getRoot<senseAD::msg::sensor::AdsfiLidarPointCloud>());
            latest[item.channel_name] = std::move(cloud);
            if (item.channel_name != kSensors[0].topic || latest[item.channel_name].points.empty()) continue;
            if (!start) start = latest[item.channel_name].ts;
            const std::vector<Candidate>* candidate_frame = nullptr;
            uint64_t candidate_ts = 0;
            if (!clusters.empty()) {
                auto it = clusters.upper_bound(latest[item.channel_name].ts);
                if (it != clusters.begin()) {
                    --it;
                    if (latest[item.channel_name].ts - it->first <= 500000000ULL) {
                        candidate_frame = &it->second;
                        candidate_ts = it->first;
                    }
                }
            }
            if (global_tracks && candidate_frame != nullptr) {
                if (candidate_ts != last_cluster_ts) {
                    UpdateGlobalTracks(&tracks, &next_track_id, *candidate_frame, candidate_ts);
                    last_cluster_ts = candidate_ts;
                }
            }
            if (lifecycle_mode) {
                if (candidate_ts != 0 && candidate_ts != last_lifecycle_cluster_ts) {
                    const auto temporal_it = temporal.find(candidate_ts);
                    if (temporal_it != temporal.end()) {
                        for (const auto& point : temporal_it->second) {
                            const int id = static_cast<int>(std::lround(point.z));
                            auto& history = temporal_history[id];
                            history.emplace_back(point.x, point.y);
                            while (history.size() > 100) history.pop_front();
                        }
                    }
                    last_lifecycle_cluster_ts = candidate_ts;
                }
                std::vector<PublishedTarget> published;
                if (!lifecycle_log.published.empty()) {
                    auto published_it = lifecycle_log.published.upper_bound(latest[item.channel_name].ts);
                    if (published_it != lifecycle_log.published.begin()) {
                        --published_it;
                        if (latest[item.channel_name].ts - published_it->first <= 1500000000ULL) {
                            published = published_it->second;
                        }
                    }
                }
                std::set<int> active_lease_ids;
                for (const auto& lease : lifecycle_log.leases) {
                    if (FindActiveLease(lifecycle_log.leases, lease.temporal_id,
                                        latest[item.channel_name].ts) != nullptr) {
                        active_lease_ids.insert(lease.temporal_id);
                    }
                }
                writer.write(RenderLifecycle(latest, latest[item.channel_name].ts, start,
                                             candidate_frame, temporal_history,
                                             lifecycle_log.leases, published,
                                             lifecycle_log.inbox_matches, active_lease_ids.size()));
            } else {
                writer.write(Render(latest, latest[item.channel_name].ts, start, candidate_frame,
                                    global_tracks ? &tracks : nullptr));
            }
            ++frames;
        } catch (const std::exception&) { }
    }
    writer.release(); std::cout << "rendered_frames=" << frames << "\n";
    return frames ? 0 : 5;
}
