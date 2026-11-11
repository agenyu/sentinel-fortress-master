#include <ad_bag/bag_reader.h>
#include <ad_msg_idl/ad_gac/radar_a_track_cm.capnp.h>
#include <capnp/serialize.h>

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <limits>
#include <map>
#include <set>
#include <string>
#include <vector>

struct Hit {
    uint64_t t = 0;
    float x = 0;
    float y = 0;
    float z = 0;
    float vx = 0;
    float vy = 0;
    float rcs = 0;
    uint8_t id = 0;
    uint8_t exist = 0;
    uint8_t mov = 0;
    uint8_t state = 0;
    uint8_t type = 0;
};

struct Summary {
    uint8_t id = 0;
    std::vector<Hit> hits;
    double path = 0;
    double displacement = 0;
    float min_x = std::numeric_limits<float>::infinity();
    float max_x = -std::numeric_limits<float>::infinity();
    float min_y = std::numeric_limits<float>::infinity();
    float max_y = -std::numeric_limits<float>::infinity();
    float min_r = std::numeric_limits<float>::infinity();
    float max_r = 0;
    std::set<std::string> sectors;
};

static std::string sector(float x, float y) {
    if (std::fabs(x) >= std::fabs(y)) return x >= 0 ? "front" : "rear";
    return y >= 0 ? "left" : "right";
}

static double dist2d(const Hit& a, const Hit& b) {
    const double dx = a.x - b.x;
    const double dy = a.y - b.y;
    return std::sqrt(dx * dx + dy * dy);
}

static void addHit(std::map<uint8_t, Summary>& summaries, const Hit& h) {
    auto& s = summaries[h.id];
    s.id = h.id;
    if (!s.hits.empty()) {
        const double dt = (h.t - s.hits.back().t) * 1e-9;
        const double d = dist2d(h, s.hits.back());
        if (dt > 0 && dt < 1.0 && d < 5.0) s.path += d;
    }
    s.hits.push_back(h);
    s.min_x = std::min(s.min_x, h.x);
    s.max_x = std::max(s.max_x, h.x);
    s.min_y = std::min(s.min_y, h.y);
    s.max_y = std::max(s.max_y, h.y);
    const float r = std::hypot(h.x, h.y);
    s.min_r = std::min(s.min_r, r);
    s.max_r = std::max(s.max_r, r);
    s.sectors.insert(sector(h.x, h.y));
}

static void finishSummary(Summary& s) {
    if (s.hits.size() >= 2) s.displacement = dist2d(s.hits.front(), s.hits.back());
}

static double durationSec(const Summary& s) {
    if (s.hits.size() < 2) return 0;
    return (s.hits.back().t - s.hits.front().t) * 1e-9;
}

static bool plausibleNearVehicle(const Hit& h) {
    if (!std::isfinite(h.x) || !std::isfinite(h.y) || !std::isfinite(h.z)) return false;
    const float r = std::hypot(h.x, h.y);
    if (r < 0.5f || r > 15.0f) return false;
    return true;
}

int main(int argc, char** argv) {
    if (argc < 3) {
        std::cerr << "usage: radar_track_audit BAG OUT_PREFIX [topic]\n";
        return 2;
    }
    const std::string topic = argc >= 4 ? argv[3] : "/sensor/radar";

    senseAD::bag::BagReaderAttribute attr;
    attr.included_channels = {topic};
    attr.iterator_type = senseAD::bag::BagIteratorType::READ_NEXT_BY_TIMESTAMP_ORDER;
    senseAD::bag::BagReader reader(argv[1], attr);
    if (!reader.IsValid()) {
        std::cerr << "invalid bag: " << argv[1] << "\n";
        return 3;
    }

    std::ofstream csv(std::string(argv[2]) + "_hits.csv");
    csv << "t_ns,id,x,y,z,vx,vy,speed,rcs,exist_probability,mov_property,track_state,track_type,sector\n";

    uint64_t frames = 0;
    uint64_t bad_frames = 0;
    uint64_t raw_hits = 0;
    uint64_t kept_hits = 0;
    uint64_t moving_hits = 0;
    std::map<uint8_t, Summary> summaries;
    std::map<std::string, uint64_t> sector_hits;

    senseAD::bag::ReadedMessage msg;
    while (reader.ReadNextMessage(&msg)) {
        frames++;
        try {
            if (msg.message_buffer.size() % sizeof(capnp::word) != 0) {
                bad_frames++;
                continue;
            }
            auto words = kj::ArrayPtr<const capnp::word>(
                reinterpret_cast<const capnp::word*>(msg.message_buffer.data()),
                msg.message_buffer.size() / sizeof(capnp::word));
            capnp::FlatArrayMessageReader message(words);
            auto radar = message.getRoot<gac::rscl::RadarTrackArray>();
            auto tracks = radar.getTrackList();
            raw_hits += tracks.size();
            for (auto tr : tracks) {
                Hit h;
                h.t = msg.timestamp;
                h.id = tr.getId();
                h.x = tr.getX();
                h.y = tr.getY();
                h.z = tr.getZ();
                h.vx = tr.getVx();
                h.vy = tr.getVy();
                h.rcs = tr.getRcs();
                h.exist = tr.getExistProbability();
                h.mov = tr.getMovProperty();
                h.state = tr.getTrackState();
                h.type = tr.getTrackType();
                if (!plausibleNearVehicle(h)) continue;
                kept_hits++;
                if (std::hypot(h.vx, h.vy) >= 0.05f) moving_hits++;
                const auto sec = sector(h.x, h.y);
                sector_hits[sec]++;
                csv << h.t << "," << static_cast<int>(h.id) << ","
                    << h.x << "," << h.y << "," << h.z << ","
                    << h.vx << "," << h.vy << "," << std::hypot(h.vx, h.vy) << ","
                    << h.rcs << "," << static_cast<int>(h.exist) << ","
                    << static_cast<int>(h.mov) << "," << static_cast<int>(h.state) << ","
                    << static_cast<int>(h.type) << "," << sec << "\n";
                addHit(summaries, h);
            }
        } catch (const std::exception&) {
            bad_frames++;
        }
    }

    std::vector<Summary> rows;
    for (auto& kv : summaries) {
        finishSummary(kv.second);
        rows.push_back(kv.second);
    }
    std::sort(rows.begin(), rows.end(), [](const Summary& a, const Summary& b) {
        if (a.sectors.size() != b.sectors.size()) return a.sectors.size() > b.sectors.size();
        if (a.path != b.path) return a.path > b.path;
        return a.hits.size() > b.hits.size();
    });

    std::cout << std::fixed << std::setprecision(3);
    std::cout << "frames=" << frames << " bad_frames=" << bad_frames
              << " raw_hits=" << raw_hits << " kept_near_vehicle_hits=" << kept_hits
              << " moving_speed_hits=" << moving_hits
              << " ids=" << rows.size() << "\n";
    std::cout << "sector_hits front=" << sector_hits["front"]
              << " left=" << sector_hits["left"]
              << " rear=" << sector_hits["rear"]
              << " right=" << sector_hits["right"] << "\n";
    std::cout << "top_tracks_by_around_vehicle_coverage\n";
    std::cout << "id,hits,duration_s,path_m,displacement_m,min_x,max_x,min_y,max_y,min_r,max_r,sectors\n";
    const size_t limit = std::min<size_t>(30, rows.size());
    for (size_t i = 0; i < limit; ++i) {
        const auto& s = rows[i];
        std::cout << static_cast<int>(s.id) << "," << s.hits.size() << ","
                  << durationSec(s) << "," << s.path << "," << s.displacement << ","
                  << s.min_x << "," << s.max_x << "," << s.min_y << "," << s.max_y << ","
                  << s.min_r << "," << s.max_r << ",";
        bool first = true;
        for (const auto& sec : s.sectors) {
            if (!first) std::cout << "|";
            std::cout << sec;
            first = false;
        }
        std::cout << "\n";
    }
}
