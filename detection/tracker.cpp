#include "tracker.h"

#include <algorithm>
#include <cmath>
#include <limits>
#include <numeric>
#include <set>
#include <unordered_set>

namespace sentinel {

namespace {
constexpr size_t kMaxTrajectoryPoints = 30;
constexpr float kInfCost = 1e6f;

int clampPositive(int v) { return std::max(1, v); }

float safeAspect(const cv::Rect& r) {
    return r.height > 0 ? static_cast<float>(r.width) / static_cast<float>(r.height) : 1.f;
}
}  // namespace

Tracker::Tracker(int max_lost_frames,
                 int min_hits,
                 float iou_threshold,
                 int output_lost_frames,
                 float high_conf_threshold,
                 float low_conf_threshold,
                 float new_track_threshold,
                 float low_match_iou_threshold)
    : max_lost_frames_(max_lost_frames)
    , min_hits_(min_hits)
    , output_lost_frames_(std::max(0, output_lost_frames))
    , iou_threshold_(iou_threshold)
    , high_conf_threshold_(high_conf_threshold)
    , low_conf_threshold_(low_conf_threshold)
    , new_track_threshold_(new_track_threshold)
    , low_match_iou_threshold_(low_match_iou_threshold)
{}

float Tracker::iou(const cv::Rect& a, const cv::Rect& b) {
    const int inter_area = (a & b).area();
    const int union_area = a.area() + b.area() - inter_area;
    if (union_area <= 0) return 0.f;
    return static_cast<float>(inter_area) / static_cast<float>(union_area);
}

cv::Point2f Tracker::centerOf(const cv::Rect& r) {
    return {r.x + r.width * 0.5f, r.y + r.height * 0.5f};
}

void Tracker::appendTrajectory(TrackedObject& obj, const cv::Point2f& center) {
    if (!obj.trajectory.empty()) {
        obj.velocity = center - obj.trajectory.back();
    }
    obj.trajectory.push_back(center);
    while (obj.trajectory.size() > kMaxTrajectoryPoints) {
        obj.trajectory.pop_front();
    }
}

cv::Vec4f Tracker::bboxToMeasurement(const cv::Rect& bbox) {
    const auto c = centerOf(bbox);
    return {c.x, c.y, safeAspect(bbox), static_cast<float>(std::max(1, bbox.height))};
}

cv::Rect Tracker::measurementToBbox(const cv::Vec4f& m) {
    const float cx = m[0];
    const float cy = m[1];
    const float aspect = std::max(0.05f, m[2]);
    const float h = std::max(1.f, m[3]);
    const float w = std::max(1.f, aspect * h);
    return cv::Rect(static_cast<int>(std::round(cx - w * 0.5f)),
                    static_cast<int>(std::round(cy - h * 0.5f)),
                    clampPositive(static_cast<int>(std::round(w))),
                    clampPositive(static_cast<int>(std::round(h))));
}

cv::KalmanFilter Tracker::createKalman(const cv::Rect& bbox) {
    // State: [cx, cy, aspect, h, vx, vy, vaspect, vh]
    // Measurement: [cx, cy, aspect, h]
    cv::KalmanFilter kf(8, 4, 0, CV_32F);
    cv::setIdentity(kf.transitionMatrix);
    for (int i = 0; i < 4; ++i) {
        kf.transitionMatrix.at<float>(i, i + 4) = 1.f;
    }

    kf.measurementMatrix = cv::Mat::zeros(4, 8, CV_32F);
    for (int i = 0; i < 4; ++i) {
        kf.measurementMatrix.at<float>(i, i) = 1.f;
    }

    cv::setIdentity(kf.processNoiseCov, cv::Scalar(1e-2));
    cv::setIdentity(kf.measurementNoiseCov, cv::Scalar(1e-1));
    cv::setIdentity(kf.errorCovPost, cv::Scalar(1.f));

    const auto m = bboxToMeasurement(bbox);
    kf.statePost = cv::Mat::zeros(8, 1, CV_32F);
    for (int i = 0; i < 4; ++i) {
        kf.statePost.at<float>(i) = m[i];
    }
    return kf;
}

cv::Rect Tracker::predictBbox(cv::KalmanFilter& kf) {
    const cv::Mat pred = kf.predict();
    return measurementToBbox({pred.at<float>(0), pred.at<float>(1), pred.at<float>(2), pred.at<float>(3)});
}

void Tracker::correctKalman(cv::KalmanFilter& kf, const cv::Rect& bbox) {
    const auto m = bboxToMeasurement(bbox);
    cv::Mat measurement(4, 1, CV_32F);
    for (int i = 0; i < 4; ++i) {
        measurement.at<float>(i) = m[i];
    }
    kf.correct(measurement);
}

std::vector<std::pair<int, int>> Tracker::hungarianMinCost(const std::vector<std::vector<float>>& cost) {
    const int n = static_cast<int>(cost.size());
    if (n == 0) return {};
    const int m = static_cast<int>(cost.front().size());
    if (m == 0) return {};

    // Hungarian implementation for rectangular matrix with n <= m. If rows > cols,
    // transpose and convert result back.
    if (n > m) {
        std::vector<std::vector<float>> transposed(m, std::vector<float>(n, 0.f));
        for (int i = 0; i < n; ++i) {
            for (int j = 0; j < m; ++j) transposed[j][i] = cost[i][j];
        }
        auto pairs = hungarianMinCost(transposed);
        std::vector<std::pair<int, int>> restored;
        restored.reserve(pairs.size());
        for (const auto& p : pairs) restored.push_back({p.second, p.first});
        return restored;
    }

    std::vector<float> u(n + 1, 0.f), v(m + 1, 0.f);
    std::vector<int> p(m + 1, 0), way(m + 1, 0);

    for (int i = 1; i <= n; ++i) {
        p[0] = i;
        int j0 = 0;
        std::vector<float> minv(m + 1, std::numeric_limits<float>::max());
        std::vector<char> used(m + 1, false);
        do {
            used[j0] = true;
            const int i0 = p[j0];
            float delta = std::numeric_limits<float>::max();
            int j1 = 0;
            for (int j = 1; j <= m; ++j) {
                if (used[j]) continue;
                const float cur = cost[i0 - 1][j - 1] - u[i0] - v[j];
                if (cur < minv[j]) {
                    minv[j] = cur;
                    way[j] = j0;
                }
                if (minv[j] < delta) {
                    delta = minv[j];
                    j1 = j;
                }
            }
            for (int j = 0; j <= m; ++j) {
                if (used[j]) {
                    u[p[j]] += delta;
                    v[j] -= delta;
                } else {
                    minv[j] -= delta;
                }
            }
            j0 = j1;
        } while (p[j0] != 0);

        do {
            const int j1 = way[j0];
            p[j0] = p[j1];
            j0 = j1;
        } while (j0 != 0);
    }

    std::vector<std::pair<int, int>> result;
    result.reserve(n);
    for (int j = 1; j <= m; ++j) {
        if (p[j] != 0) result.push_back({p[j] - 1, j - 1});
    }
    return result;
}

Tracker::MatchResult Tracker::associate(const std::vector<int>& track_ids,
                                         const std::vector<int>& detection_indices,
                                         const std::unordered_map<int, TrackState>& tracks,
                                         const std::vector<Detection>& detections,
                                         float min_iou) {
    MatchResult result;
    result.unmatched_tracks = track_ids;
    result.unmatched_detections = detection_indices;
    if (track_ids.empty() || detection_indices.empty()) return result;

    std::vector<std::vector<float>> cost(track_ids.size(), std::vector<float>(detection_indices.size(), kInfCost));
    for (size_t i = 0; i < track_ids.size(); ++i) {
        const auto tr_it = tracks.find(track_ids[i]);
        if (tr_it == tracks.end()) continue;
        for (size_t j = 0; j < detection_indices.size(); ++j) {
            const auto& det = detections[detection_indices[j]];
            const auto& obj = tr_it->second.object;
            if (obj.class_id != det.class_id) continue;
            const float ov = iou(obj.bbox, det.bbox);
            if (ov >= min_iou) cost[i][j] = 1.f - ov;
        }
    }

    const auto assignment = hungarianMinCost(cost);
    std::unordered_set<int> matched_track_rows;
    std::unordered_set<int> matched_detection_cols;
    for (const auto& [row, col] : assignment) {
        if (row < 0 || col < 0 || row >= static_cast<int>(track_ids.size()) ||
            col >= static_cast<int>(detection_indices.size())) {
            continue;
        }
        if (cost[row][col] >= kInfCost * 0.5f) continue;
        result.matches.push_back({track_ids[row], detection_indices[col]});
        matched_track_rows.insert(row);
        matched_detection_cols.insert(col);
    }

    result.unmatched_tracks.clear();
    for (int i = 0; i < static_cast<int>(track_ids.size()); ++i) {
        if (!matched_track_rows.count(i)) result.unmatched_tracks.push_back(track_ids[i]);
    }
    result.unmatched_detections.clear();
    for (int i = 0; i < static_cast<int>(detection_indices.size()); ++i) {
        if (!matched_detection_cols.count(i)) result.unmatched_detections.push_back(detection_indices[i]);
    }
    return result;
}

void Tracker::activateTrack(const Detection& det) {
    TrackState state;
    state.object.track_id = next_id_++;
    state.object.class_id = det.class_id;
    state.object.label = det.label;
    state.object.bbox = det.bbox;
    state.object.confidence = det.confidence;
    state.object.age = 1;
    state.object.lost = 0;
    appendTrajectory(state.object, centerOf(det.bbox));
    state.kf = createKalman(det.bbox);
    state.state = (min_hits_ <= 1) ? TrackLifecycle::Tracked : TrackLifecycle::Tentative;
    state.hits = 1;
    state.frame_id = frame_id_;
    state.start_frame = frame_id_;
    tracks_[state.object.track_id] = std::move(state);
}

void Tracker::updateTrack(TrackState& track, const Detection& det) {
    correctKalman(track.kf, det.bbox);
    track.object.class_id = det.class_id;
    track.object.label = det.label;
    track.object.bbox = det.bbox;
    track.object.confidence = det.confidence;
    track.object.age += 1;
    track.object.lost = 0;
    appendTrajectory(track.object, centerOf(det.bbox));
    track.hits += 1;
    track.frame_id = frame_id_;
    if (track.state == TrackLifecycle::Tentative && track.hits >= std::max(1, min_hits_)) {
        track.state = TrackLifecycle::Tracked;
    } else if (track.state == TrackLifecycle::Lost) {
        track.state = TrackLifecycle::Tracked;
    }
}

void Tracker::markLostOrRemoved(TrackState& track) {
    if (track.state == TrackLifecycle::Tentative) {
        track.state = TrackLifecycle::Removed;
        return;
    }
    track.object.lost += 1;
    track.object.velocity = {0.f, 0.f};
    if (track.object.lost > max_lost_frames_) {
        track.state = TrackLifecycle::Removed;
    } else {
        track.state = TrackLifecycle::Lost;
    }
}

void Tracker::pruneRemoved() {
    std::vector<int> to_remove;
    for (const auto& [id, track] : tracks_) {
        if (track.state == TrackLifecycle::Removed) to_remove.push_back(id);
    }
    for (int id : to_remove) tracks_.erase(id);
}

std::vector<TrackedObject> Tracker::update(const std::vector<Detection>& detections) {
    frame_id_++;

    // 1. Kalman predict all existing non-removed tracks before association.
    std::vector<int> candidate_tracks;
    candidate_tracks.reserve(tracks_.size());
    for (auto& [id, track] : tracks_) {
        if (track.state == TrackLifecycle::Removed) continue;
        track.object.bbox = predictBbox(track.kf);
        candidate_tracks.push_back(id);
    }

    // 2. ByteTrack detection split: high-score detections first, then low-score
    // detections only for recovering unmatched active tracks.
    std::vector<int> high_dets;
    std::vector<int> low_dets;
    high_dets.reserve(detections.size());
    low_dets.reserve(detections.size());
    for (int i = 0; i < static_cast<int>(detections.size()); ++i) {
        if (detections[i].confidence >= high_conf_threshold_) {
            high_dets.push_back(i);
        } else if (detections[i].confidence >= low_conf_threshold_) {
            low_dets.push_back(i);
        }
    }

    // 3. First association: tracked/lost tracks with high-score detections.
    auto first = associate(candidate_tracks, high_dets, tracks_, detections, iou_threshold_);
    std::unordered_set<int> matched_tracks;
    std::unordered_set<int> matched_dets;
    for (const auto& [track_id, det_idx] : first.matches) {
        auto it = tracks_.find(track_id);
        if (it == tracks_.end()) continue;
        updateTrack(it->second, detections[det_idx]);
        matched_tracks.insert(track_id);
        matched_dets.insert(det_idx);
    }

    // 4. Second association: still-active unmatched tracks with low-score detections.
    // Lost tracks are not matched to low-score boxes to avoid false reactivation.
    std::vector<int> active_unmatched;
    for (int track_id : first.unmatched_tracks) {
        const auto it = tracks_.find(track_id);
        if (it != tracks_.end() && it->second.state == TrackLifecycle::Tracked) {
            active_unmatched.push_back(track_id);
        }
    }
    auto second = associate(active_unmatched, low_dets, tracks_, detections, low_match_iou_threshold_);
    for (const auto& [track_id, det_idx] : second.matches) {
        auto it = tracks_.find(track_id);
        if (it == tracks_.end()) continue;
        updateTrack(it->second, detections[det_idx]);
        matched_tracks.insert(track_id);
        matched_dets.insert(det_idx);
    }

    // 5. Unmatched tracks become lost/removed.
    std::unordered_set<int> second_unmatched_set(second.unmatched_tracks.begin(), second.unmatched_tracks.end());
    for (int track_id : first.unmatched_tracks) {
        if (matched_tracks.count(track_id)) continue;
        // If this active track entered second matching and was not matched, mark it now.
        // Lost/tentative tracks from first matching are also handled here.
        auto it = tracks_.find(track_id);
        if (it == tracks_.end()) continue;
        markLostOrRemoved(it->second);
    }

    // 6. 未匹配的高置信度检测创建轨迹；低置信度检测不创建轨迹。
    for (int det_idx : first.unmatched_detections) {
        if (matched_dets.count(det_idx)) continue;
        const auto& det = detections[det_idx];
        if (det.confidence >= new_track_threshold_) {
            activateTrack(det);
        }
    }

    pruneRemoved();

    // 7. Return confirmed targets. Besides current-frame matches, expose a
    // short lost window backed by Kalman prediction so downstream multi-frame
    // logic can bridge one/two-frame detector misses without treating them as
    // new targets. Longer lost tracks remain hidden to avoid stale false alarms.
    std::vector<TrackedObject> results;
    for (const auto& [_, track] : tracks_) {
        const bool current_confirmed = track.state == TrackLifecycle::Tracked && track.object.lost == 0;
        const bool short_lost_confirmed = track.state == TrackLifecycle::Lost &&
            track.object.lost > 0 && track.object.lost <= output_lost_frames_;
        if ((current_confirmed || short_lost_confirmed) && track.object.age >= std::max(1, min_hits_)) {
            results.push_back(track.object);
        }
    }
    std::sort(results.begin(), results.end(), [](const TrackedObject& a, const TrackedObject& b) {
        return a.track_id < b.track_id;
    });
    return results;
}

void Tracker::reset() {
    tracks_.clear();
    next_id_ = 0;
    frame_id_ = 0;
}

}  // namespace sentinel
