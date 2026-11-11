#include "orsm/tracking_safety.h"

#include <algorithm>
#include <cassert>
#include <cmath>
#include <iostream>
#include <utility>
#include <vector>

using sentinel::orsm_tracking_safety::limitTotalDisplacement;
using sentinel::orsm_tracking_safety::selectLowestScore;
using sentinel::orsm_tracking_safety::shouldTakeDropLidarAnchor;

int main() {
    std::vector<std::pair<int, float>> candidates{{11, 1.4f}, {22, 0.3f}, {33, 0.8f}};
    assert(selectLowestScore(candidates) == 22);
    std::reverse(candidates.begin(), candidates.end());
    assert(selectLowestScore(candidates) == 22);  // unordered_map iteration cannot change result

    const cv::Point3f previous{10.f, 0.f, 0.f};
    const cv::Point3f predicted_plus_correction{8.5f, 0.f, 0.f};
    const auto bounded = limitTotalDisplacement(previous, predicted_plus_correction, 0.35f);
    assert(std::abs(bounded.x - 9.65f) < 1e-4f);
    assert(std::abs(bounded.y) < 1e-4f);

    assert(shouldTakeDropLidarAnchor(100, 101));
    assert(!shouldTakeDropLidarAnchor(101, 100));
    assert(!shouldTakeDropLidarAnchor(101, 101));

    std::cout << "ORSM association, displacement and anchor safety tests passed\n";
    return 0;
}
