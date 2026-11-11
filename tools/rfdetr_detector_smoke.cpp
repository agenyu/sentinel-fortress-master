#include "detection/detector.h"

#include <opencv2/imgcodecs.hpp>

#include <algorithm>
#include <iostream>
#include <string>

int main(int argc, char** argv) {
    if (argc != 3) {
        std::cerr << "Usage: rfdetr_detector_smoke <engine> <image>" << std::endl;
        return 2;
    }

    const std::string engine_path = argv[1];
    const std::string image_path = argv[2];
    cv::Mat image = cv::imread(image_path, cv::IMREAD_COLOR);
    if (image.empty()) {
        std::cerr << "Failed to read image: " << image_path << std::endl;
        return 3;
    }

    sentinel::Detector detector;
    if (!detector.init(engine_path, 0.35f, 0.45f)) {
        std::cerr << "Detector initialization failed" << std::endl;
        return 4;
    }

    const auto detections = detector.detect(image);
    std::cout << "detections=" << detections.size() << std::endl;
    for (const auto& detection : detections) {
        std::cout << "class_id=" << detection.class_id
                  << " label=" << detection.label
                  << " confidence=" << detection.confidence
                  << " bbox=" << detection.bbox.x << ',' << detection.bbox.y << ','
                  << detection.bbox.width << ',' << detection.bbox.height << std::endl;
    }

    const bool has_person = std::any_of(detections.begin(), detections.end(), [](const auto& detection) {
        return detection.class_id == 0 && detection.label == "person";
    });
    if (!has_person) {
        std::cerr << "Expected person detection was not produced" << std::endl;
        return 5;
    }
    return 0;
}
