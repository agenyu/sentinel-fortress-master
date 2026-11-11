#include "detector.h"

#include <algorithm>
#include <array>
#include <cctype>
#include <cmath>
#include <fstream>
#include <iostream>
#include <limits>
#include <numeric>
#include <unordered_map>
#include <utility>

#if __has_include(<NvInfer.h>) && __has_include(<cuda_runtime_api.h>)
#define SENTINEL_HAS_TENSORRT 1
#include <NvInfer.h>
#include <cuda_runtime_api.h>
#else
#define SENTINEL_HAS_TENSORRT 0
#endif

namespace sentinel {

namespace {

constexpr float kMeanR = 0.485f;
constexpr float kMeanG = 0.456f;
constexpr float kMeanB = 0.406f;
constexpr float kStdR = 0.229f;
constexpr float kStdG = 0.224f;
constexpr float kStdB = 0.225f;

std::string normalizeModelType(std::string model_type) {
    std::transform(model_type.begin(), model_type.end(), model_type.begin(),
                   [](unsigned char ch) { return static_cast<char>(std::tolower(ch)); });
    if (model_type == "rf-detr" || model_type == "rf_detr" || model_type == "rfdetr_base") {
        return "rfdetr";
    }
    if (model_type == "yolov5" || model_type == "yolov8" || model_type == "yolov8n") {
        return "yolo";
    }
    return model_type;
}

const std::vector<std::string>& modelLabels() {
    static const std::vector<std::string> labels = {
        "person", "bicycle", "car", "motorcycle", "airplane", "bus", "train", "truck", "boat", "traffic light",
        "fire hydrant", "stop sign", "parking meter", "bench", "bird", "cat", "dog", "horse", "sheep", "cow",
        "elephant", "bear", "zebra", "giraffe", "backpack", "umbrella", "handbag", "tie", "suitcase", "frisbee",
        "skis", "snowboard", "sports ball", "kite", "baseball bat", "baseball glove", "skateboard", "surfboard", "tennis racket", "bottle",
        "wine glass", "cup", "fork", "knife", "spoon", "bowl", "banana", "apple", "sandwich", "orange",
        "broccoli", "carrot", "hot dog", "pizza", "donut", "cake", "chair", "couch", "potted plant", "bed",
        "dining table", "toilet", "tv", "laptop", "mouse", "remote", "keyboard", "cell phone", "microwave", "oven",
        "toaster", "sink", "refrigerator", "book", "clock", "vase", "scissors", "teddy bear", "hair drier", "toothbrush",
        "wolf", "tiger", "leopard"
    };
    return labels;
}

std::string normalizedLabel(int class_id) {
    const auto& labels = modelLabels();
    if (class_id < 0 || class_id >= static_cast<int>(labels.size())) return "unknown";

    const std::string& raw = labels[class_id];
    if (raw == "person") return "person";
    if (raw == "bicycle") return "bicycle";
    if (raw == "motorcycle") return "motorcycle";  // 含多数电动车检测框
    if (raw == "car" || raw == "bus" || raw == "truck") return "vehicle";
    if (raw == "horse" || raw == "sheep" || raw == "cow" || raw == "dog" || raw == "cat" ||
        raw == "bird" || raw == "elephant" || raw == "bear" || raw == "zebra" || raw == "giraffe" ||
        raw == "wolf" || raw == "tiger" || raw == "leopard") return "animal";
    if (raw == "sports ball") return "ball";
    return raw;
}

// 已放开 bicycle(1) / motorcycle(3，含电动车)；其它噪声类仍可按需过滤。
bool isFilteredDetectionClass(int /*class_id*/) {
    return false;
}

cv::Rect rectFromCxCyWh(float cx, float cy, float w, float h, const cv::Size& image_size) {
    const bool normalized = std::max({std::abs(cx), std::abs(cy), std::abs(w), std::abs(h)}) <= 2.0f;
    if (normalized) {
        cx *= static_cast<float>(image_size.width);
        w *= static_cast<float>(image_size.width);
        cy *= static_cast<float>(image_size.height);
        h *= static_cast<float>(image_size.height);
    }
    float x1 = cx - w * 0.5f;
    float y1 = cy - h * 0.5f;
    float x2 = cx + w * 0.5f;
    float y2 = cy + h * 0.5f;
    x1 = std::clamp(x1, 0.f, static_cast<float>(image_size.width - 1));
    y1 = std::clamp(y1, 0.f, static_cast<float>(image_size.height - 1));
    x2 = std::clamp(x2, 0.f, static_cast<float>(image_size.width - 1));
    y2 = std::clamp(y2, 0.f, static_cast<float>(image_size.height - 1));
    return cv::Rect(cv::Point(static_cast<int>(std::round(x1)), static_cast<int>(std::round(y1))),
                    cv::Point(static_cast<int>(std::round(x2)), static_cast<int>(std::round(y2))));
}

cv::Rect rectFromXyXy(float x1, float y1, float x2, float y2, const cv::Size& image_size) {
    const bool normalized = std::max({std::abs(x1), std::abs(y1), std::abs(x2), std::abs(y2)}) <= 2.0f;
    if (normalized) {
        x1 *= static_cast<float>(image_size.width);
        x2 *= static_cast<float>(image_size.width);
        y1 *= static_cast<float>(image_size.height);
        y2 *= static_cast<float>(image_size.height);
    }
    x1 = std::clamp(x1, 0.f, static_cast<float>(image_size.width - 1));
    y1 = std::clamp(y1, 0.f, static_cast<float>(image_size.height - 1));
    x2 = std::clamp(x2, 0.f, static_cast<float>(image_size.width - 1));
    y2 = std::clamp(y2, 0.f, static_cast<float>(image_size.height - 1));
    return cv::Rect(cv::Point(static_cast<int>(std::round(x1)), static_cast<int>(std::round(y1))),
                    cv::Point(static_cast<int>(std::round(x2)), static_cast<int>(std::round(y2))));
}

cv::Rect rectFromCxCyWhModel(float cx, float cy, float w, float h,
                             const cv::Size& image_size, int input_w, int input_h) {
    const bool normalized = std::max({std::abs(cx), std::abs(cy), std::abs(w), std::abs(h)}) <= 2.0f;
    if (normalized) {
        return rectFromCxCyWh(cx, cy, w, h, image_size);
    }
    const float sx = image_size.width / std::max(1.0f, static_cast<float>(input_w));
    const float sy = image_size.height / std::max(1.0f, static_cast<float>(input_h));
    return rectFromCxCyWh(cx * sx, cy * sy, w * sx, h * sy, image_size);
}

cv::Rect rectFromXyXyModel(float x1, float y1, float x2, float y2,
                           const cv::Size& image_size, int input_w, int input_h) {
    const bool normalized = std::max({std::abs(x1), std::abs(y1), std::abs(x2), std::abs(y2)}) <= 2.0f;
    if (normalized) {
        return rectFromXyXy(x1, y1, x2, y2, image_size);
    }
    const float sx = image_size.width / std::max(1.0f, static_cast<float>(input_w));
    const float sy = image_size.height / std::max(1.0f, static_cast<float>(input_h));
    return rectFromXyXy(x1 * sx, y1 * sy, x2 * sx, y2 * sy, image_size);
}

float rectIou(const cv::Rect& a, const cv::Rect& b) {
    const int inter_area = (a & b).area();
    const int union_area = a.area() + b.area() - inter_area;
    if (union_area <= 0) return 0.0f;
    return static_cast<float>(inter_area) / static_cast<float>(union_area);
}

float probabilityFromModelValue(float value) {
    if (std::isfinite(value) && value >= 0.0f && value <= 1.0f) {
        return value;
    }
    if (value >= 20.0f) return 1.0f;
    if (value <= -20.0f) return 0.0f;
    return 1.0f / (1.0f + std::exp(-value));
}

int mapRfDetrClassIndex(int raw_class, int class_count) {
    const int label_count = static_cast<int>(modelLabels().size());
    if (raw_class < 0) return -1;

    // RF-DETR Base COCO uses the original sparse COCO category IDs as its
    // 91 logit slots. Downstream code uses contiguous 0..79 IDs
    // (person=0, car=2), so map explicitly instead of subtracting one.
    static constexpr std::array<int, 80> kCocoCategoryIds = {
         1,  2,  3,  4,  5,  6,  7,  8,  9, 10,
        11, 13, 14, 15, 16, 17, 18, 19, 20, 21,
        22, 23, 24, 25, 27, 28, 31, 32, 33, 34,
        35, 36, 37, 38, 39, 40, 41, 42, 43, 44,
        46, 47, 48, 49, 50, 51, 52, 53, 54, 55,
        56, 57, 58, 59, 60, 61, 62, 63, 64, 65,
        67, 70, 72, 73, 74, 75, 76, 77, 78, 79,
        80, 81, 82, 84, 85, 86, 87, 88, 89, 90
    };
    if (class_count == 91) {
        const auto it = std::find(kCocoCategoryIds.begin(), kCocoCategoryIds.end(), raw_class);
        return it == kCocoCategoryIds.end()
            ? -1
            : static_cast<int>(std::distance(kCocoCategoryIds.begin(), it));
    }

    // Fine-tuned exports normally use contiguous foreground IDs and may
    // append one background/no-object slot.
    if ((class_count == label_count || class_count == label_count + 1) && raw_class < label_count) {
        return raw_class;
    }
    return -1;
}

cv::Mat resizeDirect(const cv::Mat& bgr, int input_w, int input_h) {
    cv::Mat resized;
    cv::resize(bgr, resized, cv::Size(input_w, input_h));
    return resized;
}

#if SENTINEL_HAS_TENSORRT

class TrtLogger final : public nvinfer1::ILogger {
public:
    void log(Severity severity, const char* msg) noexcept override {
        if (severity <= Severity::kWARNING) {
            std::cerr << "[TensorRT] " << msg << std::endl;
        }
    }
};

size_t dataTypeSize(nvinfer1::DataType type) {
    switch (type) {
        case nvinfer1::DataType::kFLOAT: return 4;
        case nvinfer1::DataType::kHALF:  return 2;
        case nvinfer1::DataType::kINT8:  return 1;
        case nvinfer1::DataType::kINT32: return 4;
        case nvinfer1::DataType::kBOOL:  return 1;
        default: return 4;
    }
}

int64_t volume(const nvinfer1::Dims& dims) {
    int64_t v = 1;
    for (int i = 0; i < dims.nbDims; ++i) v *= std::max(static_cast<int64_t>(1), static_cast<int64_t>(dims.d[i]));
    return v;
}

#endif

}  // namespace

struct Detector::Impl {
#if SENTINEL_HAS_TENSORRT
    TrtLogger logger;
    nvinfer1::IRuntime* runtime{nullptr};
    nvinfer1::ICudaEngine* engine{nullptr};
    nvinfer1::IExecutionContext* context{nullptr};
    cudaStream_t stream{nullptr};
    std::vector<void*> buffers;
    std::vector<size_t> buffer_sizes;
    std::vector<int> output_indices;
    int input_index{-1};
    int input_c{3};
    int input_h{560};
    int input_w{560};

    std::string input_name;

    ~Impl() { release(); }

    void release() {
        for (void* p : buffers) {
            if (p) cudaFree(p);
        }
        buffers.clear();
        buffer_sizes.clear();
        if (stream) { cudaStreamDestroy(stream); stream = nullptr; }
#if NV_TENSORRT_MAJOR >= 10
        if (context) { delete context; context = nullptr; }
        if (engine) { delete engine; engine = nullptr; }
        if (runtime) { delete runtime; runtime = nullptr; }
#else
        if (context) { context->destroy(); context = nullptr; }
        if (engine) { engine->destroy(); engine = nullptr; }
        if (runtime) { runtime->destroy(); runtime = nullptr; }
#endif
    }
#else
    void release() {}
#endif
};

Detector::Detector() : impl_(std::make_unique<Impl>()) {}
Detector::~Detector() = default;

bool Detector::init(const std::string& engine_path,
                    float conf_threshold, float nms_threshold,
                    const std::string& model_type)
{
    engine_path_    = engine_path;
    conf_threshold_ = conf_threshold;
    nms_threshold_  = nms_threshold;
    model_type_     = normalizeModelType(model_type);
    initialized_    = false;

#if !SENTINEL_HAS_TENSORRT
        std::cerr << "[Detector] TensorRT/CUDA headers are not available in this build; "
              << "detector inference disabled. engine=" << engine_path
              << " model_type=" << model_type_ << std::endl;
    return false;
#else
    if (model_type_ != "rfdetr" && model_type_ != "yolo") {
        std::cerr << "[Detector] Unsupported detector_model_type=" << model_type
                  << "; expected rfdetr or yolo." << std::endl;
        return false;
    }
    int cuda_device_count = 0;
    const cudaError_t cuda_status = cudaGetDeviceCount(&cuda_device_count);
    std::cout << "[Detector][Init] cudaGetDeviceCount status="
              << static_cast<int>(cuda_status)
              << " device_count=" << cuda_device_count << std::endl;
    if (cuda_status != cudaSuccess || cuda_device_count <= 0) {
        std::cerr << "[Detector] CUDA device unavailable; TensorRT initialization skipped: "
                  << cudaGetErrorString(cuda_status)
                  << " device_count=" << cuda_device_count << std::endl;
        // Clear the runtime's sticky error so shutdown/cleanup paths do not
        // report this expected startup failure as a later CUDA operation error.
        (void)cudaGetLastError();
        return false;
    }

    std::ifstream file(engine_path, std::ios::binary | std::ios::ate);
    if (!file) {
        std::cerr << "[Detector] Failed to open TensorRT engine: " << engine_path << std::endl;
        return false;
    }

    const std::streamsize size = file.tellg();
    std::cout << "[Detector][Init] engine_size=" << size << std::endl;
    file.seekg(0, std::ios::beg);
    std::vector<char> engine_data(static_cast<size_t>(size));
    if (!file.read(engine_data.data(), size)) {
        std::cerr << "[Detector] Failed to read TensorRT engine: " << engine_path << std::endl;
        return false;
    }

    impl_->release();
    impl_->runtime = nvinfer1::createInferRuntime(impl_->logger);
    std::cout << "[Detector][Init] runtime=" << (impl_->runtime ? 1 : 0) << std::endl;
    if (!impl_->runtime) {
        std::cerr << "[Detector] createInferRuntime failed" << std::endl;
        return false;
    }

    impl_->engine = impl_->runtime->deserializeCudaEngine(engine_data.data(), engine_data.size());
    std::cout << "[Detector][Init] engine=" << (impl_->engine ? 1 : 0) << std::endl;
    if (!impl_->engine) {
        std::cerr << "[Detector] deserializeCudaEngine failed" << std::endl;
        return false;
    }

    impl_->context = impl_->engine->createExecutionContext();
    std::cout << "[Detector][Init] context=" << (impl_->context ? 1 : 0) << std::endl;
    if (!impl_->context) {
        std::cerr << "[Detector] createExecutionContext failed" << std::endl;
        return false;
    }

#if NV_TENSORRT_MAJOR >= 10
    const int nb = impl_->engine->getNbIOTensors();
#else
    const int nb = impl_->engine->getNbBindings();
#endif
    std::cout << "[Detector][Init] bindings=" << nb << std::endl;
    impl_->buffers.assign(nb, nullptr);
    impl_->buffer_sizes.assign(nb, 0);
    impl_->output_indices.clear();

    for (int i = 0; i < nb; ++i) {
#if NV_TENSORRT_MAJOR >= 10
        const char* name = impl_->engine->getIOTensorName(i);
        if (impl_->engine->getTensorIOMode(name) == nvinfer1::TensorIOMode::kINPUT) {
#else
        const char* name = impl_->engine->getBindingName(i);
        if (impl_->engine->bindingIsInput(i)) {
#endif
            impl_->input_index = i;
            impl_->input_name = name;
        } else {
            impl_->output_indices.push_back(i);
        }
    }
    if (impl_->input_index < 0 || impl_->output_indices.empty()) {
        std::cerr << "[Detector] Engine must have at least one input and one output" << std::endl;
        return false;
    }

#if NV_TENSORRT_MAJOR >= 10
    auto input_dims = impl_->engine->getTensorShape(impl_->input_name.c_str());
#else
    auto input_dims = impl_->engine->getBindingDimensions(impl_->input_index);
#endif
    if (input_dims.nbDims == 4) {
        if (input_dims.d[0] < 0) input_dims.d[0] = 1;
        if (input_dims.d[1] < 0) input_dims.d[1] = 3;
        if (input_dims.d[2] < 0) input_dims.d[2] = 560;
        if (input_dims.d[3] < 0) input_dims.d[3] = 560;
#if NV_TENSORRT_MAJOR >= 10
        impl_->context->setInputShape(impl_->input_name.c_str(), input_dims);
#else
        impl_->context->setBindingDimensions(impl_->input_index, input_dims);
#endif
        impl_->input_c = input_dims.d[1];
        impl_->input_h = input_dims.d[2];
        impl_->input_w = input_dims.d[3];
    } else if (input_dims.nbDims == 3) {
        if (input_dims.d[0] < 0) input_dims.d[0] = 3;
        if (input_dims.d[1] < 0) input_dims.d[1] = 560;
        if (input_dims.d[2] < 0) input_dims.d[2] = 560;
#if NV_TENSORRT_MAJOR >= 10
        impl_->context->setInputShape(impl_->input_name.c_str(), input_dims);
#else
        impl_->context->setBindingDimensions(impl_->input_index, input_dims);
#endif
        impl_->input_c = input_dims.d[0];
        impl_->input_h = input_dims.d[1];
        impl_->input_w = input_dims.d[2];
    } else {
        std::cerr << "[Detector] Unsupported input dims: nbDims=" << input_dims.nbDims << std::endl;
        return false;
    }

#if NV_TENSORRT_MAJOR >= 10
    const auto input_type = impl_->engine->getTensorDataType(impl_->input_name.c_str());
#else
    const auto input_type = impl_->engine->getBindingDataType(impl_->input_index);
#endif
    std::cout << "[Detector][Init] input_name=" << impl_->input_name
              << " input_index=" << impl_->input_index
              << " input_type=" << static_cast<int>(input_type)
              << " input_dims=" << impl_->input_w << "x" << impl_->input_h
              << " c=" << impl_->input_c << std::endl;
    if (input_type != nvinfer1::DataType::kFLOAT) {
        std::cerr << "[Detector] Only FP32 input bindings are supported by this runtime wrapper. "
                  << "Please export the engine with FP32 input." << std::endl;
        return false;
    }

    for (int i = 0; i < nb; ++i) {
#if NV_TENSORRT_MAJOR >= 10
        const char* tname = impl_->engine->getIOTensorName(i);
        nvinfer1::Dims dims = impl_->engine->getTensorShape(tname);
        const size_t bytes = static_cast<size_t>(volume(dims)) * dataTypeSize(impl_->engine->getTensorDataType(tname));
#else
        const char* tname = impl_->engine->getBindingName(i);
        nvinfer1::Dims dims = impl_->context->getBindingDimensions(i);
        const size_t bytes = static_cast<size_t>(volume(dims)) * dataTypeSize(impl_->engine->getBindingDataType(i));
#endif
        impl_->buffer_sizes[i] = bytes;
        if (cudaMalloc(&impl_->buffers[i], bytes) != cudaSuccess) {
            std::cerr << "[Detector] cudaMalloc failed for binding " << i << std::endl;
            return false;
        }
#if NV_TENSORRT_MAJOR >= 10
        impl_->context->setTensorAddress(tname, impl_->buffers[i]);
#else
        (void)tname;
#endif
    }

    if (cudaStreamCreate(&impl_->stream) != cudaSuccess) {
        std::cerr << "[Detector] cudaStreamCreate failed" << std::endl;
        return false;
    }

    initialized_ = true;
    std::cout << "[Detector] TensorRT detector initialized: engine=" << engine_path
              << " model_type=" << model_type_
              << " input=" << impl_->input_w << "x" << impl_->input_h
              << " conf=" << conf_threshold_
              << " outputs=" << impl_->output_indices.size()
              << " nms=" << (model_type_ == "yolo" ? nms_threshold_ : 0.0f)
              << std::endl;
    return true;
#endif
}

std::vector<Detection> Detector::detect(const cv::Mat& image) {
    if (!initialized_ || image.empty()) return {};

#if !SENTINEL_HAS_TENSORRT
    (void)image;
    return {};
#else
    cv::Mat input_bgr = resizeDirect(image, impl_->input_w, impl_->input_h);
    cv::Mat input_rgb;
    cv::cvtColor(input_bgr, input_rgb, cv::COLOR_BGR2RGB);
    input_rgb.convertTo(input_rgb, CV_32FC3, 1.0 / 255.0);

    std::vector<float> chw(static_cast<size_t>(impl_->input_c) * impl_->input_h * impl_->input_w);
    std::vector<cv::Mat> channels;
    for (int c = 0; c < impl_->input_c; ++c) {
        channels.emplace_back(impl_->input_h, impl_->input_w, CV_32FC1,
                              chw.data() + static_cast<size_t>(c) * impl_->input_h * impl_->input_w);
    }
    cv::split(input_rgb, channels);
    const size_t plane_size = static_cast<size_t>(impl_->input_h) * impl_->input_w;
    if (model_type_ == "rfdetr" && impl_->input_c >= 3) {
        float* r = chw.data();
        float* g = chw.data() + plane_size;
        float* b = chw.data() + plane_size * 2;
        for (size_t i = 0; i < plane_size; ++i) {
            r[i] = (r[i] - kMeanR) / kStdR;
            g[i] = (g[i] - kMeanG) / kStdG;
            b[i] = (b[i] - kMeanB) / kStdB;
        }
    }

    const size_t input_bytes = chw.size() * sizeof(float);
    if (input_bytes > impl_->buffer_sizes[impl_->input_index]) {
        std::cerr << "[Detector] Input buffer too small" << std::endl;
        return {};
    }

    cudaMemcpyAsync(impl_->buffers[impl_->input_index], chw.data(), input_bytes,
                    cudaMemcpyHostToDevice, impl_->stream);
#if NV_TENSORRT_MAJOR >= 10
    const bool enqueue_ok = impl_->context->enqueueV3(impl_->stream);
#else
    const bool enqueue_ok = impl_->context->enqueueV2(impl_->buffers.data(), impl_->stream, nullptr);
#endif
    if (!enqueue_ok) {
        std::cerr << "[Detector] TensorRT enqueue failed" << std::endl;
        return {};
    }

    struct OutputTensor {
        std::string name;
        nvinfer1::Dims dims{};
        std::vector<float> data;
    };
    std::vector<OutputTensor> outputs;
    outputs.reserve(impl_->output_indices.size());
    for (int out_idx : impl_->output_indices) {
#if NV_TENSORRT_MAJOR >= 10
        const char* out_name = impl_->engine->getIOTensorName(out_idx);
        nvinfer1::Dims out_dims = impl_->engine->getTensorShape(out_name);
        const auto out_type = impl_->engine->getTensorDataType(out_name);
#else
        const char* out_name = impl_->engine->getBindingName(out_idx);
        nvinfer1::Dims out_dims = impl_->context->getBindingDimensions(out_idx);
        const auto out_type = impl_->engine->getBindingDataType(out_idx);
#endif
        if (out_type != nvinfer1::DataType::kFLOAT) {
            std::cerr << "[Detector] RF-DETR output " << out_name
                      << " is not FP32; unsupported by current wrapper." << std::endl;
            return {};
        }
        const int64_t out_count = volume(out_dims);
        OutputTensor out;
        out.name = out_name ? out_name : "";
        out.dims = out_dims;
        out.data.resize(static_cast<size_t>(out_count));
        cudaMemcpyAsync(out.data.data(), impl_->buffers[out_idx], out.data.size() * sizeof(float),
                        cudaMemcpyDeviceToHost, impl_->stream);
        outputs.push_back(std::move(out));
    }
    cudaStreamSynchronize(impl_->stream);

    if (outputs.empty()) return {};

    const OutputTensor* boxes_tensor = nullptr;
    const OutputTensor* score_tensor = nullptr;
    const OutputTensor* packed_tensor = nullptr;
    for (const auto& out : outputs) {
        if (out.data.empty() || out.dims.nbDims <= 0) continue;
        const int last_dim = out.dims.d[out.dims.nbDims - 1];
        if (last_dim == 4 && !boxes_tensor) {
            boxes_tensor = &out;
        } else if (last_dim >= 5 && !packed_tensor) {
            packed_tensor = &out;
        } else if (!score_tensor) {
            score_tensor = &out;
        }
    }
    if (boxes_tensor) {
        for (const auto& out : outputs) {
            if (&out == boxes_tensor) continue;
            if (!score_tensor || out.data.size() > score_tensor->data.size()) {
                score_tensor = &out;
            }
        }
    }

    std::vector<Detection> results;
    const float effective_conf_threshold = std::max(0.0f, conf_threshold_);

    if (model_type_ == "yolo") {
        struct Candidate {
            Detection det;
            int class_id{-1};
        };
        std::vector<Candidate> candidates;
        for (const auto& out : outputs) {
            if (out.data.empty() || out.dims.nbDims <= 0) continue;

            std::vector<int> dims;
            dims.reserve(static_cast<size_t>(out.dims.nbDims));
            for (int i = 0; i < out.dims.nbDims; ++i) {
                if (out.dims.d[i] > 0) dims.push_back(out.dims.d[i]);
            }
            if (!dims.empty() && dims.front() == 1) dims.erase(dims.begin());
            if (dims.empty()) continue;

            int rows = 0;
            int attrs = 0;
            bool transposed = false;
            if (dims.size() == 1) {
                attrs = 6;
                rows = static_cast<int>(out.data.size() / attrs);
            } else {
                const int a = dims[dims.size() - 2];
                const int b = dims[dims.size() - 1];
                if (b >= 6 && b <= 256) {
                    rows = a;
                    attrs = b;
                } else if (a >= 6 && a <= 256) {
                    rows = b;
                    attrs = a;
                    transposed = true;
                }
            }
            if (rows <= 0 || attrs < 6) continue;
            if (static_cast<size_t>(rows) * static_cast<size_t>(attrs) > out.data.size()) continue;

            auto value_at = [&](int row, int attr) -> float {
                if (transposed) {
                    return out.data[static_cast<size_t>(attr) * rows + row];
                }
                return out.data[static_cast<size_t>(row) * attrs + attr];
            };

            int class_start = 4;
            float objectness = 1.0f;
            const int label_count = static_cast<int>(modelLabels().size());
            const bool has_objectness =
                attrs >= 6 && (attrs - 5 == label_count || attrs - 5 == label_count + 1 || attrs == 85);
            if (has_objectness) class_start = 5;

            for (int row = 0; row < rows; ++row) {
                if (has_objectness) {
                    objectness = probabilityFromModelValue(value_at(row, 4));
                    if (objectness < effective_conf_threshold) continue;
                }

                int class_id = -1;
                float class_score = 0.0f;
                if (attrs == 6) {
                    class_score = probabilityFromModelValue(value_at(row, 4));
                    class_id = static_cast<int>(std::round(value_at(row, 5)));
                    objectness = 1.0f;
                } else {
                    for (int attr = class_start; attr < attrs; ++attr) {
                        const int candidate_class = attr - class_start;
                        if (candidate_class >= label_count) break;
                        const float score = probabilityFromModelValue(value_at(row, attr));
                        if (score > class_score) {
                            class_score = score;
                            class_id = candidate_class;
                        }
                    }
                }

                if (class_id < 0 || class_id >= label_count || isFilteredDetectionClass(class_id)) continue;
                const float score = objectness * class_score;
                if (score < effective_conf_threshold) continue;

                const float a = value_at(row, 0);
                const float b = value_at(row, 1);
                const float c = value_at(row, 2);
                const float d4 = value_at(row, 3);
                cv::Rect box = (c > a && d4 > b && attrs == 6)
                    ? rectFromXyXyModel(a, b, c, d4, image.size(), impl_->input_w, impl_->input_h)
                    : rectFromCxCyWhModel(a, b, c, d4, image.size(), impl_->input_w, impl_->input_h);
                if (box.width <= 1 || box.height <= 1) continue;

                Detection det;
                det.class_id = class_id;
                det.confidence = score;
                det.bbox = box;
                det.label = normalizedLabel(class_id);
                candidates.push_back({std::move(det), class_id});
            }
        }

        std::sort(candidates.begin(), candidates.end(), [](const Candidate& lhs, const Candidate& rhs) {
            return lhs.det.confidence > rhs.det.confidence;
        });
        std::vector<char> suppressed(candidates.size(), 0);
        for (size_t i = 0; i < candidates.size(); ++i) {
            if (suppressed[i]) continue;
            results.push_back(candidates[i].det);
            for (size_t j = i + 1; j < candidates.size(); ++j) {
                if (suppressed[j]) continue;
                if (candidates[i].class_id != candidates[j].class_id) continue;
                if (rectIou(candidates[i].det.bbox, candidates[j].det.bbox) > nms_threshold_) {
                    suppressed[j] = 1;
                }
            }
        }
        return results;
    }

    if (boxes_tensor && score_tensor) {
        const int box_dim = 4;
        const int query_count = static_cast<int>(boxes_tensor->data.size() / box_dim);
        if (query_count <= 0 || score_tensor->data.size() % static_cast<size_t>(query_count) != 0) {
            std::cerr << "[Detector] RF-DETR output shape mismatch: boxes="
                      << boxes_tensor->data.size() << " scores=" << score_tensor->data.size()
                      << std::endl;
            return {};
        }
        const int class_count = static_cast<int>(score_tensor->data.size() / query_count);
        struct Candidate {
            float score{0.f};
            int query{-1};
            int class_id{-1};
            int flat_index{-1};
        };
        std::vector<Candidate> candidates;
        candidates.reserve(static_cast<size_t>(query_count));
        for (int q = 0; q < query_count; ++q) {
            for (int c = 0; c < class_count; ++c) {
                const int mapped = mapRfDetrClassIndex(c, class_count);
                if (mapped < 0) continue;
                const float score = probabilityFromModelValue(
                    score_tensor->data[static_cast<size_t>(q) * class_count + c]);
                if (score < effective_conf_threshold || isFilteredDetectionClass(mapped)) continue;
                candidates.push_back({score, q, mapped, q * class_count + c});
            }
        }
        std::sort(candidates.begin(), candidates.end(), [](const Candidate& lhs, const Candidate& rhs) {
            if (lhs.score != rhs.score) return lhs.score > rhs.score;
            return lhs.flat_index < rhs.flat_index;
        });
        // Match RF-DETR Base PostProcess(num_select=300): global top-k over
        // query/class pairs, not one argmax class per query.
        if (candidates.size() > static_cast<size_t>(query_count)) {
            candidates.resize(static_cast<size_t>(query_count));
        }

        results.reserve(candidates.size());
        for (const auto& candidate : candidates) {
            const size_t box_off = static_cast<size_t>(candidate.query) * box_dim;
            cv::Rect box = rectFromCxCyWh(boxes_tensor->data[box_off],
                                          boxes_tensor->data[box_off + 1],
                                          boxes_tensor->data[box_off + 2],
                                          boxes_tensor->data[box_off + 3],
                                          image.size());
            if (box.width <= 1 || box.height <= 1) continue;
            Detection d;
            d.class_id = candidate.class_id;
            d.confidence = candidate.score;
            d.bbox = box;
            d.label = normalizedLabel(d.class_id);
            results.push_back(std::move(d));
        }
        return results;
    }

    if (!packed_tensor || packed_tensor->dims.nbDims <= 0) {
        std::cerr << "[Detector] RF-DETR outputs did not contain boxes+scores or packed detections." << std::endl;
        return {};
    }

    const int attrs = packed_tensor->dims.d[packed_tensor->dims.nbDims - 1];
    if (attrs < 6) return {};
    const int rows = static_cast<int>(packed_tensor->data.size() / static_cast<size_t>(attrs));
    results.reserve(static_cast<size_t>(rows));
    for (int i = 0; i < rows; ++i) {
        const size_t off = static_cast<size_t>(i) * attrs;
        const float score = probabilityFromModelValue(packed_tensor->data[off + 4]);
        const int raw_class_id = static_cast<int>(std::round(packed_tensor->data[off + 5]));
        const int class_id = mapRfDetrClassIndex(raw_class_id, 91);
        if (score < effective_conf_threshold || class_id < 0 || isFilteredDetectionClass(class_id)) {
            continue;
        }
        const float a = packed_tensor->data[off];
        const float b = packed_tensor->data[off + 1];
        const float c = packed_tensor->data[off + 2];
        const float d4 = packed_tensor->data[off + 3];
        cv::Rect box = (c > a && d4 > b)
            ? rectFromXyXy(a, b, c, d4, image.size())
            : rectFromCxCyWh(a, b, c, d4, image.size());
        if (box.width <= 1 || box.height <= 1) continue;
        Detection d;
        d.class_id = class_id;
        d.confidence = score;
        d.bbox = box;
        d.label = normalizedLabel(d.class_id);
        results.push_back(std::move(d));
    }
    return results;
#endif
}

}  // namespace sentinel
