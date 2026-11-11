#include <opencv2/imgcodecs.hpp>
#include <opencv2/imgproc.hpp>

extern "C" {
#include <libavcodec/avcodec.h>
#include <libavutil/imgutils.h>
#include <libswscale/swscale.h>
}

#include <fstream>
#include <iostream>
#include <iterator>
#include <string>
#include <vector>

int main(int argc, char** argv) {
    if (argc != 3) {
        std::cerr << "usage: decode_hevc_last_frame INPUT.h265 OUTPUT.jpg\n";
        return 2;
    }
    std::ifstream input(argv[1], std::ios::binary);
    std::vector<uint8_t> bytes((std::istreambuf_iterator<char>(input)), {});
    if (bytes.empty()) {
        std::cerr << "empty input\n";
        return 3;
    }

    const AVCodec* codec = avcodec_find_decoder(AV_CODEC_ID_HEVC);
    AVCodecParserContext* parser = av_parser_init(codec->id);
    AVCodecContext* context = avcodec_alloc_context3(codec);
    AVPacket* packet = av_packet_alloc();
    AVFrame* frame = av_frame_alloc();
    if (!codec || !parser || !context || !packet || !frame || avcodec_open2(context, codec, nullptr) < 0) {
        std::cerr << "failed to initialize HEVC software decoder\n";
        return 4;
    }

    cv::Mat last_bgr;
    SwsContext* scaler = nullptr;
    auto receive_frames = [&]() {
        while (avcodec_receive_frame(context, frame) == 0) {
            last_bgr.create(frame->height, frame->width, CV_8UC3);
            scaler = sws_getCachedContext(scaler, frame->width, frame->height,
                                          static_cast<AVPixelFormat>(frame->format),
                                          frame->width, frame->height, AV_PIX_FMT_BGR24,
                                          SWS_BILINEAR, nullptr, nullptr, nullptr);
            uint8_t* dst[] = {last_bgr.data};
            int stride[] = {static_cast<int>(last_bgr.step)};
            sws_scale(scaler, frame->data, frame->linesize, 0, frame->height, dst, stride);
        }
    };

    uint8_t* cursor = bytes.data();
    int remaining = static_cast<int>(bytes.size());
    while (remaining > 0) {
        int consumed = av_parser_parse2(parser, context, &packet->data, &packet->size,
                                        cursor, remaining, AV_NOPTS_VALUE, AV_NOPTS_VALUE, 0);
        if (consumed < 0) break;
        cursor += consumed;
        remaining -= consumed;
        if (packet->size > 0 && avcodec_send_packet(context, packet) == 0) receive_frames();
    }
    avcodec_send_packet(context, nullptr);
    receive_frames();

    bool ok = !last_bgr.empty() && cv::imwrite(argv[2], last_bgr, {cv::IMWRITE_JPEG_QUALITY, 94});
    sws_freeContext(scaler);
    av_frame_free(&frame);
    av_packet_free(&packet);
    avcodec_free_context(&context);
    av_parser_close(parser);
    if (!ok) {
        std::cerr << "no decoded frame\n";
        return 5;
    }
    return 0;
}
