#include "rscl_publisher.h"

#include <iostream>

namespace sentinel {

RsclPublisher::RsclPublisher(const PublishConfig& cfg)
    : cfg_(cfg)
{}

bool RsclPublisher::init() {
    initialized_ = false;
    if (cfg_.enable_rscl_publish) {
        std::cout << "[RsclPublisher][Diagnostic] real L3 publish unavailable; using no-op publisher"
                  << std::endl;
    } else {
        std::cout << "[RsclPublisher][Diagnostic] RSCL publish disabled" << std::endl;
    }
    return true;
}

void RsclPublisher::publishSentryEvent(const AlertEvent&,
                                       const SentinelResStatusView&,
                                       uint64_t) {}

void RsclPublisher::publishSentryStateBroadcast(const StateBroadcastView&) {}

void RsclPublisher::publishSentryAction(const AlertEvent&,
                                        const SentinelResStatusView&) {}

void RsclPublisher::publishOrsmState(const ORSMAlertOutput&, uint64_t) {}

void RsclPublisher::publishOrsmAction(const ORSMAlertOutput&, uint64_t) {}

}  // namespace sentinel
