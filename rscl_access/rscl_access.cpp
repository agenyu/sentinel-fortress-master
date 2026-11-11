#include "rscl_access.h"



using senseAD::State;

RsclAccess::RsclAccess() {
    rt_ = senseAD::rscl::GetCurRuntime();
}

RsclAccess::~RsclAccess() {
    release();
}

void RsclAccess::init(int argc, char **argv, const std::string &name) {
    rt_ = senseAD::rscl::GetCurRuntime();
    rt_->Init(argc, argv);
    ComponentConfig cfg;
    cfg.timer_conf.set_name(name);
    Initialize(cfg);
    AD_LINFO(RsclAccess) << "Rscl Init";
}

void RsclAccess::release() {
    cuda_subscribers_.clear();
    subscribers_.clear();
    publishers_.clear();
    if (rt_) {
        // GetCurRuntime() 返回 RSCL 管理的单例，调用方不得释放。
        rt_->Shutdown();
        rt_ = nullptr;
    }
    AD_LINFO(RsclAccess) << "Rscl Shutdown";
}

bool RsclAccess::OnProc() {
    return true;
}
