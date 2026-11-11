


#pragma once

#include <memory>

#include <ad_rscl/component/component.h>
#include <ad_rscl/component/timer_component.h>
#include <ad_rscl/task/task.h>
#include <ad_rscl/component/component_utils.h>
#include <ad_rscl/runtime.h>
#include <ad_rscl/comm/cuda/cuda_subscriber.h>
//#include "log.h"

using senseAD::rscl::task::TimerTask;
using senseAD::rscl::component::ComponentConfig;

class RsclAccess : public senseAD::rscl::component::TimerComponent {
    DECLARE_SINGLETON(RsclAccess)
public:
    ~RsclAccess() override;

    bool OnProc() override;

    void init(int argc, char **argv, const std::string &name);

    void release();
    // ---- 新增：给外部用的 MemblockPtr 类型 ----
    // using MemblockPtr = std::shared_ptr<senseAD::rscl::comm::Memblock>;
    using MemblockPtr = std::shared_ptr<senseAD::rscl::comm::Memblock>;

    template <typename MessageT>
    static std::shared_ptr<SendMsg<MessageT>> NewMessage() {
        return std::make_shared<SendMsg<MessageT>>();
    }

    template<class M, class T>
    bool subscribe(const std::string &topic, void (T::*fp)(const std::shared_ptr<ReceivedMsg<M>> &), T *obj) {
        auto iter = subscribers_.find(topic);
        if (iter != subscribers_.end()) {
//            LOG_ERROR("Topic %s has been subscribed. ", topic.c_str());
            AD_LERROR(RsclAccess) << "Topic " << topic << " has been subscribed.";
            return false;
        }
        auto sub = InnerNode()->CreateSubscriber<M>(topic, std::bind(fp, obj, std::placeholders::_1));
        subscribers_[topic] = sub;
//        LOG_DEBUG("Subscribe topic %s success. ", topic.c_str());
        AD_LINFO(RsclAccess) << "Subscribe topic " << topic << " success.";
        return true;
    }



     /**
      * @brief 订阅 cuda Memblock 话题
      * @tparam T  回调所在类类型
      * @param topic RSCL channel_name
      * @param fp   T 成员函数: void T::OnCuda(const MemblockPtr& mb)
      * @param obj  this 指针
      */
     // ---------------- CudaSubscriber 集成（新的正确版本） ----------------
     template<class T>
     bool CudaSubscriber(const std::string &topic,
                         void (T::*fp)(const MemblockPtr &),
                         T *obj) {

         // 新增：打印当前进程的CYBER_DOMAIN_ID
         const char* domain_id = getenv("CYBER_DOMAIN_ID");
          std::cout << "订阅端CYBER_DOMAIN_ID：" << (domain_id ? domain_id : "未设置") << std::endl;

         std::cout << "开始订阅CUDA话题：" << topic << std::endl;

         auto iter = cuda_subscribers_.find(topic);
         if (iter != cuda_subscribers_.end()) {
             std::cout << "Topic %s has been subscribed. " << topic.c_str() << std::endl;
             AD_LERROR(RsclAccess) << "Topic " << topic << " has been subscribed.";
             return false;
         }
 //        auto node = this->GetPolicy()->template NativeHandle<rscl::component::Node>();
         //不确定是否与原有代码一致
         auto node = InnerNode();


         if (!node) {
               std::cout << "获取Node失败！无法订阅"<< std::endl;
                return false;
            }

         std::cout << "成功获取Node: " << node << std::endl;


         senseAD::rscl::comm::CudaSubscriberConf conf;
 //        conf.set_channel_name("center_camera_fov120");  // 正确的channel_name
         conf.set_channel_name(topic);





         std::cout << "准备构造CudaSubscriber: topic=" << topic
                   << " node=" << node << std::endl;
         auto sub = std::make_shared<senseAD::rscl::comm::CudaSubscriber>(
                 node,
                 [obj, fp](const MemblockPtr &mb) {
                     std::cout << "终于收到CUDA消息！进入回调" << std::endl;
                     (obj->*fp)(mb);
                 },
                 conf
             );
         std::cout << "CudaSubscriber构造完成: topic=" << topic << std::endl;

         cuda_subscribers_[topic] = sub;

         std::cout << "Cuda subscribe topic success: " <<  topic << std::endl;
         AD_LINFO(RsclAccess) << "Cuda subscribe topic " << topic << " success.";
         return true;
     }


    // --------------------------------------------------------------

    // ---------------- CudaSubscriber 集成结束 ----------------

    template<class M>
    bool advertise(const std::string &topic) {
        auto iter = publishers_.find(topic);
        if (iter != publishers_.end()) {
            AD_LERROR(RsclAccess) << "Topic " << topic << " has been advertised.";
            return false;
        }
        auto pub = InnerNode()->CreatePublisher<M>(topic);
        publishers_[topic] = pub;
        AD_LINFO(RsclAccess) << "Advertise topic " << topic << " success.";
        return true;
    }

    template<typename M>
    bool publish(const std::string &topic, const M &msg) const {
        auto iter = publishers_.find(topic);
        if (iter == publishers_.end()) {
            AD_LERROR(RsclAccess) << "Publish topic " << topic << " failed, topic not advertised.";
            return false;
        }
        iter->second->Publish(msg);
        return true;
    }

private:
    senseAD::rscl::Runtime* rt_;
    std::map<std::string, senseAD::rscl::comm::PublisherBase::Ptr> publishers_;
    std::map<std::string, senseAD::rscl::comm::SubscriberBase::Ptr> subscribers_;
    std::map<std::string, senseAD::rscl::comm::CudaSubscriber::Ptr> cuda_subscribers_;

};
