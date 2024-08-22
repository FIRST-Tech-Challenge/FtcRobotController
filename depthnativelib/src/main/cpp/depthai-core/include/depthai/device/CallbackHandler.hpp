#pragma once

// std
#include <functional>
#include <memory>

// shared
#include "depthai-shared/datatype/RawBuffer.hpp"
#include "depthai-shared/xlink/XLinkConstants.hpp"

// project
#include "DataQueue.hpp"
#include "depthai/xlink/XLinkConnection.hpp"

namespace dai {

class CallbackHandler {
    std::thread t;
    std::atomic<bool> running{true};
    std::shared_ptr<XLinkConnection> connection;
    std::function<std::shared_ptr<RawBuffer>(std::shared_ptr<RawBuffer>)> callback;

   public:
    void setCallback(std::function<std::shared_ptr<RawBuffer>(std::shared_ptr<RawBuffer>)> cb);
    CallbackHandler(std::shared_ptr<XLinkConnection> conn,
                    const std::string& streamName,
                    std::function<std::shared_ptr<RawBuffer>(std::shared_ptr<RawBuffer>)> cb);
    ~CallbackHandler();
};

}  // namespace dai
