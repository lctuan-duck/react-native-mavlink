#pragma once
#include <functional>
#include <vector>
#include <string>
#include <cstdint>
#include <memory>

namespace margelo::nitro::mavlink {

class Transport {
public:
    using DataCallback = std::function<void(const std::vector<uint8_t>&)>;
    using ErrorCallback = std::function<void(const std::string&)>;
    
    virtual ~Transport() = default;
    
    virtual bool start() = 0;
    virtual void stop() = 0;
    virtual bool isRunning() const = 0;
    virtual bool send(const uint8_t* data, size_t length) = 0;
    
    void setDataCallback(DataCallback cb) { dataCallback_ = std::move(cb); }
    void setErrorCallback(ErrorCallback cb) { errorCallback_ = std::move(cb); }
    
protected:
    DataCallback dataCallback_;
    ErrorCallback errorCallback_;
    
    void notifyData(const std::vector<uint8_t>& data) {
        if (dataCallback_) dataCallback_(data);
    }
    
    void notifyError(const std::string& error) {
        if (errorCallback_) errorCallback_(error);
    }
};

} // namespace margelo::nitro::mavlink
