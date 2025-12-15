#pragma once
#include <functional>
#include <string>
#include <map>
#include <vector>
#include <mutex>

namespace margelo::nitro::mavlink {

class EventEmitter {
public:
    using EventHandler = std::function<void(const std::string& data)>;
    
    EventEmitter() = default;
    ~EventEmitter() = default;
    
    void on(const std::string& eventName, EventHandler handler);
    void emit(const std::string& eventName, const std::string& data);
    void removeAllListeners(const std::string& eventName);
    void removeAllListeners();
    
private:
    std::mutex mutex_;
    std::map<std::string, std::vector<EventHandler>> handlers_;
};

} // namespace margelo::nitro::mavlink
