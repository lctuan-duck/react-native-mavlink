#include "EventEmitter.hpp"

namespace margelo::nitro::mavlink {

void EventEmitter::on(const std::string& eventName, EventHandler handler) {
    std::lock_guard<std::mutex> lock(mutex_);
    handlers_[eventName].push_back(std::move(handler));
}

void EventEmitter::emit(const std::string& eventName, const std::string& data) {
    std::lock_guard<std::mutex> lock(mutex_);
    auto it = handlers_.find(eventName);
    if (it != handlers_.end()) {
        for (auto& handler : it->second) {
            handler(data);
        }
    }
}

void EventEmitter::removeAllListeners(const std::string& eventName) {
    std::lock_guard<std::mutex> lock(mutex_);
    handlers_.erase(eventName);
}

void EventEmitter::removeAllListeners() {
    std::lock_guard<std::mutex> lock(mutex_);
    handlers_.clear();
}

} // namespace margelo::nitro::mavlink
