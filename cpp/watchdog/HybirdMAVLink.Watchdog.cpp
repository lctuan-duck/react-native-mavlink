// Watchdog thread management split from HybirdMAVLink.cpp
#include "../HybirdMAVLink.hpp"

namespace margelo::nitro::mavlink {

void HybirdMAVLink::initWatchdog() {
  watchdogRunning = true;
  watchdogThread = std::make_unique<std::thread>([this]() {
    while (watchdogRunning) {
      auto nowMs = (int64_t)std::chrono::duration_cast<std::chrono::milliseconds>(
        std::chrono::steady_clock::now().time_since_epoch()).count();
      auto last = lastHeartbeatMs.load();
      int64_t timeoutMs = 3000;
      if (heartbeatTimeoutOverrideMs.load() > 0) timeoutMs = heartbeatTimeoutOverrideMs.load();
      if (last > 0 && (nowMs - last) > timeoutMs) {
        StatusEvent st{};
        StatusText tx{};
        tx.severity = 4;
        tx.message = std::string("Heartbeat timeout: Disconnected");
        st.text = tx;
        st.timestampMs = (double)nowMs;
        for (auto& kv : statusListeners) kv.second(st);
        lastHeartbeatMs.store(0);
      }
      std::this_thread::sleep_for(std::chrono::milliseconds(500));
    }
  });
}

void HybirdMAVLink::stopWatchdog() {
  watchdogRunning = false;
  if (watchdogThread && watchdogThread->joinable()) watchdogThread->join();
}

} // namespace margelo::nitro::mavlink