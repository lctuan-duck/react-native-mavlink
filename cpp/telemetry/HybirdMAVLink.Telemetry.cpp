// Telemetry snapshot and subscriptions split from HybirdMAVLink.cpp
#include "../HybirdMAVLink.hpp"

namespace margelo::nitro::mavlink
{

  double HybirdMAVLink::onTelemetry(const std::function<void(const TelemetryEvent &)> &listener)
  {
    double t = nextToken++;
    telemetryListeners[t] = listener;
    return t;
  }
  void HybirdMAVLink::offTelemetry(double token) { telemetryListeners.erase(token); }

  std::shared_ptr<Promise<std::variant<nitro::NullType, TelemetryEvent>>> HybirdMAVLink::getTelemetrySnapshot()
  {
    auto promise = std::make_shared<Promise<std::variant<nitro::NullType, TelemetryEvent>>>();
    if (telemetrySnapshot.has_value())
    {
      promise->resolve(telemetrySnapshot.value());
    }
    else
    {
      promise->resolve(nitro::Null);
    }
    return promise;
  }

} // namespace margelo::nitro::mavlink