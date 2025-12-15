#include "../../HybirdMAVLink.hpp"
#include "../../core/MAVLinkCore.hpp"
#include <NitroModules/Promise.hpp>
#include <stdexcept>

namespace margelo::nitro::mavlink
{
  std::shared_ptr<Promise<void>> HybirdMAVLink::startUdp(const UdpOptions& options)
  {
    auto promise = Promise<void>::async();
    
    if (!core_) {
      promise->reject(std::make_exception_ptr(std::runtime_error("Core not initialized")));
      return promise;
    }
    
    UdpOptions coreOpts;
    coreOpts.port = static_cast<uint16_t>(options.port);
    coreOpts.host = options.host.has_value() ? *options.host : "0.0.0.0";
    coreOpts.remoteHost = options.remoteHost.has_value() ? *options.remoteHost : "";
    coreOpts.remotePort = options.remotePort.has_value() ? static_cast<uint16_t>(*options.remotePort) : 0;
    
    if (core_->startUDP(coreOpts)) {
      promise->resolve();
    } else {
      promise->reject(std::make_exception_ptr(std::runtime_error("Failed to start UDP")));
    }
    
    return promise;
  }

  std::shared_ptr<Promise<void>> HybirdMAVLink::stopUdp()
  {
    auto promise = Promise<void>::async();
    
    if (!core_) {
      promise->reject(std::make_exception_ptr(std::runtime_error("Core not initialized")));
      return promise;
    }
    
    core_->stopUDP();
    promise->resolve();
    
    return promise;
  }

  std::shared_ptr<Promise<void>> HybirdMAVLink::startTcp(const TcpOptions& options)
  {
    auto promise = Promise<void>::async();
    
    if (!core_) {
      promise->reject(std::make_exception_ptr(std::runtime_error("Core not initialized")));
      return promise;
    }
    
    TcpOptions coreOpts;
    coreOpts.host = options.host;
    coreOpts.port = static_cast<uint16_t>(options.port);
    
    if (core_->startTCP(coreOpts)) {
      promise->resolve();
    } else {
      promise->reject(std::make_exception_ptr(std::runtime_error("Failed to start TCP")));
    }
    
    return promise;
  }

  std::shared_ptr<Promise<void>> HybirdMAVLink::stopTcp()
  {
    auto promise = Promise<void>::async();
    
    if (!core_) {
      promise->reject(std::make_exception_ptr(std::runtime_error("Core not initialized")));
      return promise;
    }
    
    core_->stopTCP();
    promise->resolve();
    
    return promise;
  }

} // namespace margelo::nitro::mavlink
