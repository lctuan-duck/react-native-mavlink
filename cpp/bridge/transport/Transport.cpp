#include "../../HybirdMAVLink.hpp"
#include "../../core/MAVLinkCore.hpp"
#include <NitroModules/Promise.hpp>
#include <stdexcept>

namespace margelo::nitro::mavlink
{
  std::shared_ptr<Promise<void>> HybirdMAVLink::startUdp(const UdpOptions& options)
  {
    return Promise<void>::async([this, options]() {
      if (!core_) {
        throw std::runtime_error("Core not initialized");
      }
    
      UdpOptions coreOpts;
      coreOpts.port = static_cast<uint16_t>(options.port);
      coreOpts.host = options.host.has_value() ? *options.host : "0.0.0.0";
      coreOpts.remoteHost = options.remoteHost.has_value() ? *options.remoteHost : "";
      coreOpts.remotePort = options.remotePort.has_value() ? static_cast<uint16_t>(*options.remotePort) : 0;
      
      if (!core_->startUDP(coreOpts)) {
        throw std::runtime_error("Failed to start UDP");
      }
    });
  }

  std::shared_ptr<Promise<void>> HybirdMAVLink::stopUdp()
  {
    return Promise<void>::async([this]() {
      if (!core_) {
        throw std::runtime_error("Core not initialized");
      }
      
      core_->stopUDP();
    });
  }

  std::shared_ptr<Promise<void>> HybirdMAVLink::startTcp(const TcpOptions& options)
  {
    return Promise<void>::async([this, options]() {
      if (!core_) {
        throw std::runtime_error("Core not initialized");
      }
      
      TcpOptions coreOpts;
      coreOpts.host = options.host;
      coreOpts.port = static_cast<uint16_t>(options.port);
      
      if (!core_->startTCP(coreOpts)) {
        throw std::runtime_error("Failed to start TCP");
      }
    });
  }

  std::shared_ptr<Promise<void>> HybirdMAVLink::stopTcp()
  {
    return Promise<void>::async([this]() {
      if (!core_) {
        throw std::runtime_error("Core not initialized");
      }
      
      core_->stopTCP();
    });
  }

} // namespace margelo::nitro::mavlink
