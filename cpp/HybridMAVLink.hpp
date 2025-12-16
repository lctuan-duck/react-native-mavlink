#include "HybridMAVLink.hpp"

namespace margelo::nitro::math {
  class HybridMAVLink: public HybridMAVLink {
  public:
    HybridMAVLink(): HybridObject(TAG) {} // This constructor is required. TAG must have to fix errors.
  };
}