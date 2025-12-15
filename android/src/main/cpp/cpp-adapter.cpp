#include <jni.h>
#include "NitroMavlinkOnLoad.hpp"

JNIEXPORT jint JNICALL JNI_OnLoad(JavaVM* vm, void*) {
  return margelo::nitro::mavlink::initialize(vm);
}
