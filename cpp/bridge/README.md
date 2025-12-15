# Bridge Implementation

Thư mục này chứa các implementation của HybirdMAVLink bridge, được tổ chức theo chức năng.

## 📁 Cấu Trúc

```
bridge/
├── transport/
│   └── Transport.cpp          # UDP/TCP transport management
├── commands/
│   └── Commands.cpp           # MAVLink commands (COMMAND_LONG, COMMAND_INT)
├── parameters/
│   └── Parameters.cpp         # Parameter requests and updates
├── events/
│   └── Events.cpp             # Event listener registration (40+ events)
├── mission/
│   └── Mission.cpp            # Mission planning and management
└── logging/
    └── Logging.cpp            # Log data requests and transmission
```

## 🎯 Mục Đích Từng Folder

### transport/
Quản lý kết nối UDP và TCP với drone/vehicle
- `startUdp()`, `stopUdp()`
- `startTcp()`, `stopTcp()`

### commands/
Gửi commands và encode/decode messages
- `sendCommandLong()` - Gửi lệnh với float parameters
- `sendCommandInt()` - Gửi lệnh với int32 parameters
- `encode()`, `decode()` - Message encoding/decoding

### parameters/
Quản lý parameters của vehicle
- `requestParams()` - Request tất cả parameters
- `setParam()` - Set giá trị parameter

### events/
Đăng ký và quản lý event listeners
- Heartbeat, GPS, Attitude, Battery events
- Status, Mode, Arm events
- Camera, Gimbal events
- Mission events
- Parameter events
- Logging events
- Raw data events

### mission/
Quản lý mission planning
- `uploadMission()` - Upload mission waypoints
- `downloadMission()` - Download current mission
- `clearMission()` - Clear all mission items
- `setCurrentMission()` - Set current mission item
- Mission event listeners

### logging/
Quản lý logs và data transmission
- `requestLogList()` - Request danh sách logs
- `requestLogData()` - Request log data chunks
- `requestDataTransmissionHandshake()` - Data transmission setup

## 🔧 Quy Tắc Khi Thêm Code Mới

1. **Transport logic** → Thêm vào `transport/Transport.cpp`
2. **Command mới** → Thêm vào `commands/Commands.cpp`
3. **Parameter handling** → Thêm vào `parameters/Parameters.cpp`
4. **Event listener mới** → Thêm vào `events/Events.cpp`
5. **Mission feature** → Thêm vào `mission/Mission.cpp`
6. **Logging feature** → Thêm vào `logging/Logging.cpp`

## 📝 Include Paths

Tất cả files trong bridge/ folder sử dụng relative paths:
```cpp
#include "../../HybirdMAVLink.hpp"       // Header chính
#include "../../core/MAVLinkCore.hpp"    // Core orchestrator
```

## ✅ Lợi Ích Của Cấu Trúc Này

- ✅ **Dễ tìm**: Biết rõ feature nào ở folder nào
- ✅ **Dễ maintain**: Sửa 1 chức năng không ảnh hưởng code khác
- ✅ **Dễ debug**: Stack trace chỉ rõ file và folder cụ thể
- ✅ **Dễ scale**: Thêm file mới vào folder tương ứng
- ✅ **Clean code**: Mỗi folder có trách nhiệm rõ ràng
