# UDP vs TCP Options - Giải thích chi tiết

## 🎯 Tại sao UDP và TCP khác nhau?

### TCP (Connection-based)

```
Ground Station  ──connect──>  Drone Server
   (client)                    192.168.1.100:5760
```

- **Kết nối 2 chiều**: Sau khi connect, data flow cả 2 hướng
- **Chỉ cần địa chỉ server**: Client tự động được assign local port
- **Reliable**: Đảm bảo packets đến đúng thứ tự, tự động retry

### UDP (Connectionless)

```
Ground Station          Drone
  0.0.0.0:14550    ←→   192.168.1.100:14551
  (local bind)          (remote target)
```

- **Không có connection**: Mỗi packet độc lập
- **Cần 2 addresses**:
  - Local để BIND/LISTEN (receive packets)
  - Remote để SEND (send packets đến)
- **Unreliable**: Packets có thể mất, không guarantee thứ tự

---

## 📋 Options sau khi standardize

### UdpOptions

```typescript
export type UdpOptions = {
  host?: string // Local bind address (default '0.0.0.0')
  port: number // Local bind port to receive MAVLink packets
  remoteHost?: string // Remote drone address to send commands to
  remotePort?: number // Remote drone port to send commands to
  heartbeatTimeoutMs?: number
}
```

**Giải thích từng field:**

#### `host` (optional, default `'0.0.0.0'`)

- **Địa chỉ LOCAL để bind socket**
- `'0.0.0.0'` = listen trên TẤT CẢ network interfaces
- `'127.0.0.1'` = chỉ listen trên localhost
- `'192.168.1.50'` = chỉ listen trên interface có IP này

**Khi nào dùng:**

```typescript
// Listen trên tất cả interfaces (thường dùng nhất)
{ host: '0.0.0.0', port: 14550 }

// Chỉ listen trên WiFi interface (security)
{ host: '192.168.1.50', port: 14550 }

// Chỉ listen localhost cho testing
{ host: '127.0.0.1', port: 14550 }
```

#### `port` (required)

- **Port LOCAL để bind và receive MAVLink packets**
- Standard MAVLink port: `14550`
- Ground Control Station thường dùng: `14550`, `14551`

**Ví dụ:**

```typescript
// Drone broadcast telemetry đến 14550
// Ground Station nhận tại đây
{
  port: 14550
}
```

#### `remoteHost` (optional)

- **Địa chỉ IP của DRONE để SEND commands đến**
- Không cần nếu chỉ receive telemetry
- Cần nếu muốn send commands (ARM, DISARM, waypoints, etc.)

**Ví dụ:**

```typescript
// Send commands đến drone tại 192.168.1.100
{
  port: 14550,
  remoteHost: '192.168.1.100'
}
```

#### `remotePort` (optional)

- **Port của DRONE để SEND commands đến**
- Thường là `14551` cho drone
- Pair với `remoteHost`

**Ví dụ:**

```typescript
// Complete two-way communication
{
  port: 14550,              // Receive telemetry here
  remoteHost: '192.168.1.100',  // Send to drone
  remotePort: 14551         // Drone listening port
}
```

---

### TcpOptions

```typescript
export type TcpOptions = {
  host: string // Remote server address to connect to
  port: number // Remote server port to connect to
  heartbeatTimeoutMs?: number
}
```

**Giải thích:**

#### `host` (required)

- **Địa chỉ REMOTE server/drone để connect đến**
- Có thể là IP hoặc hostname

#### `port` (required)

- **Port REMOTE server/drone để connect đến**
- Standard MAVLink TCP: `5760`

**Ví dụ:**

```typescript
// Connect đến drone via TCP
{
  host: '192.168.1.100',
  port: 5760
}

// Connect đến MAVLink server
{
  host: 'drone.example.com',
  port: 5760
}
```

---

## 💡 Use Cases thực tế

### Case 1: Ground Station receive telemetry only

```typescript
// Chỉ nhận telemetry từ drone broadcast
await MAVLink.startUdp({
  port: 14550, // Listen only
})

// Drone tự động broadcast, không cần specify remote
```

### Case 2: Ground Station send commands to drone

```typescript
// Vừa nhận telemetry VÀ send commands
await MAVLink.startUdp({
  port: 14550, // Receive telemetry
  remoteHost: '192.168.1.100', // Send commands to drone
  remotePort: 14551,
})

// Giờ có thể:
await MAVLink.sendCommandLong({ command: MAV_CMD_COMPONENT_ARM_DISARM })
```

### Case 3: Multiple network interfaces

```typescript
// Laptop có WiFi (192.168.1.50) và Ethernet (10.0.0.5)
// Chỉ listen trên WiFi cho drone
await MAVLink.startUdp({
  host: '192.168.1.50', // Chỉ bind WiFi interface
  port: 14550,
  remoteHost: '192.168.1.100',
  remotePort: 14551,
})
```

### Case 4: TCP connection (simulator, SITL)

```typescript
// Connect đến simulator qua TCP
await MAVLink.startTcp({
  host: 'localhost', // Or '127.0.0.1'
  port: 5760,
})

// PX4 SITL, ArduPilot SITL thường dùng TCP:5760
```

### Case 5: Companion computer forwarding

```typescript
// Raspberry Pi trên drone nhận từ Pixhawk serial
// Forward qua WiFi đến Ground Station

// Pixhawk → RPi: Serial
// RPi → Ground Station: UDP broadcast to 255.255.255.255:14550

// Ground Station:
await MAVLink.startUdp({
  port: 14550, // Nhận broadcast từ RPi
})
```

---

## 🔍 So sánh Before/After

### Before (không consistent):

```typescript
// UDP
type UdpOptions = {
  localPort: number // ❌ Khác tên
  address?: string // ❌ Khác tên
  remoteHost?: string
  remotePort?: number
}

// TCP
type TcpOptions = {
  host: string // ✅ Standard name
  port: number // ✅ Standard name
}
```

### After (standardized):

```typescript
// UDP
type UdpOptions = {
  host?: string // ✅ Standard name (local bind)
  port: number // ✅ Standard name (local bind)
  remoteHost?: string // ✅ Clear: for sending
  remotePort?: number // ✅ Clear: for sending
}

// TCP
type TcpOptions = {
  host: string // ✅ Standard name (remote connect)
  port: number // ✅ Standard name (remote connect)
}
```

**Benefits:**

- ✅ Consistent naming: `host` + `port` cho cả UDP và TCP
- ✅ Clear semantics: Local vs Remote rõ ràng
- ✅ Better DX: Developers dễ hiểu hơn
- ✅ Comments: Giải thích purpose của mỗi field

---

## 📚 Tóm tắt

| Field        | UDP                   | TCP                   | Purpose                 |
| ------------ | --------------------- | --------------------- | ----------------------- |
| `host`       | Local bind address    | Remote server address | Where to listen/connect |
| `port`       | Local bind port       | Remote server port    | Where to listen/connect |
| `remoteHost` | Drone IP to send to   | N/A                   | UDP destination         |
| `remotePort` | Drone port to send to | N/A                   | UDP destination         |

**Quy tắc nhớ:**

- **TCP**: `host` + `port` = địa chỉ server để CONNECT đến
- **UDP**:
  - `host` + `port` = địa chỉ local để BIND/LISTEN
  - `remoteHost` + `remotePort` = địa chỉ remote để SEND đến

---

## 🎯 Migration Guide

Nếu đang dùng code cũ, cần update:

### Old code:

```typescript
await MAVLink.startUdp({
  localPort: 14550,
  address: '0.0.0.0',
})
```

### New code:

```typescript
await MAVLink.startUdp({
  port: 14550,
  host: '0.0.0.0', // Optional, this is default
})
```

Or simply:

```typescript
await MAVLink.startUdp({ port: 14550 })
```

**Breaking changes:**

- ❌ `localPort` → ✅ `port`
- ❌ `address` → ✅ `host`

**C++ implementation cũng đã được update** để match với types mới!
