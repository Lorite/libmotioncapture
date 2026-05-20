# `optitrack` backend: NatNet unicast keepalive

## TL;DR

The raw-UDP `optitrack` backend in `src/optitrack.cpp` now sends a
`NAT_KEEPALIVE` (message ID `10`) packet from the bound data socket to the
NatNet command port once per second. Without this, Motive expires the
unicast session with `NAT_DISCONNECTBYTIMEOUT` (message ID `11`) after
~10 s of streaming and the client receives no further frame data.

This is the same mechanism that the closed-source `libNatNet.so`
implements in its internal `UnicastKeepaliveThread_Func` thread.

## Symptom

Streaming from `<motive_pc>:1511` (data port) to `<client>:1511` stops
cleanly after ~10 seconds. Packet count is independent of frame rate
(~1250 packets at 120 Hz; ~200 packets at 20 Hz — both ≈ 10 s of data).
Motive's Data Streaming panel shows the client connection disappearing.

## Why **`NAT_CONNECT`** doesn't work as a keepalive

A natural first attempt is to re-send `NAT_CONNECT` (message ID `0`)
periodically. Motive accepts it and responds with a fresh `NAT_SERVERINFO`
(message ID `1`), so the round-trip "works". But the unicast session
timer is **not** reset by `NAT_CONNECT` — only by `NAT_KEEPALIVE`. The
stream still terminates at the 10 s mark. The two message IDs map to
distinct code paths on the server side.

The NatNet protocol message IDs (see `deps/NatNetSDKCrossplatform/include/NatNetTypes.h`):

| ID | Macro | Purpose |
| -- | ----- | ------- |
| `0` | `NAT_CONNECT` | Initial handshake. |
| `1` | `NAT_SERVERINFO` | Server's response to `NAT_CONNECT`. |
| `4` | `NAT_REQUEST_MODELDEF` | Ask for rigid-body / marker descriptions. |
| `5` | `NAT_MODELDEF` | Server's response with descriptions. |
| `7` | `NAT_FRAMEOFDATA` | A frame of streaming data. |
| `9` | `NAT_DISCONNECT` | Client-initiated disconnect. |
| **`10`** | **`NAT_KEEPALIVE`** | **Unicast session keepalive — what we now send.** |
| `11` | `NAT_DISCONNECTBYTIMEOUT` | Server-side timeout notification. |

## Implementation

`waitForNextFrame()` checks a `std::chrono::steady_clock` deadline at the
top of every call and sends a 4-byte header-only `NAT_KEEPALIVE` if
≥ 1000 ms has elapsed since the last send:

```cpp
auto now = std::chrono::steady_clock::now();
auto elapsed_ms = std::chrono::duration_cast<std::chrono::milliseconds>(
    now - pImpl->last_keepalive).count();
if (elapsed_ms >= 1000) {
  struct { uint16_t iMessage; uint16_t nDataBytes; } keepalive =
      {10 /* NAT_KEEPALIVE */, 0};
  boost::system::error_code ec;
  pImpl->socket.send_to(
      boost::asio::buffer(&keepalive, sizeof(keepalive)),
      pImpl->endpoint_cmd, 0, ec);
  pImpl->last_keepalive = now;
}
```

The command endpoint (`hostname:1510`) is captured into
`pImpl->endpoint_cmd` at the end of the constructor; the data socket
(bound to `0.0.0.0:1511`) is reused for sending so the keepalive's
source port matches the port Motive registered during `NAT_CONNECT`.

`pImpl->last_keepalive` is initialised at the end of the constructor.

A receive timeout of 2 s (`SO_RCVTIMEO`) is set on the data socket as a
defence-in-depth fallback. In steady-state it never fires because frames
arrive at the configured mocap rate (≥ 20 Hz). If the stream does stall
(Motive restart, transient network loss), the `boost::asio::error::would_block`
branch in `waitForNextFrame()` sends a `NAT_CONNECT` packet to attempt a
re-handshake.

## Verifying it works

The keepalives are visible on the wire as a regular 1 Hz stream of
4-byte UDP packets from the client to the command port:

```bash
sudo tcpdump -ni <iface> 'host <motive_ip> and udp and udp[8:2] = 10'
# Expect one small packet per second going out.
```

The byte filter `udp[8:2] = 10` matches packets whose first two payload
bytes equal `0x000A` (little-endian `NAT_KEEPALIVE` = 10).

## Why we send from the data socket, not a separate one

The `NAT_CONNECT` handshake's source `(IP, port)` is what Motive
registers as the unicast destination for streaming data. Subsequent
keepalives must come from the **same source port** (1511 by default), or
Motive may treat them as belonging to a different client. The data
socket — already bound to that port — is the natural sender.

## Why `<chrono>` and not a background thread

`boost::asio::ip::udp::socket` is not thread-safe for concurrent operations
without external synchronisation. Inlining the keepalive into the existing
`waitForNextFrame()` call (which already owns the socket) avoids a mutex
and avoids the thread-safety surprises around socket close on shutdown.
Frames arrive frequently enough at typical mocap rates that the 1 s
deadline is easily honoured from the polling loop.
