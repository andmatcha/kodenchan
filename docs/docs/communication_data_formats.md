source: https://github.com/ash0923/arm9_ik/commit/5e07cd34282de06bc01677d3f2798e50dc0d0b35

# Communication Data Formats

この文書は `arm9_ik` の標準通信仕様を、1ファイルで参照できるようにまとめたものです。
対象は Mac host / Ubuntu guest / miniPC / STM32 / frontend の間を流れる binary packet と JSON packet です。

## 0. Quick Reference

### Packet Index

| Packet | Transport | Direction | Primary owner | See |
|---|---|---|---|---|
| `AI` | UDP 5005 | Mac -> Ubuntu | Mac bridge / `udp_joy_bridge` | [4.1](#41-ai-ik-teleop) |
| `MC` | UDP 5005 | Mac -> Ubuntu | Mac bridge / `udp_joy_bridge` | [4.2](#42-mc-manual-currents) |
| `AC v6` | UDP 7000 + XBee | Ubuntu -> STM32 | Ubuntu `udp_ac_tx` | [4.3](#43-ac-v6-unified-command) |
| `JF` | XBee + UDP 5010 | STM32 -> Ubuntu | STM downlink / Ubuntu JF RX | [4.4](#44-jf-joint-feedback) |
| `UF` | XBee -> WS | STM32 -> frontend | Mac bridge | [4.5](#45-uf-usb-feedback) |
| Sensor Board ASCII | XBee RX serial -> WS | sensor board -> frontend | sensor board / Mac bridge | [4.6](#46-sensor-board-ascii-telemetry) / [5.5](#55-frontend-sensor-board-and-gps-pose-ws-json) |
| `GC` | SERVO serial | frontend -> servo controller | frontend / Mac bridge | [4.7](#47-gc-servo-control) |
| miniPC detection | UDP 6001 | miniPC -> Ubuntu | miniPC | [5.1](#51-minipc-detection-json) |
| miniPC command | UDP 6002 | Ubuntu -> miniPC | Ubuntu Keyboard Auto | [5.2](#52-ubuntu---minipc-command-json) |
| `cmd_ack` | UDP 6001 | miniPC -> Ubuntu | miniPC | [5.3](#53-minipc---ubuntu-cmd_ack) |
| frontend request | UDP 6100-6105 | frontend/Mac -> Ubuntu | dashboard bridge | [5.4](#54-frontend---ubuntu-request-json) |

### Component Responsibility

| Component | Owns | Sends | Receives |
|---|---|---|---|
| Frontend | Operator intent / UI display | WS actions to Mac | WS telemetry, video, packet previews |
| Mac bridge | WebSocket, XBee serial, UDP relay | `AI`, `MC`, XBee frames, frontend request JSON | `AC`, `JF`, `UF`, miniPC relay packets |
| Ubuntu | Mission state, IK geometry, `AC v6` generation | `AC v6`, miniPC command JSON, status JSON | `AI`, `MC`, `JF`, miniPC detection / ACK |
| miniPC | YOLO / RealSense, camera->base raw key center | detection JSON, `cmd_ack`, JPEG | start / phase / cancel JSON, `arm_pos` |
| STM32 | Physical arm motion, done/ready flags | `JF`, `UF` | `AC v6` via XBee/CAN |
| Sensor board | Rover GPS / IMU sampling | sensor ASCII lines via XBee RX | GPS input, IMU sensors |

### Common Lookups

| Need | Section |
|---|---|
| Ports and routes | [2. Routes And Ports](#2-routes-and-ports) |
| `AI` / `MC` flags | [3.1](#31-aimc-flags) |
| `AC.flags` | [3.2](#32-acflags) |
| shared `control_byte` | [3.3](#33-shared-control_byte) |
| `JF.flags` | [3.4](#34-jfflags) |
| Keyboard Auto `auto_flags` / `fault_code` | [3.5](#35-keyboard-auto-auto_flags) / [3.6](#36-keyboard-auto-fault_code) |
| miniPC raw key center / `arm_pos` | [6.3](#63-minipc-coordinate-contract) |
| sensor board GPS/IMU IDs | [4.6](#46-sensor-board-ascii-telemetry) |
| servo `GC` packet | [4.7](#47-gc-servo-control) |

## 1. Global Rules

| Item | Rule |
|---|---|
| Host topology | 標準構成は Mac relay です。Ubuntu guest は miniPC / STM32 と直接通信しません。 |
| UDP framing | 1 UDP datagram = 1 packet. Packets are not split across datagrams. |
| Binary endian | Little Endian. Python `struct` strings use `<`. |
| Binary CRC | CRC-16/CCITT-FALSE, little-endian field at packet tail. |
| Sequence fields | `uint8` sequence numbers may wrap. Use for freshness/liveness, not global ordering. |
| Coordinates | Keyboard Auto coordinate fields are millimeters unless stated otherwise. |
| Key labels | Keyboard labels are canonical upper-case labels such as `A`, `SPACE`, `F1`, `''`. |

## 2. Routes And Ports

### Runtime Topology

```text
miniPC  -- UDP6001(JSON detection) -->  Mac  -- UDP6001(JSON relay) --> Ubuntu(ROS2)
Ubuntu -- UDP6002(JSON cmd)        -->  Mac  -- UDP6002(JSON relay) --> miniPC

Ubuntu -- UDP7000(AC v6 binary) --> Mac -- Serial/XBee --> STM32
STM32  -- Serial/XBee --> Mac -- UDP5010(JF binary) --> Ubuntu
sensor_board -- Serial/XBee ASCII(ID,value) --> Mac(dashboard_bridge) -- WS4100 --> browser(frontend)

miniPC -- UDP4101(JPEG bytes) --> Mac(dashboard_bridge) -- WS4100 --> browser(frontend)
Mac(dashboard_bridge) -- UDP6201(JPEG bytes forward) --> Ubuntu(udp_realsense_jpeg_rx)
Ubuntu/Mac -- UDP4102(raw packet/status JSON) --> Mac(dashboard_bridge) -- WS4100 --> browser(frontend)
```

Exception: for bench checks without Ubuntu/ROS, `tools/gamepad_mode_sound_only_mac.py` can generate `AC v6` and send it directly to the XBee TX serial path.

### Port Matrix

| Port | Transport | Direction | Sender -> Receiver | Payload |
|---:|---|---|---|---|
| 5005 | UDP | Mac -> Ubuntu | dashboard_bridge -> `udp_joy_bridge` | `AI` / `MC` binary |
| 5010 | UDP | Mac -> Ubuntu | dashboard_bridge -> `udp_joint_state_rx` | `JF` binary |
| 6001 | UDP | miniPC -> Mac -> Ubuntu | miniPC -> `mac_minipc_udp_bridge` -> `udp_keyboard_detection_rx` | detection JSON + standalone `cmd_ack` |
| 6002 | UDP | Ubuntu -> Mac -> miniPC | Keyboard Auto -> `mac_minipc_udp_bridge` -> miniPC | command JSON |
| 7000 | UDP | Ubuntu -> Mac | `udp_ac_tx` -> dashboard_bridge | `AC v6` binary |
| 4100 | WS | browser <-> Mac | frontend <-> dashboard_bridge | WS JSON API |
| 4101 | UDP | miniPC -> Mac | miniPC -> dashboard_bridge | JPEG bytes |
| 6201 | UDP | Mac -> Ubuntu | dashboard_bridge -> `udp_realsense_jpeg_rx` | JPEG bytes forward |
| 4102 | UDP | Ubuntu/Mac -> Mac | Ubuntu status TX / Mac relay tee -> dashboard_bridge | raw bytes or status JSON |
| 6100 | UDP | Mac -> Ubuntu | dashboard_bridge -> `udp_keyboard_auto_request_rx` | Keyboard Auto request |
| 6101 | UDP | Mac -> Ubuntu | dashboard_bridge -> `udp_control_byte_override_rx` | AC control-byte override |
| 6102 | UDP | Mac -> Ubuntu | dashboard_bridge -> `udp_enable_override_rx` | enable override |
| 6103 | UDP | Mac -> Ubuntu | dashboard_bridge -> `udp_control_mode_override_rx` | control mode override |
| 6104 | UDP | Mac -> Ubuntu | dashboard_bridge -> `udp_ik_ready_request_rx` | IK READY request |
| 6105 | UDP | Mac -> Ubuntu | dashboard_bridge -> `udp_ac_flags_override_rx` | AC flags override |

### Serial Roles

| Role | Direction | Payload |
|---|---|---|
| XBee TX | Mac -> STM32 | `AC v6` binary |
| XBee RX | STM32 / sensor board -> Mac | mixed `JF` binary, `UF` binary, and sensor board ASCII lines |
| SERVO (GC) | Mac frontend -> servo controller | `GC` binary packets |

## 3. Shared Fields And Bit Layouts

### 3.1 `AI`/`MC.flags`

`AI` and `MC` share the same `flags` byte. Ubuntu interprets it in `udp_joy_bridge`.

| Bit | Meaning |
|---:|---|
| 0 | enable/deadman (`1=ON`) |
| 1 | gripper digital input |
| 2 | IK Parallel mode request |
| 3 | IK Panel mode request |
| 4-5 | control mode (`0=IK`, `1=MANUAL`, `2=KEYBOARD_AUTO`) |
| 6-7 | manual profile hint for UI/debug (`0=normal/default`, `1=power`, `2=sensitive`, `3=reserved`) |

`flags[7:6]` is not on the main control path. `tools/gamepad_mode_sound.py` may set it for UI/debug display. Packets generated by `dashboard_bridge.py` without profile knowledge should be treated as `0=normal/default`.

### 3.2 `AC.flags`

`AC.flags` is generated by Ubuntu `udp_ac_tx`.

| Bit | Meaning |
|---:|---|
| 0 | enable |
| 1 | gripper digital input |
| 2 | mission_panel IK submode (`1=mission`) |
| 4-5 | control mode (`0=IK`, `1=MANUAL`, `2=KEYBOARD_AUTO`) |
| 6 | `USB_READ` |

Unused bits are reserved and should be sent as `0`.

### 3.3 Shared `control_byte`

The same control byte layout appears in `AI` byte24, `MC` byte18, and `AC v6` byte30.

| Bit | Name | Meaning |
|---:|---|---|
| 0 | `KBD_PP` | Keyboard Auto push/pull pulse |
| 1 | `KBD_EN` | Keyboard Auto press enable / armed state |
| 2 | `KBD_YAMAN` | pre-READY wrist-90 handshake request |
| 3 | `NYOKKI_PUSH` | manual nyokki push pulse |
| 4 | `NYOKKI_PULL` | manual nyokki pull pulse |
| 5 | `INIT` | initialize one-shot |
| 6 | `HOME` | home pose one-shot |
| 7 | `KBD_START` | Keyboard Auto start / character-cycle active |

### 3.4 `JF.flags`

`JF.flags` is STM-side feedback for Keyboard Auto ready/done states. Ubuntu default bit mapping is configured in `keyboard_auto_controller.yaml`.

| Bit | Meaning |
|---:|---|
| 0 | `kbd_yaman_ready` |
| 1 | `stm_ready` |
| 2 | `x_align_done` |
| 3 | `global_yz_done` |
| 4 | `local_yz_done` |
| 5 | `keyboard_home_yz_done` |
| 6 | `keyboard_home_x_done` |

Mission start advances to the first character only after both miniPC start ACK and `JF.flags.bit1` (`stm_ready`) are accepted.

### 3.5 Keyboard Auto `auto_flags`

`auto_flags` is a `uint16` emitted in `AC v6` for STM/UI telemetry.

| Bit | Meaning |
|---:|---|
| 0 | AUTO active |
| 1 | global stage active (`WAIT_GLOBAL`, `WAIT_X_ALIGN`, global YZ) |
| 2 | local stage active (`WAIT_LOCAL`, local YZ) |
| 3 | global detection valid |
| 4 | local detection valid |
| 5 | X command active |
| 6 | YZ command active |
| 7 | IK valid |
| 8 | some JF done bit is high |
| 9 | global timeout |
| 10 | local timeout |
| 11 | JF timeout |
| 12 | fault stop active |

### 3.6 Keyboard Auto `fault_code`

`fault_code` is a `uint16` emitted in `AC v6`.

| Value | Meaning |
|---:|---|
| 0 | NONE |
| 1 | GLOBAL_TIMEOUT |
| 2 | LOCAL_TIMEOUT |
| 3 | JF_TIMEOUT |
| 4 | IK_FAILED |
| 5 | X_PHASE_FAILED |
| 6 | YZ_PHASE_FAILED |
| 7 | DEADMAN_OFF |
| 8 | PACKET_INVALID |
| 9 | MINIPC_ACK_TIMEOUT |
| 10 | MINIPC_ACK_REJECTED |
| 11 | STM_READY_TIMEOUT |

## 4. Wire Packets And Line Protocols

Each packet card below lists only wire-level fields. Shared bit meanings are defined in [3](#3-shared-fields-and-bit-layouts). Binary packets use little-endian fields and CRC rules from [1](#1-global-rules); line protocols define their own framing.

### 4.1 `AI` (IK teleop)

Purpose: Mac/frontend teleop input for IK control.

Route: Mac `dashboard_bridge` -> Ubuntu `udp_joy_bridge` (`0.0.0.0:5005`).

Wire:

| Item | Value |
|---|---|
| Header | `b"AI"` |
| `struct` | `<2sBBfffffB` |
| Size | 25 bytes |
| CRC | none |

Fields:

| Field | Type | Notes |
|---|---|---|
| header | `2s` | `b"AI"` |
| seq | `uint8` | sender sequence |
| flags | `uint8` | see [3.1](#31-aimc-flags) |
| vy | `float32` | normalized Y velocity, `-1..1` |
| vz | `float32` | normalized Z velocity, `-1..1` |
| omega_base | `float32` | normalized base yaw input, `-1..1` |
| omega_wrist_pitch | `float32` | normalized wrist pitch input, `-1..1` |
| omega_wrist_roll | `float32` | normalized wrist roll input, `-1..1` |
| control_byte | `uint8` | see [3.3](#33-shared-control_byte) |

### 4.2 `MC` (manual currents)

Purpose: Mac/frontend manual current command.

Route: Mac `dashboard_bridge` -> Ubuntu `udp_joy_bridge` (`0.0.0.0:5005`).

Wire:

| Item | Value |
|---|---|
| Header | `b"MC"` |
| `struct` | `<2sBB7HB` |
| Size | 19 bytes |
| CRC | none |

Fields:

| Field | Type | Notes |
|---|---|---|
| header | `2s` | `b"MC"` |
| seq | `uint8` | sender sequence |
| flags | `uint8` | see [3.1](#31-aimc-flags) |
| current[0..6] | `uint16 x7` | `0..511`, neutral=`255` |
| control_byte | `uint8` | see [3.3](#33-shared-control_byte) |

`current[0..6]` UI/operation names:

| Index | Name |
|---:|---|
| 0 | M0 BaseHorizon |
| 1 | M1 BaseRoll |
| 2 | M2 Joint1 |
| 3 | M3 Joint2 |
| 4 | M4 Joint3 |
| 5 | M5 GripperRoll |
| 6 | M6 Gripper |

### 4.3 `AC v6` (unified command)

Purpose: Ubuntu unified command packet for STM32. Ubuntu generates this packet in the ROS stack. For bench checks, `tools/gamepad_mode_sound_only_mac.py` can generate the same wire format for manual-current-only use.

Route: Ubuntu `udp_ac_tx` -> Mac `dashboard_bridge` UDP 7000 -> XBee TX -> STM32.

Wire:

| Item | Value |
|---|---|
| Header | `b"AC"` |
| `struct` | `<2sBB7H3H3hBhHHH` |
| Size | 39 bytes |
| CRC | CRC-16/CCITT-FALSE over first 37 bytes |

Fields:

| Field | Type | Notes |
|---|---|---|
| header | `2s` | `b"AC"` |
| seq | `uint8` | sender sequence |
| flags | `uint8` | see [3.2](#32-acflags) |
| current[0..6] | `uint16 x7` | manual/hold currents, neutral=`255` |
| angle[0..2] | `uint16 x3` | encoder14 targets for M2..M4 |
| vel[0..2] | `int16 x3` | rpm for M2..M4; Keyboard Auto sends `0` |
| control_byte | `uint8` | packet byte30; see [3.3](#33-shared-control_byte) |
| base_target_mm_j0 | `int16` | Keyboard Auto absolute J0/X target in mm |
| auto_flags | `uint16` | see [3.5](#35-keyboard-auto-auto_flags) |
| fault_code | `uint16` | see [3.6](#36-keyboard-auto-fault_code) |
| crc16 | `uint16` | little-endian CRC |

Keyboard Auto YZ movement uses `angle[0..2]` as repeated angle targets and keeps `vel[0..2]=0`.

### 4.4 `JF` (joint feedback)

Purpose: STM encoder feedback plus Keyboard Auto ready/done bits.

Route: STM32 -> XBee RX -> Mac `dashboard_bridge` -> Ubuntu `udp_joint_state_rx` (`0.0.0.0:5010`).

Wire:

| Item | Value |
|---|---|
| Header | `b"JF"` |
| `struct` | `<2sBB5HH` |
| Size | 16 bytes |
| CRC | CRC-16/CCITT-FALSE over first 14 bytes |

Fields:

| Field | Type | Notes |
|---|---|---|
| header | `2s` | `b"JF"` |
| seq | `uint8` | receiver liveness/freshness |
| flags | `uint8` | see [3.4](#34-jfflags) |
| encoders[0..4] | `uint16 x5` | `J0` is signed-mm linear feedback; `J1..J4` are encoder14 values |
| crc16 | `uint16` | little-endian CRC |

### 4.5 `UF` (USB feedback)

Purpose: downlink-only USB memory latitude/longitude feedback. This is separate from live rover GPS used by the map.

Route: STM32 -> XBee RX serial -> Mac `dashboard_bridge` -> frontend WS `usb_feedback`. This route does not forward `UF` to Ubuntu `udp_joint_state_rx`.

Wire:

| Item | Value |
|---|---|
| Header | `b"UF"` |
| `struct` | `<2sBBiiH` |
| Size | 14 bytes |
| CRC | CRC-16/CCITT-FALSE over first 12 bytes |

Fields:

| Field | Type | Notes |
|---|---|---|
| header | `2s` | `b"UF"` |
| seq | `uint8` | STM sequence |
| flags | `uint8` | below |
| lat_e7 | `int32` | latitude, degrees * `1e7` |
| lon_e7 | `int32` | longitude, degrees * `1e7` |
| crc16 | `uint16` | little-endian CRC |

`UF.flags`:

| Bit | Meaning |
|---:|---|
| 0 | `valid` |
| 1 | `usb_present` |
| 2 | `read_busy` |
| 3 | `read_error` |
| 4-7 | reserved, send `0` |

The Mac bridge persists accepted `UF` packets under `.logs/uf_capture/` as decoded `uf_records.jsonl`, raw `*.bin`, and `latest.json`.

### 4.6 Sensor Board ASCII telemetry

Purpose: live rover-side GPS/IMU telemetry for `/map`, `/antenna`, and operator display.

Route: sensor board UART3 -> XBee downlink -> Mac `dashboard_bridge` XBee RX serial role -> frontend WS `sensor_board` / `gps_pose`.

Wire:

| Item | Value |
|---|---|
| Encoding | ASCII |
| Framing | one newline-terminated line per sample |
| Format | `III,value\r\n` |
| `III` | 3 hex digits, sensor id |
| `value` | finite decimal float |
| Line ending | LF required; CRLF recommended |
| Max line length | 128 bytes |
| CRC | none |

Examples:

```text
415,35.12345678901\r\n
416,139.12345678901\r\n
412,90.00\r\n
```

Sensor IDs:

| ID | Field | Unit | Meaning |
|---:|---|---|---|
| `0x400` | `gyro_x` | rad/s | gyroscope X |
| `0x401` | `gyro_y` | rad/s | gyroscope Y |
| `0x402` | `gyro_z` | rad/s | gyroscope Z |
| `0x403` | `accel_x` | m/s^2 | accelerometer X |
| `0x404` | `accel_y` | m/s^2 | accelerometer Y |
| `0x405` | `accel_z` | m/s^2 | accelerometer Z |
| `0x406` | `mag_x` | uT | magnetometer X |
| `0x407` | `mag_y` | uT | magnetometer Y |
| `0x408` | `mag_z` | uT | magnetometer Z |
| `0x410` | `roll` | deg | roll angle |
| `0x411` | `pitch` | deg | pitch angle |
| `0x412` | `heading` | deg | rover heading for map arrow |
| `0x415` | `latitude` | deg | WGS84 decimal latitude |
| `0x416` | `longitude` | deg | WGS84 decimal longitude |

Current firmware formatting:

| Value class | Format |
|---|---|
| GPS latitude/longitude | `%.11f` |
| IMU / heading values | `%.2f` |

Bridge semantics:

- The XBee RX serial stream is mixed. `dashboard_bridge` parses `JF` binary, `UF` binary, and sensor board ASCII lines from the same input stream.
- Sensor board samples are latest-value cached by ID. A group of IMU/GPS values is not atomic unless a future protocol adds an explicit sequence or timestamp.
- `gps_pose` is emitted only after both `0x415` latitude and `0x416` longitude have been received.
- `gps_pose.updated_at_ms` uses the older timestamp of the latitude/longitude pair so IMU-only updates do not make stale GPS look fresh.
- Values that are not finite numbers are rejected. Producers should keep latitude in `-90..90` and longitude in `-180..180`.

### 4.7 `GC` (servo control)

Purpose: command the GNSS tracking yagi antenna servo controller from `/antenna`.

Route: frontend `/antenna` -> Mac `dashboard_bridge` WebSocket serial role `servo` / `SERVO (GC)` -> servo controller serial port.

Wire:

| Item | Value |
|---|---|
| Header | `b"GC"` |
| `struct` | `<2sHBhH` |
| Size | 9 bytes |
| CRC | CRC-16/CCITT-FALSE over first 7 bytes |

Fields:

| Byte | Field | Type | Notes |
|---:|---|---|---|
| 0..1 | header | `char[2]` | ASCII `GC` |
| 2..3 | seq | `uint16` | sender sequence, little-endian |
| 4 | type | `uint8` | see type table below |
| 5..6 | value | `int16` | type-specific data, little-endian |
| 7..8 | crc16 | `uint16` | little-endian CRC |

Types:

| Type | Name | `value` semantics |
|---:|---|---|
| `0x01` | `AUTO` | `0..3599`, antenna tracking angle in `0.1 deg` units |
| `0x02` | `MANUAL_POSITION` | `0..3599`, absolute angle in `0.1 deg` units |
| `0x03` | `MANUAL_RATE` | `-1000..1000`, rate command where `1000 = +100.0%` |
| `0x04` | `STOP` | `0` |
| `0x05` | `HOME` | `0` |

Servo semantics:

- `AUTO` and `MANUAL_POSITION` carry absolute position commands, not relative deltas.
- `/antenna` computes a 270-degree servo absolute angle for GNSS tracking and sends it as `AUTO`.
- The servo mechanical angle is not a map bearing. Map bearing is only an internal calculation input for `/antenna`.

## 5. JSON Packets

### 5.1 miniPC detection JSON

Purpose: miniPC publishes detection, mission, health, and optional ACK telemetry to Ubuntu.

Route: miniPC UDP 6001 -> Mac `mac_minipc_udp_bridge` -> Ubuntu `udp_keyboard_detection_rx`.

Required top-level fields:

```json
{
  "timestamp": 0.0,
  "t_capture": 0.0,
  "t_infer": 0.0,
  "seq": 0,
  "confidence": 0.0,
  "model_type": "global",
  "home_arm": [0.0, 0.0, 0.0],
  "keys": {
    "A": {"Xb_mm": 0.0, "Yb_mm": 0.0, "Zb_mm": 0.0}
  },
  "flags": 0,
  "frame_id": "dodai_1",
  "source": "minipc"
}
```

Optional top-level fields:

| Field | Meaning |
|---|---|
| `cmd_ack` | ACK for start/cancel/phase; see [5.3](#53-minipc---ubuntu-cmd_ack) |
| `mission_active` | miniPC mission active state |
| `mission_nonce` | active mission nonce |
| `mission_phase` | active phase, usually `global` or `local` |
| `axes_authority` | intended axis responsibility for this packet |
| `home_arm_source` | `tracked_global`, `frame_keyboard`, or `config_default` |
| `home_arm_age_sec` | age of tracked keyboard home |
| `keyboard_home_valid` | whether tracked keyboard home is valid |
| `home_arm_candidate` | same-frame Keyboard bbox candidate, or `null` |
| `global_enabled` | runtime gate for global model |
| `local_enabled` | runtime gate for local model |
| `arm_pos` | miniPC's latest stored `j3 center(arm_pos_link)` base-frame position |
| `camera_ok` | camera health |
| `packet_version` | detection JSON version |
| `health` | supervisor snapshot when enabled |
| `target_label` | active mission target label |
| `target_index` | active mission target index |
| `target_instance_count` | number of visible instances for target label |
| `target_unique` | `target_instance_count == 1` |
| `target_selected` | whether the target is present in `keys` |
| `target_reject_reason` | why target was not selected |

Detection semantics:

- `keys` labels are upper-case canonical labels. Examples: `A`, `SPACE`, `F1`, `''`, `,`, `-`, `.`, `/`, `;`, `=`, `[`, `\\`, `]`.
- `KEYBOARD` is not included in `keys`.
- `home_arm` is tracked keyboard home. It is stored from global `Keyboard` bbox camera->base conversion and reused during local phase or temporary missing detections.
- `keys[*].Xb_mm/Yb_mm/Zb_mm` are raw key centers in Ubuntu `base_frame`.
- miniPC only performs camera->base conversion. Ubuntu converts raw key center to nyokki wait point and j3 center target.
- During an active mission, miniPC normally keeps only the requested `target_label` in `keys`.

Common `target_reject_reason` values:

| Value | Meaning |
|---|---|
| `missing_target_label` | no target label requested |
| `target_not_detected` | target not visible |
| `duplicate_target_label` | same label appears multiple times |
| `target_not_stable` | stable gate not yet satisfied |

### 5.2 Ubuntu -> miniPC command JSON

Purpose: Ubuntu controls miniPC mission state and continuously refreshes `arm_pos`.

Route: Ubuntu Keyboard Auto -> Mac `mac_minipc_udp_bridge` -> miniPC UDP 6002.

Examples:

```json
{"cmd":"start","text":"A <F1>","labels":["A","SPACE","F1"],"nonce":"<uuid-hex>","arm_pos":[x_mm,y_mm,z_mm]}
{"cmd":"start","text":"A <F1>","labels":["A","SPACE","F1"],"nonce":"<new-uuid-hex>","replace_nonce":"<old-uuid-hex>","arm_pos":[x_mm,y_mm,z_mm]}
{"cmd":"cancel","nonce":"<uuid-hex>"}
{"cmd":"force_cancel","nonce":"<uuid-hex>"}
{"cmd":"phase","phase":"global","nonce":"<uuid-hex>","text":"A <F1>","label":"F1","index":2,"arm_pos":[x_mm,y_mm,z_mm]}
{"cmd":"phase","phase":"local","nonce":"<uuid-hex>","text":"A <F1>","label":"F1","index":2,"arm_pos":[x_mm,y_mm,z_mm]}
{"arm_pos":[x_mm,y_mm,z_mm]}
```

Command semantics:

- `labels` is the canonical key label array for the mission. miniPC validates `phase.label/index` against this array.
- Old `start` packets without `labels` fall back to parsing `text` into the same canonical label array.
- `replace_nonce` is retry/stop recovery. miniPC atomically replaces only when the old nonce is still active.
- Ubuntu sends `force_cancel` with the same nonce as `cancel`. It is still nonce-scoped, so delayed packets should not clear a newer mission.
- A packet with `arm_pos` and no `cmd` updates only miniPC coordinate-conversion state and does not advance ACK state.

`arm_pos` semantics:

- `arm_pos` is optional.
- It is the base-frame position of `j3 center(arm_pos_link)` in millimeters.
- miniPC stores the latest `arm_pos` snapshot and uses it for RealSense camera->base conversion.
- This packet schema does not carry `arm_pos_timestamp` or `arm_pos_age_sec`. For moving-arm analysis, compare detection `timestamp` / `t_capture` with Ubuntu command logs.

### 5.3 miniPC -> Ubuntu `cmd_ack`

miniPC returns ACKs through two routes:

| Route | Detail |
|---|---|
| standalone ACK | immediate standalone UDP JSON: `{"source":"minipc_ack","cmd_ack":...}` |
| piggyback ACK | detection JSON includes top-level `cmd_ack` |

Payload:

```json
{
  "cmd_ack": {
    "nonce": "<uuid-hex>",
    "cmd": "start|cancel|force_cancel|phase",
    "text": "ARES",
    "phase": "global|local",
    "label": "A",
    "index": 0,
    "status": "accepted|rejected|cancelled",
    "reason": "..."
  }
}
```

### 5.4 Frontend -> Ubuntu request JSON

Frontend actions go through Mac `dashboard_bridge`, which sends JSON UDP packets to Ubuntu.

#### UDP 6100: Keyboard Auto request

```json
{"cmd":"start","text":"ARES","return_mode":"manual"}
{"cmd":"cancel"}
{"cmd":"release"}
```

Text rules:

| Item | Detail |
|---|---|
| Allowed characters | `A-Z`, `0-9`, `, - . / ; = [ \\ ]`, space, apostrophe |
| Multi-character keys | use tokens: `<F1>`..`<F12>`, `<SPACE>` |
| Example | `ARES<SPACE><F10>` |
| Note | `F10` without brackets means `F`, `1`, `0` |

#### UDP 6101: AC control-byte override

```json
{"cmd":"home_pose"}
{"cmd":"initialize"}
{"control_byte":64}
```

#### UDP 6102: enable override

```json
{"cmd":"estop_on"}
{"cmd":"estop_release"}
```

#### UDP 6103: control mode override

```json
{"mode":"ik"}
{"mode":"manual"}
{"mode":"keyboard_auto"}
{"cmd":"clear"}
```

#### UDP 6104: IK READY request

```json
{"cmd":"ready"}
```

`ready` is Keyboard Auto keyboard-home capture after `RELEASE`. It is not the same as `JF.flags.bit1` STM READY.

#### UDP 6105: AC flags override

```json
{"usb_read":true}
{"usb_read":false}
{"cmd":"clear"}
```

### 5.5 Frontend Sensor Board And GPS Pose WS JSON

`dashboard_bridge` converts the sensor board ASCII line stream from [4.6](#46-sensor-board-ascii-telemetry) into two frontend-facing WebSocket message types.

#### `sensor_board`

Purpose: expose the latest decoded IMU/GPS cache and raw ID values.

Route: Mac `dashboard_bridge` -> browser WS 4100.

Shape:

```json
{
  "type": "sensor_board",
  "payload": {
    "source": "xbee_sensor_board",
    "updated_at_ms": 0,
    "present_ids": ["0x400", "0x415", "0x416"],
    "raw_values": {"0x400": 0.0, "0x415": 35.0, "0x416": 139.0},
    "gyro_rad_s": {"x": 0.0, "y": 0.0, "z": 0.0},
    "accel_mps2": {"x": 0.0, "y": 0.0, "z": 0.0},
    "mag_ut": {"x": 0.0, "y": 0.0, "z": 0.0},
    "roll_deg": 0.0,
    "pitch_deg": 0.0,
    "heading_deg": 0.0,
    "lat": 35.0,
    "lon": 139.0
  }
}
```

Field semantics:

| Field | Meaning |
|---|---|
| `source` | `xbee_sensor_board` for live XBee sensor-board data |
| `updated_at_ms` | wall-clock time when the newest cached sensor ID was received |
| `present_ids` | IDs currently present in the latest-value cache |
| `raw_values` | latest raw numeric value by `0xIII` ID |
| `gyro_rad_s`, `accel_mps2`, `mag_ut` | decoded 3-axis values, missing axes are `null` |
| `roll_deg`, `pitch_deg`, `heading_deg` | decoded orientation values, missing values are `null` |
| `lat`, `lon` | decoded GPS position, missing values are `null` |

#### `gps_pose`

Purpose: canonical frontend position payload for `/map` and `/antenna`.

Route: Mac `dashboard_bridge` -> browser WS 4100.

Shape:

```json
{
  "type": "gps_pose",
  "payload": {
    "type": "gps_pose",
    "source": "xbee_sensor_board",
    "lat": 35.0,
    "lon": 139.0,
    "updated_at_ms": 0,
    "heading_deg": 90.0,
    "roll_deg": 0.0,
    "pitch_deg": 0.0
  }
}
```

Emission rules:

- `gps_pose` is emitted only when both `lat` and `lon` are present and finite.
- `heading_deg`, `roll_deg`, and `pitch_deg` are optional and included only after their sensor IDs have been received.
- `/map` should use GPS for position and prefer IMU `heading_deg` for the rover heading arrow.
- `/antenna` uses the GPS position as rover position for GNSS tracking. The servo output remains an absolute servo angle, not a bearing.

## 6. Keyboard Auto Cross-Component Contract

This section ties the packets together. Wire formats stay in [4](#4-wire-packets-and-line-protocols) and [5](#5-json-packets).

### 6.1 Operator / Frontend Contract

| Action | Packet path | Meaning |
|---|---|---|
| `KBD_YAMAN` | frontend -> Ubuntu -> `AC.control_byte.bit2` | request STM wrist-90 pre-READY handshake |
| `READY` | UDP 6104 -> Ubuntu | capture keyboard-home posture after `RELEASE` |
| `START` | UDP 6100 -> Ubuntu | start mission after prerequisites and UI confirmations |
| `RELEASE` | UDP 6100 -> Ubuntu | hard reset Keyboard Auto context and return to `IK + parallel` |

`READY` capture is valid only once until `RELEASE`. A second capture requires `RELEASE`, then `KBD_YAMAN`, then `READY` again.

### 6.2 Start And Phase Handshake

| Stage | Ubuntu sends | miniPC / STM returns | Gate |
|---|---|---|---|
| mission start | miniPC `start` JSON + `AC.control_byte.bit7=KBD_START` | miniPC start ACK + `JF.flags.bit1` | both accepted |
| global X | `base_target_mm_j0`, X command active | `JF.flags.bit2` | X done |
| global YZ | `angle[0..2]`, `vel[0..2]=0`, YZ command active | `JF.flags.bit3` | global YZ done |
| local YZ | `angle[0..2]`, `vel[0..2]=0`, YZ command active | `JF.flags.bit4` | local YZ done |
| press | `KBD_EN`, then `KBD_PP` pulse | STM physical push/pull | wait/retract timer |
| home YZ | saved keyboard-home `angle[0..2]` | `JF.flags.bit5` | home YZ done |
| home X | saved keyboard-home X target | `JF.flags.bit6` | home X done |

### 6.3 miniPC Coordinate Contract

- miniPC publishes `keys[*].Xb_mm/Yb_mm/Zb_mm` as raw key centers in Ubuntu `base_frame`.
- miniPC receives `arm_pos` as the `arm_pos_link` / j3-center-near base-frame position.
- miniPC applies `models.*_config.offset_arm_to_cam_mm` as the j3-center -> camera-center vector.
- miniPC does not apply nyokki/tool/wrist compensation.
- Ubuntu converts raw key center -> nyokki tip wait point -> j3 center target.
- `base_target_mm_j0` is the J0/X target. It is encoded in `AC v6` as `int16`.

### 6.4 Deadman And Stop Semantics

The standard Keyboard Auto profile can run with `keyboard_auto_controller.require_deadman=false`.

This does not mean unguarded start. `START` still requires READY capture, fresh joint feedback, miniPC/STM readiness, and UI confirmations.

Stop paths:

- `RELEASE`: clears Keyboard Auto context and returns to `IK + parallel`.
- soft E-STOP / enable override: forces Ubuntu-side enable off.
- robot-side hard stop / power safety: final physical safety path.

## 7. Debugging And Logging Notes

Logging / display:

| Data | Support |
|---|---|
| `UF` | persisted under `.logs/uf_capture/` as JSONL + raw binary |
| raw `AC/JF/AI/MC` | visible through dashboard bridge telemetry as `raw_hex` |
| sensor board | decoded values are surfaced as WS `sensor_board` / `gps_pose`; bridge status includes last update and line-error counters |
| miniPC relay status | forwarded to dashboard bridge status |
| `kb_status` | published from Ubuntu and forwarded to frontend |

Caveats:

- There is no single full-mission raw log/replay file that captures `minipc_cmd`, `cmd_ack`, detection JSON, `kb_status`, and raw `AC/JF` together.
- miniPC detection JSON reports `timestamp` / `t_capture`, but this schema's `arm_pos` updates do not carry separate `arm_pos_timestamp` / `arm_pos_age_sec`.

## 8. Source Of Truth

| Scope | Path |
|---|---|
| Mac WS / telemetry / video | `tools/dashboard_bridge.py` |
| Mac manual-only optional path | `tools/gamepad_mode_sound_only_mac.py` |
| Mac miniPC relay | `tools/mac_minipc_udp_bridge.py` |
| Sensor board firmware | `tools/EmbeddedDevKit/ARES9/CubeIDE/workspace_1.19.0/ARES9_SensorBoard/Core/Src/main.c` |
| Sensor board BNO055 units | `tools/EmbeddedDevKit/ARES9/CubeIDE/workspace_1.19.0/ARES9_SensorBoard/Core/Inc/bno055.h` |
| Ubuntu uplink RX | `src/arm_ik_control/arm_ik_control/udp_joy_bridge.py` |
| Ubuntu unified AC TX | `src/arm_ik_control/arm_ik_control/udp_ac_tx.py` |
| Ubuntu JF RX | `src/arm_ik_control/arm_ik_control/udp_joint_state_rx.py` |
| Ubuntu JPEG RX | `src/arm_ik_control/arm_ik_control/udp_realsense_jpeg_rx.py` |
| Ubuntu keyboard detection RX | `src/arm_ik_control/arm_ik_control/udp_keyboard_detection_rx.py` |
| Ubuntu UI UDP RX | `src/arm_ik_control/arm_ik_control/udp_*_rx.py` |
| miniPC detection JSON schema | `minipc_ws/src/arm9_minipc/arm9_minipc/common.py` |
| miniPC command + ACK | `minipc_ws/src/arm9_minipc/arm9_minipc/cmd_protocol.py`, `minipc_ws/src/arm9_minipc/arm9_minipc/minipc_stack.py` |