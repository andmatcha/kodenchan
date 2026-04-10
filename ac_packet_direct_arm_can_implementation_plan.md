# ACv6 Packet to Arm CAN Direct Control Implementation Plan

## 目的

`/Users/jinaoyagi/workspace/ares/ARM_main_Uplink/ac_packet_can_send_summary.md` に整理されている既存の ACv6 パケット処理とアーム制御 CAN 送信を、この `kodenchan` プロジェクトのマイコン上で直接実行できる構成へ移植する。

既存システムでは次の 2 段構成になっている。

1. `ARM_communication_Uplink`: UART で `PacketACv6` を受信し、Manual Mode を uplink CAN `0x200`, `0x201` へ変換する。
2. `ARM_main_Uplink`: uplink CAN を受け、正規化、状態取得、PID、安全処理を経て motor CAN `0x200`, `0x1FF`, `0x208` を送信する。

このプロジェクトでは中間の uplink CAN を廃止し、次の 1 段構成にする。

```text
UART PacketACv6
  -> ACv6 検証 / mode 判定
  -> Manual 入力抽出
  -> 正規化 / snapshot / timeout
  -> feedback / limit / PID / 安全処理
  -> arm motor CAN 0x200, 0x1FF, 0x208
```

## 現状把握

- MCU / board: `nucleo_f303k8`, `STM32F303K8TX`
- UART: `USART2`, 115200 bps, DMA circular RX
- CAN: `hcan`, STM32 bxCAN 1 系統, PA11/PA12, 現設定は 1 Mbps 相当
- 既存コード:
  - `src/app.c`: 初期化と poll の入口
  - `src/modules/uart_async.c`: UART DMA ring 受信と printf 用 TX
  - `src/modules/can_bridge.c`: AC パケット風ストリームを読み、現状は uplink CAN `0x200`, `0x201` へ転送

今回の実装では `can_bridge` の責務を分割し、名前も「bridge」から「direct arm control」寄りに改める。既存の `uart_async` は driver 層として再利用する。

## 対象機能

初期実装の対象は Manual Mode とする。

- ACv6 packet length: 39 byte
- header: `"AC"`
- CRC: CRC16-CCITT-FALSE, `crc16` を除いた先頭 37 byte
- mode: `(flags >> 4) & 0x03`
- Manual Mode: `mode == 1`
- Manual 入力:
  - `current[0..6]`
  - `control_byte`
- motor CAN output:
  - `0x200`: motor0..3, high byte first
  - `0x1FF`: motor4..6, high byte first, byte6..7 は 0
  - `0x208`: 補助フラグ, 未使用 byte は 0

IK Mode と Keyboard Auto Mode はディスパッチ層に分岐点だけ用意し、初期段階では motor CAN に変換しない。

## レイヤー構成

### L1: HAL / Driver

責務:

- UART DMA circular RX の byte stream 読み出し
- CAN start, CAN TX mailbox 管理, CAN RX filter / callback
- GPIO から limit switch を読む
- timer tick の取得

この層は ACv6 やアーム軸の意味を知らない。

### L2: Protocol

責務:

- UART byte stream から `"AC"` header 基準で 39 byte packet を抽出
- `PacketACv6` の field 読み出し
- CRC16-CCITT-FALSE 検証
- mode 判定

この層は motor CAN ID や PID を知らない。

### L3: Application Service

責務:

- 有効な `PacketACv6` を mode ごとに振り分ける
- Manual Mode の packet を manual input snapshot に反映する
- packet timeout を監視し、入力途絶時に neutral へ戻す

この層は「ACv6 とアーム制御の接続点」に限定する。

### L4: Arm Control Domain

責務:

- raw current を normalized command に変換する
- neutral 入力時の停止保持 PID を計算する
- CAN feedback から RPM / angle を保持する
- limit switch による `base_horizon` の保護を適用する
- 最終的な motor command / auxiliary command を作る

この層は HAL の CAN 送信関数を直接呼ばない。

### L5: Arm CAN Protocol

責務:

- `ArmMotorCommand` を CAN frame `0x200`, `0x1FF`, `0x208` に pack する
- motor command は high byte first で詰める
- `0x208` 送信前に 8 byte 全体を 0 clear する

この層は UART や ACv6 を知らない。

## 推奨ディレクトリ構成

小さいファームウェアなので、細かすぎる分割は避けつつ、HAL 依存と制御ロジックを分ける。

```text
include/
  app.h
  drivers/
    uart_async.h
    can_bus.h
    limit_switch.h
  protocol/
    ac_packet_v6.h
    ac_stream_parser.h
    crc16_ccitt.h
    arm_can_protocol.h
  control/
    arm_types.h
    manual_input.h
    arm_state.h
    arm_control.h
  services/
    ac_direct_arm_service.h

src/
  app.c
  drivers/
    uart_async.c
    can_bus.c
    limit_switch.c
  protocol/
    ac_packet_v6.c
    ac_stream_parser.c
    crc16_ccitt.c
    arm_can_protocol.c
  control/
    manual_input.c
    arm_state.c
    arm_control.c
  services/
    ac_direct_arm_service.c
```

移行中は既存 `include/modules` / `src/modules` を残してもよいが、最終的には `modules/can_bridge.*` を廃止または互換ラッパー化する。

## 主要データ型案

```c
typedef struct {
  uint8_t seq;
  uint8_t flags;
  uint16_t current[7];
  uint16_t angle[3];
  int16_t vel[3];
  uint8_t control_byte;
  int16_t base_rel_mm_j0;
  uint16_t auto_flags;
  uint16_t fault_code;
  uint16_t crc16;
} AcPacketV6;

typedef struct {
  uint16_t raw_current[7];
  uint8_t control_byte;
  uint32_t updated_at_ms;
  bool valid;
} ManualInputSnapshot;

typedef struct {
  int16_t rpm[7];
  uint16_t angle[7];
  bool limit_left;
  bool limit_right;
} ArmState;

typedef struct {
  int16_t motor[7];
  uint8_t keyboard_nyokki_enabled;
  uint8_t usb_nyokki_push;
  uint8_t usb_nyokki_pull;
} ArmMotorCommand;
```

`AcPacketV6` は packed struct に頼りすぎず、初期実装では raw byte から little endian helper で読み出す方針にする。アライメント、padding、endian の混入を避けやすい。

## PacketACv6 処理仕様

`protocol/ac_stream_parser`:

- UART から受け取った byte stream を内部 ring / linear buffer に蓄積する。
- `"AC"` header まで読み飛ばす。
- 39 byte 未満なら次回 poll へ持ち越す。
- 39 byte 揃ったら CRC を検証する。
- CRC 不一致なら 1 byte ずらして再同期を試みる。
- 正常 packet だけ `AcPacketV6` として返す。

`protocol/ac_packet_v6`:

- offset は summary の PacketACv6 表に合わせる。
- `mode = (flags >> 4) & 0x03` を公開関数で返す。
- Manual Mode では `current[0..6]` と `control_byte` のみを service 層へ渡す。

## Manual 入力仕様

`control/manual_input`:

- raw neutral は `255`
- 変換式は次の通り。

```c
normalized[i] = (raw_current[i] - 255) * 64;
```

- raw が `0..511` の範囲にある前提なら normalized は `-16320..16384`
- normalized は `-16384..16384` に収まることを検査する。
- 範囲外は初期段階では packet 破棄または neutral fallback とし、実機安全確認後に `Error_Handler()` 方針を決める。既存 summary の `ARM_main_Uplink` は `Error_Handler()`。

Snapshot:

- `ManualInputSnapshot` は 39 byte ACv6 packet 単位で一括更新する。
- 旧構成の `0x200` / `0x201` 揃い待ちは不要。
- ただし「制御周期中に半端な値を読まない」ため、snapshot のコピーは短い critical section で行う。

Timeout:

- `MANUAL_UPLINK_TIMEOUT_MS = 1000` ms を初期値にする。
- timeout 時は raw current を全軸 `255`、`control_byte = 0` に戻す。
- timeout 後も CAN 送信周期では neutral / PID 停止保持の指令を出す。

## Arm Control 仕様

`control/arm_control`:

1. snapshot から raw current を読む。
2. normalized command に変換する。
3. RPM / angle feedback と limit switch を含む `ArmState` を読む。
4. normalized が 0 の軸は目標 RPM 0 の PID 出力を使う。
5. normalized が 0 ではない軸は manual 入力を優先し、その軸の PID integral / previous error を reset する。
6. `base_horizon` は左右 limit switch の状態を優先して保護する。
7. `ArmMotorCommand` を返す。

Limit rule:

| 状態 | motor0 command |
| --- | ---: |
| 左右両方 pressed | `0` |
| right pressed かつ正方向入力 | `-2000` |
| left pressed かつ負方向入力 | `+2000` |
| それ以外で入力あり | `normalized[0]` |
| それ以外で入力なし | `rpm_output[0]` |

PID parameter は既存 `ARM_main_Uplink` の値を移植する。未移植の段階では PID を 0 出力にして直接入力だけを通す feature flag を用意し、実機試験時に段階的に有効化する。

## CAN 仕様

`protocol/arm_can_protocol` で以下を pack し、`drivers/can_bus` で送信する。

### `0x200`

| byte | 内容 |
| ---: | --- |
| 0 | motor0 MSB |
| 1 | motor0 LSB |
| 2 | motor1 MSB |
| 3 | motor1 LSB |
| 4 | motor2 MSB |
| 5 | motor2 LSB |
| 6 | motor3 MSB |
| 7 | motor3 LSB |

### `0x1FF`

| byte | 内容 |
| ---: | --- |
| 0 | motor4 MSB |
| 1 | motor4 LSB |
| 2 | motor5 MSB |
| 3 | motor5 LSB |
| 4 | motor6 MSB |
| 5 | motor6 LSB |
| 6 | `0x00` |
| 7 | `0x00` |

### `0x208`

| byte | 内容 |
| ---: | --- |
| 0 | `keyboard_nyokki_enabled` |
| 1 | `0x00` |
| 2 | `0x00` |
| 3 | `usb_nyokki_push` |
| 4 | `usb_nyokki_pull` |
| 5 | `0x00` |
| 6 | `0x00` |
| 7 | `0x00` |

`control_byte` の初期 bit 解釈:

| bit | 出力先 |
| ---: | --- |
| 0 | `keyboard_nyokki_enabled` |
| 3 | `usb_nyokki_push` |
| 4 | `usb_nyokki_pull` |

bit5 initialize などは summary 上でも manual / keyboard auto 間の定義差があるため、初期実装では予約扱いにしてログまたは status counter のみで観測する。

## CAN feedback 受信

初期段階では motor command 送信を優先し、feedback 受信は次の順序で追加する。

1. CAN RX interrupt / filter を有効化する。
2. RPM feedback `0x202..0x206` を `ArmState.rpm[1..5]` に反映する。
3. encoder feedback `0x301..0x304` を `ArmState.angle[1..4]` に反映する。
4. `0x300` encoder は使用機体で必要なら `angle[0]` として追加する。

`STM32F303K8` は CAN が 1 系統なので、旧資料の CAN1/CAN2 の区別は消える。今回の single CAN bus では uplink CAN `0x200` / `0x201` を送らないため、motor command `0x200` との ID 衝突は起こさない設計にする。

## 実装ステップ

1. `uart_async` を driver 層へ移す、または既存 path のまま driver 責務として扱う。
2. `crc16_ccitt` と `ac_packet_v6` を追加し、39 byte ACv6 の header / CRC / mode / current 読み出しを単体テスト可能にする。
3. `ac_stream_parser` を追加し、既存 `can_bridge.c` 内の stream 抽出処理を移植する。
4. `arm_types`, `manual_input` を追加し、raw current から normalized command への変換と timeout neutral 化を実装する。
5. `arm_can_protocol` を追加し、`0x200`, `0x1FF`, `0x208` の pack 関数を実装する。
6. `can_bus` を追加し、標準 ID / DLC 8 の送信、TX timeout、将来の RX callback 登録をまとめる。
7. `limit_switch` を追加し、PA0 / PA1 を `base_horizon` limit として読む。ピン割り当てが違う場合は `main.h` の define を更新する。
8. `arm_state` を追加し、CAN feedback と limit の snapshot を管理する。
9. `arm_control` を追加し、normalized manual 入力、PID、limit 保護から `ArmMotorCommand` を作る。
10. `ac_direct_arm_service` を追加し、`app_poll()` から packet 取り込み、snapshot 更新、周期制御、CAN 送信を行う。
11. `app.c` を新 service 初期化 / poll に差し替え、旧 `can_bridge` 呼び出しを停止する。
12. 実機前に pack 関数と ACv6 parser のテストを追加し、想定 byte order を固定する。

## 制御周期案

初期値:

- UART packet 取り込み: `app_poll()` 毎
- motor CAN 送信: 5 ms または 10 ms 周期
- manual timeout: 1000 ms
- CAN TX mailbox wait timeout: 10 ms 以下

`app_poll()` 内では次の順に処理する。

```text
while (ac_stream_parser_next(&packet)) {
  if (ac_packet_v6_is_valid_manual(&packet)) {
    manual_input_update(&snapshot, &packet);
  }
}

if (control_period_elapsed()) {
  manual_input_apply_timeout(&snapshot);
  arm_state_update_from_feedback_and_gpio(&state);
  arm_control_make_command(&snapshot, &state, &command);
  arm_can_protocol_pack_all(&command, frames);
  can_bus_send(frames[0x200]);
  can_bus_send(frames[0x1FF]);
  can_bus_send(frames[0x208]);
}
```

## テスト計画

Host / unit test:

- CRC16-CCITT-FALSE の既知入力
- 正常 39 byte ACv6 packet の parse
- header 不一致 packet の破棄
- CRC 不一致 packet の破棄と再同期
- mode 判定: `flags` bit4-5
- raw current 正規化: `0 -> -16320`, `255 -> 0`, `511 -> 16384`
- `control_byte` bit0 / bit3 / bit4 の展開
- motor CAN `0x200`, `0x1FF`, `0x208` の byte order
- timeout 時の neutral 化
- limit switch rule による motor0 の上書き

Firmware / bench test:

- UART に ACv6 Manual packet を流し、CAN analyzer で `0x200`, `0x1FF`, `0x208` のみが出ることを確認する。
- 旧 uplink CAN `0x201` が出ないことを確認する。
- neutral packet で全 motor command が PID 停止保持または 0 に戻ることを確認する。
- `control_byte` の bit 操作で `0x208` byte0 / byte3 / byte4 が変わることを確認する。
- packet 停止から 1000 ms 後に neutral fallback することを確認する。
- limit switch 入力時に motor0 が停止または退避方向へ置き換わることを確認する。

## 未確定点

- PA0 / PA1 が実機の `base_horizon` left / right limit switch と一致しているか。
- `control_byte` bit5 initialize を Manual Mode で使うか。
- PID parameter と対象軸数を `ARM_main_Uplink` からどの値で移植するか。
- CAN feedback の `0x300` encoder を使う機体か。
- 起動直後、feedback 未受信時の PID 出力を 0 固定にするか、既存同様に現在値未初期化でも制御に入るか。
- `AutoRetransmission = DISABLE` のままで実機 CAN に対して十分か。

## 完了条件

- ACv6 39 byte packet を UART から受信し、header / CRC を検証して Manual Mode のみ処理する。
- Manual Mode packet から中間 uplink CAN を送らず、直接 motor CAN `0x200`, `0x1FF`, `0x208` を送信する。
- byte order と CAN ID が summary の motor command 側仕様に一致する。
- timeout、limit switch、範囲外値に対する安全動作が実装されている。
- parser / packer / normalization / limit rule のテストがある。
- `can_bridge` に残っている旧 uplink CAN 変換が app 経路から外れている。
