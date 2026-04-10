# kodenchan

STM32F303K8 Nucleo で `PacketACv6` を UART から受け取り、アーム用 motor CAN に直接変換して送信するファームウェアです。旧構成にあった中間 uplink CAN を挟まず、`PacketACv6` の Manual Mode 入力を `0x200`, `0x1FF`, `0x208` の CAN フレームへ変換します。

## 簡単な仕様

- MCU / board: `nucleo_f303k8`, `STM32F303K8TX`
- 使用基板: Nucleo を「こでんちゃん基板」のソケットへ取り付けて使う
- UART: `USART2`, 115200 bps, 8N1, DMA circular RX。PC 接続を想定
- CAN: STM32 bxCAN 1系統, PA11 = CAN_RX, PA12 = CAN_TX, 1 Mbps 設定。こでんちゃん基板上の CAN transceiver と RJ-45 経由でアーム側 CAN へ接続し、分電基板1 (Distribution Board 1) に直結できる
- 入力 packet: `PacketACv6`, 39 byte 固定, header は `AC`, CRC は CRC16-CCITT-FALSE
- 主な制御: `PacketACv6` の Manual Mode (`mode == 1`) を motor command と補助制御フラグへ変換
- 制御周期: 10 ms ごとに CAN command を送信
- timeout: Manual 入力が 1000 ms 更新されないと neutral 入力に戻す
- CAN監視モード: PA0 と PA1 を同時に 1000 ms 長押しすると通常制御と CAN RX monitor を切り替える

## 主な使い方

想定する構成は、Nucleo をこでんちゃん基板へ取り付け、こでんちゃん基板の RJ-45 からアーム側の分電基板1 (Distribution Board 1) へ直結する形です。こでんちゃん基板には CAN transceiver が載っており、RJ-45 のアーム側と対応した線に CANH / CANL が接続されています。USART2 は PC と接続し、PC 側から `PacketACv6` を送ります。

PC 側の送信プログラムは、特に [andmatcha/ds4map](https://github.com/andmatcha/ds4map) と組み合わせて使う想定です。ds4map の起動方法や controller mapping は ds4map 側の README を参照してください。この firmware 側では、USART2 に届いた `PacketACv6` を解釈して CAN `0x200`, `0x1FF`, `0x208` へ周期送信します。

1. Nucleo をこでんちゃん基板のソケットへ取り付ける。
2. こでんちゃん基板の RJ-45 をアーム側の分電基板1 (Distribution Board 1) へ接続する。
3. PC と Nucleo の USART2 を接続し、PC 側で `PacketACv6` を送れる状態にする。
4. PlatformIO で build / upload する。

```sh
pio run
pio run -t upload
```

5. 起動後は通常制御モードで動き、UART から受けた Manual Mode packet を CAN `0x200`, `0x1FF`, `0x208` へ周期送信する。
6. PA0 / PA1 は active low のテスト送信ボタンとして使える。PA0 と PA1 を同時に 1000 ms 長押しすると CAN RX monitor へ切り替わり、UART へ `CAN RX ...` のログを出す。もう一度同時長押しすると通常制御に戻る。

通常制御中は CAN TX も UART へ `CAN TX 0x200: ...` の形式でログ出力されます。

## 全体構成

```text
main.c
  -> app_init() / app_poll()
    -> services/uart_packet_to_can_service
      -> drivers/uart_async
      -> protocol/ac_stream_parser
      -> protocol/ac_packet_v6
      -> control/manual_input
      -> drivers/can_bus -> control/arm_state
      -> control/arm_control
      -> protocol/arm_can_protocol
      -> drivers/can_bus
    -> services/button_can_tx_service
      -> drivers/can_bus
```

各レイヤーの役割は次の通りです。

| レイヤー | 主なファイル | 役割 |
| --- | --- | --- |
| HAL / board init | `src/main.c`, `src/stm32f3xx_hal_msp.c` | UART, DMA, CAN, GPIO, clock の初期化。`main()` から `app_init()` 後に `app_poll()` を回す。 |
| app | `src/app.c` | 通常制御モードと CAN RX monitor の切り替え、各 service の呼び出し順序を決める。 |
| service | `src/services/uart_packet_to_can_service.c` | UART byte stream を packet として取り込み、manual snapshot 更新、CAN feedback 反映、10 ms 周期の制御実行、CAN 送信までを接続する。 |
| service | `src/services/button_can_tx_service.c` | PA0 / PA1 の入力を読み、単独押下時の設定済み CAN frame 送信と同時長押しのモード切替要求を扱う。押した直後の一瞬の揺れは無視する。 |
| driver | `src/drivers/uart_async.c` | USART2 の DMA circular RX と DMA TX ring buffer を提供する。packet やアームの意味は知らない。 |
| driver | `src/drivers/can_bus.c` | CAN filter 設定、標準 ID / DLC 8 の送信、RX polling、UART への CAN log 出力を提供する。 |
| protocol | `src/protocol/ac_stream_parser.c` | UART byte stream から `AC` header を探して 39 byte の有効 packet を抽出する。CRC 不一致時は 1 byte ずらして再同期する。 |
| protocol | `src/protocol/ac_packet_v6.c` | `PacketACv6` の header / CRC 検証、little endian field decode、mode 判定を行う。 |
| protocol | `src/protocol/arm_can_protocol.c` | `ArmMotorCommand` を CAN `0x200`, `0x1FF`, `0x208` の 8 byte data へ pack する。 |
| control | `src/control/manual_input.c` | raw current と control byte を保持し、timeout と正規化を行う。 |
| control | `src/control/arm_state.c` | CAN feedback から `ArmState.rpm[]` / `ArmState.angle[]` を更新する。 |
| control | `src/control/arm_control.c` | manual 入力、feedback、RPM PID から最終的な motor command を作る。 |

## PacketACv6

`PacketACv6` は 39 byte 固定長です。multi-byte field は little endian で decode します。CRC は byte 0 から byte 36 までの 37 byte に対する CRC16-CCITT-FALSE で、packet 末尾の byte 37..38 に little endian で入ります。

| byte | field | 型 | 説明 |
| ---: | --- | --- | --- |
| 0 | header[0] | char | `A` |
| 1 | header[1] | char | `C` |
| 2 | `seq` | `uint8_t` | sequence |
| 3 | `flags` | `uint8_t` | bit 4..5 が mode |
| 4..17 | `current[0..6]` | `uint16_t[7]` | Manual 入力の raw current。2 byte ずつ little endian。 |
| 18..23 | `angle[0..2]` | `uint16_t[3]` | ACv6 側 angle。現実装では decode するが制御出力には使わない。 |
| 24..29 | `vel[0..2]` | `int16_t[3]` | ACv6 側 velocity。現実装では decode するが制御出力には使わない。 |
| 30 | `control_byte` | `uint8_t` | 補助制御フラグ。CAN `0x208` へ bit 展開する。 |
| 31..32 | `base_rel_mm_j0` | `int16_t` | 現実装では decode するが制御出力には使わない。 |
| 33..34 | `auto_flags` | `uint16_t` | 現実装では decode するが制御出力には使わない。 |
| 35..36 | `fault_code` | `uint16_t` | 現実装では decode するが制御出力には使わない。 |
| 37..38 | `crc16` | `uint16_t` | CRC16-CCITT-FALSE。 |

mode は次の式で取り出します。

```c
mode = (flags >> 4) & 0x03;
```

現実装で処理するのは `mode == 1` の Manual Mode だけです。Manual Mode 以外の packet は parse されても manual snapshot を更新しません。入力が更新されなければ 1000 ms 後に neutral fallback します。

## PacketACv6 から制御値への変換

Manual Mode では `current[0..6]` と `control_byte` を使います。

`current[i]` は `raw_current[i]` として保持し、制御周期で次の式により `int16_t` の command に変換します。

```c
normalized[i] = (raw_current[i] - 255) * 64;
```

代表値は次の通りです。

| raw current | normalized command |
| ---: | ---: |
| 0 | -16320 |
| 254 | -64 |
| 255 | 0 |
| 256 | 64 |
| 511 | 16384 |

`normalized` は `-16384..16384` の範囲で検査します。範囲外なら snapshot を neutral に戻し、再度 neutral として変換します。neutral は全軸 `raw_current = 255`, `control_byte = 0` です。

## 軸、joint、CAN data の対応

現実装の軸 enum と CAN 送信先の関係は次の通りです。motor command は `int16_t` を big endian で CAN data に入れます。

| axis | enum | 制御対象 | PacketACv6 入力 | CAN送信先 | feedback 反映 |
| ---: | --- | --- | --- | --- | --- |
| 0 | `ARM_AXIS_BASE_HORIZON` | base horizon | `current[0]` | `0x200` byte 0..1 = `motor[0]` | angle `0x300` byte 0..1 little endian |
| 1 | `ARM_AXIS_BASE_ROLL` | base roll | `current[1]` | `0x200` byte 2..3 = `motor[1]` | rpm `0x202` byte 2..3 big endian, angle `0x301` |
| 2 | `ARM_AXIS_JOINT1` | joint1 | `current[2]` | `0x200` byte 4..5 = `motor[2]` | rpm `0x203`, angle `0x302` |
| 3 | `ARM_AXIS_JOINT2` | joint2 | `current[3]` | `0x200` byte 6..7 = `motor[3]` | rpm `0x204`, angle `0x303` |
| 4 | `ARM_AXIS_JOINT3` | joint3 | `current[4]` | `0x1FF` byte 0..1 = `motor[4]` | rpm `0x205`, angle `0x304` |
| 5 | `ARM_AXIS_JOINT4` | joint4 | `current[5]` | `0x1FF` byte 2..3 = `motor[5]` | rpm `0x206` |
| 6 | `ARM_AXIS_GRIPPER` | gripper | `current[6]` | `0x1FF` byte 4..5 = `motor[6]` | 現実装ではなし |

通常制御時の CAN RX filter は `0x200..0x207` と `0x300..0x307` を受ける設定ですが、`ArmState` に反映するのは上表の feedback ID です。CAN RX monitor では全 ID を受ける filter に切り替えます。

## CAN 送信 frame

通常制御モードでは 10 ms 周期で次の 3 frame を送信します。

### `0x200`: motor 0..3

| byte | 内容 |
| ---: | --- |
| 0 | `motor[0]` MSB |
| 1 | `motor[0]` LSB |
| 2 | `motor[1]` MSB |
| 3 | `motor[1]` LSB |
| 4 | `motor[2]` MSB |
| 5 | `motor[2]` LSB |
| 6 | `motor[3]` MSB |
| 7 | `motor[3]` LSB |

### `0x1FF`: motor 4..6

| byte | 内容 |
| ---: | --- |
| 0 | `motor[4]` MSB |
| 1 | `motor[4]` LSB |
| 2 | `motor[5]` MSB |
| 3 | `motor[5]` LSB |
| 4 | `motor[6]` MSB |
| 5 | `motor[6]` LSB |
| 6 | `0x00` |
| 7 | `0x00` |

### `0x208`: 補助制御

`control_byte` の bit を 1 byte ずつ展開して送ります。各 byte は `0x00` または `0x01` です。

| `control_byte` bit | `0x208` byte | 名前 |
| ---: | ---: | --- |
| 0 | 0 | `kbd_pp` |
| 1 | 1 | `kbd_en` |
| 2 | 2 | `kbd_yaman` |
| 3 | 3 | `nyokki_push` |
| 4 | 4 | `nyokki_pull` |
| 5 | 5 | `initialize` |
| 6 | 6 | `home` |
| 7 | 7 | `kbd_start` |

## 実際に送信する値

Manual Mode packet を受けた場合、送信値は次の流れで決まります。

1. `current[i]` を `(current[i] - 255) * 64` で `normalized[i]` にする。
2. `normalized[i] != 0` の軸は、その値をそのまま `motor[i]` として送る。
3. `normalized[i] == 0` の軸は、feedback RPM を 0 に寄せる PID 出力 `rpm_output[i]` を `motor[i]` として送る。
4. `control_byte` の bit0..7 を `0x208` の byte0..7 へ展開して送る。

PID 出力は `-16384..16384` に clamp されます。manual 入力は同じ範囲に入っているか検査し、外れていれば neutral fallback します。PID gain の初期値は次の通りです。

| axis | Kp | Ki | Kd |
| ---: | ---: | ---: | ---: |
| 0 | 1.0 | 0.0 | 0.0 |
| 1 | 1.0 | 0.0 | 0.0 |
| 2 | 2.2 | 0.0 | 0.0 |
| 3 | 2.0 | 0.0 | 0.0 |
| 4 | 0.2 | 0.0 | 0.0 |
| 5 | 0.13 | 0.0 | 0.03 |
| 6 | 1.0 | 0.0 | 0.0 |

PA0 / PA1 のテスト送信では、通常制御モードかつ同時長押し中でない場合に次の CAN frame を送ります。`BUTTON_CAN_TX_REPEAT_WHILE_PRESSED` が有効なので、押し続けると 10 ms 周期で繰り返し送信します。

| 入力 | CAN ID | data | 意味 |
| --- | ---: | --- | --- |
| PA0 active low | `0x200` | `00 00 00 00 FF FF 00 00` | `motor[2]`, joint1 に `-1` を送る |
| PA1 active low | `0x200` | `00 00 00 00 00 01 00 00` | `motor[2]`, joint1 に `+1` を送る |

## 制御の内部実装

`uart_packet_to_can_service_poll()` は `app_poll()` から繰り返し呼ばれ、次の順序で処理します。

1. `uart_async_read()` で USART2 DMA RX buffer から最大 32 byte ずつ読み出し、`ac_stream_parser` に流す。
2. `ac_stream_parser_next()` が返す valid packet をすべて消費する。
3. packet が Manual Mode なら `manual_input_update_from_packet()` で snapshot を更新する。
4. `can_bus_poll()` で CAN feedback を取り込み、`arm_state_handle_can_feedback()` で RPM / angle を更新する。
5. 10 ms 経過していれば `manual_input_apply_timeout()` を実行する。
6. snapshot を `ManualInput` に正規化する。範囲外なら neutral へ戻す。
7. `arm_control_make_command()` で manual command と PID 出力を合成する。
8. `arm_can_protocol_pack_manual_command()` で `0x200`, `0x1FF`, `0x208` へ pack する。
9. `can_bus_send()` で各 frame を送信する。CAN 送信失敗は `Error_Handler()` へ入る。

neutral 入力時も周期送信は続きます。これにより、入力途絶や範囲外入力の後も「何も送らない」状態ではなく、neutral または RPM 0 PID の指令を継続します。

## モード切り替え

このプロジェクトには 2 種類の mode があります。

### PacketACv6 の mode

`PacketACv6.flags` の bit4..5 から取得する mode です。

| mode | 現実装の扱い |
| ---: | --- |
| `0` | parse はできるが manual snapshot は更新しない |
| `1` | Manual Mode として処理する |
| `2` | parse はできるが manual snapshot は更新しない |
| `3` | parse はできるが manual snapshot は更新しない |

Manual Mode 以外を受けた場合、直ちに CAN command を停止するのではなく、最後の Manual 入力が 1000 ms を超えて更新されない時点で neutral fallback します。

### ファームウェアの動作 mode

`app.c` が持つ通常制御モード / CAN RX monitor の切り替えです。PA0 と PA1 を同時に 1000 ms 長押しすると切り替わります。

| 動作 mode | CAN TX | CAN RX filter | UART log | 主な処理 |
| --- | --- | --- | --- | --- |
| 通常制御モード | 有効 | `0x200..0x207`, `0x300..0x307` | `CAN TX ...` | `PacketACv6` を motor CAN へ変換し、button 単独押下 frame も送信する。 |
| CAN RX monitor | 無効 | 全 ID | `CAN RX ...` | CAN 受信内容を UART へ表示する。manual 入力と PID は reset され続け、CAN TX は行わない。 |

CAN RX monitor に入ると `can_bus_set_tx_enabled(false)` で TX を止め、未送信 mailbox を abort します。通常制御へ戻ると parser / manual input / PID を reset し、通常 filter に戻してから TX を再度有効にします。

## 変更しやすい設定

| 設定 | 場所 | 現在値 |
| --- | --- | --- |
| 制御周期 | `SERVICE_CONTROL_PERIOD_MS` | 10 ms |
| manual timeout | `MANUAL_INPUT_TIMEOUT_MS` | 1000 ms |
| CAN TX mailbox wait | `CAN_TX_TIMEOUT_MS` | 10 ms |
| button poll | `BUTTON_CAN_TX_POLL_PERIOD_MS` | 10 ms |
| ボタン入力の判定待ち時間 | `BUTTON_CAN_TX_DEBOUNCE_MS` | 20 ms。押した直後の一瞬の揺れを無視するための時間。 |
| button repeat | `BUTTON_CAN_TX_REPEAT_WHILE_PRESSED` | 有効 |
| button repeat period | `BUTTON_CAN_TX_REPEAT_PERIOD_MS` | 10 ms |
| mode toggle hold | `BUTTON_CAN_TX_MODE_TOGGLE_HOLD_MS` | 1000 ms |
| PA0 test frame | `BUTTON_CAN_TX_PA0_STD_ID`, `BUTTON_CAN_TX_PA0_DATA` | `0x200`, `00 00 00 00 FF FF 00 00` |
| PA1 test frame | `BUTTON_CAN_TX_PA1_STD_ID`, `BUTTON_CAN_TX_PA1_DATA` | `0x200`, `00 00 00 00 00 01 00 00` |

## 現実装の注意点

- `PacketACv6` の `angle`, `vel`, `base_rel_mm_j0`, `auto_flags`, `fault_code` は decode していますが、現在の CAN command 生成には使っていません。
- Manual Mode 以外の packet は現状では制御対象外です。
- CAN は標準 ID / data frame / DLC 8 で送信します。
- motor command は CAN data 上では big endian、`PacketACv6` の multi-byte field は little endian です。
- feedback RPM は `0x202..0x206` の byte 2..3 を big endian signed として読みます。
- feedback angle は `0x300..0x304` の byte 0..1 を little endian unsigned として読みます。
