# kodenchan UART–CAN bridge

Nucleo-STM32F303K8 の ST-LINK micro USB Virtual COM Port（USART2）と、基板上の CAN の間を双方向変換するファームウェアです。変換仕様は [`docs/uart_can_bridge.md`](docs/uart_can_bridge.md) に従います。

## 通信設定

| インターフェース | 設定 |
| --- | --- |
| USART2 / Virtual COM Port | 115200 bps、8 data bits、parity none、1 stop bit、flow control none |
| CAN | 1 Mbps、11 bit standard data frame、Normal mode |

USART2 は PA2（TX）/ PA15（RX）で ST-LINK の micro USB Virtual COM Port につながります。UART RX/TX は DMA を使用します。

## UART → CAN（uplink）

USART2はCRC-16/CCITT-FALSE付きの39 byte固定長`PacketACv6`を受理します。`flags`のmodeに応じてCAN出力を切り替え、headerまたはCRCが不正なpacketは送信しません。旧`M` / `I` / `B` packetは受理しません。

| PacketACv6 mode | CAN output |
| --- | --- |
| `0`: IK | `0x210` / `0x211` / `0x212`、各DLC 8 |
| `1`: Manual | `0x200` / `0x201`、各DLC 8 |
| `2`: Keyboard Auto | `0x501` / `0x502` / `0x503`、各DLC 8 |

mode 3は予約値としてCANを送信しません。IKの`vel`、Keyboard Autoの`fault_code`、両modeの`current[0]`, `[1]`, `[5]`, `[6]`もPacketACv6からCANへ反映します。

また、`CAN_ID,DATA\r` または `CAN_ID,DATA\n` の ASCII 行を CAN standard frame（DLC 4）へ変換します。`DATA` は signed 32 bit として読み、CAN data には big-endian で格納します。CAN ID `0` は受理しますが送信しません。

## CAN → UART（downlink）

CAN hardware filter が受理する ID は `0x203..0x208`、`0x300..0x3FF`、`0x400..0x4FF`、`0x500..0x5FF` です。

- `0x203` と `0x204` の最新値を約 50 Hz の 16 byte `JF` packet に変換します。
- `0x205`、`0x206`、`0x207`、`0x208` の DLC 8 frame を順番に連結し、40 byte `UF` packetへ変換します。frame 間隔が 250 ms を超えた場合や順序が不正な場合は組み立てを破棄します。
- その他の受理 ID は最新値を最大16 ID分保持し、約 10 Hz で `0x%03X,%d\r\n` 形式へ変換します。`0x410..0x413` は DLC 4 以上なら big-endian signed int32、その他はゼロ埋めされた8 byteの合計値です。
- UF の活動中と最後の UF 活動から250 msは、JFとASCII出力を抑止します。

`JF` と `UF` は共通の sequence counter と CRC-16/CCITT-FALSE を使用します。

## ビルドと書き込み

```sh
pio run
pio run -t upload
```

## テスト

STM32 HALのUART/CAN入出力とtickをモック化し、実際のbridge serviceとCRC実装をNative環境でテストします。

```sh
pio test -e native
```

テスト対象にはPacketACv6のIK / Manual / Keyboard Auto全mode、uplink ASCII、`JF` / `UF`、downlink ASCIIと、CRC不正、予約mode、旧`M/I/B`拒否、DLC不足、UF順序・timeout・優先期間、最新値table、tick wraparoundが含まれます。

CubeMX管理の初期化設定は今回変更していません。将来 CubeMX で反映したい改善は [`docs/cubemx_uart_can_bridge_future_improvements.md`](docs/cubemx_uart_can_bridge_future_improvements.md) にまとめています。

## 実装構成

```text
main.c
  -> app_init() / app_poll()
    -> services/uart_can_bridge_service
       -> drivers/uart_async
       -> drivers/can_bus
       -> protocol/crc16_ccitt
```

以前のPID、ボタン送信、CANモニターモードは撤去されています。
