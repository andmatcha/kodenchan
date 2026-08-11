# uplink / downlink における UART–CAN 間の受け渡し

## 1. 調査範囲と要約

`xbee_communication/remote/uplink` と `downlink` の現行 C 実装を調査した。

- **uplink**: USART1/USART3 で受信したバイナリパケットまたは ASCII 行を CAN1 の標準データフレームへ変換する。
- **downlink**: CAN1 で受信したデータを `JF` / `UF` バイナリパケットまたは ASCII 行へ変換し、USART6 から送信する。USART3 で受信した ASCII 行も、最新値として USART6 へ中継する。
- CAN1 は両側とも 11 bit 標準 ID、Normal mode、実装および `.ioc` 上は 1 Mbit/s である。
- 実際に通信処理で使う UART はすべて 8N1、フロー制御なし。現行 `main.c` では uplink の USART1/3 と downlink の USART3/6 は 115200 bit/s である。ただし uplink の `.ioc` には USART1/3 が 57600 bit/s と残っており、不一致がある。

概略は次の通りである。

```text
上り:  UART (USART1/3) -> uplink -> CAN1
下り:  CAN1 -> downlink -> UART (USART6)
                         ^
                         +-- USART3 の ASCII 最新値も合流
```

## 2. uplink: UART から CAN

### 2.1 UART 入力の振り分け

| UART | 入力 | 受信方法 | 備考 |
|---|---|---|---|
| USART1 (RX: PA10) | アーム用 `M` / `I` / `B` バイナリ、ローバー用 ASCII | 512 byte circular DMA | ASCII は先頭が文字 `0` の行だけを開始と認識する |
| USART3 (RX: PC5) | ローバー用 ASCII | 512 byte circular DMA | 行頭の制限なし |

改行は LF または CR のどちらでも行終端になる。USART1 は先頭バイトが `M`、`I`、`B` ならヘッダ別の固定長だけを集め、それ以外は `0` を除いて読み捨てる。

### 2.2 アーム用 UART バイナリパケットの共通則

- 構造体は `packed` で、複数 byte 整数と末尾 CRC は STM32 のネイティブな **little-endian** で格納される。
- CRC は **CRC-16/CCITT-FALSE**（初期値 `0xFFFF`、多項式 `0x1021`）で、末尾の `crc16` 自身を除く全 byte が計算対象である。
- CRC または固定長が不正なら CAN へ送らない。
- UART の `seq` は検証・並べ替えには使わず、CAN フレームにも渡さない。

#### `M` (Manual、19 byte)

| UART byte | フィールド |
|---:|---|
| 0 | header = `M` |
| 1 | `seq` (`uint8`) |
| 2..15 | `current[0..6]` (`uint16` × 7、LE) |
| 16 | `control_byte` |
| 17..18 | `crc16` (LE) |

| CAN ID | DLC | CAN data byte 0..7 |
|---:|---:|---|
| `0x200` | 8 | `current[0]`, `[1]`, `[2]`, `[3]` を各 uint16 LE |
| `0x201` | 8 | `current[4]`, `[5]`, `[6]` を各 uint16 LE、byte 6=`control_byte`、byte 7=`0` |

#### `I` (IK、19 byte)

| UART byte | フィールド |
|---:|---|
| 0 | header = `I` |
| 1 | `seq` (`uint8`) |
| 2..9 | `current[0..3]` (`uint16` × 4、LE) |
| 10..15 | `angle[0..2]` (`uint16` × 3、LE) |
| 16 | `control_byte` |
| 17..18 | `crc16` (LE) |

| CAN ID | DLC | CAN data byte 0..7 |
|---:|---:|---|
| `0x210` | 8 | `angle[0]`, `[1]`, `[2]` を各 uint16 LE、byte 6..7=`0` |
| `0x211` | 8 | `vel[0..2]` を各 int16 LE、byte 6=`control_byte`、byte 7=`0` |
| `0x212` | 8 | UART の `current[0]`, `[1]`, `[2]`, `[3]` を各 uint16 LE |

現行 UART パケットに `vel` フィールドはなく、正規化用構造体がゼロ初期化されるため、`0x211` の `vel[0..2]` は常に `0` になる。

#### `B` (Keyboard Auto、15 byte)

| UART byte | フィールド |
|---:|---|
| 0 | header = `B` |
| 1 | `seq` (`uint8`) |
| 2..7 | `angle[0..2]` (`uint16` × 3、LE) |
| 8 | `control_byte` |
| 9..10 | `base_rel_mm_j0` (`int16`、LE) |
| 11..12 | `auto_flags` (`uint16`、LE) |
| 13..14 | `crc16` (LE) |

| CAN ID | DLC | CAN data byte 0..7 |
|---:|---:|---|
| `0x501` | 8 | `angle[0]`, `[1]`, `[2]` を各 uint16 LE、byte 6..7=`base_rel_mm_j0` (int16 LE) |
| `0x502` | 8 | byte 0=`control_byte`、byte 1..2=`auto_flags` (LE)、byte 3..4=`fault_code` (LE)、byte 5..7=`0` |
| `0x503` | 8 | uint16 LE の `255` を 4 個、すなわち `FF 00 FF 00 FF 00 FF 00` |

`fault_code` は UART の `B` パケットに存在せず、現行実装では常に `0` である。

### 2.3 ローバー用 UART ASCII から CAN

入力形式は次の通りである。

```text
CAN_ID,DATA\r または CAN_ID,DATA\n
```

- `CAN_ID`: C の基数自動判定で解釈するため、例 `0x123` または `291`。範囲は `0x000..0x7FF`。
- `DATA`: 符号付き整数として基数自動判定し、その後 `uint32_t` に変換する。
- 出力: CAN 標準データフレーム、DLC 4。`DATA` は **big-endian** で byte 0..3 に置く。
- CAN ID `0` は構文上受理されるが、送信関数で何も送らず終了する。

例: `0x123,100\r\n` は CAN ID `0x123`、DLC 4、data `00 00 00 64` になる。

## 3. downlink: CAN から UART

### 3.1 CAN 受信範囲

ハードウェアフィルタが受理する標準 ID は次の範囲である。

- 個別 ID: `0x203..0x208`
- 範囲: `0x300..0x3FF`、`0x400..0x4FF`、`0x500..0x5FF`

したがって、後述する「その他 ID」の処理は全 11 bit ID に対するものではなく、このフィルタを通った ID だけに適用される。

### 3.2 アーム状態: CAN `0x203` / `0x204` から `JF`

| CAN ID | CAN data の解釈 |
|---:|---|
| `0x203` | byte 0..1=`encoder[0]`、2..3=`encoder[1]` (各 uint16 LE)、byte 4=`flags` |
| `0x204` | byte 0..1=`encoder[2]`、2..3=`encoder[3]`、4..5=`encoder[4]` (各 uint16 LE) |

両 ID を一度以上受信すると、以降は保持している最新値を約 50 Hz で 16 byte の `JF` パケットにし、USART6 から DMA 送信する。

| UART byte | `JF` フィールド |
|---:|---|
| 0..1 | ASCII `JF` |
| 2 | `seq` (`uint8`、送信ごとに加算) |
| 3 | `flags` |
| 4..13 | `encoders[0..4]` (`uint16` × 5、LE) |
| 14..15 | CRC-16/CCITT-FALSE (LE、byte 0..13 が対象) |

`0x203` / `0x204` について受信 DLC の明示的な検査はない。DLC が不足した場合にどう扱うべきかという仕様は不明であり、実装上はゼロ初期化された受信バッファの不足部分を読む。

### 3.3 UF テキスト: CAN `0x205..0x208` から `UF`

CAN ID の昇順をフレーム番号 0..3 とし、各フレームの 8 byte を連結する。

| CAN ID | 復元先 |
|---:|---|
| `0x205` | payload byte 0..7 |
| `0x206` | payload byte 8..15 |
| `0x207` | payload byte 16..23 |
| `0x208` | payload byte 24..31 |

各フレームは DLC 8 以上が必要で、`0x205` から順番に受信する必要がある。途中の受信間隔が 250 ms を超えた場合や順序が飛んだ場合は組み立てを破棄する。4 枚揃うと、次の 40 byte `UF` パケットを USART6 へ送る。

| UART byte | `UF` フィールド |
|---:|---|
| 0..1 | ASCII `UF` |
| 2 | `seq` (`JF` と共通のカウンタ) |
| 3 | `flags` = `0x13` (`VALID | USB_PRESENT | END`) |
| 4 | `chunk_index` = `0` |
| 5 | `payload_len`: payload 内で最初の NUL より前の長さ、最大 32 |
| 6..37 | CAN 4 フレームを連結した `payload[32]` |
| 38..39 | CRC-16/CCITT-FALSE (LE、byte 0..37 が対象) |

UF の受信・送信中および最後の UF 活動から 250 ms の間は、`JF` と後述のローバー ASCII の新規送信を止め、保留済みの通常送信も破棄して UF を優先する。

### 3.4 その他の CAN からローバー ASCII

受信値は ID ごとの「最新値」テーブルへ保存し、約 10 Hz で USART6 から次の形式で送信する。

```text
0x%03X,%d\r\n
```

CAN data の数値化は ID により異なる。

| CAN ID | 値の解釈 |
|---:|---|
| `0x410..0x413` | DLC 4 以上の場合、byte 0..3 を big-endian の符号付き int32 として解釈 |
| 上記以外でフィルタを通る ID | 受信バッファ byte 0..7 の符号なし合計値。受信バッファは毎回ゼロ初期化される |

最新値テーブルは最大 16 ID で、同じ ID は上書きされる。16 ID を超えると古いスロットを巡回して置換する。

### 3.5 USART3 から USART6 への ASCII 中継

downlink の USART3 (RX: PC5) は、割り込みで次の行を受け取る。

```text
ID,VALUE\r または ID,VALUE\n
```

ID と VALUE を最新値テーブルへ保存し、CAN 由来の最新値とは別テーブルで保持する。その後、CAN 由来と同じ `0x%03X,%d\r\n` 形式に整形し、約 10 Hz で USART6 (TX: PC6) へ送る。この経路は UART 同士の中継であり、CAN へは送らない。

## 4. 関係性の要点

| 入力側 | 中間表現 | 出力側 | 保持されない情報 |
|---|---|---|---|
| uplink `M/I/B` UART パケット | 正規化したアーム値 | 固定 ID の CAN 2～3 フレーム | UART `seq`、CRC。`I` の速度は入力自体がなく 0 |
| uplink ローバー ASCII | CAN ID + uint32 値 | 任意の標準 ID、DLC 4 | ASCII 表現、改行 |
| downlink CAN `0x203/204` | encoder 5 個 + flags | `JF` UART パケット | 元の CAN ID 境界は `encoders[]` の位置に変換 |
| downlink CAN `0x205..208` | 32 byte 連結 payload | `UF` UART パケット | 各 CAN フレームの個別時刻。ID は位置情報へ変換 |
| downlink のその他 CAN | ID ごとの最新整数値 | ASCII 行 | 生 CAN data（値へ集約）、過去サンプル |

## 5. コードから確定できない点

- `current`、`angle`、`vel`、encoder の物理単位、スケール、符号の意味は不明。
- `control_byte`、`auto_flags`、`JF.flags` の各 bit の意味は、この `uplink` / `downlink` 内だけでは不明。
- CAN ID `0x200` 系、`0x210` 系、`0x501` 系、`0x203..0x208` および汎用 ID のシステム全体での所有者・意味は不明。
- `M/I/B` の UART 送信元、`JF/UF` の最終受信先、USART3 の接続先はコメントから用途名を推測できるだけで、物理配線は不明。
- UF payload の文字コードと、32 byte 中の NUL より後ろを受信側がどう扱うべきかは不明。実装は `payload_len` を最初の NUL までとしても、payload 自体は 32 byte 全て送る。
- uplink の `B` に `fault_code` がない理由、`0x503` に 255 を 4 個固定送信する意味、`I` の `vel` を常に 0 とする意図は不明。
- downlink が一般 CAN フレームを「8 byte の合計値」にする設計意図は不明。

## 6. 主な参照実装

- uplink: `include/communication_types.h`, `include/communication_config.h`, `src/xbee_uart.c`, `src/xbee_protocol.c`, `src/can_bridge.c`, `src/rover_link.c`, `src/main.c`
- downlink: `include/communication_types.h`, `include/communication_config.h`, `src/can_bridge.c`, `src/xbee_protocol.c`, `src/xbee_uart.c`, `src/rover_link.c`, `src/scheduler.c`, `src/main.c`
