# 新UF Text Feedback Packet案

## 目的

現行 `UF` packet は USB memory から読み取った緯度経度を `lat_e7` / `lon_e7` として返す 14 byte 固定長 packet である。
新仕様では、読み取ったテキストデータを座標へ解釈せず、読み取った byte 列のまま frontend へ返す。

実装負荷を下げるため、現行 `UF` の考え方をできるだけ踏襲し、状態管理を最小限にする。

## 必要最小限案

現行 header `b"UF"`、`seq`、`flags`、packet 末尾 CRC を維持する。
`lat_e7` / `lon_e7` の代わりに、固定長の text payload を持たせる。
追加する状態情報は、chunk 番号と終端フラグだけにする。

1 packet あたりの有効 payload は最大 32 byte とする。
packet 自体は常に 40 byte 固定長で送信する。

Wire:

| Item | Value |
|---|---|
| Header | `b"UF"` |
| `struct` | `<2sBBBB32sH` |
| Size | 40 bytes |
| Payload bytes | 32 bytes |
| CRC | CRC-16/CCITT-FALSE over first 38 bytes |

Fields:

| Byte | Field | Type | Notes |
|---:|---|---|---|
| 0..1 | header | `char[2]` | ASCII `UF` |
| 2 | seq | `uint8` | STM sequence。packet ごとに increment |
| 3 | flags | `uint8` | 現行 `UF.flags` に終端フラグだけ追加 |
| 4 | chunk_index | `uint8` | 0 origin の chunk 番号。read ごとに 0 から開始 |
| 5 | payload_len | `uint8` | この packet の有効 payload byte 数。`0..32` |
| 6..37 | payload | `uint8[32]` | 読み取った text byte 列をそのまま格納 |
| 38..39 | crc16 | `uint16` | little endian CRC |

`UF.flags` は現行定義を維持し、bit4 だけ終端フラグとして追加する。

| Bit | Meaning |
|---:|---|
| 0 | `valid` |
| 1 | `usb_present` |
| 2 | `read_busy` |
| 3 | `read_error` |
| 4 | `end`。この packet が最後の text chunk |
| 5-7 | reserved, send `0` |

## 分割と終端

送信側は読み取った text byte 列を先頭から 32 byte ごとに分割して送る。

- data packet は `valid | usb_present` を立てる。
- data packet の `payload_len` は `1..32` とする。
- `payload_len` より後ろの `payload` byte は padding。receiver は復元 text に含めない。
- `chunk_index` は 0 から始め、packet ごとに 1 ずつ増やす。
- 最後の data packet には `end` フラグを立てる。
- 空ファイルの場合は、`chunk_index=0`, `payload_len=0`, `valid | usb_present | end` の packet を 1 個送る。

この案では `chunk_count`、`total_len`、`transfer_id` は持たない。
receiver は `chunk_index` の連続性を確認しながら、packet 到着順に payload を連結する。

## 送信例

読み取った text が 70 byte の場合:

| Packet | payload_len | flags | payload |
|---:|---:|---|---|
| 1 | 32 | `valid | usb_present` | byte 0..31 |
| 2 | 32 | `valid | usb_present` | byte 32..63 |
| 3 | 6 | `valid | usb_present | end` | byte 64..69 |

## 想定テキストの1パケット収容確認

`payload[32]` は32 byteまで格納できる。
次の想定テキストは、いずれも本文だけなら32 byte以内に収まるため、data packet 1 個で送信できる。
1 packet で送る場合は `chunk_index=0` とし、その data packet に `end` フラグも立てる。

| Text | UTF-8 bytes | Fits in one data packet |
|---|---:|---|
| `38.12345, -110.98765` | 20 | yes |
| `LAT: 38.12345` | 13 | yes |
| `LON: -110.98765` | 15 | yes |
| `38° 07.407' N, 110° 39.259' W` | 31 | yes |
| `Lat 38 07 24.42 N` | 17 | yes |
| `Lon 110 39 15.54 W` | 18 | yes |
| `N38.12345 W110.98765` | 20 | yes |

上表の byte 数は改行文字を含まない。
末尾に LF (`\n`) だけを含める場合も最大32 byteに収まる。
ただし `38° 07.407' N, 110° 39.259' W` に CRLF (`\r\n`) まで含めると33 byteになり、2 個の data packet に分割される。

## Busy / Error

現行 flags をそのまま使う。

Busy packet:

- `flags = usb_present | read_busy`
- `chunk_index = 0`
- `payload_len = 0`
- `payload` は全 byte `0x00`

Error packet:

- `flags = read_error`
- USB を検出できている場合は `usb_present` も立てる。
- `chunk_index = 0`
- `payload_len = 0`
- `payload` は全 byte `0x00`

Error packet を受けた receiver は、組み立て中の text buffer を破棄する。

## Receiver 最小実装

Mac bridge は CRC OK の `UF` packet だけを処理する。

保持する状態は次の 3 つでよい。

| State | Meaning |
|---|---|
| `buffer` | 組み立て中の raw byte 列 |
| `expected_chunk_index` | 次に受け取るべき chunk 番号 |
| `active` | text 組み立て中かどうか |

処理ルール:

- `read_busy` packet は UI に busy を通知するだけで、`buffer` は変更しない。
- `read_error` packet は `buffer` を破棄し、error を通知する。
- `valid` packet かつ `chunk_index` が `expected_chunk_index` と一致するなら、`payload[0:payload_len]` を `buffer` に追加する。
- `end` フラグが立っていれば、その時点の `buffer` を complete text として frontend へ渡す。
- text byte 列は復元時に変換しない。UTF-8 decode や改行変換が必要な場合は、表示直前の UI 側だけで行う。
- `chunk_index` が期待値と一致しない場合は packet 抜けまたは重複とみなし、`buffer` を破棄して incomplete として扱う。
- `end` フラグが来ない場合に備え、最後の valid data packet から一定時間後に `buffer` を破棄する。初期値は 5 秒でよい。

## 現行UFからの差分

| Item | 現行UF | 新UF案 |
|---|---|---|
| Size | 14 bytes | 40 bytes |
| Header | `b"UF"` | `b"UF"` |
| `seq` | あり | あり |
| `flags` | 4bit使用 | bit4 に `end` だけ追加 |
| Data | `lat_e7`, `lon_e7` | `chunk_index`, `payload_len`, `payload[32]` |
| CRC | first 12 bytes | first 38 bytes |

## 実装メモ

- STM32 側の `UF_PACKET_LEN` は `40`、`UF_PACKET_CRC_OFFSET` は `38` にする。
- `uf_packet_encode()` は `flags`, `chunk_index`, `payload_len`, `payload[32]` を受ける形にする。
- `payload_len > 32` は送信側で作らない。受信側で見つけた場合は破棄する。
- `chunk_index` は `uint8` なので、1 回の read は最大 256 chunks、つまり 8192 byte までをこの簡易仕様の上限とする。
- 現行の CAN feedback `0x209` は 8 byte の `lat_e7/lon_e7` 前提なので、USB read 元が別 CAN node の場合は、CAN 側も 32 byte text chunk を渡せる経路へ変える必要がある。
- この案は再送、順不同受信、複数 transfer の同時進行には対応しない。1 回の USB read に対して、chunk 番号順に data packet を送り、最後の packet に `end` を立てるだけの単純な仕様とする。
