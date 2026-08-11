# UART–CAN bridge tests

`test_uart_can_bridge/test_main.c` は UART、CAN、HAL tickだけをモック化し、本番の `uart_can_bridge_service.c` と `crc16_ccitt.c` をNative環境で実行する。

```sh
pio test -e native
```

主な検証範囲:

- CRC-16/CCITT-FALSE既知値
- `PacketACv6`のIK / Manual / Keyboard Auto全modeについて、全CAN frame、DLC、byte配置を検証
- ACの`vel`、`fault_code`、IK/Keyboard Auto用`current[0]`, `[1]`, `[5]`, `[6]`
- AC予約mode、CRC不正・byte欠落後のheader再同期、不完全packet、旧`M` / `I` / `B` packetの拒否
- uplink ASCIIの基数自動判定、signed int32、big-endian、CR/LF、ID 0
- CRC不正、範囲外ASCII、空値、余分な文字、長すぎる行からの復帰
- mixed binary/ASCII stream
- downlink `JF` の両frame待ち、50 Hz、短DLCのゼロ埋め、CRC
- downlink `UF` の4 frame連結、payload length、flags、CRC、順序、DLC、250 ms timeout
- `JF` / `UF` 共通sequence
- downlink ASCIIのbyte合計と `0x410..0x413` signed big-endian
- 最新値の更新、16 ID上限と巡回置換
- UF優先中の通常出力破棄・250 ms抑止
- HAL tickの32 bit wraparound
