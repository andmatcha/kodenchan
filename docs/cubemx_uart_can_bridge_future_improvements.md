# UART–CAN bridge の CubeMX 将来改善案

今回の実装では `kodenchan.ioc` と CubeMX生成のマイコン初期化設定を変更していない。現在の設定でも USART2 115200 bps、CAN 1 Mbps で動作するが、将来 CubeMX から再生成する際は以下を検討する。

## 現在の前提

- Nucleo-STM32F303K8 の ST-LINK micro USB Virtual COM Portを USART2 の唯一の UART 入出力窓口とする。
- USART2 は PA2（TX）/ PA15（RX）、115200 bps、8N1、フロー制御なし。
- USART2 RX/TX は DMA1 Channel 6/7 を使用する。
- CAN は PA11（RX）/ PA12（TX）、Normal mode、1 Mbps。
- APB1 peripheral clock 32 MHz、prescaler 4、BS1 5 TQ、BS2 2 TQ、SJW 1 TQで 1 Mbps、sample point 75%となっている。

## 推奨する CubeMX 設定変更

### 1. CAN bit timing の sample point 改善

配線長、transceiver、ノード数に余裕を持たせるため、32 MHz CAN clockで次の 1 Mbps 設定を候補とする。

| 項目 | 候補値 |
| --- | ---: |
| Prescaler | 2 |
| Time Quantum / bit | 16 |
| BS1 | 13 TQ |
| BS2 | 2 TQ |
| SJW | 1 TQ |
| Sample point | 87.5% |

実機では CANH/CANL の波形、終端抵抗、全ノードの oscillator tolerance を確認してから採用する。

### 2. CAN Auto Retransmission を有効化

現在は `AutoRetransmission = DISABLE` である。一時的な arbitration loss や ACK error で UART入力由来の frame が失われにくいよう、有効化を推奨する。再送によって古い command が残ることを許容できるかはシステム全体で確認する。

### 3. CAN Auto Bus-Off を有効化

現在は `AutoBusOff = DISABLE` である。継続的な CAN error 時の保護として有効化を推奨する。復帰方法は、Auto Wake-Upの採否と合わせて決める。

### 4. CAN RX FIFO interrupt の利用

現在のユーザーコードは main loop で FIFO0/FIFO1 を pollingしている。高負荷時の FIFO overrunを避けるには、CubeMXで CAN RX FIFO0 message pending interruptを有効化し、割り込み側では軽量な software queueへ退避する構成を検討する。UART送信や CRC計算は割り込み内で行わない。

### 5. interrupt priority の整理

DMA1 Channel 6/7、USART2、CAN RX/TX が同一の最高 priorityにならないよう、データ欠落リスクと処理時間を基準に priorityを設計する。特に UART DMA TX completeが長時間遅延すると送信 ring bufferが詰まるため、CAN処理との優先順位を実機負荷試験で決める。

### 6. UART DMA Receive-to-Idle の検討

現在は circular DMAの write indexを pollingしている。CubeMX/HALの対応状況を確認し、Receive-to-Idle DMAと software ring bufferの組み合わせへ移行すると、main loop負荷を下げつつ overrunを検出しやすくなる。採用時もバイナリパケット途中の UART IDLEを packet終端として扱わないこと。

## 単一USART2へ統合したことによる制約

元仕様にある downlink の「USART3で受信した ASCII 最新値を USART6へ送る」経路は、入力と出力を同じ USART2へ統合すると自己ループになり、同じ `ID,VALUE` 行を uplink入力と中継入力に識別できない。このため現在の bridgeでは USART2 のASCII入力を UART→CAN としてのみ扱う。

将来このUART間中継も必要なら、CubeMXでUARTを追加するのではなく、まずプロトコルに方向またはmessage typeを示す framingを追加し、単一USART2上で曖昧性なく multiplexできる仕様に改訂することを推奨する。

## 再生成時の注意

- `main.c` の `USER CODE` 範囲にある `app_init()` / `app_poll()` 呼び出しを維持する。
- `stm32f3xx_it.c` の DMA1 Channel 6/7 と USART2 handlerを維持する。
- CAN filterは `src/drivers/can_bus.c` のユーザーコードで設定しており、`0x203..0x208`、`0x300..0x5FF` の指定範囲以外を受理しない。
- 再生成後は `pio run` に加え、115200 bpsでの連続 mixed binary/ASCII入力、CAN 1 Mbps高負荷、UART TX飽和、tick counter wraparoundを実機確認する。
