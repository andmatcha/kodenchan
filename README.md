# こでんちゃん CANモニター

CAN（1 Mbit/s）で受信した全フレームをUSART2へ順番に出力します。CAN受信割り込みではリングバッファへ格納するだけにし、UART出力はメインループで行うため、シリアル出力中もCAN FIFOを回収できます。

## アップロード

```
pio run -t upload
```


## モニター方法

接続されているデバイスが１つのみの場合
```
pio device monitor
```

複数接続されている場合
```
pio device monitor -p ポート名(例/dev/cu.~~~) -b 115200
```

## デバイスのポート確認

```
pio device list
```

## 出力形式

1行が1フレームで、受信順に次の形式で表示されます。

```text
200#0000000000000000
1FF#0102030405060708
12345678#AABB
300#R4
```

- 標準ID: 3桁の16進数
- 拡張ID: 8桁の16進数
- `#` 以降: ペイロードを16進数で連結
- リモートフレーム: `R` とDLC

1秒ごとに累積統計も表示します。

```text
STAT rx=310 output=310 queued=0 queue_drop=0 fifo_overrun=0 uart_drop=0 fifo_full=0 read_error=0 can_error=0 queue_peak=3
```

正常時は `rx = output + queued + queue_drop + uart_drop` となり、全件を表示できている場合は `queued=0 queue_drop=0 fifo_overrun=0 uart_drop=0` です。
