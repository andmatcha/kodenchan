# こでんちゃん

## アップロード

```
pio run -t upload
```


## モニター方法

接続されているデバイスが１つのみの場合
```
pio device monitor --baud 38400
```

複数接続されている場合
```
pio device monitor -p ポート名(例/dev/cu.~~~) -b 38400
```

## デバイスのポート確認

```
pio device list
```
