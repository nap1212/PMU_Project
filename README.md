[test1.2_STM32_GPS_PPS_仕様書.md](https://github.com/user-attachments/files/28501587/test1.2_STM32_GPS_PPS_.md)

# PMU Synchrophasor 計測システム 仕様書

**ファイル名**: `test1.2_STM32_GPS_PPS.ino`  
**対象マイコン**: STM32F1 / STM32F4 系 (Arduino フレームワーク / STM32duino)  
**作成日**: 2026-06-02  
**準拠規格**: IEEE C37.118.1 (Synchrophasor 計測標準)

---

## 1. システム概要

交流電圧信号 (ZMPT101B センサー) をリアルタイムで計測し、L76K GPS HAT の 1PPS 信号で UTC 時刻に同期した **絶対フェーザー** を算出する PMU (Phasor Measurement Unit) システム。

| 項目 | 内容 |
|------|------|
| プラットフォーム | STM32F1 / STM32F4 (Blue Pill / Nucleo 等) |
| 計測対象 | 単相交流電圧 (50/60 Hz) |
| センサー | ZMPT101B AC 電圧トランスフォーマモジュール |
| 時刻同期 | L76K GPS HAT (NMEA + 1PPS) |
| アルゴリズム | 最小二乗法 (LS) + 離散フーリエ変換 (DFT) |
| 出力 | USB-CDC シリアル (115200 bps) |

---

## 2. ハードウェア仕様

### 2.1 使用部品

| 部品 | 型番 / 種別 | 備考 |
|------|------------|------|
| マイコン | STM32F103C8T6 等 (STM32F1/F4) | Arduino フレームワーク使用 |
| AC 電圧センサー | ZMPT101B モジュール | 3.3V 系に要調整 |
| GPS モジュール | L76K GPS HAT | NMEA + 1PPS 出力 |

### 2.2 ピン配線

| 信号 | STM32 ピン | 方向 | 備考 |
|------|-----------|------|------|
| ZMPT101B OUT | PA0 (A0) | 入力 | ADC1_IN0 / アナログ入力 |
| L76K PPS | PB0 | 入力 | 外部割り込み RISING エッジ |
| L76K TX → STM32 RX | PA10 (Serial1 RX) | 入力 | NMEA 受信 |
| L76K RX → STM32 TX | PA9 (Serial1 TX) | 出力 | GPS コマンド送信用 |
| USB-CDC (デバッグ) | USB (PA11/PA12) | 出力 | シリアルモニタ |

> **注意**: ボードによって Serial1 のピン番号が異なる場合がある。  
> Nucleo-64 等では Serial2 (PA2/PA3) を使用すること。

### 2.3 電気的仕様

| 項目 | 値 | 備考 |
|------|-----|------|
| 動作電圧 | 3.3 V | STM32 I/O 電圧 |
| ADC 基準電圧 | 3.3 V | `ADC_REF_V` |
| ADC 分解能 | 12-bit (0〜4095) | STM32 デフォルト |
| PPS 入力レベル | 3.3 V CMOS | L76K 出力と直結可 |
| ZMPT101B 中点電圧 | 1.65 V (= 3.3 V / 2) | オフセット調整要確認 |

---

## 3. ソフトウェア仕様

### 3.1 サンプリング仕様

| パラメータ | 値 | 定数名 |
|-----------|-----|--------|
| サンプリング周波数 | 2000 Hz | `FS` |
| バッファサイズ | 200 サンプル | `BUF_SIZE` |
| 計測時間幅 | 100 ms (5 サイクル @ 50 Hz) | — |
| サンプル間隔 | 500 µs | `Ts = 1/FS` |
| 1 サイクルあたりサンプル数 | 40 サンプル | — |
| 使用タイマー | TIM3 | `SAMPLE_TIMER` |

### 3.2 GPS 時刻同期仕様

| 項目 | 仕様 |
|------|------|
| GPS モジュール | L76K GPS HAT |
| NMEA ボーレート | 9600 bps (`GPS_BAUD`) |
| 解析センテンス | `$GPRMC` / `$GNRMC` |
| PPS 精度 (L76K) | ±100 ns (typ.) |
| 有効 Δt 範囲 | 0〜999,999 µs (0〜1 秒) |
| PPS 有効判定 | `pps_valid == true` かつ `dt_us < 1,000,000` |

### 3.3 フェーザー算出仕様

#### 3.3.1 LS (最小二乗法) フェーザー

| 項目 | 仕様 |
|------|------|
| モデル | $x[k] = A\cos(\omega_0 k T_s) + B\sin(\omega_0 k T_s)$ |
| 推定方法 | 疑似逆行列 $(H^T H)^{-1} H^T x$ |
| 振幅 | $X_m = \sqrt{A^2 + B^2}$ [V ピーク, センサー出力] |
| 相対位相 | $\phi_{LS} = \arctan2(-B,\ A)$ [deg] (バッファ先頭 $k=0$ 基準) |
| **絶対位相** | $\phi_{abs} = \phi_{LS} - 2\pi f_0 \Delta t$ [deg] **(GPS UTC 整秒基準)** |

#### 3.3.2 DFT フェーザー

| 項目 | 仕様 |
|------|------|
| 点数 | N = BUF_SIZE = 200 点 |
| 対象 bin | $k = \text{round}(f_0 / (F_s/N))$ |
| 振幅 | $2|X[k]|/N$ [V ピーク] |
| 位相 | $\angle X[k]$ [deg] |

#### 3.3.3 高調波解析 (DFT)

| 次数 | 周波数 (50 Hz 系) | 備考 |
|------|-----------------|------|
| 基本波 | 50 Hz | — |
| 2 次 | 100 Hz | DFT 推定 |
| 3 次 | 150 Hz | DFT 推定 |
| THD | $\sqrt{H_2^2 + H_3^2} / H_1 \times 100$ [%] | — |

#### 3.3.4 TVE (Total Vector Error)

| 項目 | 仕様 |
|------|------|
| 定義 | IEEE C37.118.1 式 |
| 基準値 | LS フェーザー |
| 被評価値 | DFT フェーザー |
| 合格基準 | TVE ≤ 1 % (`TVE_LIMIT`) |
| 計算式 | $\text{TVE} = \dfrac{\sqrt{(\tilde{X}_r - X_r)^2 + (\tilde{X}_i - X_i)^2}}{\sqrt{X_r^2 + X_i^2}} \times 100\ [\%]$ |

#### 3.3.5 周波数推定

| 項目 | 仕様 |
|------|------|
| 方法 | 立下りゼロクロス法 |
| 外れ値除外 | ±20 % 閾値 |
| 最大検出ゼロクロス数 | 20 点 |
| 有効範囲 | 40〜70 Hz (範囲外は公称値 `F0_NOM` でフォールバック) |

#### 3.3.6 ROCOF (周波数変化率)

| 項目 | 仕様 |
|------|------|
| 方法 | 連続周波数推定値の差分 |
| 更新間隔 | BUF_SIZE / FS = 0.1 s |
| 単位 | Hz/s |

### 3.4 センサー変換比

| パラメータ | 値 | 定数名 |
|-----------|-----|--------|
| SENSOR_RATIO | 500.0 | `SENSOR_RATIO` |
| 実電圧 [V pk] | センサー出力 [V pk] × SENSOR_RATIO | — |

> `SENSOR_RATIO` は ZMPT101B の実配線・負荷抵抗に応じてキャリブレーションすること。

---

## 4. シリアル出力仕様

### 4.1 デバッグシリアル (Serial / USB-CDC)

| 項目 | 仕様 |
|------|------|
| ボーレート | 115200 bps |
| フォーマット | テキスト (UTF-8) |
| 更新レート | 約 10 Hz (BUF_SIZE / FS = 100 ms 毎) |

### 4.2 出力内容

```
==========================================
  PMU Synchrophasor — STM32 + L76K GPS
  IEEE C37.118.1 準拠 (LS + DFT + 1PPS)
==========================================
--- GPS 1PPS 時刻同期 ---
GPS Fix        : 有効 (A)
UTC Time       : HH:MM:SS UTC
PPS Valid      : Yes
dt (PPS->buf)  : xxx.xxx ms

--- 信号基本情報 ---
DC Offset      : x.xxxx V
RMS (sensor)   : x.xxxx V
Real RMS       : xxx.xx V

--- 周波数 ---
Frequency      : xx.xxx Hz  [正常 50Hz]
ROCOF          : x.xxxx Hz/s

--- LS フェーザー (最小二乗法) ---
  Amp (sensor) : x.xxxx V (pk)
  Amp (real)   : xxx.xx V (pk)
  Amp RMS(real): xxx.xx V
  Phase (rel)  : xxx.xxx deg  (バッファ先頭 k=0 基準)
  Phase (abs)  : xxx.xxx deg  [GPS UTC整秒基準 / IEEE C37.118.1] ★
  cos coeff A  : x.xxxxx V
  sin coeff B  : x.xxxxx V

--- DFT フェーザー ---
--- 高調波 (DFT) ---
--- TVE (Total Vector Error) ---
```

### 4.3 シリアルプロッター出力

`PLOT_MODE` マクロで切替。

| PLOT_MODE | 出力フォーマット |
|-----------|----------------|
| 0 | `AC:値,LS_est:値,DFT_est:値` (波形) |
| 1 | `Amp_LS:値,Amp_DFT:値,Phase_LS:値,Phase_Abs:値,Freq:値,TVE:値` (フェーザー) |

---

## 5. 動作条件・制約

| 項目 | 条件 |
|------|------|
| GPS Fix 取得 | 初回起動時 2〜5 分程度かかる場合がある |
| 絶対位相有効条件 | `gps_fix == true` かつ `pps_valid == true` かつ `dt_us < 1,000,000` |
| GPS Fix なし時 | 相対位相 (`Phase_rel`) のみ出力。絶対位相は `---` 表示 |
| 信号なし時 | 周波数推定値 0、公称値 (`F0_NOM = 50 Hz`) でフォールバック |
| micros() オーバーフロー | unsigned 差分演算により約 71 分周期のオーバーフローを自動補正 |

---

## 6. 設定パラメータ一覧

| 定数名 | デフォルト値 | 説明 |
|--------|------------|------|
| `PIN_ADC` | PA0 | ZMPT101B アナログ入力ピン |
| `PIN_PPS` | PB0 | GPS PPS 入力ピン |
| `GPS_SERIAL` | Serial1 | GPS NMEA UART |
| `SAMPLE_TIMER` | TIM3 | サンプリング用タイマー |
| `BUF_SIZE` | 200 | サンプルバッファサイズ |
| `FS` | 2000.0 | サンプリング周波数 [Hz] |
| `F0_NOM` | 50.0 | 公称周波数 [Hz] |
| `ADC_REF_V` | 3.3 | ADC 基準電圧 [V] |
| `ADC_BITS` | 4095.0 | ADC 最大値 (12-bit) |
| `GPS_BAUD` | 9600 | GPS UART ボーレート |
| `SENSOR_RATIO` | 500.0 | センサー変換比 (要キャリブレーション) |
| `PLOT_MODE` | 1 | シリアルプロッター出力モード |
| `TVE_LIMIT` | 1.0 | TVE 合格閾値 [%] |

---

## 7. 開発環境

| 項目 | 推奨環境 |
|------|---------|
| IDE | Arduino IDE 2.x / VS Code + Arduino Extension |
| ボードパッケージ | STM32duino (STM32 Arduino Core) v2.x 以上 |
| 追加ライブラリ | なし (HardwareTimer は STM32duino に内包) |
| コンパイラ | arm-none-eabi-gcc |
| 書き込み | ST-Link / USB DFU (ボード依存) |
