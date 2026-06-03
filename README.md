[PMU_Project_Spec.md](https://github.com/user-attachments/files/28542681/PMU_Project_Spec.md)
# PMU Synchrophasor Measurement — 仕様書

**バージョン**: 2.0  
**対象ボード**: STM32 Nucleo-F446RE  
**GPSモジュール**: Waveshare L76K GPS HAT  
**準拠規格**: IEEE C37.118.1  

---

## 目次

1. [プロジェクト概要](#1-プロジェクト概要)
2. [ハードウェア構成](#2-ハードウェア構成)
3. [ソフトウェア構成](#3-ソフトウェア構成)
4. [STM32CubeMX設定](#4-stm32cubemx設定)
5. [アルゴリズム](#5-アルゴリズム)
6. [APIリファレンス](#6-apiリファレンス)
7. [出力フォーマット](#7-出力フォーマット)
8. [キャリブレーション](#8-キャリブレーション)
9. [ビルド・書き込み手順](#9-ビルド書き込み手順)
10. [既知の制限事項](#10-既知の制限事項)

---

## 1. プロジェクト概要

### 1.1 目的

ZMPT101B 交流電圧センサーで系統電圧の同期フェーザー（Synchrophasor）を計測するシステム。
L76K GPS モジュールの PPS（1秒パルス）信号で UTC 時刻同期を行い、各計測値に精密タイムスタンプを付与する。

最小二乗法（LS）および離散フーリエ変換（DFT）の2手法でフェーザーを推定し、IEEE C37.118.1 に基づく TVE（Total Vector Error）評価を行う。

### 1.2 計測項目

| 計測項目 | 説明 |
|---------|------|
| タイムスタンプ | GPS同期 UTC (ISO 8601 形式、1ms 分解能) |
| 電圧 RMS | センサー出力および実電圧換算値 |
| 周波数 | ゼロクロス法による系統周波数推定 |
| ROCOF | 周波数変化率 (Rate of Change of Frequency) |
| LS フェーザー | 振幅・位相（最小二乗法） |
| DFT フェーザー | 振幅・位相・bin（離散フーリエ変換） |
| 高調波 | 2次・3次高調波の振幅・位相 |
| THD | 全高調波歪率 |
| TVE | LS基準のDFTフェーザー誤差 [%] |

### 1.3 サンプリング設計

| パラメータ | 値 |
|-----------|-----|
| サンプリング周波数 Fs | 2000 Hz |
| サンプル間隔 | 0.5 ms |
| バッファサイズ | 200 サンプル |
| 1バッファの時間長 | 100 ms（5サイクル @ 50Hz） |
| フレームレート | 10 フレーム/秒 |
| 公称周波数 | 50 Hz |

---

## 2. ハードウェア構成

### 2.1 使用機器

| 機器 | 型番・仕様 |
|------|-----------|
| マイコンボード | STM32 Nucleo-F446RE |
| 電圧センサー | ZMPT101B（交流電圧トランスモジュール） |
| GPS モジュール | Waveshare L76K GPS HAT |
| 通信 | ST-Link 仮想COMポート（USB、115200bps） |

### 2.2 配線

#### ZMPT101B → Nucleo

| ZMPT101B | Nucleo ピン | 説明 |
|----------|------------|------|
| OUT | PA0 (A0) | ADC入力 |
| VCC | 3.3V | 電源 |
| GND | GND | グランド |

> **注意**: STM32 GPIO 最大入力電圧は 3.3V。センサー出力が 3.3V を超えないようボリュームで調整すること。

#### L76K GPS HAT → Nucleo

| L76K GPS HAT | Nucleo ピン | 説明 |
|-------------|------------|------|
| UART TX | PA10 (D2) | NMEA データ受信 |
| UART RX | PA9 (D8) | コマンド送信（省略可） |
| 1PPS | PB0 (A3) | 秒同期パルス（立ち上がり） |
| 3.3V | 3.3V | 電源 |
| GND | GND | グランド |

### 2.3 システム構成図

```
[系統電圧]
    ↓
[ZMPT101B] ──────────────→ PA0 (ADC1_IN0)
                                 ↓
[L76K GPS HAT]               [STM32F446RE]  ──→ [USART2] ──→ [PC/ログ]
  UART TX ──→ PA10 (USART1_RX)   ↑
  1PPS    ──→ PB0  (EXTI0)   [TIM2 2000Hz]
```

---

## 3. ソフトウェア構成

### 3.1 開発環境

| 項目 | 内容 |
|------|------|
| IDE | STM32CubeIDE |
| HAL | STM32F4 HAL ライブラリ |
| 言語 | C (C11) |

### 3.2 ファイル構成

```
プロジェクト/
├── Core/
│   ├── Inc/
│   │   ├── main.h          # CubeIDE 自動生成
│   │   ├── pmu.h           # PMU モジュール ヘッダ
│   │   └── gps.h           # GPS モジュール ヘッダ
│   └── Src/
│       ├── main.c          # CubeIDE 自動生成 + ユーザー追記
│       ├── pmu.c           # PMU モジュール 実装
│       └── gps.c           # GPS モジュール 実装
├── Drivers/                # HAL ライブラリ（自動生成）
├── .gitignore
└── *.ioc                   # CubeMX 設定ファイル
```

### 3.3 モジュール構成

```
main.c
├── PMU_Init()              起動メッセージ出力
├── GPS_Init()              GPS 初期化
├── HAL_TIM_Base_Start_IT() TIM2 開始
└── PMU_Task()              メインループ処理
    └── ProcessBuffer()     バッファ解析・全計算・CSV出力
        ├── LS_Phasor()             最小二乗法フェーザー推定
        ├── DFT_Phasor()            DFT フェーザー推定
        ├── EstimateFrequency()     ゼロクロス周波数推定
        ├── EstimateROCOF()         ROCOF 推定
        ├── CalcTVE()               TVE 計算
        └── GPS_TickToTimestamp()   タイムスタンプ生成

割り込み系:
TIM2 割り込み → PMU_TIM_Callback() → HAL_ADC_Start_IT()
                                              ↓
ADC 完了割り込み → PMU_ADC_Callback() → adcBuf[writeIdx] に格納
                                         writeIdx==0 のとき g_bufStartTick 記録

USART1 受信割り込み → GPS_RxCallback() → NMEA パース → s_time 更新
PB0 EXTI 割り込み   → GPS_PPS_Callback() → s_ppsTick 記録
```

---

## 4. STM32CubeMX設定

### 4.1 クロック (Clock Configuration タブ)

HCLK に `180` と入力して Enter（自動設定される）

| クロック | 値 |
|---------|-----|
| HCLK | 180 MHz |
| APB1 Timer clock | 90 MHz |
| APB2 clock | 90 MHz |

### 4.2 TIM2（サンプリングタイマー 2000Hz）

**Timers → TIM2**

| 項目 | 設定値 |
|------|--------|
| Clock Source | Internal Clock |
| Counter Mode | Up |
| Prescaler | 0 |
| Counter Period | **44999** |

> 90,000,000 ÷ (0+1) ÷ (44999+1) = **2000 Hz**

NVIC タブ → `TIM2 global interrupt` → ✅ Enabled

### 4.3 ADC1（電圧計測）

**Analog → ADC1** → IN0 (PA0) チェック

| 項目 | 設定値 |
|------|--------|
| Resolution | 12 bits |
| Continuous Conversion | Disabled |
| External Trigger | Software Trigger |
| Data Alignment | Right |
| Clock Prescaler | PCLK2 divided by 4（→ 22.5 MHz） |
| Sampling Time | 480 Cycles |

NVIC タブ → `ADC1 global interrupt` → ✅ Enabled

### 4.4 USART2（計測値出力 / デバッグ）

**Connectivity → USART2**

| 項目 | 設定値 |
|------|--------|
| Mode | Asynchronous |
| Baud Rate | 115200 bps |
| Word Length | 8 Bits / Parity None / Stop Bits 1 |
| TX ピン | PA2（自動割り当て） |
| RX ピン | PA3（自動割り当て） |

NVIC: 不要（ブロッキング送信）

### 4.5 USART1（GPS NMEA受信）

**Connectivity → USART1**

| 項目 | 設定値 |
|------|--------|
| Mode | Asynchronous |
| Baud Rate | **9600 bps** |
| Word Length | 8 Bits / Parity None / Stop Bits 1 |
| TX ピン | PA9（自動割り当て） |
| RX ピン | PA10（自動割り当て） |

NVIC タブ → `USART1 global interrupt` → ✅ Enabled

### 4.6 PB0（GPS PPS 信号）

Pinout ビューで **PB0** をクリック → `GPIO_EXTI0` を選択

**System Core → GPIO → PB0**

| 項目 | 設定値 |
|------|--------|
| GPIO mode | External Interrupt - Rising edge |
| Pull-up/Pull-down | Pull-Down |
| User Label | GPS_PPS |

**System Core → NVIC** → `EXTI line0 interrupt` → ✅ Enabled

### 4.7 NVIC 有効確認チェックリスト

| 割り込み | 有効 |
|---------|------|
| TIM2 global interrupt | ✅ |
| ADC1 global interrupt | ✅ |
| USART1 global interrupt | ✅ |
| EXTI line0 interrupt | ✅ |

---

## 5. アルゴリズム

### 5.1 最小二乗法（LS）フェーザー推定

信号モデル:
```
x[k] = A·cos(ω₀·k·Ts) + B·sin(ω₀·k·Ts)
```

疑似逆行列で解く:
```
[A, B]ᵀ = (HᵀH)⁻¹ Hᵀ x
振幅: Xm = √(A² + B²)
位相: φ  = atan2(-B, A)  [rad → deg]
```

### 5.2 DFT フェーザー推定

```
対象 bin: k = round(f₀ × N / Fs)
X[k] = Σ x[n]·exp(-j2πkn/N)
振幅 = 2|X[k]| / N
位相 = angle(X[k])
```

### 5.3 周波数推定（ゼロクロス法）

1. DC オフセット除去
2. 立下りゼロクロス時刻を線形補間で検出（最大 20点）
3. 隣接ゼロクロス間隔から周波数を算出
4. ±20% 外れ値を除外してロバスト化

### 5.4 ROCOF 推定

```
ROCOF = (f_cur - f_prev) / T_update   [Hz/s]
T_update = 200 / 2000 = 0.1 s
```

### 5.5 TVE（Total Vector Error）

IEEE C37.118.1 定義:
```
TVE = √((Xr_est - Xr_ref)² + (Xi_est - Xi_ref)²)
    / √(Xr_ref² + Xi_ref²)  × 100  [%]
合格基準: TVE ≤ 1%
```

### 5.6 GPS タイムスタンプ生成

```
バッファ収集開始時刻 g_bufStartTick = HAL_GetTick() (writeIdx == 0 のとき)

タイムスタンプ = GPS_UTC_at_PPS + (tick - ppsTick) [ms]

PLOT_MODE 0: サンプル i のタイムスタンプ
    sampleTick = g_bufStartTick + (i / 2)   ← 0.5ms間隔を1ms分解能で近似

PLOT_MODE 1: フレームのタイムスタンプ
    frameTick = g_bufStartTick               ← 計測ウィンドウ開始時刻
```

---

## 6. APIリファレンス

### PMU モジュール

#### `PMU_Init(void)`
起動メッセージを UART 出力。HAL 初期化後・タイマー開始前に呼ぶ。

#### `PMU_TIM_Callback(void)`
TIM2 周期割り込みから呼ぶ。ADC 変換を開始する。

#### `PMU_ADC_Callback(uint16_t raw)`
ADC 変換完了割り込みから呼ぶ。サンプルをバッファに格納。
`writeIdx == 0` のとき `g_bufStartTick` を記録する。

#### `PMU_Task(void)`
メインループから継続的に呼ぶ。バッファが揃ったら解析・CSV出力を実行。

---

### GPS モジュール

#### `GPS_Init(void)`
内部状態を初期化する。

#### `GPS_RxCallback(uint8_t byte)`
USART1 受信割り込みから 1バイトずつ渡す。GPRMC/GNRMC 文を自動検出・解析。

#### `GPS_PPS_Callback(void)`
PB0 EXTI 割り込みから呼ぶ。PPS 時刻を `HAL_GetTick()` で記録する。

#### `GPS_IsValid(void) → bool`
GPS 測位が有効かどうかを返す。

#### `GPS_GetTimestamp(char *buf, uint16_t len)`
現在時刻の ISO 8601 UTC 文字列を生成。測位無効時は `"NO_FIX"`。

#### `GPS_TickToTimestamp(uint32_t tick, char *buf, uint16_t len)`
指定した `HAL_GetTick()` 値を UTC 文字列に変換。
過去のバッファ収集時刻など任意の時刻のタイムスタンプ生成に使用。

---

### main.c への追記

```c
/* USER CODE BEGIN Includes */
#include "pmu.h"
#include "gps.h"

/* USER CODE BEGIN 0 */
static uint8_t gpsRxByte;

/* USER CODE BEGIN 2 */
GPS_Init();
PMU_Init();
HAL_UART_Receive_IT(&huart1, &gpsRxByte, 1);
HAL_TIM_Base_Start_IT(&htim2);

/* USER CODE BEGIN 3 */   // while(1) の中
PMU_Task();

/* USER CODE BEGIN 4 */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim) {
    if (htim->Instance == TIM2) PMU_TIM_Callback();
}
void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef *hadc) {
    if (hadc->Instance == ADC1)
        PMU_ADC_Callback((uint16_t)HAL_ADC_GetValue(hadc));
}
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart) {
    if (huart->Instance == USART1) {
        GPS_RxCallback(gpsRxByte);
        HAL_UART_Receive_IT(&huart1, &gpsRxByte, 1);
    }
}
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin) {
    if (GPIO_Pin == GPS_PPS_Pin) GPS_PPS_Callback();
}
```

---

## 7. 出力フォーマット

### 7.1 PLOT_MODE 0（波形 CSV）

サンプルごとにタイムスタンプ付き。バッファ1個につき 200行出力。

```
Timestamp,AC_V,LS_est_V,DFT_est_V
2024-06-03T12:34:56.000Z,0.1234,0.1235,0.1233
2024-06-03T12:34:56.000Z,0.1256,0.1255,0.1254
2024-06-03T12:34:56.001Z,0.1278,0.1277,0.1276
...
```

> 注: サンプル間隔 0.5ms に対し `HAL_GetTick()` は 1ms 分解能のため、2サンプルごとに同一タイムスタンプになる。

### 7.2 PLOT_MODE 1（フェーザー CSV）― PMU 標準出力

フレームごとにタイムスタンプ付き。10行/秒。ヘッダは起動時に1回のみ出力。

```
Timestamp,Freq_Hz,ROCOF_Hz_s,AmpLS_Vpk,PhaseLS_deg,AmpDFT_Vpk,PhaseDFT_deg,TVE_pct,RMS_V
2024-06-03T12:34:56.000Z,50.0010,0.00010,141.4200,0.1500,141.4000,0.1600,0.1200,99.9700
2024-06-03T12:34:56.100Z,50.0020,0.00010,141.4300,0.1400,141.4100,0.1500,0.1100,99.9800
```

### 7.3 詳細テキスト出力（常時）

PLOT_MODE に関わらず、CSV の後に詳細テキストを出力する。

```
========================================
  PMU Synchrophasor Measurement Result
  IEEE C37.118.1 準拠 (LS + DFT)
========================================
Timestamp      : 2024-06-03T12:34:56.000Z
DC Offset      : 1.6502 V
RMS (sensor)   : 0.2012 V
Real RMS       : 100.60 V

--- 周波数 ---
Frequency      : 50.001 Hz  [正常 50Hz]
ROCOF          : 0.0001 Hz/s

--- LS (最小二乗法) フェーザー ---
  Amp (sensor) : 0.2845 V (pk)
  Amp (real)   : 142.25 V (pk)
  Amp RMS(real): 100.58 V
  Phase        : 0.150 deg
  cos coeff A  :  0.28449 V
  sin coeff B  : -0.00074 V

--- DFT フェーザー ---
  Bin          : 5  (50.00 Hz)
  Amp (sensor) : 0.2843 V (pk)
  Amp (real)   : 142.15 V (pk)
  Phase        : 0.160 deg

--- 高調波 (DFT) ---
  2nd (100 Hz): 0.002 V (pk)  45.00 deg
  3rd (150 Hz): 0.001 V (pk)  90.00 deg
  THD          : 0.70 %

--- TVE (Total Vector Error) ---
  TVE          : 0.1200 %  [合格 <=1%]
  Basis        : LS=ref, DFT=eval
========================================
```

---

## 8. キャリブレーション

### 8.1 SENSOR_RATIO の調整

```c
#define SENSOR_RATIO    500.0f   // pmu.c — 実測値で変更
```

**調整手順:**

1. 既知の RMS 電圧（例: 100 Vrms）を入力
2. シリアル出力の `RMS (sensor)` の値を確認
3. `SENSOR_RATIO = 実際の電圧 ÷ RMS (sensor)` で計算
4. 定数を更新して再ビルド

### 8.2 DC オフセット確認

`DC Offset` が `VDDA/2 = 1.65 V` 付近であることを確認。
大きくずれている場合は ZMPT101B のバイアス回路を確認すること。

### 8.3 GPS 測位確認

起動後、屋外または窓際に設置して測位完了を待つ（通常 1〜5分）。
`Timestamp` が `NO_FIX` から日時文字列に変われば測位完了。

---

## 9. ビルド・書き込み手順

### 9.1 リポジトリのクローン

```bash
git clone https://github.com/nap1212/PMU_Project.git
```

### 9.2 STM32CubeIDE でのインポート

1. **File → Import → General → Existing Projects into Workspace**
2. クローンしたフォルダを選択 → **Finish**

### 9.3 ビルド

**Project → Build All**（または Ctrl+B）

### 9.4 書き込み・実行

Nucleo を USB 接続後、**Run → Run**（または F11）

### 9.5 シリアルモニタ接続

| 項目 | 設定値 |
|------|--------|
| ポート | ST-Link 仮想 COM ポート |
| ボーレート | 115200 bps |
| 改行コード | CRLF |

---

## 10. 既知の制限事項

| 制限事項 | 内容 |
|---------|------|
| タイムスタンプ分解能 | HAL_GetTick() は 1ms 分解能。0.5ms 間隔のサンプルは 2サンプルで同一タイムスタンプになる |
| PLOT_MODE 0 の出力遅延 | 200サンプル分の UART 送信に約 780ms かかるため計測は連続しない |
| GPS 測位時間 | コールドスタート時は測位まで最大 5分かかる場合がある |
| 分・時またぎの精度 | タイムスタンプは分・時のロールオーバーに対応しているが日付変更には未対応 |
| 周波数推定精度 | ゼロクロス法のため高調波・ノイズの影響を受ける |
| GPS 絶対時刻同期のみ | GPS による絶対位置情報は未使用 |
| 単相のみ | 三相計測は未対応 |
| SENSOR_RATIO 固定 | 温度・経年変化によるセンサードリフトの自動補正なし |
