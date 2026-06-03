[PMU_Project_Spec.md](https://github.com/user-attachments/files/28540738/PMU_Project_Spec.md)
# PMU Synchrophasor Measurement — 仕様書

**バージョン**: 1.0  
**対象ボード**: STM32 Nucleo-F446RE  
**準拠規格**: IEEE C37.118.1  

---

## 目次

1. [プロジェクト概要](#1-プロジェクト概要)
2. [ハードウェア構成](#2-ハードウェア構成)
3. [ソフトウェア構成](#3-ソフトウェア構成)
4. [STM32周辺機能設定](#4-stm32周辺機能設定)
5. [アルゴリズム](#5-アルゴリズム)
6. [APIリファレンス](#6-apiリファレンス)
7. [出力フォーマット](#7-出力フォーマット)
8. [キャリブレーション](#8-キャリブレーション)
9. [ビルド手順](#9-ビルド手順)
10. [既知の制限事項](#10-既知の制限事項)

---

## 1. プロジェクト概要

### 1.1 目的

ZMPT101B 交流電圧センサーを用いて系統電圧の同期フェーザー（Synchrophasor）を計測するシステム。最小二乗法（LS）および離散フーリエ変換（DFT）の2手法でフェーザーを推定し、IEEE C37.118.1 に基づく TVE（Total Vector Error）評価を行う。

### 1.2 計測項目

| 計測項目 | 説明 |
|---------|------|
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
| バッファサイズ | 200 サンプル |
| 1バッファの時間長 | 100 ms（5サイクル @ 50Hz） |
| 公称周波数 | 50 Hz |

---

## 2. ハードウェア構成

### 2.1 使用機器

| 機器 | 型番・仕様 |
|------|-----------|
| マイコンボード | STM32 Nucleo-F446RE |
| 電圧センサー | ZMPT101B（交流電圧トランスモジュール） |
| 通信 | ST-Link 仮想COMポート（USB） |

### 2.2 配線

| ZMPT101B ピン | Nucleo ピン | 説明 |
|-------------|------------|------|
| OUT | PA0 (A0) | ADC入力 |
| VCC | 3.3V | 電源 |
| GND | GND | グランド |

> **注意**: STM32 の GPIO 最大入力電圧は 3.3V。ZMPT101B の出力が 3.3V を超えないようセンサー側のボリュームで調整すること。

### 2.3 ブロック図

```
[系統電圧] → [ZMPT101B] → [PA0 / ADC1] → [STM32F446RE] → [USART2] → [PC]
                                              ↑
                                          [TIM2 2000Hz]
```

---

## 3. ソフトウェア構成

### 3.1 開発環境

| 項目 | 内容 |
|------|------|
| IDE | STM32CubeIDE |
| HAL | STM32 HAL ライブラリ |
| 言語 | C (C11) |

### 3.2 ファイル構成

```
プロジェクト/
├── Core/
│   ├── Inc/
│   │   ├── main.h          # CubeIDE 自動生成
│   │   └── pmu.h           # PMU モジュール ヘッダ
│   └── Src/
│       ├── main.c          # CubeIDE 自動生成 + 追記
│       └── pmu.c           # PMU モジュール 実装
├── Drivers/                # HAL ライブラリ（自動生成）
├── .gitignore
└── *.ioc                   # CubeMX 設定ファイル
```

### 3.3 モジュール構成

```
main.c
└── PMU_Init()          起動メッセージ出力
└── PMU_Task()          メインループ処理（バッファ監視・解析）
    └── ProcessBuffer() バッファ解析・全計算・UART出力
        ├── LS_Phasor()          最小二乗法フェーザー推定
        ├── DFT_Phasor()         DFTフェーザー推定
        ├── EstimateFrequency()  ゼロクロス周波数推定
        ├── EstimateROCOF()      ROCOF推定
        └── CalcTVE()            TVE計算

割り込み系:
HAL_TIM_PeriodElapsedCallback() → PMU_TIM_Callback()  → HAL_ADC_Start_IT()
HAL_ADC_ConvCpltCallback()      → PMU_ADC_Callback()  → adcBuf へ格納
```

---

## 4. STM32周辺機能設定

### 4.1 クロック

| 項目 | 設定値 |
|------|--------|
| HCLK | 180 MHz |
| APB1 Timer clock | 90 MHz |
| APB2 clock | 90 MHz |

### 4.2 TIM2（サンプリングタイマー）

| 項目 | 設定値 |
|------|--------|
| Clock Source | Internal Clock |
| Counter Mode | Up |
| Prescaler | 0 |
| Counter Period | 44999 |
| 実際の周波数 | 90,000,000 ÷ 45,000 = **2000 Hz** |
| NVIC | TIM2 global interrupt 有効 |

### 4.3 ADC1（電圧計測）

| 項目 | 設定値 |
|------|--------|
| 入力チャンネル | IN0 (PA0) |
| 解像度 | 12 bit (0〜4095) |
| 基準電圧 | VDDA = 3.3 V |
| Trigger | Software |
| Clock Prescaler | PCLK2 divided by 4 (22.5 MHz) |
| Sampling Time | 480 Cycles |
| NVIC | ADC1 global interrupt 有効 |

### 4.4 USART2（デバッグ出力）

| 項目 | 設定値 |
|------|--------|
| Mode | Asynchronous |
| Baud Rate | 115200 bps |
| Word Length | 8 bit |
| Parity | None |
| Stop Bits | 1 |
| TX ピン | PA2 |
| RX ピン | PA3 |

---

## 5. アルゴリズム

### 5.1 最小二乗法（LS）フェーザー推定

信号モデル:

```
x[k] = A·cos(ω₀·k·Ts) + B·sin(ω₀·k·Ts)
```

行列形式で疑似逆行列を解く:

```
[A, B]ᵀ = (HᵀH)⁻¹ Hᵀ x
```

フェーザー出力:

```
振幅: Xm = √(A² + B²)
位相: φ  = atan2(-B, A)  [rad]
```

### 5.2 DFT フェーザー推定

対象 bin:

```
k = round(f₀ / (Fs / N))
```

DFT 計算:

```
X[k] = Σ x[n]·exp(-j2πkn/N)
振幅 = 2|X[k]| / N
位相 = angle(X[k])
```

### 5.3 周波数推定（ゼロクロス法）

1. DC オフセット除去
2. 立下りゼロクロス時刻を線形補間で検出（最大20点）
3. 隣接ゼロクロス間隔の平均値から周波数を算出
4. ±20% の外れ値を除外してロバスト性を確保

### 5.4 ROCOF 推定

```
ROCOF = (f_cur - f_prev) / T_update   [Hz/s]
T_update = BUF_SIZE / Fs = 0.1 s
```

### 5.5 TVE（Total Vector Error）

IEEE C37.118.1 定義:

```
TVE = √( (Xr_est - Xr_ref)² + (Xi_est - Xi_ref)² )
    / √( Xr_ref² + Xi_ref² )  × 100  [%]
```

- リファレンス: LS フェーザー  
- 被評価値: DFT フェーザー  
- 合格基準: TVE ≤ 1%（IEEE Class P/M）

---

## 6. APIリファレンス

### `PMU_Init(void)`
起動メッセージを UART 出力する。HAL 初期化後・タイマー開始前に呼ぶ。

### `PMU_TIM_Callback(void)`
TIM2 周期割り込みから呼ぶ。ADC 変換を開始する。

```c
// main.c に追記
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim) {
    if (htim->Instance == TIM2)
        PMU_TIM_Callback();
}
```

### `PMU_ADC_Callback(uint16_t raw)`
ADC 変換完了割り込みから呼ぶ。サンプルをバッファに格納する。

```c
// main.c に追記
void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef *hadc) {
    if (hadc->Instance == ADC1)
        PMU_ADC_Callback((uint16_t)HAL_ADC_GetValue(hadc));
}
```

### `PMU_Task(void)`
メインループから継続的に呼ぶ。バッファが揃ったら解析・出力を実行する。

---

## 7. 出力フォーマット

### 7.1 PLOT_MODE 0（波形データ）

Arduino Serial Plotter / CSV 対応フォーマット。200行/バッファ。

```
AC:<value>,LS_est:<value>,DFT_est:<value>
```

### 7.2 PLOT_MODE 1（フェーザー情報）

1行/バッファ。

```
Amp_LS:<V>,Amp_DFT:<V>,Phase_LS:<deg>,Phase_DFT:<deg>,Freq:<Hz>,TVE:<%>
```

### 7.3 詳細テキスト出力（常時）

```
========================================
  PMU Synchrophasor Measurement Result
  IEEE C37.118.1 準拠 (LS + DFT)
========================================
DC Offset      : x.xxxx V
RMS (sensor)   : x.xxxx V
Real RMS       : xxx.xx V

--- 周波数 ---
Frequency      : xx.xxx Hz  [正常 50Hz]
ROCOF          : x.xxxx Hz/s

--- LS (最小二乗法) フェーザー ---
  Amp (sensor) : x.xxxx V (pk)
  Amp (real)   : xxx.xx V (pk)
  Amp RMS(real): xxx.xx V
  Phase        : xxx.xxx deg
  cos coeff A  : x.xxxxx V
  sin coeff B  : x.xxxxx V

--- DFT フェーザー ---
  Bin          : xx  (xx.xx Hz)
  Amp (sensor) : x.xxxx V (pk)
  Amp (real)   : xxx.xx V (pk)
  Phase        : xxx.xxx deg

--- 高調波 (DFT) ---
  2nd (100 Hz): x.xxx V (pk)  xxx.xx deg
  3rd (150 Hz): x.xxx V (pk)  xxx.xx deg
  THD          : x.xx %

--- TVE (Total Vector Error) ---
  TVE          : x.xxxx %  [合格 <=1%]
  Basis        : LS=ref, DFT=eval
========================================
```

---

## 8. キャリブレーション

### 8.1 SENSOR_RATIO の調整

`pmu.c` の定数を実測値で更新する。

```c
#define SENSOR_RATIO    500.0f   // ← 実測値で変更
```

**調整手順:**

1. 既知の RMS 電圧（例: 100 Vrms）を入力
2. シリアルモニタで `RMS (sensor)` の値を確認
3. `SENSOR_RATIO = 実際の電圧 / RMS (sensor)` で計算
4. 定数を更新して再ビルド

### 8.2 ADC 中点確認

DC オフセットが `VDDA/2 = 1.65 V` 付近であることを確認。  
大きくずれている場合は ZMPT101B のバイアス回路を確認すること。

---

## 9. ビルド手順

### 9.1 リポジトリのクローン

```bash
git clone https://github.com/nap1212/PMU_Project.git
```

### 9.2 STM32CubeIDE でのインポート

1. **File → Import → General → Existing Projects into Workspace**
2. クローンしたフォルダを選択
3. **Finish**

### 9.3 ビルド

**Project → Build All**（または Ctrl+B）

### 9.4 書き込み

Nucleo を USB 接続後、**Run → Run**（または F11）

---

## 10. 既知の制限事項

| 制限事項 | 内容 |
|---------|------|
| PLOT_MODE 0 の出力遅延 | 200サンプル分のUART送信に約780msかかるため、計測は連続しない |
| 周波数推定精度 | ゼロクロス法のため高調波・ノイズの影響を受ける |
| 同期信号なし | GPS等による絶対時刻同期は未実装 |
| 単相のみ | 三相計測は未対応 |
| SENSOR_RATIO 固定 | 温度・経年変化によるセンサードリフトの自動補正なし |
