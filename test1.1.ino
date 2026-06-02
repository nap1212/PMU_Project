/*
 * ===========================================================================
 *  PMU Synchrophasor Measurement — Arduino Mega 2560
 *  Sensor  : ZMPT101B (AC voltage transformer module)
 *  Standard: IEEE C37.118.1 参照アルゴリズム準拠
 *            - Least Squares (LS)  フェーザー推定
 *            - DFT                 高調波補完・TVE算出
 *  Output  : シリアルモニタ (詳細) + シリアルプロッター (波形/フェーザー)
 *
 *  ピン配線:
 *    ZMPT101B OUT → A0 (ADC入力)
 *    VCC → 5V / GND → GND
 *
 *  サンプリング設計 (50Hz系統):
 *    Fs = 2000 Hz  (1サイクル40サンプル)
 *    BUF_SIZE = 200 サンプル = 5サイクル
 *    タイマー1 CTC モード: 16MHz / (Prescaler=8 / OCR1A=999) = 2000Hz
 * ===========================================================================
 */

#include <math.h>
#include <avr/io.h>
#include <avr/interrupt.h>

/* ─── 設定 ─────────────────────────────────────── */
#define PIN_ADC         A0          /* ZMPT101B 信号入力ピン          */
#define BUF_SIZE        200         /* サンプルバッファサイズ          */
#define FS              2000.0f     /* サンプリング周波数 [Hz]         */
#define F0_NOM          50.0f       /* 公称周波数 [Hz] (50 or 60)     */
#define ADC_REF_V       5.0f        /* ADC 基準電圧 [V]               */
#define ADC_BITS        1023.0f     /* 10-bit ADC 最大値              */

/* ZMPT101B 変換比 (要キャリブレーション)
   センサー出力 RMS [V] → 実電圧 RMS [V]
   例: 100Vrms / 計測RMS値 で求める                                   */
#define SENSOR_RATIO    500.0f

/* シリアルプロッター出力モード
   PLOT_MODE 0 : 波形 (ADC電圧, LS推定波形, DFT推定波形)
   PLOT_MODE 1 : フェーザー情報 (振幅, 位相, 周波数, TVE)            */
#define PLOT_MODE       0

/* TVE 閾値 [%] — IEEE C37.118.1: Class P/M ともに 1%               */
#define TVE_LIMIT       1.0f

/* ─── バッファ・フラグ ──────────────────────────── */
static volatile uint16_t adcBuf[BUF_SIZE];
static volatile uint16_t writeIdx   = 0;
static volatile bool    bufReady    = false;

/* ─── タイマー1 割り込み (2000Hz サンプリング) ────── */
ISR(TIMER1_COMPA_vect)
{
  if (!bufReady)
  {
    adcBuf[writeIdx] = analogRead(PIN_ADC);
    writeIdx++;
    if (writeIdx >= BUF_SIZE)
    {
      writeIdx = 0;
      bufReady = true;
    }
  }
}

/* ─── タイマー1 初期化 ──────────────────────────── */
static void Timer1_Init(void)
{
  /*  CTC モード, Prescaler = 8
   *  OCR1A = (F_CPU / Prescaler / Fs) - 1
   *         = (16000000 / 8 / 2000) - 1 = 999
   */
  cli();
  TCCR1A = 0;
  TCCR1B = 0;
  TCNT1  = 0;
  OCR1A  = 999;
  TCCR1B |= (1 << WGM12);   /* CTC モード          */
  TCCR1B |= (1 << CS11);    /* Prescaler = 8       */
  TIMSK1 |= (1 << OCIE1A);  /* 比較一致割り込み有効 */
  sei();
}

/* ===========================================================================
 *  最小二乗法 (Least Squares) フェーザー推定
 *  モデル: x[k] = A*cos(ω0*k*Ts) + B*sin(ω0*k*Ts)
 *  解:     [A, B]^T = (H^T H)^{-1} H^T x  (疑似逆行列)
 *  フェーザー振幅 Xm = sqrt(A^2 + B^2)
 *  位相          φ  = atan2(-B, A) [rad]
 * =========================================================================*/
typedef struct {
  float amp;      /* 振幅 Xm [V センサー出力]  */
  float phase;    /* 位相 φ  [deg]             */
  float A;        /* cos 係数                  */
  float B;        /* sin 係数                  */
} Phasor;

static Phasor LS_Phasor(const float *x, uint16_t N, float f0, float Ts)
{
  /*
   * HtH = H^T H は 2x2 対称行列
   *   HtH[0][0] = Σ cos²  HtH[0][1] = Σ cos*sin
   *   HtH[1][0] = HtH[0][1]          HtH[1][1] = Σ sin²
   * Htx = H^T x は 2x1 ベクトル
   *   Htx[0] = Σ cos*x[k]  Htx[1] = Σ sin*x[k]
   */
  double HtH00 = 0, HtH01 = 0, HtH11 = 0;
  double Htx0  = 0, Htx1  = 0;

  double omega = 2.0 * M_PI * f0 * Ts;

  for (uint16_t k = 0; k < N; k++)
  {
    double c = cos(omega * k);
    double s = sin(omega * k);
    HtH00 += c * c;
    HtH01 += c * s;
    HtH11 += s * s;
    Htx0  += c * x[k];
    Htx1  += s * x[k];
  }

  /* 2x2 逆行列: det = HtH00*HtH11 - HtH01^2 */
  double det = HtH00 * HtH11 - HtH01 * HtH01;

  Phasor p = {0};
  if (fabs(det) < 1e-12) return p;  /* 特異行列ガード */

  double A = ( HtH11 * Htx0 - HtH01 * Htx1) / det;
  double B = (-HtH01 * Htx0 + HtH00 * Htx1) / det;

  p.A     = (float)A;
  p.B     = (float)B;
  p.amp   = (float)sqrt(A * A + B * B);
  p.phase = (float)(atan2(-B, A) * 180.0 / M_PI);  /* deg */

  return p;
}

/* ===========================================================================
 *  DFT フェーザー推定 (基本波 + 高調波)
 *  N 点 DFT の bin k = round(f0 / (Fs/N)) を計算
 *  X[k] = Σ x[n] * exp(-j2πkn/N)
 *  振幅 = 2|X[k]|/N,  位相 = angle(X[k]) [deg]
 * =========================================================================*/
typedef struct {
  float amp;    /* [V] */
  float phase;  /* [deg] */
  float freq;   /* [Hz] — DFT bin 周波数 */
  int   bin;    /* DFT bin インデックス   */
} DFTPhasor;

static DFTPhasor DFT_Phasor(const float *x, uint16_t N, float targetFreq, float Fs_local)
{
  int bin = (int)roundf(targetFreq / (Fs_local / N));
  if (bin <= 0 || bin >= (int)(N / 2)) {
    DFTPhasor d = {0}; return d;
  }

  double re = 0, im = 0;
  double twoPiK_N = 2.0 * M_PI * bin / N;

  for (uint16_t n = 0; n < N; n++)
  {
    re += x[n] * cos(twoPiK_N * n);
    im -= x[n] * sin(twoPiK_N * n);
  }

  DFTPhasor d;
  d.bin   = bin;
  d.freq  = bin * (Fs_local / N);
  d.amp   = (float)(2.0 * sqrt(re * re + im * im) / N);
  d.phase = (float)(atan2(im, re) * 180.0 / M_PI);
  return d;
}

/* ===========================================================================
 *  TVE (Total Vector Error) 計算
 *  IEEE C37.118.1 式: TVE = sqrt( (Xr_est - Xr_true)^2 + (Xi_est - Xi_true)^2 )
 *                          / sqrt( Xr_true^2 + Xi_true^2 )
 *  ここでは LS をリファレンス、DFT を被評価値として計算
 * =========================================================================*/
static float CalcTVE(const Phasor *ref, const DFTPhasor *dft)
{
  /* 実部・虚部に変換 (RMS フェーザー表現: Xm/√2) */
  float phiRef = ref->phase * (float)M_PI / 180.0f;
  float phiDft = dft->phase * (float)M_PI / 180.0f;

  float Xr_ref = (ref->amp / sqrtf(2.0f)) * cosf(phiRef);
  float Xi_ref = (ref->amp / sqrtf(2.0f)) * sinf(phiRef);
  float Xr_dft = (dft->amp / sqrtf(2.0f)) * cosf(phiDft);
  float Xi_dft = (dft->amp / sqrtf(2.0f)) * sinf(phiDft);

  float num = sqrtf((Xr_dft - Xr_ref) * (Xr_dft - Xr_ref)
                  + (Xi_dft - Xi_ref) * (Xi_dft - Xi_ref));
  float den = sqrtf(Xr_ref * Xr_ref + Xi_ref * Xi_ref);

  if (den < 1e-9f) return 0.0f;
  return (num / den) * 100.0f;  /* % */
}

/* ===========================================================================
 *  周波数推定 — ゼロクロス法 + 外れ値除外 (元コードから継承・改良)
 * =========================================================================*/
static float EstimateFrequency(const float *x, uint16_t N, float Ts)
{
  float zeroCrossTimes[20];
  int   zcCount   = 0;
  float dcOffset  = 0;

  for (uint16_t i = 0; i < N; i++) dcOffset += x[i];
  dcOffset /= N;

  float prev = x[0] - dcOffset;
  bool  wasPos = (prev >= 0);

  for (uint16_t i = 1; i < N && zcCount < 20; i++)
  {
    float cur   = x[i] - dcOffset;
    bool  isPos = (cur >= 0);

    if (wasPos && !isPos)   /* 立下りゼロクロス */
    {
      float t = (float)(i - 1)
              + fabsf(prev) / (fabsf(prev) + fabsf(cur));
      zeroCrossTimes[zcCount++] = t * Ts;
    }
    wasPos = isPos;
    prev   = cur;
  }

  if (zcCount < 2) return 0.0f;

  /* 外れ値除外 (±20%) */
  float intervals[19];
  float sumAll = 0;
  int   iCount = zcCount - 1;

  for (int i = 0; i < iCount; i++)
  {
    intervals[i] = zeroCrossTimes[i + 1] - zeroCrossTimes[i];
    sumAll += intervals[i];
  }
  float roughAvg = sumAll / iCount;

  float sumValid = 0; int validN = 0;
  for (int i = 0; i < iCount; i++)
  {
    if (fabsf(intervals[i] - roughAvg) / roughAvg <= 0.2f)
    {
      sumValid += intervals[i]; validN++;
    }
  }

  if (validN < 1) return 0.0f;
  return 1.0f / (sumValid / validN);
}

/* ===========================================================================
 *  ROCOF 推定 — 連続した周波数推定値の差分
 *  簡易実装: 前回周波数との差分 / 更新間隔
 * =========================================================================*/
static float g_prevFreq    = 0.0f;
static float g_updateIntv  = (float)BUF_SIZE / FS;  /* バッファ長[s] */

static float EstimateROCOF(float curFreq)
{
  float rocof = 0.0f;
  if (g_prevFreq > 1.0f && curFreq > 1.0f)
    rocof = (curFreq - g_prevFreq) / g_updateIntv;
  g_prevFreq = curFreq;
  return rocof;
}

/* ===========================================================================
 *  メイン処理バッファ解析
 * =========================================================================*/
static void ProcessBuffer(void)
{
  const float Ts = 1.0f / FS;

  /* ─ ADC → 電圧変換 (float配列) ─ */
  float volt[BUF_SIZE];
  float dcOff = 0;
  for (uint16_t i = 0; i < BUF_SIZE; i++)
  {
    volt[i] = adcBuf[i] * (ADC_REF_V / ADC_BITS);
    dcOff  += volt[i];
  }
  dcOff /= BUF_SIZE;

  /* DCオフセット除去 */
  float ac[BUF_SIZE];
  for (uint16_t i = 0; i < BUF_SIZE; i++)
    ac[i] = volt[i] - dcOff;

  /* ─ RMS ─ */
  float sumSq = 0;
  for (uint16_t i = 0; i < BUF_SIZE; i++) sumSq += ac[i] * ac[i];
  float rms = sqrtf(sumSq / BUF_SIZE);

  /* ─ 周波数 & ROCOF ─ */
  float freq  = EstimateFrequency(volt, BUF_SIZE, Ts);
  float rocof = EstimateROCOF(freq);

  /* 周波数が不正な場合は公称値でフォールバック */
  float f0 = (freq > 40.0f && freq < 70.0f) ? freq : F0_NOM;

  /* ─ 最小二乗法フェーザー推定 ─ */
  Phasor ls = LS_Phasor(ac, BUF_SIZE, f0, Ts);

  /* ─ DFT フェーザー推定 (基本波) ─ */
  DFTPhasor dft = DFT_Phasor(ac, BUF_SIZE, f0, FS);

  /* ─ 高調波 (2次・3次) DFT ─ */
  DFTPhasor h2 = DFT_Phasor(ac, BUF_SIZE, f0 * 2.0f, FS);
  DFTPhasor h3 = DFT_Phasor(ac, BUF_SIZE, f0 * 3.0f, FS);

  /* ─ TVE ─ */
  float tve = CalcTVE(&ls, &dft);

  /* ─ 実電圧換算 ─ */
  float realRms  = rms           * SENSOR_RATIO;
  float realAmpLS  = ls.amp      * SENSOR_RATIO;
  float realAmpDFT = dft.amp     * SENSOR_RATIO;

  /* =========================================================
   *  シリアルプロッター出力
   *  Arduino IDE のシリアルプロッターは "ラベル:値" 形式で表示
   * ========================================================= */
#if PLOT_MODE == 0
  /* 波形プロット: センサーAC電圧 / LS推定 / DFT推定 */
  for (uint16_t i = 0; i < BUF_SIZE; i++)
  {
    /* LS推定波形再構成 */
    float lsWave  = ls.A  * cosf(2.0f * (float)M_PI * f0 * Ts * i)
                  + ls.B  * sinf(2.0f * (float)M_PI * f0 * Ts * i);
    /* DFT推定波形再構成 */
    float dftAmpPk = dft.amp;
    float dftWave  = dftAmpPk * cosf(2.0f * (float)M_PI * f0 * Ts * i
                               + dft.phase * (float)M_PI / 180.0f);

    Serial.print("AC:");      Serial.print(ac[i],    4);
    Serial.print(",LS_est:"); Serial.print(lsWave,   4);
    Serial.print(",DFT_est:");Serial.println(dftWave, 4);
  }

#else
  /* フェーザー情報プロット */
  Serial.print("Amp_LS:");  Serial.print(realAmpLS,  2);
  Serial.print(",Amp_DFT:");Serial.print(realAmpDFT, 2);
  Serial.print(",Phase_LS:");Serial.print(ls.phase,  2);
  Serial.print(",Phase_DFT:");Serial.print(dft.phase,2);
  Serial.print(",Freq:");   Serial.print(freq,       3);
  Serial.print(",TVE:");    Serial.println(tve,      3);
#endif

  /* =========================================================
   *  シリアルモニタ 詳細出力
   * ========================================================= */
  Serial.println(F(""));
  Serial.println(F("========================================"));
  Serial.println(F("  PMU Synchrophasor Measurement Result"));
  Serial.println(F("  IEEE C37.118.1 準拠 (LS + DFT)"));
  Serial.println(F("========================================"));

  /* --- 基本情報 --- */
  Serial.print(F("DC Offset      : ")); Serial.print(dcOff, 4);
  Serial.print(F(" V"));
  if (fabsf(dcOff - ADC_REF_V / 2.0f) > 0.3f)
    Serial.print(F("  [WARNING: 中点(2.5V)からずれています]"));
  Serial.println();

  Serial.print(F("RMS (sensor)   : ")); Serial.print(rms,     4); Serial.println(F(" V"));
  Serial.print(F("Real RMS       : ")); Serial.print(realRms, 2); Serial.println(F(" V"));

  /* --- 周波数 & ROCOF --- */
  Serial.println(F(""));
  Serial.println(F("--- 周波数 ---"));
  Serial.print(F("Frequency      : ")); Serial.print(freq, 3); Serial.print(F(" Hz"));
  if      (freq > 49.0f && freq < 51.0f) Serial.print(F("  [正常 50Hz]"));
  else if (freq > 59.0f && freq < 61.0f) Serial.print(F("  [正常 60Hz]"));
  else if (freq > 1.0f)                  Serial.print(F("  [異常!]"));
  else                                   Serial.print(F("  [信号なし]"));
  Serial.println();

  Serial.print(F("ROCOF          : ")); Serial.print(rocof, 4); Serial.println(F(" Hz/s"));

  /* --- 最小二乗法フェーザー --- */
  Serial.println(F(""));
  Serial.println(F("--- LS (最小二乗法) フェーザー ---"));
  Serial.print(F("  Amp (sensor) : ")); Serial.print(ls.amp,     4); Serial.println(F(" V (pk)"));
  Serial.print(F("  Amp (real)   : ")); Serial.print(realAmpLS,  2); Serial.println(F(" V (pk)"));
  Serial.print(F("  Amp RMS(real): ")); Serial.print(realAmpLS / sqrtf(2.0f), 2); Serial.println(F(" V"));
  Serial.print(F("  Phase        : ")); Serial.print(ls.phase,   3); Serial.println(F(" deg"));
  Serial.print(F("  cos coeff A  : ")); Serial.print(ls.A,       5); Serial.println(F(" V"));
  Serial.print(F("  sin coeff B  : ")); Serial.print(ls.B,       5); Serial.println(F(" V"));

  /* --- DFT フェーザー --- */
  Serial.println(F(""));
  Serial.println(F("--- DFT フェーザー ---"));
  Serial.print(F("  Bin          : ")); Serial.print(dft.bin);
  Serial.print(F("  ("));              Serial.print(dft.freq, 2); Serial.println(F(" Hz)"));
  Serial.print(F("  Amp (sensor) : ")); Serial.print(dft.amp,    4); Serial.println(F(" V (pk)"));
  Serial.print(F("  Amp (real)   : ")); Serial.print(realAmpDFT, 2); Serial.println(F(" V (pk)"));
  Serial.print(F("  Phase        : ")); Serial.print(dft.phase,  3); Serial.println(F(" deg"));

  /* --- 高調波 --- */
  Serial.println(F(""));
  Serial.println(F("--- 高調波 (DFT) ---"));
  Serial.print(F("  2nd ("));  Serial.print(f0 * 2, 0); Serial.print(F(" Hz): "));
  Serial.print(h2.amp * SENSOR_RATIO, 3); Serial.print(F(" V (pk)  "));
  Serial.print(h2.phase, 2); Serial.println(F(" deg"));
  Serial.print(F("  3rd ("));  Serial.print(f0 * 3, 0); Serial.print(F(" Hz): "));
  Serial.print(h3.amp * SENSOR_RATIO, 3); Serial.print(F(" V (pk)  "));
  Serial.print(h3.phase, 2); Serial.println(F(" deg"));

  /* 全高調波歪率 THD */
  float thd = 0;
  if (ls.amp > 1e-6f)
    thd = sqrtf(h2.amp * h2.amp + h3.amp * h3.amp) / ls.amp * 100.0f;
  Serial.print(F("  THD          : ")); Serial.print(thd, 2); Serial.println(F(" %"));

  /* --- TVE (IEEE C37.118.1) --- */
  Serial.println(F(""));
  Serial.println(F("--- TVE (Total Vector Error) ---"));
  Serial.print(F("  TVE          : ")); Serial.print(tve, 4); Serial.print(F(" %"));
  if (tve <= TVE_LIMIT)
    Serial.print(F("  [合格 ≤1%]"));
  else
    Serial.print(F("  [不合格 >1%]"));
  Serial.println();

  Serial.print(F("  Basis        : LS=ref, DFT=eval"));
  Serial.println();

  Serial.println(F("========================================"));
  Serial.println(F(""));
}

/* ===========================================================================
 *  setup / loop
 * =========================================================================*/
void setup(void)
{
  Serial.begin(115200);
  analogReference(DEFAULT);   /* 5V 基準 */
  pinMode(PIN_ADC, INPUT);

  delay(500);
  Serial.println(F("PMU Arduino Mega — 起動"));
  Serial.print(F("Fs = "));    Serial.print(FS,       0); Serial.println(F(" Hz"));
  Serial.print(F("BUF  = "));  Serial.print(BUF_SIZE);    Serial.println(F(" samples"));
  Serial.print(F("f0_nom = ")); Serial.print(F0_NOM,  0); Serial.println(F(" Hz"));
  Serial.println(F("タイマー1 初期化..."));

  Timer1_Init();

  Serial.println(F("計測開始"));
  Serial.println(F(""));
}

void loop(void)
{
  if (bufReady)
  {
    /* bufReady=true の間 ISR は if(!bufReady) をスキップするため
     * adcBuf への書き込みは発生しない — 安全に読み出せる          */
    ProcessBuffer();

    /* 次のバッファ収集を再開 */
    cli();
    writeIdx = 0;
    bufReady = false;
    sei();
  }
}
