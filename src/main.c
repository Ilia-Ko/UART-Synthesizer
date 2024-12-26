#include "stm32h743xx.h"

#include "stm32h7xx_ll_rcc.h"
#include "stm32h7xx_ll_bus.h"
#include "stm32h7xx_ll_utils.h"
#include "stm32h7xx_ll_exti.h"
#include "stm32h7xx_ll_gpio.h"
#include "stm32h7xx_ll_tim.h"
#include "stm32h7xx_ll_adc.h"
#include "stm32h7xx_ll_dac.h"
#include "stm32h7xx_ll_opamp.h"
#include "stm32h7xx_ll_usart.h"

#include "arm_math.h"
#include "string.h"
#include "stdio.h"

// GPIO, ADC3 on AHB4
// DAC1, OPAMP1(2), TIM2 - TIM5 on APB1
// ADC1, ADC2, DMA1, DMA2 on AHB1
// USART1 on APB2
// RNG on AHB2

#define HCLK_FREQ (96*1000*1000UL)

// LED on |PA1| (active LOW)
// DAC1.OUT1 on |PA4|
// DAC1.OUT2 on |PA5|
// ADC3.INP0 on |PC2|
// USART1.TX on |PA9|, PB6,PB14 // USART2.TX on PA2,PD5   // USART6.TX on PG14,PC6    // USART3.TX on PD8,PC10,PB10
// USART1.RX on |PA10|,PB7,PB15 // USART2.RX on PA3,PD6   // USART6.RX on PG9 ,PC7    // USART3.RX on PD9,PC11,PB11

#define TRUE (2+2 == 4)
#define FALSE (!TRUE)
#define VOLATILE(typ,x) (*(volatile typ*) &(x))
typedef unsigned long bool;

// --- --- --- UART defines --- --- ---
#define BAUD_RATE (2000*1000UL)     // Signalling rate for UART to maintain 1152 kbps audio transfer
#define RX_BUF_SIZE (256*3)         // RAM buffer length to receive commands
#define RX_BUF_HALF (RX_BUF_SIZE/2) // Half of the RX buffer
#define TX_BUF_SIZE 256             // RAM buffer length to compose messages
#define CMD_BUF_SIZE 128            // Command buffer
#define STREAM_STOP_1 0x29b8c56cU   // First 4 stream stop bytes
#define STREAM_STOP_2 0x1fedca8dU   // Second 4 stream stop bytes
#define MIN_CMD_LENGTH 4            // The shortest command
#define MAX_CMD_LENGTH 128          // The longest command
#define MSG_PONG        "pong"
#define MSG_OVERFLOW    "Command rewinded"
#define MSG_INVALID     "Invalid command"
#define MSG_INVD_WAVE   "Invalid wave kind. "
#define MSG_INVD_AMPL   "Invalid amplitude spec. "
#define MSG_INVD_RND    "Invalid entropy source. "
#define MSG_INVD_TONE   "Invalid tone. "
#define MSG_INVD_NOISE  "Invalid noise kind. "
#define MSG_INVD_FREQ   "Tone out of range. "
#define MSG_INVD_BEAT   "Invalid beat duration. "
#define MSG_INVD_IMPR   "Invalid improvisation. "
#define MSG_INVD_SCAL   "Invalid scale. "
#define MSG_INVD_COEF   "Invalid filter coefficient. "
#define MSG_INVD_FILT   "Invalid filter specification. "
#define MSG_INVD_MUTAB  "Invalid mutability. "
#define MSG_INVD_PATT   "Invalid pattern beat. "
#define MSG_INVD_EQFR   "Invalid frequency spec. "
#define MSG_EQ_OUTRANGE "Frequency out of range. "
#define MSG_ILL_FILTER  "Warning: IIR denominator 0-th coefficient too close to zero! Replacing with 1. "
#define MSG_SET_RNG     "Defined entropy source: hardware RNG"
#define MSG_SET_ADC     "Defined entropy source: floating ADC"
#define MSG_SET_FILT    "Filter ON:"
#define MSG_SET_NOFILT  "Filter OFF:"
#define MSG_BEAT_DEF    "Beat duration redefined"
#define MSG_WAVE_DEF    "Wave type redefined"
#define MSG_FILT_DEF    "Filter coeffs redefined"
#define MSG_PATT_DEF    "Beat pattern redefined"
#define MSG_EQ_DEF      "Equalizer frequencies redefined"
// Commands
#define CMD_PING        "ping"          // > ping
#define CMD_STATUS      "status"        // > status
#define CMD_STOP        "stop"          // > stop
#define CMD_FIRE        "fire "         // > fire A 300         > fire 314.2 200
#define CMD_IMPROV      "impr "         // > impr chaos 400     > impr scale f/maj 250      > impr board 300
#define CMD_PLAY        "play"          // > play ['path/to/file.flac' (processed by PC)]
#define CMD_SET_WAVE    "set wave "     // > set wave sin 1.0   > set wave rect 0.5
#define CMD_SET_RAND    "set entropy "  // > set entropy RNG    > set entropy ADC
#define CMD_SET_FILT    "set filter "   // > set filter on      > set filter off    > set filter 1.0 -4 +3e-2 0.0 / 1 -0.1 0 5
#define CMD_SET_BEAT    "set beat "     // > set beat 250
#define CMD_SET_PATT    "set pattern "  // > set pattern 1 1 0.5 0.5 1  > set pattern mutable   > set pattern stable
#define CMD_SET_EQ      "set equalizer "// > set equalizer 100.0 800.0 365.0 440.0 3600.0
#define CMD_GET_WAVE    "get wave"      // > get wave
#define CMD_GET_RAND    "get entropy"   // > get entropy
#define CMD_GET_FILT    "get filter"    // > get filter
#define CMD_GET_BEAT    "get beat"      // > get beat
#define CMD_GET_PATT    "get pattern"   // > get pattern
#define CMD_GET_EQ      "get equalizer" // > get equalizer
// Improvisation modes
#define STR_IMPR_LADDER "ladder"
#define STR_IMPR_CHAOS  "chaos"
#define STR_IMPR_RKEYS  "board"
#define STR_IMPR_SCALE  "scale"
// Noise types
#define STR_NOISE       "noise "
#define STR_NOISE_WHITE "white"
#define STR_NOISE_PINK  "pink"
#define STR_NOISE_RED   "red"
#define STR_NOISE_BLUE  "blue"
#define STR_NOISE_VIOLET "violet"
#define STR_NOISE_UV    "uv"
// Pattern strings
#define STR_PATT_STABLE     "stable"
#define STR_PATT_MUTABILITY "mutability "
// Workflow
enum REGIME { IDLE, PAUSING, ONESHOT, IMPROV, STREAM }; // Operation regimes
enum IMPROV_MODE { LADDER, RAND_FREQS, RAND_KEYS, RAND_KEYS_SCALE };    // Improvisation modes
enum RND_SOURCE { HARD_RNG, ADC_FLOAT };    // Entropy sources
enum NOISE_TYPE { NOISE_WHITE, NOISE_PINK, NOISE_RED, NOISE_BLUE, NOISE_VIOLET, NOISE_UV };

// Some filters for noise synthesis
// Differencing filters
#define DIFF_LEN 5
#define DIFF2_LEN 5
const float32_t DIFF_B[DIFF_LEN] = {-1/12.0f, 2/3.0f, 0.0f, -2/3.0f, 1/12.0f};
const float32_t DIFF2_B[DIFF2_LEN] = {-1/12.0f, 4/3.0f, -5/2.0f, 4/3.0f, -1/12.0f};
// Pinking filter by Julius Orion Smith III
#define PINK_LEN 4
const float32_t PINK_B[PINK_LEN] = {0.049922035f, -0.095993537f, 0.050612699f, -0.004408786f};
const float32_t PINK_A[PINK_LEN] = {1.0f,         -2.494956002f, 2.017265875f, -0.522189400f};

// UART globals
LL_RCC_ClocksTypeDef clk;   // RCC clock frequencies
char rxBuf[RX_BUF_SIZE];    // Receive buffer
char txBuf[TX_BUF_SIZE];    // Send buffer
char cmdBuf[CMD_BUF_SIZE];  // Command buffer
uint32_t cmdLen;            // Length of command inside CMD buf
uint32_t rxPos;             // Current position inside RX buffer, used by UART
uint32_t sPos;              // Current sample position in RX buffer, used by DAC
bool isOverflow;            // Whether buffer was overflown
volatile bool isPending;    // Whether a command is pending
enum REGIME regime;         // Current operation regime
enum IMPROV_MODE improvMode;// Current improvisation mode (active in IMPROV regime)
void SendMessage(const char* msg, uint32_t msgLen);


// --- --- --- Sound defines --- --- ---
#define MAX_CODE (1 << 12)  // DAC maximal sample value
#define MID_CODE (1 << 11)  // DAC medium sample value
#define BASE_FREQ 48000     // Count of cycles per 1 second
#define EMA_K (1 - 1 / (5e-3f * BASE_FREQ)) // Smooth exponential 5ms transitions
#define BASE_TONE(a) (BASE_TONES[(a)-'A'])  // Converts a tone from 'A' to 'G' into its frequency
#define HALF_TONE 1.0594630943592953f       // 2^(1/12)
#define SQRT2 1.4142135623730951f           // sqrt(2)
#define MAX_SCALE_LEN 12    // Maximal length of any scale
#define NUM_SCALES 16       // Count of implemented scales
#define FILT_DEPTH 6        // Maximal power of numerator and denominator of IIR filter
#define FILT_MEM 1024       // Input and output memory depth for equalized IIR filter
#define MAX_PATTERN 16      // Maximal length of beat pattern
enum WAVE {
    WAVE_SIN, WAVE_SIN3, WAVE_SIN5, WAVE_RECT, WAVE_SAWTOOTH, WAVE_TRIANGULAR
};
#define STR_WAVE_SIN    "sin"
#define STR_WAVE_SIN3   "sin3"
#define STR_WAVE_SIN5   "sin5"
#define STR_WAVE_RECT   "rect"
#define STR_WAVE_SAWTH  "saw"
#define STR_WAVE_TRIANG "3ang"
// Selection of base tone
const float32_t BASE_TONES[7] = {   /* A, B, C, D, E, F, G */
    440.0f,
    493.8833012561239f,
    261.6255653005987f,
    293.6647679174076f,
    329.6275569128699f,
    349.2282314330039f,
    391.9954359817492f,
};
const float32_t HT_POW[MAX_SCALE_LEN+1] = {
    1.0f,
    1.059463094359295f,
    1.122462048309373f,
    1.189207115002721f,
    1.259921049894873f,
    1.334839854170034f,
    1.414213562373095f,
    1.498307076876682f,
    1.587401051968199f,
    1.681792830507429f,
    1.781797436280678f,
    1.887748625363387f,
    2.0f
};
// Selection of scale
#define STR_SCALE_CHROM "chrom"
#define STR_SCALE_MAJOR "maj"
#define STR_SCALE_NAT_MINOR "nat.min"
#define STR_SCALE_MEL_MINOR "mel.min"
#define STR_SCALE_HAR_MINOR "har.min"
#define STR_SCALE_PENT_MAJOR "pent.maj"
#define STR_SCALE_PENT_MINOR "pent.min"
#define STR_SCALE_PENT_JAPAN "japanese"
#define STR_SCALE_LYDIAN     "lydian"
#define STR_SCALE_MIXOLYDIAN "mixolydian"
#define STR_SCALE_DORIAN     "dorian"
#define STR_SCALE_PHRYGIAN   "phrygian"
#define STR_SCALE_LOCRIAN    "locrian"
#define STR_SCALE_BLUES      "blues"
#define STR_SCALE_BLUES_7    "blues-7"
#define STR_SCALE_BLUES_9    "blues-9"
enum SCALE {
    CHROMATIC = 0, MAJOR, MINOR_NATUR, MINOR_MELOD, MINOR_HARMON,
    PENT_MAJOR, PENT_MINOR, PENT_JAPAN, LYDIAN, MIXOLYDIAN,
    DORIAN, PHRYGIAN, LOCRIAN, BLUES, BLUES_7, BLUES_9
};
const uint32_t SCALES[MAX_SCALE_LEN * NUM_SCALES] = {    /* Table of defined scales */
    0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11,   /* Chromatic scale*/
    0, 2, 4, 5, 7, 9, 11, 0, 0, 0, 0, 0,    /* Major scale */
    0, 2, 3, 5, 7, 8, 10, 0, 0, 0, 0, 0,    /* Natural minor scale */
    0, 2, 3, 5, 7, 9, 11, 0, 0, 0, 0, 0,    /* Melodic minor scale */
    0, 2, 3, 5, 7, 8, 11, 0, 0, 0, 0, 0,    /* Harmonic minor scale */
    0, 2, 4, 7, 9, 0,  0, 0, 0, 0, 0, 0,    /* Pentatonic major scale */
    0, 3, 5, 7, 10,0,  0, 0, 0, 0, 0, 0,    /* Pentatonic minor scale */
    0, 1, 5, 7, 8, 0,  0, 0, 0, 0, 0, 0,    /* Pentatonic Japanese scale */
    0, 2, 4, 6, 7, 9, 11, 0, 0, 0, 0, 0,    /* Lydian scale */
    0, 2, 4, 5, 7, 9, 10, 0, 0, 0, 0, 0,    /* Mixolydian scale */
    0, 2, 3, 5, 7, 9, 10, 0, 0, 0, 0, 0,    /* Dorian scale */
    0, 1, 3, 5, 7, 8, 10, 0, 0, 0, 0, 0,    /* Phrygian scale */
    0, 2, 3, 5, 6, 8, 10, 0, 0, 0, 0, 0,    /* Locrian scale */
    0, 3, 5, 6, 7, 10, 0, 0, 0, 0, 0, 0,    /* Blues hexatonic */
    0, 2, 3, 5, 6, 9, 10, 0, 0, 0, 0, 0,    /* Blues diatonic */
    0, 2, 3, 4, 5, 7, 9, 10,11, 0, 0, 0,    /* Blues nonatonic */
};
const uint32_t SCALE_SIZE[NUM_SCALES] = {    /* List of defined scale lengths */
    12, 7, 7, 7, 7,
    5, 5, 5,
    7, 7, 7, 7, 7,
    6, 7, 9
};

// Sound globals
enum WAVE waveType = WAVE_SIN;  // Kind of wave to synthesize
float32_t amp = 0.97f;          // Wave amplitude
uint32_t cycle = 0;             // Count of cycles within one second
uint32_t seconds = 0;           // Count of seconds (should be kept as small as possible with modulus)
float32_t phi = 0.0f;           // Phase adjustment, necessary for fractional frequencies
float32_t oldFreq = 0.0f;       // Previous tone frequency
float32_t freq = 0.0f;          // Current tone frequency
float32_t ema = 0.0f;           // Exponential transitions between tones
bool isNoise = FALSE;           // Whether to emit noise instead of normal waves
// Random globals
enum RND_SOURCE rndSource;      // Current entropy source
bool doFilter;                  // Whether to filter the random sequence
uint32_t  rndMax;               // Maximal random value
float32_t rndNorm;              // 1 / (float32_t) rndMax
enum NOISE_TYPE noiseType = NOISE_WHITE;    // Selected noise model
float32_t noiseDiv;             // Common noise divider (to keep noise within range [-1,+1])
float32_t IIR_A[FILT_DEPTH+1];  // IIR filter coefficients for denominator
float32_t IIR_B[FILT_DEPTH+1];  // IIR filter coefficients for numerator
float32_t iirFastInp[FILT_DEPTH];   // Short-term cyclic buffer for IIR-filtered input sequence
float32_t iirFastOut[FILT_DEPTH];   // Short-term cyclic buffer for IIR-filtered output sequence
float32_t iirInp[FILT_MEM];     // Long-term cyclic buffer of IIR-filtered input samples
float32_t iirOut[FILT_MEM];     // Long-term cyclic buffer of IIR-filtered output samples
uint32_t iirPsc[FILT_DEPTH+1];  // IIR prescalers: indices of samples within the long-term buffer, corresponding to active frequencies
uint32_t iirPos;                // Cyclic position in the long-term or short-term buffers
uint32_t (*Entropy)(void);      // Source of random numbers
// Scale globals
enum SCALE scIdx;               // Index of current scale
uint32_t scLen;                 // Actual length of current scale
float32_t scale[3*MAX_SCALE_LEN];   // Frequencies of current scale
// Improvisation globals
uint32_t beatDurMS = 200;       // Beat duration in milliseconds
uint32_t pattLen;               // Actual length of beat pattern
uint32_t pattPos;               // Index of beat inside beat pattern
float32_t mutability;           // Pattern mutability
float32_t mutQuant;             // Mutation quantum
float32_t pattern[MAX_PATTERN];     // Beat pattern
float32_t (*ImproviseNext)(void);   // Improvisator routine
bool isRising;                  // Ladder mode (active im LADDER IMPROV_MODE)

// --- --- --- Wave processing --- --- ---
void SetScale(enum SCALE sc, float32_t baseTone) {      // Compute scale of frequencies into 'scale'
    scIdx = sc;
    scLen = SCALE_SIZE[scIdx];

    // Construct central scale
    for (uint32_t i = 0; i < scLen; i++) {
        uint32_t powIdx = SCALES[scIdx * MAX_SCALE_LEN + i];
        scale[i] = baseTone * HT_POW[powIdx];
    }
    if (2*scLen <= MAX_SCALE_LEN) {
        for (uint32_t i = 0; i < scLen; i++) {
            scale[scLen+i] = 2 * scale[i];
        }
        scLen *= 2;
    }

    // Construct lower and higher transposed scales
    for (uint32_t i = 0; i < scLen; i++) {
        scale[2*scLen+i] = 2*scale[i];
        scale[scLen+i] = scale[i];
        scale[i] /= 2;
    }
    scLen *= 3;

    // Do not go way too high
    uint32_t origLen = SCALE_SIZE[scIdx];
    if (2*origLen <= MAX_SCALE_LEN) scLen -= origLen;
}
void ChangeFreq(float32_t newFreq) {    // Set the next wave frequency to be synthesized
    oldFreq = freq;
    ema = EMA_K;
    freq = newFreq;
}
void FireOneShot(uint32_t durMS) {      // Issue a 'durMS' milliseconds delay
    LL_TIM_DisableCounter(TIM3);            // Stop TIM3 anyway
    LL_TIM_SetCounter(TIM3, 0);
    LL_TIM_SetAutoReload(TIM3, 2*durMS);    // Program TIM3 for 'durMS' delay
    LL_TIM_EnableCounter(TIM3);             // Fire TIM3
}
void RestartBaseTIM(void) { // Reset base timer TIM2 and start it again
    LL_TIM_DisableCounter(TIM2);
    LL_TIM_SetCounter(TIM2, 0);
    seconds = 0;
    cycle = 0;
    LL_TIM_EnableCounter(TIM2);
}
uint32_t EntropyRNG(void) { // Get next random from hardware RNG
    while (0 == (RNG->SR & RNG_SR_DRDY_Msk));   // Wait until RNG accumulates the next random
    return RNG->DR;
}
uint32_t EntropyADC(void) { // Get next random from floating ADC
    // while (!LL_ADC_IsActiveFlag_EOC(ADC3));
    LL_ADC_ClearFlag_EOC(ADC3);

    uint32_t rnd = LL_ADC_REG_ReadConversionData12(ADC3);
    LL_ADC_REG_StartConversion(ADC3);
    // LL_ADC_REG_StartConversionSWStart(ADC3);    // Launch a new conversion for future
    return rnd;
}
void FillRandom(float32_t *buf, uint32_t count) {       // Fill some buffer with random floats within [-1,+1] range
    for (uint32_t i = 0; i < count; i++) {
        buf[i] = 2 * Entropy() * rndNorm - 1.0f;
    }
}
void DefineEntropySource(enum RND_SOURCE src) {         // Setup globals for entropy source 'src'
    switch (src) {
    case HARD_RNG:
        rndMax = UINT32_MAX;
        Entropy = &EntropyRNG;
        break;
    case ADC_FLOAT:
        // rndMax = 1U << 12;
        rndMax = 100U;  // Noise is rather weak on the floating ADC input
        Entropy = &EntropyADC;
        break;
    }

    rndNorm = 1.0f / (float32_t) rndMax;    // Update random normalizator

    // Regenerate random sequences if entropy source has changed
    if (rndSource != src) {
        FillRandom(iirFastOut, FILT_DEPTH);
        memset(iirOut, 0, sizeof(iirOut));  // Somewhat long operation
    }
    rndSource = src;    // Remember new entropy source
}
float32_t FilterSeq(uint32_t nextRnd, float32_t norm) { // Perform one step of short-term IIR-filtering
    float32_t nextR = 2.0f * (nextRnd * norm) - 1.0f;   // Work with real numbers from -1 to +1
    float32_t s = IIR_B[0] * nextR;
    uint32_t idx = iirPos;
    for (uint32_t i = 1; i < FILT_DEPTH; i++) {
        s += IIR_B[i] * iirFastInp[idx];
        s -= IIR_A[i] * iirFastOut[idx];
        idx++;
        if (idx >= FILT_DEPTH) idx = 0;
    }
    s /= IIR_A[0];  // Use non-unit 0-th denominator coefficient as global scaler

    // Update input and output samples and cyclic position
    iirFastInp[iirPos] = nextR;
    iirFastOut[iirPos] = s;
    if (iirPos > 0) iirPos--;
    else iirPos = FILT_DEPTH - 1;
    return (1.0f + s) / 2.0f;   // Return number from 0 to 1
}
float32_t FilterSample(float32_t next) {                // Perform one step of long-term filtering, takes -1 <= next <= +1
    // Update position in the cyclic buffer
    if (iirPos > 0) iirPos--;
    else iirPos = FILT_MEM - 1;

    // Store the next input sample
    iirInp[iirPos] = next;

    // Calculate the next output sample
    float32_t s = IIR_B[0] * iirInp[(iirPos+iirPsc[0]) % FILT_MEM];
    for (uint32_t i = 1; i <= FILT_DEPTH; i++) {
        uint32_t idx = (iirPos + iirPsc[i]) % FILT_MEM;
        s += IIR_B[i] * iirInp[idx];
        s -= IIR_A[i] * iirOut[idx];
    }
    s /= IIR_A[0];  // Use non-unit 0-th denominator coefficient as global scaler

    // Store the next output sample
    iirOut[iirPos] = s;
    return s;
}
void DefineFilter(const float32_t cA[FILT_DEPTH], const float cB[FILT_DEPTH]) { // Setup globals for IIR-filtering
    // Update the IIR coefficients
    memcpy(IIR_A, cA, sizeof(float32_t)*FILT_DEPTH);
    memcpy(IIR_B, cB, sizeof(float32_t)*FILT_DEPTH);

    // Avoid near-zero division
    if (cA[0] < 1e-6f && cA[0] > -1e-6f) {
        if (LL_USART_IsEnabled(USART1)) SendMessage(MSG_ILL_FILTER, sizeof MSG_ILL_FILTER - 1);
        IIR_A[0] = 1.0f;
    }
}

// --- --- --- Wave generation modes --- --- ---
float32_t Next_Ladder() {
    float32_t newF;
    if (isRising) newF = freq * HALF_TONE;
    else newF = freq / HALF_TONE;

    if (newF >= 880.0f) {
        newF = 880.0f;
        isRising = FALSE;
    } else if (newF <= 110.0f) {
        newF = 110.0f;
        isRising = TRUE;
    }

    return newF;
}
float32_t Next_RandomFreqs() {
    if (doFilter)
        return 110.0f + 770.0f * FilterSeq(Entropy(), rndNorm);
    else
        return 110.0f + 770.0f * (Entropy() * rndNorm);
}
float32_t Next_RandomKeys() {
    uint32_t rnd;
    if (doFilter) rnd = FilterSeq(Entropy(), rndNorm) * rndMax;
    else rnd = Entropy();

    uint32_t powIdx = rnd % (12U * 4U);
    uint32_t locIdx = powIdx % 12U;
    uint32_t outIdx = powIdx / 12U;

    return 110.0f * HT_POW[locIdx] * (1U << outIdx);
}
float32_t Next_RandomKeys_Scale() {
    uint32_t rnd;
    if (doFilter) rnd = FilterSeq(Entropy(), rndNorm) * rndMax;
    else rnd = Entropy();

    return scale[rnd % scLen];
}

// --- --- --- UART utilities --- --- ---
void SendByte(char byte) {
    while (!LL_USART_IsActiveFlag_TC(USART1));      // Wait until previous transmission is completed
    LL_USART_TransmitData8(USART1, byte);           // Send the next byte
}
void SendMessage(const char* msg, uint32_t msgLen) {    // Send a sequence of bytes via USART1
    for (uint32_t i = 0; i < msgLen; i++) {
        while (!LL_USART_IsActiveFlag_TC(USART1));  // Wait until previous transmission is completed
        LL_USART_TransmitData8(USART1, msg[i]);     // Send the next byte
        LL_GPIO_TogglePin(GPIOG, LL_GPIO_PIN_14);   // Change UART LED state
    }
}
bool ParseTone(const char* str, uint32_t strLen, float32_t* freq) {
    bool res = FALSE;

    char tone = str[0];
    char lcTone = tone | 0x20;
    float32_t newF;
    if (lcTone >= 'a' && lcTone <= 'g') {
        // > f
        // > С+5
        // > С-3
        // Determine main frequency
        newF = BASE_TONE(0x20 ^ lcTone);    // Base tone from uppercase character
        if (0 != (tone & 0x20)) newF /= HALF_TONE;  // Apply flat if char was lowercase
        // Check for octave modifier
        if (1 == strLen) {
            res = TRUE;
        } else {
            // Parse octave shift
            int oct = 0;
            int nArg = sscanf(str+1, "%d", &oct);
            if (1 != nArg) return FALSE;
            // Check octave ranges
            if (oct < -4 || oct > +5) {
                SendMessage(MSG_INVD_TONE, sizeof MSG_INVD_TONE - 1);
                return FALSE;
            }
            // Shift frequency by several octaves
            if (oct < 0)
                newF /= (float32_t) (1 << (-oct));
            else
                newF *= (float32_t) (1 << oct);
            res = TRUE;
        }
        // Output argument
        *freq = newF;
    }

    return res;
}
bool ExecCommand(char* cmd, uint32_t cmdLen) {    // Execute user command and return success flag
    bool res = FALSE;
    uint32_t pos = 0;
    // Parse the command
    if (0 == strncmp(cmd, CMD_PING, sizeof CMD_PING)) {             // Compare incl. 0
        SendMessage(MSG_PONG, sizeof MSG_PONG-1);
        res = TRUE;
    } else if (0 == strncmp(cmd, CMD_STATUS, sizeof CMD_STATUS)) {  // Compare incl. 0
        // > status
        pos = snprintf(txBuf, TX_BUF_SIZE, "%.1f MHz ", clk.HCLK_Frequency * 1e-6f);
        switch (regime) {
        case IDLE:
            pos += snprintf(txBuf+pos, TX_BUF_SIZE-pos, "idle");
            break;
        case PAUSING:
            pos += snprintf(txBuf+pos, TX_BUF_SIZE-pos, "pausing");
            break;
        case ONESHOT:
            uint32_t durMS = LL_TIM_GetAutoReload(TIM3)/2;
            uint32_t remMS = durMS - LL_TIM_GetCounter(TIM3)/2;
            if (isNoise) {
                pos += snprintf(txBuf+pos, TX_BUF_SIZE-pos, "firing ");
                switch (noiseType) {
                case NOISE_WHITE:
                    pos += snprintf(txBuf+pos, TX_BUF_SIZE-pos, "white ");
                    break;
                case NOISE_PINK:
                    pos += snprintf(txBuf+pos, TX_BUF_SIZE-pos, "pink ");
                    break;
                case NOISE_RED:
                    pos += snprintf(txBuf+pos, TX_BUF_SIZE-pos, "red ");
                    break;
                case NOISE_BLUE:
                    pos += snprintf(txBuf+pos, TX_BUF_SIZE-pos, "blue ");
                    break;
                case NOISE_VIOLET:
                    pos += snprintf(txBuf+pos, TX_BUF_SIZE-pos, "violet ");
                    break;
                case NOISE_UV:
                    pos += snprintf(txBuf+pos, TX_BUF_SIZE-pos, "uv ");
                    break;
                }
                pos += snprintf(txBuf+pos, TX_BUF_SIZE-pos, "noise");
            } else {
                pos += snprintf(txBuf+pos, TX_BUF_SIZE-pos, "firing freq = %.3f Hz", freq);
            }
            pos += snprintf(txBuf+pos, TX_BUF_SIZE-pos, ", duration = %ld ms, remaining %ld ms", durMS, remMS);
            break;
        case IMPROV:
            pos += snprintf(txBuf+pos, TX_BUF_SIZE-pos, "improvisation ");
            switch (improvMode) {
            case LADDER:
                pos += snprintf(txBuf+pos, TX_BUF_SIZE-pos, "'ladder'");
                break;
            case RAND_FREQS:
                pos += snprintf(txBuf+pos, TX_BUF_SIZE-pos, "'chaos'");
                break;
            case RAND_KEYS:
                pos += snprintf(txBuf+pos, TX_BUF_SIZE-pos, "'board'");
                break;
            case RAND_KEYS_SCALE:
                pos += snprintf(txBuf+pos, TX_BUF_SIZE-pos, "'scale ");
                switch (scIdx) {
                case CHROMATIC:
                    pos += snprintf(txBuf+pos, TX_BUF_SIZE-pos, "%s'", STR_SCALE_CHROM);
                    break;
                case MAJOR:
                    pos += snprintf(txBuf+pos, TX_BUF_SIZE-pos, "%s'", STR_SCALE_MAJOR);
                    break;
                case MINOR_NATUR:
                    pos += snprintf(txBuf+pos, TX_BUF_SIZE-pos, "%s'", STR_SCALE_NAT_MINOR);
                    break;
                case MINOR_MELOD:
                    pos += snprintf(txBuf+pos, TX_BUF_SIZE-pos, "%s'", STR_SCALE_MEL_MINOR);
                    break;
                case MINOR_HARMON:
                    pos += snprintf(txBuf+pos, TX_BUF_SIZE-pos, "%s'", STR_SCALE_HAR_MINOR);
                    break;
                case PENT_MAJOR:
                    pos += snprintf(txBuf+pos, TX_BUF_SIZE-pos, "%s'", STR_SCALE_PENT_MAJOR);
                    break;
                case PENT_MINOR:
                    pos += snprintf(txBuf+pos, TX_BUF_SIZE-pos, "%s'", STR_SCALE_PENT_MINOR);
                    break;
                case PENT_JAPAN:
                    pos += snprintf(txBuf+pos, TX_BUF_SIZE-pos, "%s'", STR_SCALE_PENT_JAPAN);
                    break;
                case LYDIAN:
                    pos += snprintf(txBuf+pos, TX_BUF_SIZE-pos, "%s'", STR_SCALE_LYDIAN);
                    break;
                case MIXOLYDIAN:
                    pos += snprintf(txBuf+pos, TX_BUF_SIZE-pos, "%s'", STR_SCALE_MIXOLYDIAN);
                    break;
                case DORIAN:
                    pos += snprintf(txBuf+pos, TX_BUF_SIZE-pos, "%s'", STR_SCALE_DORIAN);
                    break;
                case PHRYGIAN:
                    pos += snprintf(txBuf+pos, TX_BUF_SIZE-pos, "%s'", STR_SCALE_PHRYGIAN);
                    break;
                case LOCRIAN:
                    pos += snprintf(txBuf+pos, TX_BUF_SIZE-pos, "%s'", STR_SCALE_LOCRIAN);
                    break;
                case BLUES:
                    pos += snprintf(txBuf+pos, TX_BUF_SIZE-pos, "%s'", STR_SCALE_BLUES);
                    break;
                case BLUES_7:
                    pos += snprintf(txBuf+pos, TX_BUF_SIZE-pos, "%s'", STR_SCALE_BLUES_7);
                    break;
                case BLUES_9:
                    pos += snprintf(txBuf+pos, TX_BUF_SIZE-pos, "%s'", STR_SCALE_BLUES_9);
                    break;
                default:
                    pos += snprintf(txBuf+pos, TX_BUF_SIZE-pos, "unknown'");
                    break;
                }
                pos += snprintf(txBuf+pos, TX_BUF_SIZE-pos, " based on %.3f Hz", scale[0]);
                break;
            default:
                pos += snprintf(txBuf+pos, TX_BUF_SIZE-pos, "'unknown'");
                break;
            }
            pos += snprintf(txBuf+pos, TX_BUF_SIZE-pos, ", beat = %ld ms", beatDurMS);
            break;
        case STREAM:
            pos += snprintf(txBuf+pos, TX_BUF_SIZE-pos, "streaming 12-bit stereo 48 kHz (1152 kbps)");
            break;
        default:
            pos += snprintf(txBuf+pos, TX_BUF_SIZE-pos, "undefined");
            break;
        }
        // Send status message
        SendMessage(txBuf, pos);
        res = TRUE;
    } else if (0 == strncmp(cmd, CMD_STOP, sizeof CMD_STOP)) {      // Compare incl. 0
        // > stop
        regime = PAUSING;
        ChangeFreq(0.0f);   // Zero signal
        FireOneShot(5);     // 5 ms transition
        res = TRUE;
    } else if (0 == strncmp(cmd, CMD_FIRE, sizeof CMD_FIRE - 1)) {
        // > fire f 500
        // > fire С+5 100
        // > fire С-3 100
        // > fire 314.2 333
        // > fire noise red 666
        pos = sizeof CMD_FIRE - 1;  // Position after the first space 'fire '
        char tone = cmd[pos];
        char lcTone = tone | 0x20;
        float32_t newF;
        int durMS;
        if (lcTone >= 'a' && lcTone <= 'g') {
            // > fire f 500
            // > fire С+5 100
            // > fire С-3 100
            // Set main frequency
            newF = BASE_TONE(0x20 ^ lcTone);
            if (0 != (tone & 0x20)) newF /= HALF_TONE;
            if (cmd[pos+1] == ' ') {
                // Parse duration only
                int nArg = sscanf(cmd+pos+2, "%d", &durMS);
                if (nArg != 1) return FALSE;
            } else {
                // Parse octave shift and duration
                int oct = 0;
                int nArg = sscanf(cmd+pos+1, "%d %d", &oct, &durMS);
                if (2 != nArg) return FALSE;
                // Check octave ranges
                if (oct < -4 || oct > +5) {
                    SendMessage(MSG_INVD_TONE, sizeof MSG_INVD_TONE - 1);
                    return FALSE;
                }
                // Shift frequency by several octaves
                if (oct < 0)
                    newF /= (float32_t) (1 << (-oct));
                else
                    newF *= (float32_t) (1 << oct);
            }
        } else if (tone >= '1' && tone <= '9') {
            // > fire 314.2 333
            int nArg = sscanf(cmd+pos, "%f %d", &newF, &durMS);
            if (nArg != 2) return FALSE;
            // Check frequency ranges
            if (newF < 1e-3f || newF > 3e4f) {
                SendMessage(MSG_INVD_FREQ, sizeof MSG_INVD_FREQ - 1);
                return FALSE;
            }
        } else if (0 == strncmp(cmd+pos, STR_NOISE, sizeof STR_NOISE - 1)) {
            // > fire noise red 666
            pos += sizeof STR_NOISE - 1;    // Position after 'fire noise '
            if (0 == strncmp(cmd+pos, STR_NOISE_WHITE, sizeof STR_NOISE_WHITE - 1)) {
                pos += sizeof STR_NOISE_WHITE - 1;  // Position after 'fire noise white '
                noiseType = NOISE_WHITE;
                noiseDiv = 1.5f;
            } else if (0 == strncmp(cmd+pos, STR_NOISE_PINK, sizeof STR_NOISE_PINK - 1)) {
                pos += sizeof STR_NOISE_PINK - 1;   // Position after 'fire noise pink '
                noiseType = NOISE_PINK;
                noiseDiv = 0.3f;
            } else if (0 == strncmp(cmd+pos, STR_NOISE_RED, sizeof STR_NOISE_RED - 1)) {
                pos += sizeof STR_NOISE_RED - 1;    // Position after 'fire noise red '
                noiseType = NOISE_RED;
                iirOut[0] = 0.0f;   // Reset white noise quadrature
                noiseDiv = 2.0f;
            } else if (0 == strncmp(cmd+pos, STR_NOISE_BLUE, sizeof STR_NOISE_BLUE - 1)) {
                pos += sizeof STR_NOISE_BLUE - 1;   // Position after 'fire noise blue '
                noiseType = NOISE_BLUE;
                noiseDiv = 0.05f;
            } else if (0 == strncmp(cmd+pos, STR_NOISE_VIOLET, sizeof STR_NOISE_VIOLET - 1)) {
                pos += sizeof STR_NOISE_VIOLET - 1; // Position after 'fire noise violet '
                noiseType = NOISE_VIOLET;
                noiseDiv = 1.0f;
            } else if (0 == strncmp(cmd+pos, STR_NOISE_UV, sizeof STR_NOISE_UV - 1)) {
                pos += sizeof STR_NOISE_UV - 1;     // Position after 'fire noise uv '
                noiseType = NOISE_UV;
                noiseDiv = 0.1f;
            } else {
                // Invalid noise type
                SendMessage(MSG_INVD_NOISE, sizeof MSG_INVD_NOISE - 1);
                return FALSE;
            }
            int nArg = sscanf(cmd+pos, " %d", &durMS);
            if (1 != nArg) return FALSE;
            // Reset noise generation
            isNoise = TRUE;
            newF = 0;
            iirPos = 1;
            memset(iirOut, 0, sizeof(float32_t)*FILT_DEPTH);
        } else {
            // Failed to parse tone
            SendMessage(MSG_INVD_TONE, sizeof MSG_INVD_TONE - 1);
            return FALSE;
        }
        // Check duration range
        if (durMS < 1 || durMS > (1 << 16)) {
            SendMessage(MSG_INVD_BEAT, sizeof MSG_INVD_BEAT - 1);
            return FALSE;
        }
        // Fire the tone
        regime = ONESHOT;
        ChangeFreq(newF);
        RestartBaseTIM();
        FireOneShot(durMS);
        res = TRUE;
    } else if (0 == strncmp(cmd, CMD_IMPROV, sizeof CMD_IMPROV - 1)) {
        // > impr ladder 500
        // > impr chaos 400
        // > impr scale f/maj 250
        // > impr scale A-1/hmin 250
        // > impr board 333
        pos = sizeof CMD_IMPROV - 1;
        if (0 == strncmp(cmd+pos, STR_IMPR_LADDER, sizeof STR_IMPR_LADDER - 1)) {
            // > impr ladder 500
            improvMode = LADDER;
            ImproviseNext = &Next_Ladder;
            freq = Next_RandomKeys();
            isRising = (Entropy() % 2 == 0);
            pos += sizeof STR_IMPR_LADDER - 1;
        } else if (0 == strncmp(cmd+pos, STR_IMPR_CHAOS, sizeof STR_IMPR_CHAOS - 1)) {
            // > impr chaos 400
            improvMode = RAND_FREQS;
            ImproviseNext = &Next_RandomFreqs;
            pos += sizeof STR_IMPR_CHAOS - 1;
        } else if (0 == strncmp(cmd+pos, STR_IMPR_RKEYS, sizeof STR_IMPR_RKEYS - 1)) {
            // > impr board 333
            improvMode = RAND_KEYS;
            ImproviseNext = &Next_RandomKeys;
            pos += sizeof STR_IMPR_RKEYS - 1;
        } else if (0 == strncmp(cmd+pos, STR_IMPR_SCALE, sizeof STR_IMPR_SCALE - 1)) {
            // > impr scale f/maj 250
            // > impr scale A-1/hmin 250
            improvMode = RAND_KEYS_SCALE;
            ImproviseNext = &Next_RandomKeys_Scale;
            pos += sizeof STR_IMPR_SCALE - 1;
            if (' ' != cmd[pos]) return FALSE;  // Ensure space after 'scale'
            pos++;
            // Find '/' delimiter
            const char* delim = (char*) memchr(cmd+pos, '/', cmdLen-pos);
            if (delim == NULL) return FALSE;
            uint32_t posDelim = delim - cmd;    // Index of '/' in 'cmd'
            // Extract base tone
            float32_t baseTone = BASE_TONE('F');
            res = ParseTone(cmd+pos, posDelim-pos, &baseTone);
            if (!res) return FALSE;
            // Match scale name
            pos = posDelim + 1;
            if (0 == strncmp(cmd+pos, STR_SCALE_CHROM, sizeof STR_SCALE_CHROM - 1)) {
                SetScale(CHROMATIC, baseTone);
                pos += sizeof STR_SCALE_CHROM - 1;
            } else if (0 == strncmp(cmd+pos, STR_SCALE_MAJOR, sizeof STR_SCALE_MAJOR - 1)) {
                SetScale(MAJOR, baseTone);
                pos += sizeof STR_SCALE_MAJOR - 1;
            } else if (0 == strncmp(cmd+pos, STR_SCALE_NAT_MINOR, sizeof STR_SCALE_NAT_MINOR - 1)) {
                SetScale(MINOR_NATUR, baseTone);
                pos += sizeof STR_SCALE_NAT_MINOR - 1;
            } else if (0 == strncmp(cmd+pos, STR_SCALE_MEL_MINOR, sizeof STR_SCALE_MEL_MINOR - 1)) {
                SetScale(MINOR_MELOD, baseTone);
                pos += sizeof STR_SCALE_MEL_MINOR - 1;
            } else if (0 == strncmp(cmd+pos, STR_SCALE_HAR_MINOR, sizeof STR_SCALE_HAR_MINOR - 1)) {
                SetScale(MINOR_HARMON, baseTone);
                pos += sizeof STR_SCALE_HAR_MINOR - 1;
            } else if (0 == strncmp(cmd+pos, STR_SCALE_PENT_MAJOR, sizeof STR_SCALE_PENT_MAJOR - 1)) {
                SetScale(PENT_MAJOR, baseTone);
                pos += sizeof STR_SCALE_PENT_MAJOR - 1;
            } else if (0 == strncmp(cmd+pos, STR_SCALE_PENT_MINOR, sizeof STR_SCALE_PENT_MINOR - 1)) {
                SetScale(PENT_MINOR, baseTone);
                pos += sizeof STR_SCALE_PENT_MINOR - 1;
            } else if (0 == strncmp(cmd+pos, STR_SCALE_PENT_JAPAN, sizeof STR_SCALE_PENT_JAPAN - 1)) {
                SetScale(PENT_JAPAN, baseTone);
                pos += sizeof STR_SCALE_PENT_JAPAN - 1;
            } else if (0 == strncmp(cmd+pos, STR_SCALE_LYDIAN, sizeof STR_SCALE_LYDIAN - 1)) {
                SetScale(LYDIAN, baseTone);
                pos += sizeof STR_SCALE_LYDIAN - 1;
            } else if (0 == strncmp(cmd+pos, STR_SCALE_MIXOLYDIAN, sizeof STR_SCALE_MIXOLYDIAN - 1)) {
                SetScale(MIXOLYDIAN, baseTone);
                pos += sizeof STR_SCALE_MIXOLYDIAN - 1;
            } else if (0 == strncmp(cmd+pos, STR_SCALE_DORIAN, sizeof STR_SCALE_DORIAN - 1)) {
                SetScale(DORIAN, baseTone);
                pos += sizeof STR_SCALE_DORIAN - 1;
            } else if (0 == strncmp(cmd+pos, STR_SCALE_PHRYGIAN, sizeof STR_SCALE_PHRYGIAN - 1)) {
                SetScale(PHRYGIAN, baseTone);
                pos += sizeof STR_SCALE_PHRYGIAN - 1;
            } else if (0 == strncmp(cmd+pos, STR_SCALE_LOCRIAN, sizeof STR_SCALE_LOCRIAN - 1)) {
                SetScale(LOCRIAN, baseTone);
                pos += sizeof STR_SCALE_LOCRIAN - 1;
            } else if (0 == strncmp(cmd+pos, STR_SCALE_BLUES_9, sizeof STR_SCALE_BLUES_9 - 1)) {
                SetScale(BLUES_9, baseTone);
                pos += sizeof STR_SCALE_BLUES_9 - 1;
            } else if (0 == strncmp(cmd+pos, STR_SCALE_BLUES_7, sizeof STR_SCALE_BLUES_7 - 1)) {
                SetScale(BLUES_7, baseTone);
                pos += sizeof STR_SCALE_BLUES_7 - 1;
            } else if (0 == strncmp(cmd+pos, STR_SCALE_BLUES, sizeof STR_SCALE_BLUES - 1)) {
                SetScale(BLUES, baseTone);
                pos += sizeof STR_SCALE_BLUES - 1;
            } else {
                SendMessage(MSG_INVD_SCAL, sizeof MSG_INVD_SCAL - 1);
                return FALSE;
            }
        } else {
            SendMessage(MSG_INVD_IMPR, sizeof MSG_INVD_IMPR - 1);
            return FALSE;
        }
        // Ensure space before beat duration and parse it
        if (0 != cmd[pos]) {
            int beatMS;
            int nArg = sscanf(cmd+pos, " %d", &beatMS);
            if (nArg != 1 || beatMS < 1 || beatMS > (1 << 16)) {
                SendMessage(MSG_INVD_BEAT, sizeof MSG_INVD_BEAT - 1);
                return FALSE;
            }
            beatDurMS = beatMS;
        }
        // Enter improvisation mode
        regime = IMPROV;
        RestartBaseTIM();
        LL_TIM_GenerateEvent_UPDATE(TIM3);  // Start improvisation
        res = TRUE;
    } else if (0 == strncmp(cmd, CMD_PLAY, sizeof CMD_PLAY - 1)) {
        // Fill RX buffer with signed mid codes (with zeros)
        memset(rxBuf, 0x00, RX_BUF_SIZE);
        /* DAC starts playing codes from the upper half of the buffer
            while UART starts writing samples to the lower half */
        sPos = RX_BUF_HALF;
        // Enter streaming mode
        LL_TIM_EnableCounter(TIM2); // Ensure that base TIM2 is running
        regime = STREAM;
        res = TRUE;
    } else if (0 == strncmp(cmd, CMD_SET_WAVE, sizeof CMD_SET_WAVE - 1)) {
        // > set wave sin3 0.314
        pos = sizeof CMD_SET_WAVE - 1;  // Position after 'set wave '
        enum WAVE wvTyp;
        if (0 == strncmp(cmd+pos, STR_WAVE_SIN5, sizeof STR_WAVE_SIN5 - 1)) {
            wvTyp = WAVE_SIN5;
            pos += sizeof STR_WAVE_SIN5 - 1;
        } else if (0 == strncmp(cmd+pos, STR_WAVE_SIN3, sizeof STR_WAVE_SIN3 - 1)) {
            wvTyp = WAVE_SIN3;
            pos += sizeof STR_WAVE_SIN3 - 1;
        } else if (0 == strncmp(cmd+pos, STR_WAVE_SIN, sizeof STR_WAVE_SIN - 1)) {
            wvTyp = WAVE_SIN;
            pos += sizeof STR_WAVE_SIN - 1;
        } else if (0 == strncmp(cmd+pos, STR_WAVE_RECT, sizeof STR_WAVE_RECT - 1)) {
            wvTyp = WAVE_RECT;
            pos += sizeof STR_WAVE_RECT - 1;
        } else if (0 == strncmp(cmd+pos, STR_WAVE_SAWTH, sizeof STR_WAVE_SAWTH - 1)) {
            wvTyp = WAVE_SAWTOOTH;
            pos += sizeof STR_WAVE_SAWTH - 1;
        } else if (0 == strncmp(cmd+pos, STR_WAVE_TRIANG, sizeof STR_WAVE_TRIANG - 1)) {
            wvTyp = WAVE_TRIANGULAR;
            pos += sizeof STR_WAVE_TRIANG - 1;
        } else {
            SendMessage(MSG_INVD_WAVE, sizeof MSG_INVD_WAVE - 1);
            return FALSE;
        }
        // Parse wave amplitude (if present)
        float32_t newAmp;
        bool locRes = TRUE;
        int nArg = sscanf(cmd+pos, " %f", &newAmp);
        if (1 == nArg) {
            if (newAmp > 0) amp = newAmp;
            else locRes = FALSE;
        } else if (0 != cmd[pos]) {
            locRes = FALSE;
        }
        if (!locRes) {
            SendMessage(MSG_INVD_AMPL, sizeof MSG_INVD_AMPL - 1);
            return FALSE;
        }
        waveType = wvTyp;   // Confirm new wave type
        SendMessage(MSG_WAVE_DEF, sizeof MSG_WAVE_DEF - 1);
        res = TRUE;
    } else if (0 == strncmp(cmd, CMD_GET_WAVE, sizeof CMD_GET_WAVE)) {  // Compare incl. 0
        // > get wave
        pos = snprintf(txBuf, TX_BUF_SIZE, "Wave type: ");
        switch (waveType) {
        case WAVE_SIN:
            pos += snprintf(txBuf+pos, TX_BUF_SIZE-pos, "sine (ARM approx.)");
            break;
        case WAVE_SIN3:
            pos += snprintf(txBuf+pos, TX_BUF_SIZE-pos, "3rd power of sine");
            break;
        case WAVE_SIN5:
            pos += snprintf(txBuf+pos, TX_BUF_SIZE-pos, "5th power of sine");
            break;
        case WAVE_RECT:
            pos += snprintf(txBuf+pos, TX_BUF_SIZE-pos, "rectangular");
            break;
        case WAVE_SAWTOOTH:
            pos += snprintf(txBuf+pos, TX_BUF_SIZE-pos, "sawtooth");
            break;
        case WAVE_TRIANGULAR:
            pos += snprintf(txBuf+pos, TX_BUF_SIZE-pos, "triangular");
            break;
        default:
            pos += snprintf(txBuf+pos, TX_BUF_SIZE-pos, "invalid");
            break;
        }
        // Append wave amplitude
        pos += snprintf(txBuf+pos, TX_BUF_SIZE-pos, ", amplitude: %.3f", amp);
        SendMessage(txBuf, pos);
        res = TRUE;
    } else if (0 == strncmp(cmd, CMD_SET_RAND, sizeof CMD_SET_RAND - 1)) {
        // > set entropy RNG
        // > set entropy ADC
        if (sizeof CMD_SET_RAND - 1 + 3 != cmdLen) return FALSE;
        pos = sizeof CMD_SET_RAND - 1;
        if (0 == strncmp(cmd+pos, "RNG", 3)) {
            DefineEntropySource(HARD_RNG);
            SendMessage(MSG_SET_RNG, sizeof MSG_SET_RNG - 1);
            res = TRUE;
        } else if (0 == strncmp(cmd+pos, "ADC", 3)) {
            DefineEntropySource(ADC_FLOAT);
            SendMessage(MSG_SET_ADC, sizeof MSG_SET_ADC - 1);
            res = TRUE;
        } else {
            SendMessage(MSG_INVD_RND, sizeof MSG_INVD_RND - 1);
            res = FALSE;
        }
    } else if (0 == strncmp(cmd, CMD_GET_RAND, sizeof CMD_GET_RAND)) {  // Compare incl. 0
        // > get entropy
        switch (rndSource) {
        case HARD_RNG:
            SendMessage(MSG_SET_RNG, sizeof MSG_SET_RNG - 1);
            break;
        case ADC_FLOAT:
            SendMessage(MSG_SET_ADC, sizeof MSG_SET_ADC - 1);
            break;
        default:
            SendMessage(MSG_INVD_RND, sizeof MSG_INVD_RND - 1);
            break;
        }
        res = TRUE;
    } else if (0 == strncmp(cmd, CMD_SET_FILT, sizeof CMD_SET_FILT - 1)) {
        // > set filter on
        // > set filter off
        // > set filter 1.0 -4.0 0.0 3.14 1e-5 / 1
        // > set filter 1.0 0.0 -3.0 / -2 0 2.71
        pos = sizeof CMD_SET_FILT - 1;
        if (0 == strncmp(cmd+pos, "on", 3)) {
            doFilter = TRUE;
            FillRandom(iirFastOut, FILT_DEPTH);
            SendMessage(MSG_SET_FILT, sizeof MSG_SET_FILT - 2);     // Without semicolon
            res = TRUE;
        } else if (0 == strncmp(cmd+pos, "off", 4)) {
            doFilter = FALSE;
            FillRandom(iirFastOut, FILT_DEPTH);
            SendMessage(MSG_SET_NOFILT, sizeof MSG_SET_NOFILT - 2); // Without semicolon
            res = TRUE;
        } else {
            pos--;  // Position of the first space after 'set filter'
            const char* delim = memchr(cmd+pos, '/', cmdLen-pos);   // Find the next '/' delimiter'
            if (NULL == delim) {
                SendMessage(MSG_INVD_FILT, sizeof MSG_INVD_FILT - 1);
                return FALSE;
            }
            uint32_t divSep = delim - cmd + 1;  // Position after the '/' delimiter
            float32_t cfs_a[FILT_DEPTH+1];
            float32_t cfs_b[FILT_DEPTH+1];
            memset(cfs_a, 0, sizeof(cfs_a));    // Reset coeffs with 0.0f
            memset(cfs_b, 0, sizeof(cfs_b));    // Reset coeffs with 0.0f
            uint32_t nc = 0;    // Count of coefficients
            uint32_t nArg;
            // Split command line by '/' delimiter
            nArg = divSep-2;
            while (' ' == cmd[nArg]) {
                cmd[nArg] = 0;
                if (0 == nArg) break;
                nArg--;
            }
            // Parse numerator coefficients
            while (pos < divSep) {
                const char* delim = memchr(cmd+pos, ' ', cmdLen-pos);   // Find the next space
                if (NULL == delim) break;
                pos = delim - cmd + 1;  // Position after the next space
                if (pos >= divSep) break;
                if (FILT_DEPTH+1 == nc) {
                    uint32_t pos1 = snprintf(txBuf, TX_BUF_SIZE, "Filter numerator too long, truncated to %d coeffs. ", FILT_DEPTH);
                    SendMessage(txBuf, pos1);
                    break;
                }
                nArg = sscanf(cmd+pos, "%f", cfs_b+nc); // Parse the next numerator coefficient
                if (1 != nArg) {    // Failed to parse the next coefficient
                    SendMessage(MSG_INVD_COEF, sizeof MSG_INVD_COEF - 1);
                    return FALSE;
                }
                nc++;   // Next coefficient
            }
            // Parse denominator coefficients
            nc = 0;
            pos = divSep;
            while (pos < cmdLen) {
                const char* delim = memchr(cmd+pos, ' ', cmdLen-pos);   // Find the next space
                if (NULL == delim) break;
                pos = delim - cmd + 1;  // Position after the next space
                if (FILT_DEPTH+1 == nc) {
                    uint32_t pos1 = snprintf(txBuf, TX_BUF_SIZE, "Filter denominator too long, truncated to %d coeffs. ", FILT_DEPTH);
                    SendMessage(txBuf, pos1);
                    break;
                }
                nArg = sscanf(cmd+pos, "%f", cfs_a+nc); // Parse the next denominator coefficient
                if (1 != nArg) {    // Failed to parse the next coefficient
                    SendMessage(MSG_INVD_COEF, sizeof MSG_INVD_COEF - 1);
                    return FALSE;
                }
                nc++;   // Next coefficient
            }
            // Confirm new filter coefficients
            DefineFilter(cfs_a, cfs_b);
            FillRandom(iirFastOut, FILT_DEPTH);
            SendMessage(MSG_FILT_DEF, sizeof MSG_FILT_DEF - 1);
            res = TRUE;
        }
    } else if (0 == strncmp(cmd, CMD_GET_FILT, sizeof CMD_GET_FILT)) {  // Compare incl. 0
        // > get filter
        if (doFilter) {
            memcpy(txBuf, MSG_SET_FILT, sizeof MSG_SET_FILT-1);
            pos = sizeof MSG_SET_FILT - 1;
        } else {
            memcpy(txBuf, MSG_SET_NOFILT, sizeof MSG_SET_NOFILT-1);
            pos = sizeof MSG_SET_NOFILT - 1;
        }
        for (uint32_t i = 0; i <= FILT_DEPTH; i++)
            pos += snprintf(txBuf+pos, TX_BUF_SIZE-pos, " %.6f", IIR_B[i]);
        txBuf[pos+0] = ' ';
        txBuf[pos+1] = '/';
        pos += 2;
        for (uint32_t i = 0; i <= FILT_DEPTH; i++)
            pos += snprintf(txBuf+pos, TX_BUF_SIZE-pos, " %.6f", IIR_A[i]);
        SendMessage(txBuf, pos);
        res = TRUE;
    } else if (0 == strncmp(cmd, CMD_SET_BEAT, sizeof CMD_SET_BEAT - 1)) {
        // > set beat 250
        pos = sizeof CMD_SET_BEAT - 1;  // Position after the first space after 'set beat '
        int beatMS;
        uint32_t nArg = sscanf(cmd+pos, "%d", &beatMS);
        if (nArg != 1 || beatMS < 1) {
            SendMessage(MSG_INVD_BEAT, sizeof MSG_INVD_BEAT - 1);
            return FALSE;
        }
        beatDurMS = beatMS;
        SendMessage(MSG_BEAT_DEF, sizeof MSG_BEAT_DEF - 1);
        res = TRUE;
    } else if (0 == strncmp(cmd, CMD_GET_BEAT, sizeof CMD_GET_BEAT)) {  // Compare incl. 0
        // > get beat
        pos = snprintf(txBuf, TX_BUF_SIZE, "Beat = %ld ms", beatDurMS);
        SendMessage(txBuf, pos);
        res = TRUE;
    } else if (0 == strncmp(cmd, CMD_SET_PATT, sizeof CMD_SET_PATT - 1)) {
        // > set pattern mutability 0.314
        // > set pattern stable
        // > set pattern 1 1 1/2 1/2 3/7
        pos = sizeof CMD_SET_PATT - 1;  // Position after the space 'set pattern '
        if (0 == strncmp(cmd+pos, STR_PATT_STABLE, sizeof STR_PATT_STABLE)) {   // Compare incl. 0
            // > set pattern stable
            mutability = -1.0f;
            pos = snprintf(txBuf, TX_BUF_SIZE, "Pattern is stable");
            SendMessage(txBuf, pos);
            res = TRUE;
        } else if (0 == strncmp(cmd+pos, STR_PATT_MUTABILITY, sizeof STR_PATT_MUTABILITY - 1)) {
            // > set pattern mutability 0.314
            pos += sizeof STR_PATT_MUTABILITY - 1;  // Position after the last space
            float32_t mut;
            uint32_t nArg = sscanf(cmd+pos, "%f", &mut);
            if (nArg != 1) {
                SendMessage(MSG_INVD_MUTAB, sizeof MSG_INVD_MUTAB);
                return FALSE;
            }
            mutability = mut;
            pos = snprintf(txBuf, TX_BUF_SIZE, "Pattern mutability = %.2f", mutability);
            SendMessage(txBuf, pos);
            res = TRUE;
        } else if ('1' <= cmd[pos] && cmd[pos] <= '9') {
            // > set pattern 1 1 1/2 1/2 3/7
            pos--;  // Position of the first space in beat sequence
            bool locRes = TRUE;
            int parts[2];
            uint32_t nb = 0;    // Count of beats (prevent aggressive optimizations here)
            float32_t patt[MAX_PATTERN];// Buffer for beats
            while (pos < cmdLen) {
                const char* delim = memchr(cmd+pos, ' ', cmdLen-pos);   // Find the next space
                if (NULL == delim) break;
                pos = delim - cmd + 1;   // Position after the next space
                if (MAX_PATTERN == nb) {
                    uint32_t pos1 = snprintf(txBuf, TX_BUF_SIZE, "Pattern spec. too long, truncated to %d beats. ", MAX_PATTERN);
                    SendMessage(txBuf, pos1);
                    break;
                }
                uint32_t nArg = sscanf(cmd+pos, "%d/%d", parts+0, parts+1); // Parse the next beat
                if (0 >= nArg) {        // Failed to parse the next beat
                    locRes = FALSE;
                    break;
                } else if (1 == nArg) { // Integer beat
                    if (parts[0] < 1) {
                        locRes = FALSE;
                        break;
                    }
                    patt[nb] = (float32_t) parts[0];
                } else {                // Fractional beat
                    if (parts[0] < 1 || parts[1] < 1) {
                        locRes = FALSE;
                        break;
                    }
                    patt[nb] = parts[0] / (float32_t) parts[1];
                }
                // Next beat
                nb++;
            }
            // Confirm the new pattern if it is compliant
            if (locRes) {
                pattLen = nb;   // Update pattern length
                // Copy pattern and select mutation quantum
                float32_t minB = 1e9;
                float32_t maxB = -1e9;
                for (uint32_t i = 0; i < pattLen; i++) {
                    pattern[i] = patt[i];
                    if (patt[i] > maxB) maxB = patt[i];
                    if (patt[i] < minB) minB = patt[i];
                }
                arm_sqrt_f32(minB * maxB, &mutQuant);
                mutQuant = ceilf(mutQuant / minB) * minB / 2;
                SendMessage(MSG_PATT_DEF, sizeof MSG_PATT_DEF - 1);
            } else {
                SendMessage(MSG_INVD_PATT, sizeof MSG_INVD_PATT - 1);
            }
            res = locRes;
        }
    } else if (0 == strncmp(cmd, CMD_GET_PATT, sizeof CMD_GET_PATT)) {  // Compare incl. 0
        // > get pattern
        pos = snprintf(txBuf, TX_BUF_SIZE, "Beat pattern (1 = %ld ms): [", beatDurMS);
        for (uint32_t i = 0; i < pattLen; i++) {
            const float32_t dur = pattern[i];
            const float32_t inv = 1 / dur;
            if (fabsf(dur - (int) dur) < 1e-5f) {
                pos += snprintf(txBuf+pos, TX_BUF_SIZE-pos, "%d ", (int) dur);
            } else if (fabsf(inv - (int) inv) < 1e-5f) {
                pos += snprintf(txBuf+pos, TX_BUF_SIZE-pos, "1/%d ", (int) inv);
            } else {
                pos += snprintf(txBuf+pos, TX_BUF_SIZE-pos, "%.4f ", dur);
            }
        }
        pos--;  // Erase the last space
        if (mutability <= 0.0f) {
            pos += snprintf(txBuf+pos, TX_BUF_SIZE-pos, "], stable");
        } else {
            pos += snprintf(txBuf+pos, TX_BUF_SIZE-pos,
                "], mutability = %.2f, mutation quantum = %.4f", mutability, mutQuant);
        }
        SendMessage(txBuf, pos);
        res = TRUE;
    } else if (0 == strncmp(cmd, CMD_SET_EQ, sizeof CMD_SET_EQ - 1)) {
        // > set equalizer 100.0 800.0 365.0 440.0 3600.0
        pos = sizeof CMD_SET_EQ - 2;    // Position after 'set equalizer'
        uint32_t nc = 0;    // Count of frequencies
        uint32_t freqs[FILT_DEPTH+1];
        memset(freqs, 0, sizeof(freqs));    // Reset frequency indices with 0
        uint32_t nArg;
        while (pos < cmdLen) {
            const char* delim = memchr(cmd+pos, ' ', cmdLen-pos);   // Find the next space
            if (NULL == delim) break;
            pos = delim - cmd + 1;  // Position after the next space
            if (FILT_DEPTH+1 == nc) {
                uint32_t pos1 = snprintf(txBuf, TX_BUF_SIZE, "Equalizer spec. too long, truncated to %d freqs. ", FILT_DEPTH+1);
                SendMessage(txBuf, pos1);
                break;
            }
            float32_t freq;
            bool locRes = FALSE;
            nArg = sscanf(cmd+pos, "%f", &freq);    // Parse the next frequency
            if (1 == nArg) {    // Check the next frequency
                if (freq > 0.0f && freq <= BASE_FREQ) {
                    freq = BASE_FREQ / freq;
                    uint32_t fIdx = freq - 1U;
                    if (fIdx < FILT_MEM) {
                        freqs[nc] = fIdx;
                        locRes = TRUE;
                    }
                }
            } else {
                SendMessage(MSG_INVD_EQFR, sizeof MSG_INVD_EQFR - 1);
                return FALSE;
            }
            if (!locRes) {
                SendMessage(MSG_EQ_OUTRANGE, sizeof MSG_EQ_OUTRANGE - 1);
                return FALSE;
            }
            nc++;   // Next frequency
        }
        // Confirm the new equalizer frequencies
        memcpy(iirPsc, freqs, (FILT_DEPTH+1) * sizeof(uint32_t));
        SendMessage(MSG_EQ_DEF, sizeof MSG_EQ_DEF - 1);
        res = TRUE;
    } else if (0 == strncmp(cmd, CMD_GET_EQ, sizeof CMD_GET_EQ)) {      // Compare incl. 0
        // > get equalizer
        pos = snprintf(txBuf, TX_BUF_SIZE, "Equalizer");
        if (doFilter)
            pos += snprintf(txBuf+pos, TX_BUF_SIZE-pos, " ON:");
        else
            pos += snprintf(txBuf+pos, TX_BUF_SIZE-pos, " OFF:");
        for (uint32_t i = 0; i <= FILT_DEPTH; i++) {
            float32_t freq = BASE_FREQ / (1.0f + iirPsc[i]);
            pos += snprintf(txBuf+pos, TX_BUF_SIZE-pos, " %.1f", freq);
        }
        SendMessage(txBuf, pos);
        res = TRUE;
    }

    return res;
}

// === === === Initialization routines === === ===
void SetupHSE(void) {   // Switch to HSE clock
    // Increase system voltage scale to the VOS1
    PWR->CR3 |= PWR_CR3_LDOEN;  // Ensure that LDO voltage regulator is enabled
    uint32_t d3cr = PWR->D3CR;
    d3cr &= ~PWR_D3CR_VOS;
    d3cr |= (3 << PWR_D3CR_VOS_Pos);    // Set VOS1 voltage scale
    PWR->D3CR = d3cr;
    while (0 == (PWR->D3CR & PWR_D3CR_VOSRDY));

    // Start HSE clock
    LL_RCC_HSE_Enable();
    while (!LL_RCC_HSE_IsReady());  // Wait until HSE becomes ready

    // Configure PLL so that SYSCLK and ABx, APBx frequency were 96 MHz and all other frequencies were 48 MHz (including RNG)
    // 0. HSE @ 25 MHz = REF_CK
    // 1. REF_CK / PLLM = VCO_IN = 1 MHz (thus PLLM = 25)           Must be: 1 MHz <= VCO_IN <= 2 MHz
    // 2. VCO_IN * PLLN = VCO_OUT = 384 MHz (thus PLLN = 384)       Must be: 192 MHz <= VCO_OUT <= 836 MHz
    // 3. VCO_OUT / PLLP = SYS_CK = 96 MHz (thus PLLP = 4)          Must be: PLL_OUT <= 180 MHz
    // 4. VCO_OUT / PLLQ = USB_RNG = 48 MHz (thus PLLQ = 8)         Must be: USB_RNG = 48 MHz
    uint32_t pllM = 25 << RCC_PLLCKSELR_DIVM1_Pos;  // This factor is specific for PLL1
    uint32_t pllN = (384-1) << RCC_PLL1DIVR_N1_Pos; // Multiplier for VCO
    uint32_t pllP = (4-1) << RCC_PLL1DIVR_P1_Pos;   // Divider for SYS_CK
    uint32_t pllQ = (8-1) << RCC_PLL1DIVR_Q1_Pos;   // Divider for RNG
    LL_RCC_PLL1_Disable();
    while (LL_RCC_PLL1_IsReady());
    RCC->PLLCKSELR &= ~RCC_PLLCKSELR_DIVM1_Msk;
    RCC->PLLCKSELR |= RCC_PLLCKSELR_PLLSRC_HSE | pllM;  // Use HSE as input for PLL
    RCC->PLLCFGR &= ~RCC_PLLCFGR_PLL1RGE;   // Select PLL1 input (VCO_IN) between 1 and 2 MHz
    RCC->PLLCFGR |= RCC_PLLCFGR_PLL1VCOSEL; // Select low-frequency VCO (VCO_IN is 1 to 2 MHz)
    RCC->PLLCFGR &= ~RCC_PLLCFGR_PLL1FRACEN;// Disable sigma-delta modulator (fractional mode)
    RCC->PLLCFGR |= RCC_PLLCFGR_DIVP1EN;    // Enable P divider
    RCC->PLLCFGR |= RCC_PLLCFGR_DIVQ1EN;    // Enable Q divider
    RCC->PLLCFGR &=~RCC_PLLCFGR_DIVR1EN;    // Disable R divider
    RCC->PLL1DIVR = pllN | pllP | pllQ;     // Configure PLL dividers and multipliers

    // Start the PLL1
    LL_RCC_PLL1_Enable();
    while (!LL_RCC_PLL1_IsReady()); // Wait until PLL1 becomes ready

    // Configure bus prescalers: CPU 96 MHz, AHBx 96 MHz, APBx 96 MHz
    LL_RCC_SetSysPrescaler(LL_RCC_SYSCLK_DIV_1);    // Divide SYS_CLK by 1 to obtain 96 MHz CPU frequency
    LL_RCC_SetAHBPrescaler(LL_RCC_AHB_DIV_1);       // AHB freq must be <= 240 MHz
    LL_RCC_SetAPB1Prescaler(LL_RCC_APB1_DIV_1);     // APB1 freq must be <= 120 MHz
    LL_RCC_SetAPB2Prescaler(LL_RCC_APB2_DIV_1);     // APB2 freq must be <= 120 MHz
    LL_RCC_SetAPB3Prescaler(LL_RCC_APB3_DIV_1);     // APB3 freq must be <= 120 MHz
    LL_RCC_SetAPB4Prescaler(LL_RCC_APB4_DIV_1);     // APB4 freq must be <= 120 MHz
    LL_RCC_SetTIMPrescaler(LL_RCC_TIM_PRESCALER_FOUR_TIMES);    // This makes TIM clocks run AHB frequency
    LL_RCC_SetRNGClockSource(LL_RCC_RNG_CLKSOURCE_PLL1Q);
    LL_RCC_SetUSARTClockSource(LL_RCC_USART16_CLKSOURCE_PCLK2);
    LL_RCC_SetSysClkSource(LL_RCC_SYS_CLKSOURCE_PLL1);

    // Decrease the count of FLASH wait states (to accomodate its latency)
    // 1 WS for 70 < HCLK < 140 MHz, 2 WS for 140 < HCLK < 210 MHz (VOS1 voltage range)
    // 1 WS for 45 < HCLK <  90 MHz, 2 WS for  90 < HCLK < 135 MHz (default VOS3 voltage range)
    const uint32_t FLATENCY = FLASH_ACR_LATENCY_1WS;
    uint32_t flACR = FLASH->ACR;
    flACR &= ~FLASH_ACR_LATENCY_Msk;
    flACR |= FLATENCY;
    FLASH->ACR = flACR;
    if ((FLASH->ACR & FLASH_ACR_LATENCY_Msk) != FLATENCY) while (TRUE);
}
void SetupDAC(void) {       // Configure DAC1.OUT1 on PA4, DAC1.OUT2 on PA5 analog pins and OPAMP1.OUT on PC4, OPAMP2.OUT on PE7
    // Setup GPIO PA4 and PA5 analog pins
    LL_AHB4_GRP1_EnableClock(LL_AHB4_GRP1_PERIPH_GPIOA);
    LL_GPIO_SetPinMode(GPIOA, LL_GPIO_PIN_4, LL_GPIO_MODE_ANALOG);
    LL_GPIO_SetPinMode(GPIOA, LL_GPIO_PIN_5, LL_GPIO_MODE_ANALOG);

    // Configure DAC channels 1 and 2 to be triggered by software
    LL_APB1_GRP1_EnableClock(LL_APB1_GRP1_PERIPH_DAC12);
    LL_DAC_SetOutputMode(DAC1, LL_DAC_CHANNEL_1, LL_DAC_OUTPUT_MODE_NORMAL);
    LL_DAC_SetOutputMode(DAC1, LL_DAC_CHANNEL_2, LL_DAC_OUTPUT_MODE_NORMAL);
    // LL_DAC_SetOutputConnection(DAC1, LL_DAC_CHANNEL_1, LL_DAC_OUTPUT_CONNECT_INTERNAL);
    // LL_DAC_SetOutputConnection(DAC1, LL_DAC_CHANNEL_2, LL_DAC_OUTPUT_CONNECT_INTERNAL);
    LL_DAC_SetOutputConnection(DAC1, LL_DAC_CHANNEL_1, LL_DAC_OUTPUT_CONNECT_GPIO);
    LL_DAC_SetOutputConnection(DAC1, LL_DAC_CHANNEL_2, LL_DAC_OUTPUT_CONNECT_GPIO);
    LL_DAC_SetOutputBuffer(DAC1, LL_DAC_CHANNEL_1, LL_DAC_OUTPUT_BUFFER_ENABLE);
    LL_DAC_SetOutputBuffer(DAC1, LL_DAC_CHANNEL_2, LL_DAC_OUTPUT_BUFFER_ENABLE);
    LL_DAC_SetTriggerSource(DAC1, LL_DAC_CHANNEL_1, LL_DAC_TRIG_SOFTWARE);
    LL_DAC_SetTriggerSource(DAC1, LL_DAC_CHANNEL_2, LL_DAC_TRIG_SOFTWARE);
    LL_DAC_EnableTrigger(DAC1, LL_DAC_CHANNEL_1);
    LL_DAC_EnableTrigger(DAC1, LL_DAC_CHANNEL_2);

    // // Setup GPIO PC4 and PE7 in analog mode
    // LL_AHB4_GRP1_EnableClock(LL_AHB4_GRP1_PERIPH_GPIOC);
    // LL_AHB4_GRP1_EnableClock(LL_AHB4_GRP1_PERIPH_GPIOE);
    // LL_GPIO_SetPinMode(GPIOC, LL_GPIO_PIN_4, LL_GPIO_MODE_ANALOG);
    // LL_GPIO_SetPinMode(GPIOE, LL_GPIO_PIN_7, LL_GPIO_MODE_ANALOG);

    // // Setup OPAMP1 and OPAMP2 to amplify DAC1 channels
    // LL_APB1_GRP2_EnableClock(LL_APB1_GRP2_PERIPH_OPAMP);
    // LL_OPAMP_SetMode(OPAMP1, LL_OPAMP_MODE_PGA);
    // LL_OPAMP_SetMode(OPAMP2, LL_OPAMP_MODE_PGA);
    // LL_OPAMP_SetInputInverting(OPAMP1, LL_OPAMP_INPUT_INVERT_CONNECT_NO);
    // LL_OPAMP_SetInputInverting(OPAMP2, LL_OPAMP_INPUT_INVERT_CONNECT_NO);
    // LL_OPAMP_SetInputNonInverting(OPAMP1, LL_OPAMP_INPUT_NONINVERT_DAC);
    // LL_OPAMP_SetInputNonInverting(OPAMP2, LL_OPAMP_INPUT_NONINVERT_DAC);
    // LL_OPAMP_SetPGAGain(OPAMP1, LL_OPAMP_PGA_GAIN_4_OR_MINUS_3);
    // LL_OPAMP_SetPGAGain(OPAMP2, LL_OPAMP_PGA_GAIN_4_OR_MINUS_3);
    // LL_OPAMP_Enable(OPAMP1);
    // LL_OPAMP_Enable(OPAMP2);

    // Reset DAC buffers and turn DAC on
    LL_DAC_ConvertDualData12RightAligned(DAC1, 0x000, 0x000);
    LL_DAC_Enable(DAC1, LL_DAC_CHANNEL_1);
    LL_DAC_Enable(DAC1, LL_DAC_CHANNEL_2);
}
void SetupRandoms(void) {   // Configure RNG and ADC3.INP0 to measure floating analog PC2 pin
    // Configure GPIO analog PC0 pin
    LL_AHB4_GRP1_EnableClock(LL_AHB4_GRP1_PERIPH_GPIOC);
    LL_GPIO_SetPinMode(GPIOC, LL_GPIO_PIN_2, LL_GPIO_MODE_ANALOG);

    // Prepare ADC to collect the noise from analog pin
    LL_AHB4_GRP1_EnableClock(LL_AHB4_GRP1_PERIPH_ADC3);
    LL_ADC_SetChannelPreselection(ADC3, LL_ADC_CHANNEL_0);
    LL_ADC_SetResolution(ADC3, LL_ADC_RESOLUTION_8B);
    // LL_ADC_SetDataAlignment(ADC3, LL_ADC_DATA_ALIGN_RIGHT);
    LL_ADC_REG_SetSequencerLength(ADC3, LL_ADC_REG_SEQ_SCAN_DISABLE);
    LL_ADC_REG_SetSequencerRanks(ADC3, LL_ADC_REG_RANK_1, LL_ADC_CHANNEL_0);
    LL_ADC_REG_SetTriggerSource(ADC3, LL_ADC_REG_TRIG_SOFTWARE);
    LL_ADC_REG_SetContinuousMode(ADC3, LL_ADC_REG_CONV_SINGLE);

    // LL_ADC_EnableIT_EOCS(ADC1);
    // NVIC_EnableIRQ(ADC_IRQn);

    // Enable ADC without interrupts
    LL_ADC_Enable(ADC3);
    LL_ADC_REG_StartConversion(ADC3);

    // Enable hardware RNG as well
    LL_AHB2_GRP1_EnableClock(LL_AHB2_GRP1_PERIPH_RNG);
    RNG->CR |= RNG_CR_RNGEN;

    // Initialize random globals
    DefineEntropySource(HARD_RNG);
    doFilter = FALSE;
    memset(IIR_A, 0, sizeof(IIR_A));
    memset(IIR_B, 0, sizeof(IIR_B));
    IIR_A[0] = 1.0f;
    IIR_B[0] = 1.0f;
    iirPos = 0;
    for (uint32_t i = 0; i <= FILT_DEPTH; i++)
        iirPsc[i] = i;

    // Somewhat long operations
    memset(iirInp, 0, sizeof(iirInp));
    memset(iirOut, 0, sizeof(iirOut));
    FillRandom(iirFastInp, FILT_DEPTH);
    FillRandom(iirFastOut, FILT_DEPTH);
}
void SetupBaseTimer(uint32_t timFreq) { // Configure TIM2 to generate IRQs at 'BASE_FREQ' frequency
    // Enable TIM2 counter
    LL_APB1_GRP1_EnableClock(LL_APB1_GRP1_PERIPH_TIM2);
    LL_TIM_SetCounterMode(TIM2, LL_TIM_COUNTERMODE_UP);

    // Configure frequencies
    uint32_t arr = timFreq / BASE_FREQ - 1;
    LL_TIM_SetPrescaler(TIM2, 0);       // Do not divide APB1 frequency here
    LL_TIM_SetAutoReload(TIM2, arr);    // Divide by smth -> BASE_FREQ Hz

    // Configure TIM2 interrupts
    LL_TIM_EnableIT_UPDATE(TIM2);
    NVIC_EnableIRQ(TIM2_IRQn);

    // Start the timer
    LL_TIM_EnableCounter(TIM2);
}
void SetupMSTimer(uint32_t timFreq) {   // Configure TIM3 in one-shot mode
    // Enable TIM3 counter
    LL_APB1_GRP1_EnableClock(LL_APB1_GRP1_PERIPH_TIM3);
    LL_TIM_SetCounterMode(TIM3, LL_TIM_COUNTERMODE_UP);
    // Set one-pulse mode
    LL_TIM_SetOnePulseMode(TIM3, LL_TIM_ONEPULSEMODE_SINGLE);
    LL_TIM_DisableARRPreload(TIM3);     // ARR change takes effect immediately

    // Configure frequency
    uint32_t psc = timFreq / 2000 - 1;
    LL_TIM_SetPrescaler(TIM3, psc);     // Divide by smth -> 2 kHz
    LL_TIM_SetAutoReload(TIM3, 0);      // Setup ARR later

    // Update, but do not start TIM3 now
    LL_TIM_GenerateEvent_UPDATE(TIM3);
    LL_TIM_DisableCounter(TIM3);

    // Configure TIM3 interrupts
    LL_TIM_EnableIT_UPDATE(TIM3);
    NVIC_EnableIRQ(TIM3_IRQn);
}
void SetupDebugTimer(uint32_t timFreq) {// Configure TIM5 to generate IRQs at 2 Hz frequency
    // Setup GPIO PA1 for LED
    LL_AHB4_GRP1_EnableClock(LL_AHB4_GRP1_PERIPH_GPIOA);
    LL_GPIO_SetPinMode(GPIOA, LL_GPIO_PIN_1, LL_GPIO_MODE_OUTPUT);
    LL_GPIO_ResetOutputPin(GPIOA, LL_GPIO_PIN_1);   // Turn off LED by default

    // Enable TIM5 counter
    LL_APB1_GRP1_EnableClock(LL_APB1_GRP1_PERIPH_TIM5);
    LL_TIM_SetCounterMode(TIM5, LL_TIM_COUNTERMODE_UP);

    // Configure frequency
    uint32_t arr = timFreq / 10000 / 2 - 1;
    LL_TIM_SetPrescaler(TIM5, 9999);    // Divide by 10000
    LL_TIM_SetAutoReload(TIM5, arr);    // Divide by smth -> 2 Hz

    // Configure TIM5 interrupts
    LL_TIM_EnableIT_UPDATE(TIM5);
    NVIC_EnableIRQ(TIM5_IRQn);

    // Start the timer
    LL_TIM_EnableCounter(TIM5);
}
void SetupUART(uint32_t usartFreq) {    // Configure UART on GPIOA pins PA9 (TX) and PA10 (RX)
    // Configure GPIO PE3 for input from user button
    LL_AHB4_GRP1_EnableClock(LL_AHB4_GRP1_PERIPH_GPIOE);
    LL_GPIO_SetPinMode(GPIOE, LL_GPIO_PIN_3, LL_GPIO_MODE_INPUT);
    LL_GPIO_SetPinPull(GPIOE, LL_GPIO_PIN_3, LL_GPIO_PULL_UP);

    // Configure EXTI line 3 interrupt
    LL_APB4_GRP1_EnableClock(LL_APB4_GRP1_PERIPH_SYSCFG);
    LL_SYSCFG_SetEXTISource(LL_SYSCFG_EXTI_PORTE, LL_SYSCFG_EXTI_LINE3);    // Select PE3 for EXTI3
    LL_EXTI_EnableFallingTrig_0_31(LL_EXTI_LINE_3);
    LL_EXTI_EnableIT_0_31(LL_EXTI_LINE_3);
    NVIC_EnableIRQ(EXTI3_IRQn); // Enable IRQ from EXTI3 line on NVIC

    // Configure GPIO pins PA9 and PA10
    LL_AHB4_GRP1_EnableClock(LL_AHB4_GRP1_PERIPH_GPIOA);    // Enable GPIOA block
    LL_GPIO_SetPinMode(GPIOA, LL_GPIO_PIN_9,  LL_GPIO_MODE_ALTERNATE);
    LL_GPIO_SetPinMode(GPIOA, LL_GPIO_PIN_10, LL_GPIO_MODE_ALTERNATE);
    LL_GPIO_SetAFPin_8_15(GPIOA, LL_GPIO_PIN_9,  LL_GPIO_AF_7);
    LL_GPIO_SetAFPin_8_15(GPIOA, LL_GPIO_PIN_10, LL_GPIO_AF_7);

    // Configure basic UART parameters
    LL_APB2_GRP1_EnableClock(LL_APB2_GRP1_PERIPH_USART1);   // Enable USART1 block
    LL_USART_SetParity(USART1, LL_USART_PARITY_NONE);       // Turn off parity control
    LL_USART_SetDataWidth(USART1, LL_USART_DATAWIDTH_8B);   // Use 8-bit words
    LL_USART_SetStopBitsLength(USART1, LL_USART_STOPBITS_1);// Use 1 stop bit
    LL_USART_SetHWFlowCtrl(USART1, LL_USART_HWCONTROL_NONE);// Disable hardware flow control
    LL_USART_DisableAutoBaudRate(USART1);
    LL_USART_DisableFIFO(USART1);
    // Set baud rate
    LL_USART_SetBaudRate(USART1, usartFreq, LL_USART_PRESCALER_DIV1, LL_USART_OVERSAMPLING_16, BAUD_RATE);

    // Setup UART operation regime
    LL_USART_DisableOverrunDetect(USART1);  // Do not generate false interrupts with ORE
    LL_USART_EnableDirectionTx(USART1);     // Turn on UART transmitter
    LL_USART_EnableDirectionRx(USART1);     // Turn on UART receiver
    LL_USART_EnableIT_RXNE(USART1);         // Enable IRQ when receive buffer is not empty 'RXNE'

    // Attach IRQ handler on RXNE interrupt (receive buffer not empty)
    NVIC_SetPriority(USART1_IRQn, 32);  // Higher priority than for TIMx
    NVIC_EnableIRQ(USART1_IRQn);        // See 'stm32f743xx.h' line 98
    // IRQ handler is named 'USART1_IRQHandler', see 'startup_stm32f743xx.s' line 185

    // Initialize receive buffer
    rxBuf[RX_BUF_SIZE-1] = 0;   // Ensure terminating zero at the end of buffer
    rxPos = 0;
    isOverflow = FALSE;
    isPending = FALSE;

    // Start UART operation
    LL_USART_Enable(USART1);
}

// === === === Main routine === === ===
int main(void) {
    // Use HSE as main clock
    SetupHSE();

    // Setup system clock
    // LL_RCC_GetSystemClocksFreq(&clk);   // Get all frequencies (probably, there is a bug)
    clk.SYSCLK_Frequency = HCLK_FREQ;
    clk.CPUCLK_Frequency = HCLK_FREQ;
    clk.HCLK_Frequency = HCLK_FREQ;
    clk.PCLK1_Frequency = HCLK_FREQ;
    clk.PCLK2_Frequency = HCLK_FREQ;
    clk.PCLK3_Frequency = HCLK_FREQ;
    clk.PCLK4_Frequency = HCLK_FREQ;
    LL_Init1msTick(clk.HCLK_Frequency); // Configure generic millisecond clock

    // Check that selected base frequency is viable
    while (clk.PCLK1_Frequency % BASE_FREQ != 0);

    // Configure peripherals
    SetupDebugTimer(clk.HCLK_Frequency);
    SetupRandoms();
    SetupDAC();
    SetupBaseTimer(clk.HCLK_Frequency);
    SetupMSTimer(clk.HCLK_Frequency);
    SetupUART(clk.PCLK2_Frequency);

    // Setup basic beat pattern 4/4
    beatDurMS = 250;
    memset(pattern, 0, sizeof pattern);
    pattern[0] = 1.0f;
    pattern[1] = 1.0f;
    pattern[2] = 1.0f;
    pattern[3] = 1.0f;
    pattLen = 4;
    pattPos = 0;
    mutability = -1.0f; // Stable pattern
    mutQuant = 0.5f;

    // Play ascending sequence at startup
    waveType = WAVE_SIN;
    regime = ONESHOT;
    LL_TIM_EnableCounter(TIM2); // Ensure that base timer is running
    for (uint32_t i = 0; i < 7; i++) {
        uint32_t powIdx = SCALES[MAJOR * MAX_SCALE_LEN + i];
        ChangeFreq(BASE_TONE('F')/2 * HT_POW[powIdx]);
        LL_mDelay(80);
    }
    ChangeFreq(BASE_TONE('F'));
    FireOneShot(200);

    // Process user commands in main loop instead of UART IRQ
    while (TRUE) {
        if (isPending) {
            bool res = ExecCommand(cmdBuf, cmdLen);
            isPending = FALSE;
            if (!res) SendMessage(MSG_INVALID, sizeof MSG_INVALID);
        }
    }
}


// === === === Interrupt handlers === === ===
void TIM5_IRQHandler(void) {    // This IRQ is triggered 2 times per second by TIM5
    LL_TIM_ClearFlag_UPDATE(TIM5);  // Clear the interrupt flag
    LL_GPIO_TogglePin(GPIOA, LL_GPIO_PIN_1);    // Change status/UART LED state
}
void TIM3_IRQHandler(void) {    // This IRQ is triggered when TIM3 stops in one-pulse mode
    LL_TIM_ClearFlag_UPDATE(TIM3);  // Clear the interrupt flag

    uint32_t dur;   // Local variable to use inside 'switch'

    switch (regime) {
    case PAUSING:
        // Pausing process finished
        regime = IDLE;
        isNoise = FALSE;
        seconds = 0;        // Keep seconds in normal ranges
        break;
    case ONESHOT:
        // One-shot finished
        regime = PAUSING;
        isNoise = FALSE;
        ChangeFreq(0.0f);   // Zero frequency (no signal)
        FireOneShot(5);     // 5 ms transition
        break;
    case IMPROV:
        // Suggest the next tone to be played
        ChangeFreq(ImproviseNext());
        dur = beatDurMS * pattern[pattPos]; // Follow the beat pattern
        pattPos = (pattPos + 1) % pattLen;  // Move on to the next beat
        if (0 == pattPos && pattLen > 1) {
            // Mutate pattern if needed
            float32_t r = Entropy() * rndNorm;
            if (r <= mutability) {
                // Design a mutation: apply a mutation quantum at a random position
                uint32_t idx = Entropy() % (pattLen - 1);   // Where to apply mutation
                bool l2r = FALSE;
                if (pattern[idx] > mutQuant && pattern[idx+1] > mutQuant) {
                    // Move beat in random direction
                    l2r = (Entropy() % 2 == 0);
                } else if (pattern[idx] > mutQuant && pattern[idx+1] <= mutQuant) {
                    // Move beat from left to right
                    l2r = TRUE;
                } else if (pattern[idx] <= mutQuant && pattern[idx+1] > mutQuant) {
                    // Move beat from right to left
                    l2r = FALSE;
                }
                // Mutate beat pattern
                if (l2r) {
                    pattern[idx] -= mutQuant;
                    pattern[idx+1] += mutQuant;
                } else {
                    pattern[idx] += mutQuant;
                    pattern[idx+1] -= mutQuant;
                }
            }
        }
        // Play next beat
        FireOneShot(dur);
        break;
    default:
        break;
    }
}
void TIM2_IRQHandler(void) {    // This IRQ is triggered 'BASE_FREQ' times per second by TIM2
    LL_TIM_ClearFlag_UPDATE(TIM2);      // Clear the interrupt flag
    uint32_t sampleL, sampleR;

    if (STREAM == regime) {
        if ((sPos % RX_BUF_HALF) == 0) {
            // Detect the end of audio stream
            if (STREAM_STOP_1 == *((uint32_t*) (rxBuf+sPos))) {
                if (STREAM_STOP_2 == *((uint32_t*) (rxBuf+sPos+4))) {
                    regime = IDLE;
                    rxBuf[RX_BUF_SIZE-1] = 0;   // Ensure terminating zero at the end of buffer
                    rxPos = 0;
                    return;
                }
            }
            // Otherwise request the next chunk
            SendByte('+');
        }
        // Get samples from RX buffer half
        sampleL = *((uint32_t*) (rxBuf + sPos + 0));    // Load the 1st signed 12-bit sample from RX buffer
        sampleL += MID_CODE;
        sampleL &= 0x0FFFU;
        sampleR = *((uint32_t*) (rxBuf + sPos + 1));    // Load the 2nd signed 12-bit sample from RX buffer
        sampleR >>= 4;
        sampleR += MID_CODE;
        sampleR &= 0x0FFFU;

        // Get samples from RX buffer half
        // sampleL = *((int16_t*) (rxBuf + sPos + 0)); // Load the 1st signed 16-bit sample from RX buffer
        // sampleR = *((int16_t*) (rxBuf + sPos + 2)); // Load the 2nd signed 16-bit sample from RX buffer
        // sampleL += (1U << 15);  // Convert signed 16-bit sample into unsigned 16-bit sample
        // sampleR += (1U << 15);  // Convert signed 16-bit sample into unsigned 16-bit sample
        // sampleL &= 0xFFFFU;     // Remove 17-th bit after s2u conversion
        // sampleR &= 0xFFFFU;     // Remove 17-th bit after s2u conversion
        // sampleL >>= 4;          // Reduce 16-bit precision to 12-bit
        // sampleR >>= 4;          // Reduce 16-bit precision to 12-bit

        // Get samples from RX buffer half
        // uint32_t sampLR = *((uint32_t*) (rxBuf+sPos));
        // sampLR &= 0xFFF0FFF0;
        // sampLR >>= 4;
        // sampLR += 0x08000800;
        // sampleL = sampLR;
        // sampleR = sampLR >> 16;
        if (doFilter) {
            // Apply filter to streamed audio samples if needed
            float32_t sL = 2.0f * (sampleL / (float32_t) MAX_CODE) - 1;
            float32_t sR = 2.0f * (sampleR / (float32_t) MAX_CODE) - 1;
            sL = FilterSample(sL);
            sR = FilterSample(sR);
            // Scale streamed audio by the amplitude and clamp if needed
            sL *= amp;
            sR *= amp;
            if (sL > +1.1f) sL = +1.0f;
            if (sR > +1.1f) sR = +1.0f;
            if (sL < -1.1f) sL = -1.0f;
            if (sR < -1.1f) sR = -1.0f;
            sampleL = MID_CODE * (1.0f + sL) - 1U;
            sampleR = MID_CODE * (1.0f + sR) - 1U;
        }
        sPos = (sPos + 3U) % RX_BUF_SIZE;   // Move to the next two 16-bit samples in buffer
        // sPos = (sPos + 4U) % RX_BUF_SIZE;   // Move to the next two 16-bit samples in buffer
    } else if (IDLE == regime) {
        // Stop base timer in IDLE state
        LL_TIM_DisableCounter(TIM2);
        return;
    } else {
        // Wave generation regimes

        // Update software-level time counters
        cycle = (cycle + 1U) % BASE_FREQ;   // Global cycle counter (within one second)
        if (0 == cycle) {
            seconds += 1;                   // Global seconds counter
            // Phase sanitization
            if (ema < 0.2f / MAX_CODE) {
                phi = freq*seconds + phi/(2*PI);    // Phase compensation, relevant for fractional frequencies
                phi = 2*PI * (phi - (int) phi);
                seconds = 0;                // Keep seconds as small as possible
            }
        }

        // Compute next wave sample
        float32_t s1 = 0.0f, s2 = 0.0f;
        if (isNoise) {
            // Synthesize white noise
            s1 = 2.0f * (Entropy() * rndNorm) - 1;  // Zero-centered white noise sample
            uint32_t npos, idx1;
            if (iirPos > 0) npos = iirPos - 1;
            else npos = FILT_MEM - 1;
            // Modify noise if needed
            switch (noiseType) {
            case NOISE_WHITE:
                if (doFilter) {
                    s2 = FilterSample(s1);    // Optionally filtered
                } else {
                    s2 = s1;
                    iirInp[npos] = s1;
                    iirOut[npos] = s2;
                    iirPos = npos;  // Update cyclic position manually
                }
                break;
            case NOISE_PINK:
                // Pink noise as white noise filtered with special IIR
                iirInp[npos] = s1;
                s2 = PINK_B[0] * iirInp[npos];
                idx1 = npos;
                for (uint32_t i = 1; i < PINK_LEN; i++) {
                    if (idx1 < FILT_MEM-1) idx1++;
                    else idx1 = 0;
                    s2 += PINK_B[i] * iirInp[idx1];
                    s2 -= PINK_A[i] * iirOut[idx1];
                }
                iirOut[npos] = s2;
                iirPos = npos;      // Update cyclic position manually
                break;
            case NOISE_RED:
                // Red noise as white noise integration
                iirInp[npos] = s1;
                s2 = 0.98f * iirOut[iirPos] + s1;   // Accumulate leaky quadrature of white noise
                iirOut[npos] = s2;
                iirPos = npos;      // Update cyclic position manually
                // Update noise divider
                s1 = fabsf(s2);
                if (s1 > noiseDiv) noiseDiv = 1.5f * s1;
                break;
            case NOISE_BLUE:
                // Blue noise as 1st-order differenced pink noise
                iirInp[npos] = s1;
                s2 = PINK_B[0] * iirInp[npos];
                idx1 = npos;
                for (uint32_t i = 1; i < PINK_LEN; i++) {
                    if (idx1 < FILT_MEM-1) idx1++;
                    else idx1 = 0;
                    s2 += PINK_B[i] * iirInp[idx1];
                    s2 -= PINK_A[i] * iirOut[idx1];
                }
                iirOut[npos] = s2;
                iirPos = npos;      // Update cyclic position manually
                // 1st-order differencing of pink noise
                s2 = 0.0f;
                idx1 = npos;
                for (uint32_t i = 0; i < DIFF_LEN; i++) {
                    if (idx1 < FILT_MEM-1) idx1++;
                    else idx1 = 0;
                    s2 += DIFF_B[i] * iirOut[idx1];
                }
                break;
            case NOISE_VIOLET:
                // Violet noise as white noise 1st-order differentiation
                iirInp[npos] = s1;
                s2 = 0.0f;
                idx1 = npos;
                for (uint32_t i = 0; i < DIFF_LEN; i++) {
                    if (idx1 < FILT_MEM-1) idx1++;
                    else idx1 = 0;
                    s2 += DIFF_B[i] * iirInp[idx1];
                }
                iirOut[npos] = s2;
                iirPos = npos;      // Update cyclic position manually
                break;
            case NOISE_UV:
                // Ultra-violet noise as 2nd-order differenced pink noise
                iirInp[npos] = s1;
                s2 = PINK_B[0] * iirInp[npos];
                idx1 = npos;
                for (uint32_t i = 1; i < PINK_LEN; i++) {
                    if (idx1 < FILT_MEM-1) idx1++;
                    else idx1 = 0;
                    s2 += PINK_B[i] * iirInp[idx1];
                    s2 -= PINK_A[i] * iirOut[idx1];
                }
                iirOut[npos] = s2;
                iirPos = npos;      // Update cyclic position manually
                // 2nd-order differencing of pink noise
                s2 = 0.0f;
                idx1 = npos;
                for (uint32_t i = 0; i < DIFF2_LEN; i++) {
                    if (idx1 < FILT_MEM-1) idx1++;
                    else idx1 = 0;
                    s2 += DIFF2_B[i] * iirOut[idx1];
                }
                break;
            }
            // Post-scale the noise
            s2 /= noiseDiv;
            // Take pausing into account
            if (PAUSING == regime) {
                s1 = s2;
                s2 = 0;
            } else {
                s1 = 0;
            }
        } else {
            // Synthesize conventional waveform
            float32_t t = 2*PI * (seconds + cycle / ((float32_t) BASE_FREQ));
            switch (waveType) {
            case WAVE_SIN:
                // Monochromatic sine wave (+ smooth transition between mono waves)
                s1 = arm_sin_f32(t * oldFreq + phi);
                s2 = arm_sin_f32(t * freq + phi);
                break;
            case WAVE_SIN3:
                // Cubic sine wave (+ smooth transition between waves)
                s1 = arm_sin_f32(t * oldFreq + phi);
                s2 = arm_sin_f32(t * freq + phi);
                s1 = s1*s1*s1;
                s2 = s2*s2*s2;
                break;
            case WAVE_SIN5:
                // Fifth power of sine wave (+ smooth transition between waves)
                s1 = arm_sin_f32(t * oldFreq + phi);
                s2 = arm_sin_f32(t * freq + phi);
                s1 = s1*s1*s1*s1*s1;
                s2 = s2*s2*s2*s2*s2;
                break;
            case WAVE_RECT:
                // Rectangular wave form
                s1 = (t * oldFreq + phi) / (2*PI);
                s2 = (t * freq + phi) / (2*PI);
                s1 = s1 - floorf(s1);
                s2 = s2 - floorf(s2);
                s1 = (s1 < 0.5f) ? 1 : -1;
                s2 = (s2 < 0.5f) ? 1 : -1;
                break;
            case WAVE_SAWTOOTH:
                // Sawtooth wave form
                s1 = (t * oldFreq + phi) / (2*PI);
                s2 = (t * freq + phi) / (2*PI);
                s1 = s1 - floorf(s1);
                s2 = s2 - floorf(s2);
                s1 = 2*s1 - 1;
                s2 = 2*s2 - 1;
                break;
            case WAVE_TRIANGULAR:
                // Triangular wave form
                s1 = (t * oldFreq + phi) / (2*PI);
                s2 = (t * freq + phi) / (2*PI);
                s1 = s1 - floorf(s1);
                s2 = s2 - floorf(s2);
                s1 = (s1 < 0.5f) ? 4*s1-1 : 3-4*s1;
                s2 = (s2 < 0.5f) ? 4*s2-1 : 3-4*s2;
                break;
            default:
                s1 = 0;
                s2 = 0;
                break;
            }
        }

        // Smooth exponential transition between waves
        float32_t s = ema * s1 + (1-ema) * s2;
        ema *= EMA_K;
        // Scale by common amplitude
        s = amp * s;
        if (s > +1.0f) s = +1;
        if (s < -1.0f) s = -1;
        // Convert to DAC sample
        sampleL = MID_CODE * (1.0f + s) - 1U;
        sampleR = sampleL;
    }

    // Convert next sample
    LL_DAC_ConvertDualData12RightAligned(DAC1, sampleL, sampleR);
    LL_DAC_TrigSWConversion(DAC1, LL_DAC_CHANNEL_1 | LL_DAC_CHANNEL_2);
}
void USART1_IRQHandler(void) {  // This IRQ is triggered when USART1 receives the next symbol
    // Read received byte, this clears RXNE flag automatically
    char recvByte = LL_USART_ReceiveData8(USART1);

    // Change UART LED state
    LL_GPIO_TogglePin(GPIOA, LL_GPIO_PIN_1);

    // Audio streaming mode
    if (STREAM == regime) {
        rxBuf[rxPos] = recvByte;
        rxPos = (rxPos + 1U) % RX_BUF_SIZE; // Cyclic storing of audio samples
        return;
    }

    // Append the byte to RX buffer
    if (rxPos > RX_BUF_SIZE-2) {    // Check buffer overflow
        isOverflow = TRUE;
        rxPos = 0;                  // Rewind the buffer
    }
    // It is safe to append the byte
    rxBuf[rxPos] = recvByte;
    rxPos++;

    // Check whether a command is not completed yet
    if ('\r' != recvByte && '\n' != recvByte) return;

    if (isOverflow) {
        // Reset buffer
        rxPos = 0;
        isOverflow = FALSE;
        // Report overflow
        SendMessage(MSG_OVERFLOW, sizeof MSG_OVERFLOW);
    } else {
        // Basic checks before issuing the command
        cmdLen = rxPos - 1U;    // Command length without terminating character
        bool noOp = ('\n' == rxBuf[0] || '\r' == rxBuf[0]); // Whether command begins with no-op
        bool res = noOp && (cmdLen <= 1U);  // Do not complain about 1 or 2 no-ops in a row
        if (cmdLen >= MIN_CMD_LENGTH && cmdLen <= MAX_CMD_LENGTH) {
            // Copy user command into CMD buffer
            if (cmdLen > CMD_BUF_SIZE - 1) cmdLen = CMD_BUF_SIZE - 1;
            memcpy(cmdBuf, rxBuf, cmdLen);
            cmdBuf[cmdLen] = 0; // Ensure terminating zero
            isPending = TRUE;
            res = TRUE;
        }
        // Reset RX buffer for the next command
        rxPos = 0;
        LL_GPIO_ResetOutputPin(GPIOA, LL_GPIO_PIN_1);   // Turn off UART LED
        // Complain about invalid commands
        if (!res) SendMessage(MSG_INVALID, sizeof MSG_INVALID - 1);
    }
}
void EXTI3_IRQHandler(void) {
    // Clear interrupt flag in Pending Register for EXTI block
    LL_EXTI_ClearFlag_0_31(LL_EXTI_LINE_3);

    // Stop synthesis
    if (STREAM == regime) SendByte('0');
    regime = IDLE;

    // Reset UART as well
    rxPos = 0;
    rxBuf[RX_BUF_SIZE-1] = 0;
    LL_GPIO_ResetOutputPin(GPIOA, LL_GPIO_PIN_1);
}
