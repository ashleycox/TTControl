/*
 * TT Control, advanced sinusoidal control of multi-phase turntable motors
 * Created by Ashley Cox at The Blind Man’s Workshop
 * https://theblindmansworkshop.com
 * No part of this code may be used or reproduced for commercial purposes without written permission and contractual agreement
 * All external libraries and frameworks are the property of their respective authors and governed by their respective licenses
 */

#include "waveform.h"
#include "hal.h"
#include "system_monitor.h"
#include <math.h>

// Global pointer for ISR access. Only one WaveformGenerator exists in this sketch, so a static thunk is simpler than passing context through the IRQ API.
static WaveformGenerator* _waveformInstance = nullptr;
static const uint16_t PWM_WRAP_VALUE = 1023;
static const float PWM_CLOCK_DIVIDER = 2.44f;
static const float FALLBACK_SAMPLE_RATE_HZ = 50000.0f;
static const double DDS_ACCUMULATOR_SCALE = 4294967296.0;

// FIR coefficients are normalized to roughly unity gain so filtering does not change the requested output amplitude.
const float FIR_COEFFS_GENTLE[8] = {0.0, 0.0, 0.1, 0.4, 0.4, 0.1, 0.0, 0.0};
const float FIR_COEFFS_MEDIUM[8] = {0.05, 0.05, 0.1, 0.3, 0.3, 0.1, 0.05, 0.05};
const float FIR_COEFFS_AGGRESSIVE[8] = {0.1, 0.1, 0.1, 0.2, 0.2, 0.1, 0.1, 0.1};

WaveformGenerator::WaveformGenerator() {
    _enabled = false;
    _swapPending = false;
    _stateLock = false;
    _waveformInstance = this;
    
    // Initialize States
    _activeState = &_stateA;
    _pendingState = &_stateB;
    
    // Defaults for _stateA
    _stateA.frequency = 50.0;
    _stateA.amplitude = 0.0;
    _stateA.phaseInc = 0;
    _stateA.filterType = FILTER_NONE;
    _stateA.iirAlpha = 0.0;
    _stateA.firProfile = FIR_GENTLE;
    for(int i=0; i<4; i++) _stateA.phaseOffsets[i] = 0;
    
    // Start pending and active states identical so the first swap is safe even before any settings have been applied.
    *_pendingState = *((WaveformState*)_activeState);
    
    _lutSize = LUT_MAX_SIZE;
    _sampleRateHz = FALLBACK_SAMPLE_RATE_HZ;
    
    // Initialize per-channel state
    for(int i=0; i<4; i++) {
        _phaseAcc[i] = 0; // Only _phaseAcc[0] is used as master, others are derived
        _iirPrev[i] = 0.0;
        _lastSamples[i] = 0;
        for(int j=0; j<8; j++) _firBuffer[i][j] = 0;
    }
    _firIndex = 0;
    // Number of top accumulator bits used as the LUT index.
    _lutShift = 32 - (int)log2(_lutSize);
    
    _currentBufferIndex = 0;
    _lastBufferFillMs = 0;
    _bufferFillCount = 0;
    _dmaIrqCount = 0;
    _dmaRearmCount = 0;
    _dmaDesyncCount = 0;
    _dmaStarted = false;
}

void WaveformGenerator::begin() {
    generateLUT();
    setupPWM();
    setupDMA();
    
    // Pre-fill both ping-pong buffers before DMA starts so PWM never reads uninitialized sample memory.
    fillBuffer(0);
    fillBuffer(1);
    
    // Start one channel per PWM slice. Chaining keeps the paired ping-pong channels running after this point.
    dma_channel_start(_dmaChan0);
    dma_channel_start(_dmaChan2);
    _dmaStarted = true;
}

void WaveformGenerator::setupPWM() {
    /*
     * RP2040 GPIO to PWM Slice Mapping:
     * GPIO 0 -> Slice 0 A
     * GPIO 1 -> Slice 0 B
     * GPIO 2 -> Slice 1 A
     * GPIO 3 -> Slice 1 B
     */
    
    gpio_set_function(PIN_PWM_PHASE_A, GPIO_FUNC_PWM);
    gpio_set_function(PIN_PWM_PHASE_B, GPIO_FUNC_PWM);
    gpio_set_function(PIN_PWM_PHASE_C, GPIO_FUNC_PWM);
#if ENABLE_4_CHANNEL_SUPPORT
    gpio_set_function(PIN_PWM_PHASE_D, GPIO_FUNC_PWM);
#endif
    
    _pwmSlice0 = pwm_gpio_to_slice_num(PIN_PWM_PHASE_A);
    _pwmSlice1 = pwm_gpio_to_slice_num(PIN_PWM_PHASE_C);
    
    pwm_config config = pwm_get_default_config();
    
    /*
     * Set PWM frequency to approximately 50 kHz. The DDS phase increment is
     * derived from the same wrap/divider values below so clock changes do not
     * silently retune the motor.
     * SysClock = 125MHz (usually)
     * Wrap = 1023 (10-bit)
     * Div = 125000000 / (50000 * 1024) = ~2.44
     */
    pwm_config_set_wrap(&config, PWM_WRAP_VALUE);
    pwm_config_set_clkdiv(&config, PWM_CLOCK_DIVIDER);
    _sampleRateHz = ((float)clock_get_hz(clk_sys) / PWM_CLOCK_DIVIDER) / ((float)PWM_WRAP_VALUE + 1.0f);
    if (!isfinite(_sampleRateHz) || _sampleRateHz <= 0.0f) {
        _sampleRateHz = FALLBACK_SAMPLE_RATE_HZ;
    }
    
    // Configure both carrier slices while stopped, then align their counters before enabling them in one register write.
    pwm_init(_pwmSlice0, &config, false);
    pwm_init(_pwmSlice1, &config, false);
    pwm_set_counter(_pwmSlice0, 0);
    pwm_set_counter(_pwmSlice1, 0);
    pwm_set_mask_enabled((1u << _pwmSlice0) | (1u << _pwmSlice1));
}

void WaveformGenerator::setupDMA() {
    // Claim four channels: ping/pong for slice 0 and ping/pong for slice 1.
    _dmaChan0 = dma_claim_unused_channel(true);
    _dmaChan1 = dma_claim_unused_channel(true);
    _dmaChan2 = dma_claim_unused_channel(true);
    _dmaChan3 = dma_claim_unused_channel(true);
    
    // --- Slice 0 (Phase A & B) ---
    dma_channel_config c0 = dma_channel_get_default_config(_dmaChan0);
    channel_config_set_transfer_data_size(&c0, DMA_SIZE_32);
    channel_config_set_read_increment(&c0, true);
    channel_config_set_write_increment(&c0, false);
    channel_config_set_dreq(&c0, DREQ_PWM_WRAP0 + _pwmSlice0); // Pace by PWM wrap
    channel_config_set_chain_to(&c0, _dmaChan1); // Chain to Chan 1
    
    dma_channel_configure(
        _dmaChan0, &c0,
        &pwm_hw->slice[_pwmSlice0].cc, // Write to PWM CC register
        _dmaBufferSlice0[0],           // Read from Buffer 0
        DMA_BUFFER_SIZE,               // Transfer count
        false                          // Don't start yet
    );
    
    dma_channel_config c1 = dma_channel_get_default_config(_dmaChan1);
    channel_config_set_transfer_data_size(&c1, DMA_SIZE_32);
    channel_config_set_read_increment(&c1, true);
    channel_config_set_write_increment(&c1, false);
    channel_config_set_dreq(&c1, DREQ_PWM_WRAP0 + _pwmSlice0);
    channel_config_set_chain_to(&c1, _dmaChan0); // Chain back to Chan 0
    
    dma_channel_configure(
        _dmaChan1, &c1,
        &pwm_hw->slice[_pwmSlice0].cc,
        _dmaBufferSlice0[1],           // Read from Buffer 1
        DMA_BUFFER_SIZE,
        false
    );
    
    // --- Slice 1 (Phase C & D) ---
    dma_channel_config c2 = dma_channel_get_default_config(_dmaChan2);
    channel_config_set_transfer_data_size(&c2, DMA_SIZE_32);
    channel_config_set_read_increment(&c2, true);
    channel_config_set_write_increment(&c2, false);
    channel_config_set_dreq(&c2, DREQ_PWM_WRAP0 + _pwmSlice1);
    channel_config_set_chain_to(&c2, _dmaChan3);
    
    dma_channel_configure(
        _dmaChan2, &c2,
        &pwm_hw->slice[_pwmSlice1].cc,
        _dmaBufferSlice1[0],
        DMA_BUFFER_SIZE,
        false
    );
    
    dma_channel_config c3 = dma_channel_get_default_config(_dmaChan3);
    channel_config_set_transfer_data_size(&c3, DMA_SIZE_32);
    channel_config_set_read_increment(&c3, true);
    channel_config_set_write_increment(&c3, false);
    channel_config_set_dreq(&c3, DREQ_PWM_WRAP0 + _pwmSlice1);
    channel_config_set_chain_to(&c3, _dmaChan2);
    
    dma_channel_configure(
        _dmaChan3, &c3,
        &pwm_hw->slice[_pwmSlice1].cc,
        _dmaBufferSlice1[1],
        DMA_BUFFER_SIZE,
        false
    );
    
    /*
     * Slice 0 and slice 1 run in lockstep from the same PWM settings, so slice 0
     * interrupts are enough to know which buffer in both slices is free.
     */
    dma_channel_set_irq0_enabled(_dmaChan0, true);
    dma_channel_set_irq0_enabled(_dmaChan1, true);
    
    irq_set_exclusive_handler(DMA_IRQ_0, WaveformGenerator::dmaInterruptHandler);
    irq_set_enabled(DMA_IRQ_0, true);
}

void __not_in_flash_func(WaveformGenerator::dmaInterruptHandler)() {
    if (_waveformInstance) {
        // Check which channel triggered
        if (dma_hw->ints0 & (1u << _waveformInstance->_dmaChan0)) {
            dma_hw->ints0 = (1u << _waveformInstance->_dmaChan0); // Clear IRQ
            
            /*
             * Chan 0 (and Chan 2) finished. Chan 1 (and Chan 3) are now running.
             * Reset read addresses for Chan 0 and Chan 2 so they are ready when chained back.
             */
            _waveformInstance->_dmaIrqCount++;
            bool pairedBusy = dma_channel_is_busy(_waveformInstance->_dmaChan2);
            if (pairedBusy) {
                _waveformInstance->_dmaDesyncCount++;
            } else {
                _waveformInstance->rearmDmaChannel(_waveformInstance->_dmaChan2, _waveformInstance->_dmaBufferSlice1[0]);
            }
            _waveformInstance->rearmDmaChannel(_waveformInstance->_dmaChan0, _waveformInstance->_dmaBufferSlice0[0]);
            
            // Signal that Buffer 0 is free to be refilled
            _waveformInstance->_currentBufferIndex = 0; 
        }
        if (dma_hw->ints0 & (1u << _waveformInstance->_dmaChan1)) {
            dma_hw->ints0 = (1u << _waveformInstance->_dmaChan1); // Clear IRQ
            
            /*
             * Chan 1 (and Chan 3) finished. Chan 0 (and Chan 2) are now running.
             * Reset read addresses for Chan 1 and Chan 3.
             */
            _waveformInstance->_dmaIrqCount++;
            bool pairedBusy = dma_channel_is_busy(_waveformInstance->_dmaChan3);
            if (pairedBusy) {
                _waveformInstance->_dmaDesyncCount++;
            } else {
                _waveformInstance->rearmDmaChannel(_waveformInstance->_dmaChan3, _waveformInstance->_dmaBufferSlice1[1]);
            }
            _waveformInstance->rearmDmaChannel(_waveformInstance->_dmaChan1, _waveformInstance->_dmaBufferSlice0[1]);
            
            // Signal that Buffer 1 is free to be refilled
            _waveformInstance->_currentBufferIndex = 1;
        }
    }
}

void __not_in_flash_func(WaveformGenerator::update)() {
    /*
     * Refill only the buffer not currently being read by DMA. The DMA IRQ also
     * records the freed buffer, but the busy check is the final guard against
     * overwriting active sample memory.
     */
    
    bool chan0Busy = dma_channel_is_busy(_dmaChan0);
    bool chan1Busy = dma_channel_is_busy(_dmaChan1);
    bool chan2Busy = dma_channel_is_busy(_dmaChan2);
    bool chan3Busy = dma_channel_is_busy(_dmaChan3);
    
    static int lastFilledBuffer = -1;
    static bool desyncRecorded = false;

    bool slice0Ping = chan0Busy && !chan1Busy;
    bool slice0Pong = chan1Busy && !chan0Busy;
    bool slice1Ping = chan2Busy && !chan3Busy;
    bool slice1Pong = chan3Busy && !chan2Busy;
    bool inSync = (slice0Ping && slice1Ping) || (slice0Pong && slice1Pong);

    if (!inSync) {
        if (!desyncRecorded) {
            _dmaDesyncCount++;
            desyncRecorded = true;
        }
        return;
    }
    desyncRecorded = false;
    
    if (slice0Ping && lastFilledBuffer != 1) {
        // Chan 0 is reading Buffer 0, so Buffer 1 is free to fill
        uint32_t startUs = time_us_32();
        fillBuffer(1);
        systemMonitor.recordCore1WorkMicros(time_us_32() - startUs);
        lastFilledBuffer = 1;
    }
    else if (slice0Pong && lastFilledBuffer != 0) {
        // Chan 1 is reading Buffer 1, so Buffer 0 is free to fill
        uint32_t startUs = time_us_32();
        fillBuffer(0);
        systemMonitor.recordCore1WorkMicros(time_us_32() - startUs);
        lastFilledBuffer = 0;
    }
}

void __not_in_flash_func(WaveformGenerator::fillBuffer)(int bufferIndex) {
    _lastBufferFillMs = millis();
    _bufferFillCount++;

    if (!enabledAtomic()) {
        // Fill with zero duty while disabled. MotorController ramps amplitude to zero before disabling for normal stops.
        for (int i = 0; i < DMA_BUFFER_SIZE; i++) {
            _dmaBufferSlice0[bufferIndex][i] = 0;
            _dmaBufferSlice1[bufferIndex][i] = 0;
        }
        return;
    }

    // Apply a pending Core 0 settings update between buffers so every sample in the buffer uses one coherent state.
    if (swapPendingAtomic()) {
        lockState();
        if (swapPendingAtomic()) {
            WaveformState* temp = (WaveformState*)_activeState;
            _activeState = _pendingState;
            _pendingState = temp;
            *_pendingState = *((WaveformState*)_activeState);
            storeSwapPending(false);
        }
        unlockState();
    }
    
    const volatile WaveformState* state = _activeState;
    
    for (int i = 0; i < DMA_BUFFER_SIZE; i++) {
        // Calculate samples for enabled phases; unused channels stay at the neutral sample before the 512 PWM offset is applied.
        int16_t samples[4];
        for (int ch = 0; ch < 4; ch++) {
            samples[ch] = (ch < MAX_ACTIVE_PHASE_OUTPUTS) ? generateSample(ch) : 0;
            _lastSamples[ch] = samples[ch];
        }
        
        // Advance the master phase once per sample. Per-channel phase offsets are added inside generateSample().
        _phaseAcc[0] += state->phaseInc;
        
        /*
         * Pack into 32-bit words for DMA
         * Slice 0: Phase A (GPIO 0) -> Channel A (Low 16), Phase B (GPIO 1) -> Channel B (High 16)
         * Slice 1: Phase C (GPIO 2) -> Channel A (Low 16), Phase D (GPIO 3) -> Channel B (High 16)
         */
        
        // Offset to 0-1023 range (Center 512)
        uint16_t valA = (uint16_t)(512 + samples[0]);
        uint16_t valB = (uint16_t)(512 + samples[1]);
        uint16_t valC = (uint16_t)(512 + samples[2]);
        uint16_t valD = (uint16_t)(512 + samples[3]);
        
        // Clamp to the 10-bit PWM range after offsetting the signed samples.
        if (valA > 1023) valA = 1023;
        if (valB > 1023) valB = 1023;
        if (valC > 1023) valC = 1023;
        if (valD > 1023) valD = 1023;
        
        _dmaBufferSlice0[bufferIndex][i] = (valB << 16) | valA;
        _dmaBufferSlice1[bufferIndex][i] = (valD << 16) | valC;
    }
}

void WaveformGenerator::generateLUT() {
    // Full-scale LUT entries are +/-511 so adding the 512 PWM midpoint produces legal 10-bit duty values after clamping.
    for (int i = 0; i < _lutSize; i++) {
        float angle = (2.0 * PI * i) / _lutSize;
        _lut[i] = (int16_t)(sin(angle) * 511.0);
    }
}

void WaveformGenerator::lockState() {
    while (__atomic_test_and_set(&_stateLock, __ATOMIC_ACQUIRE)) {
        tight_loop_contents();
    }
}

void WaveformGenerator::unlockState() {
    __atomic_clear(&_stateLock, __ATOMIC_RELEASE);
}

void WaveformGenerator::configure(const SpeedSettings& s) {
    // Configure phase and filtering without changing the current frequency.
    lockState();
    _pendingState->filterType = (FilterType)s.filterType;
    _pendingState->iirAlpha = isfinite(s.iirAlpha) ? s.iirAlpha : 0.5f;
    _pendingState->firProfile = (FirProfile)s.firProfile;
    
    for(int i=0; i<4; i++) {
        _pendingState->phaseOffsets[i] = phaseOffsetToAccumulator(s.phaseOffset[i]);
    }
    storeSwapPending(true);
    unlockState();
}

float WaveformGenerator::getFrequency() {
    lockState();
    float freq = _pendingState->frequency;
    unlockState();
    return freq;
}

void WaveformGenerator::setFrequency(float freq) {
    if (!isfinite(freq)) freq = 0.0f;
    if (freq > MAX_OUTPUT_FREQUENCY_HZ) freq = MAX_OUTPUT_FREQUENCY_HZ;
    if (freq < -MAX_OUTPUT_FREQUENCY_HZ) freq = -MAX_OUTPUT_FREQUENCY_HZ;
    lockState();
    _pendingState->frequency = freq;
    _pendingState->phaseInc = frequencyToPhaseIncrement(freq);
    storeSwapPending(true);
    unlockState();
}

void WaveformGenerator::setAmplitude(float amp) {
    // Public amplitude is normalized; MotorController applies settings limits before calling this function.
    if (!isfinite(amp)) amp = 0.0;
    if (amp < 0.0) amp = 0.0;
    if (amp > 1.0) amp = 1.0;
    lockState();
    _pendingState->amplitude = amp;
    storeSwapPending(true);
    unlockState();
}

void WaveformGenerator::updateSettings(float freq, const SpeedSettings& s) {
    // Atomically publish a complete waveform tune: frequency, filters, and phase offsets. This is the preferred path during speed changes.
    if (!isfinite(freq)) freq = 0.0f;
    if (freq > MAX_OUTPUT_FREQUENCY_HZ) freq = MAX_OUTPUT_FREQUENCY_HZ;
    if (freq < -MAX_OUTPUT_FREQUENCY_HZ) freq = -MAX_OUTPUT_FREQUENCY_HZ;
    lockState();
    _pendingState->frequency = freq;
    _pendingState->phaseInc = frequencyToPhaseIncrement(freq);
    
    _pendingState->filterType = (FilterType)s.filterType;
    _pendingState->iirAlpha = isfinite(s.iirAlpha) ? s.iirAlpha : 0.5f;
    _pendingState->firProfile = (FirProfile)s.firProfile;
    
    for(int i=0; i<4; i++) {
        _pendingState->phaseOffsets[i] = phaseOffsetToAccumulator(s.phaseOffset[i]);
    }
    
    storeSwapPending(true);
    unlockState();
}

void WaveformGenerator::setEnabled(bool e) {
    storeEnabled(e);
    if (!e) {
        // Leave DMA/PWM running and let subsequent buffers go to zero. That keeps output shutdown deterministic without reconfiguring timing hardware.
    }
}

int16_t WaveformGenerator::getSample(int channel) {
    if (channel < 0 || channel >= 4) return 0;
    return _lastSamples[channel];
}

uint32_t WaveformGenerator::getLastBufferFillMs() const {
    return _lastBufferFillMs;
}

uint32_t WaveformGenerator::getBufferFillCount() const {
    return _bufferFillCount;
}

uint32_t WaveformGenerator::getDmaIrqCount() const {
    return _dmaIrqCount;
}

uint32_t WaveformGenerator::getDmaRearmCount() const {
    return _dmaRearmCount;
}

uint32_t WaveformGenerator::getDmaDesyncCount() const {
    return _dmaDesyncCount;
}

float WaveformGenerator::getSampleRateHz() const {
    return _sampleRateHz;
}

bool WaveformGenerator::isDmaRunning() const {
    return _dmaStarted &&
           (dma_channel_is_busy(_dmaChan0) || dma_channel_is_busy(_dmaChan1) ||
            dma_channel_is_busy(_dmaChan2) || dma_channel_is_busy(_dmaChan3));
}

uint32_t WaveformGenerator::frequencyToPhaseIncrement(float freq) const {
    if (!isfinite(freq)) freq = 0.0f;
    float sampleRate = _sampleRateHz;
    if (!isfinite(sampleRate) || sampleRate <= 0.0f) sampleRate = FALLBACK_SAMPLE_RATE_HZ;

    double incD = (double)freq * (DDS_ACCUMULATOR_SCALE / (double)sampleRate);
    if (!isfinite(incD)) return 0;

    int64_t inc = llround(incD);
    return (uint32_t)inc;
}

uint32_t WaveformGenerator::phaseOffsetToAccumulator(float degrees) const {
    if (!isfinite(degrees)) degrees = 0.0f;
    double normalized = fmod((double)degrees, 360.0);
    if (normalized < 0.0) normalized += 360.0;
    double scaled = floor((normalized / 360.0) * DDS_ACCUMULATOR_SCALE);
    if (scaled < 0.0 || scaled >= DDS_ACCUMULATOR_SCALE) return 0;
    return (uint32_t)scaled;
}

void __not_in_flash_func(WaveformGenerator::rearmDmaChannel)(int channel, const uint32_t* readAddr) {
    dma_channel_set_trans_count(channel, DMA_BUFFER_SIZE, false);
    dma_channel_set_read_addr(channel, readAddr, false);
    _dmaRearmCount++;
}

bool WaveformGenerator::enabledAtomic() const {
    return __atomic_load_n(&_enabled, __ATOMIC_ACQUIRE);
}

bool WaveformGenerator::swapPendingAtomic() const {
    return __atomic_load_n(&_swapPending, __ATOMIC_ACQUIRE);
}

void WaveformGenerator::storeEnabled(bool enabled) {
    __atomic_store_n(&_enabled, enabled, __ATOMIC_RELEASE);
}

void WaveformGenerator::storeSwapPending(bool pending) {
    __atomic_store_n(&_swapPending, pending, __ATOMIC_RELEASE);
}

int16_t __not_in_flash_func(WaveformGenerator::generateSample)(int channel) {
    const volatile WaveformState* state = _activeState;
    
    // Use the upper accumulator bits for the LUT index and the next ten bits for linear interpolation between adjacent LUT samples.
    uint32_t phase = _phaseAcc[0] + state->phaseOffsets[channel];
    uint16_t index = phase >> _lutShift;
    uint16_t frac = (phase >> (_lutShift - 10)) & 0x3FF; 
    
    uint16_t nextIndex = (index + 1);
    if (nextIndex >= _lutSize) nextIndex = 0;
    
    int16_t s1 = _lut[index];
    int16_t s2 = _lut[nextIndex];
    
    int32_t val = s1 + (((s2 - s1) * (int32_t)frac) >> 10);
    val = (int32_t)(val * state->amplitude);
    
    if (state->filterType == FILTER_IIR) {
        // Lightweight one-pole smoothing for users who need gentler edges.
        float alpha = state->iirAlpha;
        float out = alpha * val + (1.0 - alpha) * _iirPrev[channel];
        _iirPrev[channel] = out;
        val = (int16_t)out;
    }
    else if (state->filterType == FILTER_FIR) {
        // The FIR history is per channel so phase outputs do not bleed together.
        for (int i = 7; i > 0; i--) {
            _firBuffer[channel][i] = _firBuffer[channel][i-1];
        }
        _firBuffer[channel][0] = val;
        
        float sum = 0;
        const float* coeffs;
        if (state->firProfile == FIR_GENTLE) coeffs = FIR_COEFFS_GENTLE;
        else if (state->firProfile == FIR_MEDIUM) coeffs = FIR_COEFFS_MEDIUM;
        else coeffs = FIR_COEFFS_AGGRESSIVE;
        
        for (int i = 0; i < 8; i++) {
            sum += _firBuffer[channel][i] * coeffs[i];
        }
        val = (int16_t)sum;
    }
    
    return (int16_t)val;
}
