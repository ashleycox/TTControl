/*
 * TT Control, advanced sinusoidal control of multi-phase turntable motors
 * Created by Ashley Cox at The Blind Man’s Workshop
 * https://theblindmansworkshop.com
 * No part of this code may be used or reproduced for commercial purposes without written permission and contractual agreement
 * All external libraries and frameworks are the property of their respective authors and governed by their respective licenses
 */

#include "waveform.h"
#include "hal.h"
#include <math.h>

// Global pointer for ISR access
static WaveformGenerator* _waveformInstance = nullptr;

// FIR Coefficients (Example for 8-tap)
const float FIR_COEFFS_GENTLE[8] = {0.0, 0.0, 0.1, 0.4, 0.4, 0.1, 0.0, 0.0};
const float FIR_COEFFS_MEDIUM[8] = {0.05, 0.05, 0.1, 0.3, 0.3, 0.1, 0.05, 0.05};
const float FIR_COEFFS_AGGRESSIVE[8] = {0.1, 0.1, 0.1, 0.2, 0.2, 0.1, 0.1, 0.1};

WaveformGenerator::WaveformGenerator() {
    _enabled = false;
    _swapPending = false;
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
    
    *_pendingState = *((WaveformState*)_activeState); // Copy to pending
    
    _lutSize = LUT_MAX_SIZE;
    _lut = new int16_t[_lutSize];
    
    // Initialize per-channel state
    for(int i=0; i<4; i++) {
        _phaseAcc[i] = 0; // Only _phaseAcc[0] is used as master, others are derived
        _iirPrev[i] = 0.0;
        for(int j=0; j<8; j++) _firBuffer[i][j] = 0;
    }
    _firIndex = 0;
    _lutShift = 32 - (int)log2(_lutSize); // Calculate shift for LUT indexing
    
    _currentBufferIndex = 0;
}

void WaveformGenerator::begin() {
    generateLUT();
    setupPWM();
    setupDMA();
    
    // Pre-fill buffers
    fillBuffer(0);
    fillBuffer(1);
    
    // Start DMA
    dma_channel_start(_dmaChan0);
    dma_channel_start(_dmaChan2);
}

void WaveformGenerator::setupPWM() {
    // RP2040 GPIO to PWM Slice Mapping:
    // GPIO 0 -> Slice 0 A
    // GPIO 1 -> Slice 0 B
    // GPIO 2 -> Slice 1 A
    // GPIO 3 -> Slice 1 B
    
    gpio_set_function(PIN_PWM_PHASE_A, GPIO_FUNC_PWM);
    gpio_set_function(PIN_PWM_PHASE_B, GPIO_FUNC_PWM);
    gpio_set_function(PIN_PWM_PHASE_C, GPIO_FUNC_PWM);
    gpio_set_function(PIN_PWM_PHASE_D, GPIO_FUNC_PWM);
    
    _pwmSlice0 = pwm_gpio_to_slice_num(PIN_PWM_PHASE_A);
    _pwmSlice1 = pwm_gpio_to_slice_num(PIN_PWM_PHASE_C);
    
    pwm_config config = pwm_get_default_config();
    
    // Set PWM frequency to ~50kHz
    // SysClock = 125MHz (usually)
    // Wrap = 1023 (10-bit)
    // Div = 125000000 / (50000 * 1024) = ~2.44
    pwm_config_set_wrap(&config, 1023);
    pwm_config_set_clkdiv(&config, 2.44f);
    
    pwm_init(_pwmSlice0, &config, true);
    pwm_init(_pwmSlice1, &config, true);
}

void WaveformGenerator::setupDMA() {
    // Claim DMA channels
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
    
    // Enable Interrupts
    // We only need interrupts from one channel per slice to know when a buffer is done.
    // Actually, we need to know when *any* buffer finishes so we can refill it.
    // Since they run in lockstep (same PWM freq), we can just listen to Slice 0's channels.
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
            
            // Chan 0 (and Chan 2) finished. Chan 1 (and Chan 3) are now running.
            // Reset read addresses for Chan 0 and Chan 2 so they are ready when chained back.
            dma_channel_set_read_addr(_waveformInstance->_dmaChan0, _waveformInstance->_dmaBufferSlice0[0], false);
            dma_channel_set_read_addr(_waveformInstance->_dmaChan2, _waveformInstance->_dmaBufferSlice1[0], false);
            
            // Signal that Buffer 0 is free to be refilled
            _waveformInstance->_currentBufferIndex = 0; 
        }
        if (dma_hw->ints0 & (1u << _waveformInstance->_dmaChan1)) {
            dma_hw->ints0 = (1u << _waveformInstance->_dmaChan1); // Clear IRQ
            
            // Chan 1 (and Chan 3) finished. Chan 0 (and Chan 2) are now running.
            // Reset read addresses for Chan 1 and Chan 3.
            dma_channel_set_read_addr(_waveformInstance->_dmaChan1, _waveformInstance->_dmaBufferSlice0[1], false);
            dma_channel_set_read_addr(_waveformInstance->_dmaChan3, _waveformInstance->_dmaBufferSlice1[1], false);
            
            // Signal that Buffer 1 is free to be refilled
            _waveformInstance->_currentBufferIndex = 1;
        }
    }
}

void __not_in_flash_func(WaveformGenerator::update)() {
    // Check if we need to refill a buffer
    // We can check the DMA busy status or use the flag set by ISR.
    // Since we have double buffering, we want to fill the *inactive* buffer.
    
    // Simple polling approach:
    // If Chan 0 is busy, we can touch Buffer 1.
    // If Chan 1 is busy, we can touch Buffer 0.
    
    bool chan0Busy = dma_channel_is_busy(_dmaChan0);
    bool chan1Busy = dma_channel_is_busy(_dmaChan1);
    
    // Safety: If both are busy (transition) or neither (stopped), do nothing?
    // Actually, one should always be busy.
    
    static int lastFilledBuffer = -1;
    
    if (chan0Busy && lastFilledBuffer != 1) {
        // Chan 0 is reading Buffer 0, so Buffer 1 is free to fill
        fillBuffer(1);
        lastFilledBuffer = 1;
    }
    else if (chan1Busy && lastFilledBuffer != 0) {
        // Chan 1 is reading Buffer 1, so Buffer 0 is free to fill
        fillBuffer(0);
        lastFilledBuffer = 0;
    }
}

void __not_in_flash_func(WaveformGenerator::fillBuffer)(int bufferIndex) {
    if (!_enabled) {
        // Fill with zeros
        for (int i = 0; i < DMA_BUFFER_SIZE; i++) {
            _dmaBufferSlice0[bufferIndex][i] = 0;
            _dmaBufferSlice1[bufferIndex][i] = 0;
        }
        return;
    }

    // Handle State Swap
    if (_swapPending) {
        WaveformState* temp = (WaveformState*)_activeState;
        _activeState = _pendingState;
        _pendingState = temp;
        *_pendingState = *((WaveformState*)_activeState);
        _swapPending = false;
    }
    
    const volatile WaveformState* state = _activeState;
    
    for (int i = 0; i < DMA_BUFFER_SIZE; i++) {
        // Calculate samples for all 4 phases
        int16_t samples[4];
        for (int ch = 0; ch < 4; ch++) {
            samples[ch] = getSample(ch);
        }
        
        // Advance Master Phase
        _phaseAcc[0] += state->phaseInc;
        
        // Pack into 32-bit words for DMA
        // Slice 0: Phase A (GPIO 0) -> Channel A (Low 16), Phase B (GPIO 1) -> Channel B (High 16)
        // Slice 1: Phase C (GPIO 2) -> Channel A (Low 16), Phase D (GPIO 3) -> Channel B (High 16)
        
        // Offset to 0-1023 range (Center 512)
        uint16_t valA = (uint16_t)(512 + samples[0]);
        uint16_t valB = (uint16_t)(512 + samples[1]);
        uint16_t valC = (uint16_t)(512 + samples[2]);
        uint16_t valD = (uint16_t)(512 + samples[3]);
        
        // Clamp
        if (valA > 1023) valA = 1023;
        if (valB > 1023) valB = 1023;
        if (valC > 1023) valC = 1023;
        if (valD > 1023) valD = 1023;
        
        _dmaBufferSlice0[bufferIndex][i] = (valB << 16) | valA;
        _dmaBufferSlice1[bufferIndex][i] = (valD << 16) | valC;
    }
}

void WaveformGenerator::generateLUT() {
    for (int i = 0; i < _lutSize; i++) {
        float angle = (2.0 * PI * i) / _lutSize;
        _lut[i] = (int16_t)(sin(angle) * 511.0);
    }
}

void WaveformGenerator::configure(const SpeedSettings& s) {
    _pendingState->filterType = (FilterType)s.filterType;
    _pendingState->iirAlpha = s.iirAlpha;
    _pendingState->firProfile = (FirProfile)s.firProfile;
    
    for(int i=0; i<4; i++) {
        double normalized = s.phaseOffset[i] / 360.0;
        _pendingState->phaseOffsets[i] = (uint32_t)(normalized * 4294967296.0);
    }
    _swapPending = true;
}

void WaveformGenerator::setFrequency(float freq) {
    _pendingState->frequency = freq;
    // Recalculate Phase Increment based on PWM frequency (50kHz)
    // Inc = Freq * (1/50000) * 2^32
    // Inc = Freq * 0.00002 * 4294967296.0
    // Inc = Freq * 85899.34592
    double inc = freq * 85899.34592;
    _pendingState->phaseInc = (uint32_t)inc;
    _swapPending = true;
}

void WaveformGenerator::setAmplitude(float amp) {
    if (amp < 0.0) amp = 0.0;
    if (amp > 1.0) amp = 1.0;
    _pendingState->amplitude = amp;
    _swapPending = true;
}

void WaveformGenerator::updateSettings(float freq, const SpeedSettings& s) {
    _pendingState->frequency = freq;
    
    // Recalculate Phase Increment
    double inc = freq * 85899.34592;
    _pendingState->phaseInc = (uint32_t)inc;
    
    _pendingState->filterType = (FilterType)s.filterType;
    _pendingState->iirAlpha = s.iirAlpha;
    _pendingState->firProfile = (FirProfile)s.firProfile;
    
    for(int i=0; i<4; i++) {
        double normalized = s.phaseOffset[i] / 360.0;
        _pendingState->phaseOffsets[i] = (uint32_t)(normalized * 4294967296.0);
    }
    
    _swapPending = true;
}

void WaveformGenerator::setEnabled(bool e) {
    _enabled = e;
    if (!e) {
        // Force zero output immediately?
        // Or just let the buffer fill with zeros next time.
        // For immediate stop, we might want to pause DMA or clear PWM.
        // But filling with zeros is safer for smooth stop.
    }
}

int16_t __not_in_flash_func(WaveformGenerator::getSample)(int channel) {
    const volatile WaveformState* state = _activeState;
    
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
        float alpha = state->iirAlpha;
        float out = alpha * val + (1.0 - alpha) * _iirPrev[channel];
        _iirPrev[channel] = out;
        val = (int16_t)out;
    }
    else if (state->filterType == FILTER_FIR) {
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
