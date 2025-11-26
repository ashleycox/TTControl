/*
 * TT Control, advanced sinusoidal control of multi-phase turntable motors
 * Created by Ashley Cox at The Blind Man’s Workshop
 * https://theblindmansworkshop.com
 * No part of this code may be used or reproduced for commercial purposes without written permission and contractual agreement
 * All external libraries and frameworks are the property of their respective authors and governed by their respective licenses
 */

#ifndef WAVEFORM_H
#define WAVEFORM_H

#include <Arduino.h>
#include "config.h"
#include "types.h"
#include "globals.h"

extern "C" {
    #include "pico/stdlib.h"
    #include "hardware/dma.h"
    #include "hardware/pwm.h"
    #include "hardware/irq.h"
}

/**
 * @brief Generates 4-phase sinusoidal waveforms using Direct Digital Synthesis (DDS).
 * 
 * Runs on Core 1 for high-precision timing.
 * Supports:
 * - Variable frequency and amplitude
 * - Phase offsets
 * - Digital filtering (IIR/FIR)
 * - PWM output generation
 */
class WaveformGenerator {
public:
    WaveformGenerator();
    
    // Initialize hardware and LUTs
    void begin();
    
    // Main generation loop (Call in loop1)
    // NOTE: With DMA, this is no longer a tight loop, but a buffer management task
    void update(); 
    
    // --- Control ---
    void setFrequency(float freq);
    float getFrequency() { return _pendingState->frequency; }
    
    void setAmplitude(float amp); // 0.0 to 1.0
    
    void updateSettings(float freq, const SpeedSettings& s);
    
    void setEnabled(bool enabled);
    
    // --- Configuration ---
    void configure(const SpeedSettings& settings);

    // --- Interrupt Handler ---
    static void dmaInterruptHandler();

private:
    // Double Buffering State
    struct WaveformState {
        float frequency;
        uint32_t phaseInc;
        uint32_t phaseOffsets[4];
        float amplitude; // 0.0 - 1.0
        FilterType filterType;
        float iirAlpha;
        FirProfile firProfile;
    };
    
    WaveformState _stateA;
    WaveformState _stateB;
    
    // Pointers to active and pending states
    // Volatile to ensure atomic-like access (pointer swap is atomic on 32-bit ARM)
    volatile WaveformState* _activeState;
    WaveformState* _pendingState;
    
    // Flags
    bool _enabled;
    volatile bool _swapPending; // Flag to tell ISR to swap
    
    // Internal State (Not buffered, maintained by ISR)
    uint32_t _phaseAcc[4];
    float _iirPrev[4];
    float _firBuffer[4][8]; // [Channel][Tap]
    uint8_t _firIndex;
    
    int16_t* _lut;
    int _lutSize;
    int _lutShift;
    
    // DMA / PWM State
    static const int DMA_BUFFER_SIZE = 256; // Number of samples per buffer
    // 2 Slices, 2 Buffers per slice (Ping-Pong), Buffer Size
    // Slice 0 controls Phase A & B (GPIO 0, 1)
    // Slice 1 controls Phase C & D (GPIO 2, 3)
    // Buffer format: 32-bit words. Top 16 bits = Channel B/D, Bottom 16 bits = Channel A/C
    uint32_t _dmaBufferSlice0[2][DMA_BUFFER_SIZE];
    uint32_t _dmaBufferSlice1[2][DMA_BUFFER_SIZE];
    
    int _dmaChan0; // Slice 0 Ping
    int _dmaChan1; // Slice 0 Pong
    int _dmaChan2; // Slice 1 Ping
    int _dmaChan3; // Slice 1 Pong
    
    uint _pwmSlice0;
    uint _pwmSlice1;
    
    volatile int _currentBufferIndex; // 0 or 1
    
    void generateLUT();
    int16_t getSample(int channel);
    void fillBuffer(int bufferIndex);
    void setupPWM();
    void setupDMA();
};

#endif // WAVEFORM_H
