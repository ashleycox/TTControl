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
    #include "hardware/clocks.h"
    #include "hardware/dma.h"
    #include "hardware/pwm.h"
    #include "hardware/irq.h"
    #include "pico/critical_section.h"
}

/**
 * @brief Generates 4-phase sinusoidal waveforms using Direct Digital Synthesis (DDS).
 * 
 * Runs on Core 1 for high-precision timing. Core 0 only publishes pending
 * settings; Core 1 swaps them into the active state while filling DMA buffers.
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
    
    /*
     * Main generation loop (Call in loop1)
     * NOTE: With DMA, this is no longer a tight loop, but a buffer management task
     */
    void update(); 
    
    // --- Control ---
    void setFrequency(float freq);
    float getFrequency();
    
    void setAmplitude(float amp); // 0.0 to 1.0
    
    void updateSettings(float freq, const SpeedSettings& s, uint8_t phaseMode);
    
    void setEnabled(bool enabled);
    
    // --- Configuration ---
    void configure(const SpeedSettings& settings);

    // --- Interrupt Handler ---
    static void dmaInterruptHandler();
    
    // --- Dashboard Diagnostics ---
    int16_t getSample(int channel);
    uint32_t getLastBufferFillMs() const;
    uint32_t getBufferFillCount() const;
    uint32_t getDmaIrqCount() const;
    uint32_t getDmaRearmCount() const;
    uint32_t getDmaDesyncCount() const;
    float getSampleRateHz() const;
    bool isDmaRunning() const;
    uint32_t getClippingCount(int channel) const;
    float getModulationHeadroomPercent(int channel);
    float getAppliedPhaseDegrees(int channel) const;
    float getAppliedChannelGainPercent(int channel) const;

private:
    // Double-buffered configuration state. Frequency, phase, amplitude, and filters are copied as a unit so Core 1 never sees a partially changed tune.
    struct WaveformState {
        float frequency;
        uint32_t phaseInc;
        uint32_t phaseOffsets[4];
        float channelGain[4];
        float phaseSlewDegreesPerSecond;
        float gainSlewPercentPerSecond;
        float amplitude; // 0.0 - 1.0
        FilterType filterType;
        float iirAlpha;
        FirProfile firProfile;
        uint8_t activePhaseOutputs;
    };
    
    WaveformState _stateA;
    WaveformState _stateB;
    
    /*
     * Pointers to active and pending states
     * Volatile to ensure atomic-like access (pointer swap is atomic on 32-bit ARM)
     */
    volatile WaveformState* _activeState;
    WaveformState* _pendingState;
    
    // Flags shared between Core 0 control calls and Core 1 buffer generation.
    volatile bool _enabled;
    volatile bool _swapPending; // Flag to tell ISR to swap
    critical_section_t _stateLock;
    
    // Internal sample history maintained only by Core 1. The master accumulator is channel 0; other channels derive phase by adding offsets.
    uint32_t _phaseAcc[4];
    float _iirPrev[4];
    float _firBuffer[4][8]; // [Channel][Tap]
    uint8_t _firIndex;
    volatile int16_t _lastSamples[4];
    uint32_t _appliedPhaseOffsets[4];
    float _appliedChannelGain[4];
    bool _appliedTuningInitialized;
    volatile uint32_t _clippingCount[4];
    
    int16_t _lut[LUT_MAX_SIZE];
    int _lutSize;
    int _lutShift;
    float _sampleRateHz;
    
    // DMA / PWM State
    static const int DMA_BUFFER_SIZE = 256; // Number of samples per buffer
    /*
     * 2 Slices, 2 Buffers per slice (Ping-Pong), Buffer Size
     * Slice 0 controls Phase A & B (GPIO 0, 1)
     * Slice 1 controls Phase C & D (GPIO 2, 3)
     * Buffer format: 32-bit words. Top 16 bits = Channel B/D, Bottom 16 bits = Channel A/C
     */
    uint32_t _dmaBufferSlice0[2][DMA_BUFFER_SIZE];
    uint32_t _dmaBufferSlice1[2][DMA_BUFFER_SIZE];
    
    int _dmaChan0; // Slice 0 Ping
    int _dmaChan1; // Slice 0 Pong
    int _dmaChan2; // Slice 1 Ping
    int _dmaChan3; // Slice 1 Pong
    
    uint _pwmSlice0;
    uint _pwmSlice1;
    
    volatile int _currentBufferIndex; // Last buffer freed by DMA IRQ, 0 or 1
    volatile uint32_t _lastBufferFillMs;
    volatile uint32_t _bufferFillCount;
    volatile uint32_t _dmaIrqCount;
    volatile uint32_t _dmaRearmCount;
    volatile uint32_t _dmaDesyncCount;
    volatile bool _slice1RearmPending[2];
    bool _dmaStarted;
    
    void generateLUT();
    void lockState();
    void unlockState();
    void fillBuffer(int bufferIndex);
    int16_t generateSample(int channel);
    void setupPWM();
    void setupDMA();
    uint32_t frequencyToPhaseIncrement(float freq) const;
    uint32_t phaseOffsetToAccumulator(float degrees) const;
    void rearmDmaChannel(int channel, const uint32_t* readAddr);
    void updateAppliedTuning(const volatile WaveformState* state);
    bool enabledAtomic() const;
    bool swapPendingAtomic() const;
    void storeEnabled(bool enabled);
    void storeSwapPending(bool pending);
};

#endif // WAVEFORM_H
