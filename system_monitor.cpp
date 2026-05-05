/*
 * TT Control, advanced sinusoidal control of multi-phase turntable motors
 * Created by Ashley Cox at The Blind Man’s Workshop
 * https://theblindmansworkshop.com
 * No part of this code may be used or reproduced for commercial purposes without written permission and contractual agreement
 * All external libraries and frameworks are the property of their respective authors and governed by their respective licenses
 */

#include "system_monitor.h"
#include <LittleFS.h>

extern "C" {
extern const uint8_t __flash_binary_start;
extern const uint8_t __flash_binary_end;
}

SystemMonitor systemMonitor;

SystemMonitor::SystemMonitor()
    : _core0LoopStartUs(0),
      _windowStartUs(0),
      _core0WindowBusyUs(0),
      _core1WindowBusyUs(0),
      _snapshot{} {
}

void SystemMonitor::begin() {
    _windowStartUs = micros();
    _core0LoopStartUs = 0;
    _core0WindowBusyUs = 0;
    __atomic_store_n(&_core1WindowBusyUs, 0, __ATOMIC_RELAXED);
    refreshMemoryAndFlash();
}

void SystemMonitor::beginCore0Loop() {
    _core0LoopStartUs = micros();
}

void SystemMonitor::endCore0Loop() {
    if (_core0LoopStartUs == 0) return;
    uint32_t now = micros();
    _core0WindowBusyUs += now - _core0LoopStartUs;
    _core0LoopStartUs = 0;
}

void SystemMonitor::update() {
    uint32_t now = micros();
    uint32_t elapsedUs = now - _windowStartUs;
    if (elapsedUs < 1000000UL) return;

    uint32_t core1BusyUs = __atomic_exchange_n(&_core1WindowBusyUs, 0, __ATOMIC_RELAXED);
    _snapshot.core0LoadPercent = min(100.0f, (_core0WindowBusyUs * 100.0f) / elapsedUs);
    _snapshot.core1LoadPercent = min(100.0f, (core1BusyUs * 100.0f) / elapsedUs);

    _core0WindowBusyUs = 0;
    _windowStartUs = now;
    refreshMemoryAndFlash();
}

void SystemMonitor::recordCore1WorkMicros(uint32_t durationUs) {
    __atomic_fetch_add(&_core1WindowBusyUs, durationUs, __ATOMIC_RELAXED);
}

SystemMetricsSnapshot SystemMonitor::snapshot() const {
    return _snapshot;
}

void SystemMonitor::refreshMemoryAndFlash() {
    int heapTotal = rp2040.getTotalHeap();
    int heapUsed = rp2040.getUsedHeap();
    int heapFree = rp2040.getFreeHeap();
    int psramTotal = rp2040.getTotalPSRAMHeap();
    int psramUsed = rp2040.getUsedPSRAMHeap();
    int psramFree = rp2040.getFreePSRAMHeap();

    _snapshot.heapTotalBytes = heapTotal > 0 ? (uint32_t)heapTotal : 0;
    _snapshot.heapUsedBytes = heapUsed > 0 ? (uint32_t)heapUsed : 0;
    _snapshot.heapFreeBytes = heapFree > 0 ? (uint32_t)heapFree : 0;
    _snapshot.psramTotalBytes = psramTotal > 0 ? (uint32_t)psramTotal : 0;
    _snapshot.psramUsedBytes = psramUsed > 0 ? (uint32_t)psramUsed : 0;
    _snapshot.psramFreeBytes = psramFree > 0 ? (uint32_t)psramFree : 0;

#ifdef PICO_FLASH_SIZE_BYTES
    _snapshot.flashTotalBytes = PICO_FLASH_SIZE_BYTES;
#else
    _snapshot.flashTotalBytes = 0;
#endif

    uintptr_t sketchStart = (uintptr_t)&__flash_binary_start;
    uintptr_t sketchEnd = (uintptr_t)&__flash_binary_end;
    _snapshot.sketchUsedBytes = sketchEnd > sketchStart ? (uint32_t)(sketchEnd - sketchStart) : 0;

#ifdef FS_START
    _snapshot.sketchCapacityBytes = FS_START;
#else
    _snapshot.sketchCapacityBytes = 0;
#endif

#if defined(FS_START) && defined(FS_END)
    _snapshot.filesystemCapacityBytes = FS_END > FS_START ? (uint32_t)(FS_END - FS_START) : 0;
#else
    _snapshot.filesystemCapacityBytes = 0;
#endif

    FSInfo fsInfo;
    if (LittleFS.info(fsInfo)) {
        _snapshot.filesystemMounted = true;
        _snapshot.filesystemTotalBytes = (uint32_t)fsInfo.totalBytes;
        _snapshot.filesystemUsedBytes = (uint32_t)fsInfo.usedBytes;
    } else {
        _snapshot.filesystemMounted = false;
        _snapshot.filesystemTotalBytes = 0;
        _snapshot.filesystemUsedBytes = 0;
    }
}
