/*
 * TT Control, advanced sinusoidal control of multi-phase turntable motors
 * Created by Ashley Cox at The Blind Man's Workshop
 * https://theblindmansworkshop.com
 * No part of this code may be used or reproduced for commercial purposes without written permission and contractual agreement
 * All external libraries and frameworks are the property of their respective authors and governed by their respective licenses
 */

#ifndef NETWORK_MANAGER_H
#define NETWORK_MANAGER_H

#include <Arduino.h>
#include "config.h"

#if NETWORK_ENABLE
#include <WiFi.h>
#include <DNSServer.h>
#endif

enum NetworkMode : uint8_t {
    NETWORK_MODE_AP = 0,
    NETWORK_MODE_STA = 1,
    NETWORK_MODE_STA_AP = 2
};

enum WebHomePage : uint8_t {
    WEB_HOME_DASHBOARD = 0,
    WEB_HOME_CONTROL = 1,
    WEB_HOME_SETTINGS = 2,
    WEB_HOME_CALIBRATE = 3,
    WEB_HOME_NETWORK = 4,
    WEB_HOME_PRESETS = 5,
    WEB_HOME_BENCH = 6,
    WEB_HOME_DIAGNOSTICS = 7,
    WEB_HOME_ERRORS = 8,
    WEB_HOME_PAGE_COUNT = 9
};

struct NetworkConfig {
    uint32_t magic;
    uint16_t version;
    bool enabled;
    uint8_t mode;
    char hostname[NETWORK_HOSTNAME_MAX + 1];
    char ssid[NETWORK_SSID_MAX + 1];
    char password[NETWORK_PASSWORD_MAX + 1];
    bool dhcp;
    uint8_t staticIp[4];
    uint8_t gateway[4];
    uint8_t subnet[4];
    uint8_t dns[4];
    bool apFallback;
    char apSsid[NETWORK_SSID_MAX + 1];
    char apPassword[NETWORK_PASSWORD_MAX + 1];
    uint8_t apChannel;
    bool readOnlyMode;
    char webPin[NETWORK_WEB_PIN_MAX + 1];
    uint8_t webHomePage;
};

class NetworkManager {
public:
    NetworkManager();

    void begin();
    void update();
    void restart();
    void stop();
    void save();
    void resetDefaults();

    bool isAvailable() const;
    bool isEnabled() const;
    bool isConnected() const;
    bool isApActive() const;
    bool isSetupApOpen() const;
    bool isServerAvailable() const;
    bool hasStationCredentials() const;
    bool isWebAccessLocked() const;
    bool verifyWebPin(const char* pin) const;

    NetworkConfig& getConfig();
    const NetworkConfig& getConfig() const;

    const char* statusText() const;
    String modeText() const;
    String ipText() const;
    String stationIpText() const;
    String apIpText() const;
    String ssidText() const;
    String macText() const;
    int rssi() const;
    uint8_t connectedClientCount() const;

private:
    NetworkConfig _config;
    bool _loaded;
    bool _apActive;
    bool _staStarted;
    uint32_t _connectStartMs;
    uint32_t _lastReconnectMs;
    char _statusText[48];
#if NETWORK_ENABLE
    DNSServer _dnsServer;
#endif

    void setDefaults();
    void load();
    void start();
    void startStation();
    void startAccessPoint();
    void stopAccessPoint();
    void setStatus(const char* text);
};

extern NetworkManager networkManager;

#endif // NETWORK_MANAGER_H
