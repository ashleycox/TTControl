/*
 * TT Control, advanced sinusoidal control of multi-phase turntable motors
 * Created by Ashley Cox at The Blind Man's Workshop
 * https://theblindmansworkshop.com
 * No part of this code may be used or reproduced for commercial purposes without written permission and contractual agreement
 * All external libraries and frameworks are the property of their respective authors and governed by their respective licenses
 */

#include "network_manager.h"
#include "settings.h"
#include <LittleFS.h>
#include <stddef.h>

static const char* NETWORK_CONFIG_FILE = "/network.bin";

NetworkManager networkManager;

static void copyBounded(char* target, size_t targetSize, const char* source) {
    if (!target || targetSize == 0) return;
    if (!source) source = "";
    strncpy(target, source, targetSize - 1);
    target[targetSize - 1] = 0;
}

#if NETWORK_ENABLE
static IPAddress ipFromBytes(const uint8_t bytes[4]) {
    return IPAddress(bytes[0], bytes[1], bytes[2], bytes[3]);
}

static void bytesFromIp(uint8_t bytes[4], uint8_t a, uint8_t b, uint8_t c, uint8_t d) {
    bytes[0] = a;
    bytes[1] = b;
    bytes[2] = c;
    bytes[3] = d;
}
#else
static void bytesFromIp(uint8_t bytes[4], uint8_t a, uint8_t b, uint8_t c, uint8_t d) {
    bytes[0] = a;
    bytes[1] = b;
    bytes[2] = c;
    bytes[3] = d;
}
#endif

NetworkManager::NetworkManager()
    : _loaded(false),
      _apActive(false),
      _staStarted(false),
      _connectStartMs(0),
      _lastReconnectMs(0) {
    setDefaults();
}

void NetworkManager::begin() {
    load();
#if NETWORK_ENABLE
    if (_config.enabled) {
        start();
    } else {
        setStatus("Disabled");
    }
#else
    setStatus("Unavailable");
#endif
}

void NetworkManager::update() {
#if NETWORK_ENABLE
    if (!_config.enabled) return;

    if (_apActive) {
        _dnsServer.processNextRequest();
    }

    uint32_t now = millis();
    bool wantsStation = _config.mode == NETWORK_MODE_STA || _config.mode == NETWORK_MODE_STA_AP;
    if (wantsStation && _staStarted && WiFi.status() == WL_CONNECTED) {
        setStatus(_apActive ? "Connected + setup AP" : "Connected");
        return;
    }

    if (wantsStation && _staStarted && WiFi.status() != WL_CONNECTED) {
        if (_config.apFallback && !_apActive && now - _connectStartMs > 15000) {
            startAccessPoint();
        }

        if (now - _lastReconnectMs > 30000) {
            startStation();
        }
    }
#endif
}

void NetworkManager::restart() {
    stop();
#if NETWORK_ENABLE
    if (_config.enabled) {
        start();
    } else {
        setStatus("Disabled");
    }
#else
    setStatus("Unavailable");
#endif
}

void NetworkManager::stop() {
#if NETWORK_ENABLE
    stopAccessPoint();
    WiFi.disconnect(true);
    WiFi.mode(WIFI_OFF);
#endif
    _apActive = false;
    _staStarted = false;
}

void NetworkManager::save() {
    _config.magic = NETWORK_CONFIG_MAGIC;
    _config.version = NETWORK_CONFIG_VERSION;

    File f = LittleFS.open(NETWORK_CONFIG_FILE, "w");
    if (!f) return;
    f.write((const uint8_t*)&_config, sizeof(NetworkConfig));
    f.close();
}

void NetworkManager::resetDefaults() {
    setDefaults();
    save();
}

bool NetworkManager::isAvailable() const {
#if NETWORK_ENABLE
    return true;
#else
    return false;
#endif
}

bool NetworkManager::isEnabled() const {
    return _config.enabled;
}

bool NetworkManager::isConnected() const {
#if NETWORK_ENABLE
    return _staStarted && WiFi.status() == WL_CONNECTED;
#else
    return false;
#endif
}

bool NetworkManager::isApActive() const {
    return _apActive;
}

bool NetworkManager::isSetupApOpen() const {
    return _apActive && _config.apPassword[0] == 0;
}

bool NetworkManager::isServerAvailable() const {
    return isConnected() || _apActive;
}

bool NetworkManager::hasStationCredentials() const {
    return _config.ssid[0] != 0;
}

bool NetworkManager::isWebAccessLocked() const {
    return _config.readOnlyMode && _config.webPin[0] != 0;
}

bool NetworkManager::verifyWebPin(const char* pin) const {
    if (!_config.readOnlyMode) return true;
    if (!_config.webPin[0]) return true;
    if (!pin) return false;
    if (strlen(pin) > NETWORK_WEB_PIN_MAX) return false;
    return strcmp(pin, _config.webPin) == 0;
}

NetworkConfig& NetworkManager::getConfig() {
    return _config;
}

const NetworkConfig& NetworkManager::getConfig() const {
    return _config;
}

const char* NetworkManager::statusText() const {
    return _statusText;
}

String NetworkManager::modeText() const {
    if (_config.mode == NETWORK_MODE_STA) return "Station";
    if (_config.mode == NETWORK_MODE_STA_AP) return "Station + setup AP";
    return "Setup AP";
}

String NetworkManager::ipText() const {
    if (isConnected()) return stationIpText();
    if (_apActive) return apIpText();
    return "";
}

String NetworkManager::stationIpText() const {
#if NETWORK_ENABLE
    if (isConnected()) return WiFi.localIP().toString();
#endif
    return "";
}

String NetworkManager::apIpText() const {
#if NETWORK_ENABLE
    if (_apActive) return WiFi.softAPIP().toString();
#endif
    return "";
}

String NetworkManager::ssidText() const {
#if NETWORK_ENABLE
    if (isConnected()) return WiFi.SSID();
    if (_apActive) return String(_config.apSsid);
#endif
    return String(_config.ssid);
}

String NetworkManager::macText() const {
#if NETWORK_ENABLE
    return WiFi.macAddress();
#else
    return "";
#endif
}

int NetworkManager::rssi() const {
#if NETWORK_ENABLE
    if (isConnected()) return WiFi.RSSI();
#endif
    return 0;
}

uint8_t NetworkManager::connectedClientCount() const {
#if NETWORK_ENABLE
    if (_apActive) return WiFi.softAPgetStationNum();
#endif
    return 0;
}

void NetworkManager::setDefaults() {
    memset(&_config, 0, sizeof(_config));
    _config.magic = NETWORK_CONFIG_MAGIC;
    _config.version = NETWORK_CONFIG_VERSION;
    _config.enabled = true;
    _config.mode = NETWORK_MODE_AP;
    _config.dhcp = true;
    _config.apFallback = true;
    _config.apChannel = NETWORK_DEFAULT_AP_CHANNEL;
    _config.readOnlyMode = false;
    _config.webHomePage = WEB_HOME_DASHBOARD;
    _config.hiddenSsid = false;
    copyBounded(_config.hostname, sizeof(_config.hostname), NETWORK_DEFAULT_HOSTNAME);
    copyBounded(_config.apSsid, sizeof(_config.apSsid), NETWORK_DEFAULT_AP_SSID);
    copyBounded(_config.apPassword, sizeof(_config.apPassword), NETWORK_DEFAULT_AP_PASSWORD);
    copyBounded(_config.webPin, sizeof(_config.webPin), NETWORK_DEFAULT_WEB_PIN);
    bytesFromIp(_config.staticIp, 192, 168, 1, 60);
    bytesFromIp(_config.gateway, 192, 168, 1, 1);
    bytesFromIp(_config.subnet, 255, 255, 255, 0);
    bytesFromIp(_config.dns, 1, 1, 1, 1);
    setStatus("Not started");
}

void NetworkManager::load() {
    _loaded = true;
    if (!LittleFS.exists(NETWORK_CONFIG_FILE)) {
        setDefaults();
        return;
    }

    File f = LittleFS.open(NETWORK_CONFIG_FILE, "r");
    if (!f) {
        setDefaults();
        return;
    }

    NetworkConfig loaded;
    memset(&loaded, 0, sizeof(loaded));
    size_t readSize = f.read((uint8_t*)&loaded, sizeof(NetworkConfig));
    f.close();

    if (loaded.magic != NETWORK_CONFIG_MAGIC) {
        setDefaults();
        return;
    }

    bool migrated = false;
    if (loaded.version == 1 && readSize >= offsetof(NetworkConfig, readOnlyMode)) {
        loaded.readOnlyMode = false;
        copyBounded(loaded.webPin, sizeof(loaded.webPin), NETWORK_DEFAULT_WEB_PIN);
        loaded.webHomePage = WEB_HOME_DASHBOARD;
        loaded.version = NETWORK_CONFIG_VERSION;
        migrated = true;
    } else if (loaded.version == 2 && readSize >= offsetof(NetworkConfig, webHomePage)) {
        loaded.webHomePage = WEB_HOME_DASHBOARD;
        loaded.version = NETWORK_CONFIG_VERSION;
        migrated = true;
    } else if (loaded.version == 3 && readSize >= offsetof(NetworkConfig, hiddenSsid)) {
        loaded.hiddenSsid = false;
        loaded.version = NETWORK_CONFIG_VERSION;
        migrated = true;
    } else if (loaded.version != NETWORK_CONFIG_VERSION || readSize != sizeof(NetworkConfig)) {
        setDefaults();
        return;
    }

    _config = loaded;
    _config.hostname[NETWORK_HOSTNAME_MAX] = 0;
    _config.ssid[NETWORK_SSID_MAX] = 0;
    _config.password[NETWORK_PASSWORD_MAX] = 0;
    _config.apSsid[NETWORK_SSID_MAX] = 0;
    _config.apPassword[NETWORK_PASSWORD_MAX] = 0;
    _config.webPin[NETWORK_WEB_PIN_MAX] = 0;

    if (_config.mode > NETWORK_MODE_STA_AP) _config.mode = NETWORK_MODE_AP;
    if (_config.apChannel < 1 || _config.apChannel > 13) _config.apChannel = NETWORK_DEFAULT_AP_CHANNEL;
    if (_config.webHomePage >= WEB_HOME_PAGE_COUNT) _config.webHomePage = WEB_HOME_DASHBOARD;
    if (_config.hostname[0] == 0) copyBounded(_config.hostname, sizeof(_config.hostname), NETWORK_DEFAULT_HOSTNAME);
    if (_config.apSsid[0] == 0) copyBounded(_config.apSsid, sizeof(_config.apSsid), NETWORK_DEFAULT_AP_SSID);
    if (_config.webPin[0] == 0) copyBounded(_config.webPin, sizeof(_config.webPin), NETWORK_DEFAULT_WEB_PIN);
    if (migrated) save();
}

void NetworkManager::start() {
#if NETWORK_ENABLE
    if (!_config.enabled) {
        setStatus("Disabled");
        return;
    }

    if (_config.mode == NETWORK_MODE_AP) {
        WiFi.mode(WIFI_AP);
        startAccessPoint();
        return;
    }

    if (_config.mode == NETWORK_MODE_STA_AP) {
        WiFi.mode(WIFI_AP_STA);
        startAccessPoint();
        startStation();
        return;
    }

    WiFi.mode(WIFI_STA);
    startStation();
#else
    setStatus("Unavailable");
#endif
}

void NetworkManager::startStation() {
#if NETWORK_ENABLE
    if (_config.ssid[0] == 0) {
        setStatus("No station SSID");
        if (_config.apFallback && !_apActive) {
            startAccessPoint();
        }
        return;
    }

    WiFi.setHostname(_config.hostname);
    if (!_config.dhcp) {
        WiFi.config(ipFromBytes(_config.staticIp), ipFromBytes(_config.dns),
                    ipFromBytes(_config.gateway), ipFromBytes(_config.subnet));
    }

    if (_config.password[0] == 0) {
        WiFi.beginNoBlock(_config.ssid);
    } else {
        WiFi.beginNoBlock(_config.ssid, _config.password);
    }

    _staStarted = true;
    _connectStartMs = millis();
    _lastReconnectMs = _connectStartMs;
    setStatus(_config.hiddenSsid ? "Connecting hidden" : "Connecting");
#endif
}

void NetworkManager::startAccessPoint() {
#if NETWORK_ENABLE
    IPAddress apIP(192, 168, 4, 1);
    WiFi.softAPConfig(apIP, apIP, IPAddress(255, 255, 255, 0));

    bool ok;
    if (_config.apPassword[0] == 0) {
        ok = WiFi.softAP(_config.apSsid, nullptr, _config.apChannel);
    } else {
        ok = WiFi.softAP(_config.apSsid, _config.apPassword, _config.apChannel);
    }

    _apActive = ok;
    if (ok) {
        _dnsServer.start(53, "*", apIP);
    }
    setStatus(ok ? "Setup AP active" : "AP start failed");
#endif
}

void NetworkManager::stopAccessPoint() {
#if NETWORK_ENABLE
    if (_apActive) {
        _dnsServer.stop();
        WiFi.softAPdisconnect(false);
    }
#endif
    _apActive = false;
}

void NetworkManager::setStatus(const char* text) {
    copyBounded(_statusText, sizeof(_statusText), text);
}
