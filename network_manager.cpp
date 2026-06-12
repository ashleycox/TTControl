/*
 * TT Control, advanced sinusoidal control of multi-phase turntable motors
 * Created by Ashley Cox at The Blind Man's Workshop
 * https://theblindmansworkshop.com
 * No part of this code may be used or reproduced for commercial purposes without written permission and contractual agreement
 * All external libraries and frameworks are the property of their respective authors and governed by their respective licenses
 */

#include "network_manager.h"
#include "globals.h"
#include "settings.h"
#include <LittleFS.h>
#include <stddef.h>

static const char* NETWORK_CONFIG_FILE = "/network.bin";
static const uint32_t NETWORK_CONFIG_BLOB_MAGIC = 0x54544E43UL; // "TTNC"
static const uint16_t NETWORK_CONFIG_BLOB_VERSION = 1;

NetworkManager networkManager;

namespace {
struct NetworkConfigFileHeader {
    uint32_t magic;
    uint16_t formatVersion;
    uint16_t headerSize;
    uint32_t configVersion;
    uint32_t payloadSize;
    uint32_t crc32;
};

uint32_t networkCrc32(const uint8_t* data, size_t length) {
    uint32_t crc = 0xFFFFFFFFUL;
    while (length--) {
        crc ^= *data++;
        for (uint8_t i = 0; i < 8; i++) {
            crc = (crc >> 1) ^ (0xEDB88320UL & (-(int32_t)(crc & 1)));
        }
    }
    return ~crc;
}

bool makeNetworkSidecarPath(const char* path, const char* suffix, char* out, size_t outSize) {
    if (!path || !suffix || !out || outSize == 0) return false;
    int written = snprintf(out, outSize, "%s%s", path, suffix);
    return written > 0 && (size_t)written < outSize;
}

bool readWrappedNetworkConfig(const char* path, NetworkConfig& target, size_t& readSize) {
    File f = LittleFS.open(path, "r");
    if (!f) return false;

    NetworkConfigFileHeader header;
    if (f.read((uint8_t*)&header, sizeof(header)) != sizeof(header)) {
        f.close();
        return false;
    }

    if (header.magic != NETWORK_CONFIG_BLOB_MAGIC ||
        header.formatVersion != NETWORK_CONFIG_BLOB_VERSION ||
        header.headerSize != sizeof(NetworkConfigFileHeader) ||
        header.configVersion > NETWORK_CONFIG_VERSION ||
        header.payloadSize == 0 ||
        header.payloadSize > sizeof(NetworkConfig)) {
        f.close();
        return false;
    }

    memset(&target, 0, sizeof(target));
    readSize = f.read((uint8_t*)&target, header.payloadSize);
    f.close();

    if (readSize != header.payloadSize) return false;
    if (networkCrc32((const uint8_t*)&target, readSize) != header.crc32) return false;
    return true;
}

bool readRawNetworkConfig(const char* path, NetworkConfig& target, size_t& readSize) {
    File f = LittleFS.open(path, "r");
    if (!f) return false;

    memset(&target, 0, sizeof(target));
    readSize = f.read((uint8_t*)&target, sizeof(NetworkConfig));
    f.close();

    return readSize >= offsetof(NetworkConfig, readOnlyMode) &&
           target.magic == NETWORK_CONFIG_MAGIC;
}

bool readNetworkConfigBlob(const char* path, NetworkConfig& target, size_t& readSize, bool& wrapped) {
    wrapped = true;
    if (readWrappedNetworkConfig(path, target, readSize)) return true;

    wrapped = false;
    return readRawNetworkConfig(path, target, readSize);
}

bool loadNetworkConfigBlob(const char* path, NetworkConfig& target, size_t& readSize, bool& wrapped) {
    if (readNetworkConfigBlob(path, target, readSize, wrapped)) return true;

    char backupPath[40];
    if (makeNetworkSidecarPath(path, ".bak", backupPath, sizeof(backupPath))) {
        return readNetworkConfigBlob(backupPath, target, readSize, wrapped);
    }
    return false;
}

bool writeNetworkConfigBlob(const char* path, const NetworkConfig& source) {
    char tmpPath[40];
    char backupPath[40];
    if (!makeNetworkSidecarPath(path, ".tmp", tmpPath, sizeof(tmpPath))) return false;
    if (!makeNetworkSidecarPath(path, ".bak", backupPath, sizeof(backupPath))) return false;

    LittleFS.remove(tmpPath);

    File f = LittleFS.open(tmpPath, "w");
    if (!f) return false;

    NetworkConfigFileHeader header;
    header.magic = NETWORK_CONFIG_BLOB_MAGIC;
    header.formatVersion = NETWORK_CONFIG_BLOB_VERSION;
    header.headerSize = sizeof(NetworkConfigFileHeader);
    header.configVersion = NETWORK_CONFIG_VERSION;
    header.payloadSize = sizeof(NetworkConfig);
    header.crc32 = networkCrc32((const uint8_t*)&source, sizeof(source));

    bool ok = f.write((const uint8_t*)&header, sizeof(header)) == sizeof(header);
    ok = ok && f.write((const uint8_t*)&source, sizeof(source)) == sizeof(source);
    f.close();

    if (!ok) {
        LittleFS.remove(tmpPath);
        return false;
    }

    LittleFS.remove(backupPath);
    bool hadOriginal = LittleFS.exists(path);
    if (hadOriginal && !LittleFS.rename(path, backupPath)) {
        LittleFS.remove(tmpPath);
        return false;
    }

    if (!LittleFS.rename(tmpPath, path)) {
        if (hadOriginal) LittleFS.rename(backupPath, path);
        LittleFS.remove(tmpPath);
        return false;
    }

    if (hadOriginal) LittleFS.remove(backupPath);
    return true;
}
}

static void copyBounded(char* target, size_t targetSize, const char* source) {
    // All config strings are fixed-width fields in NetworkConfig. Always leave a terminator so older/corrupt files cannot leak into adjacent fields.
    if (!target || targetSize == 0) return;
    if (!source) source = "";
    strncpy(target, source, targetSize - 1);
    target[targetSize - 1] = 0;
}

#if NETWORK_ENABLE
static IPAddress ipFromBytes(const uint8_t bytes[4]) {
    // Store IPs as bytes in flash to keep NetworkConfig independent of Wi-Fi library object layout.
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
      _ecoStandbySuspended(false),
      _deviceUnlocked(true),
      _connectStartMs(0),
      _lastReconnectMs(0) {
    setDefaults();
}

void NetworkManager::begin() {
    // Load config first. The network may stay fully stopped in Eco standby mode even when networking is enabled.
    load();
#if NETWORK_ENABLE
    if (_config.enabled) {
        if (shouldSuspendForStandby()) {
            _ecoStandbySuspended = true;
            setStatus("Eco standby");
        } else {
            start();
        }
    } else {
        setStatus("Disabled");
    }
#else
    setStatus("Unavailable");
#endif
}

void NetworkManager::update() {
#if NETWORK_ENABLE
    // Eco standby turns Wi-Fi off while the motor controller is in standby.
    if (_config.enabled && shouldSuspendForStandby()) {
        if (!_ecoStandbySuspended || _apActive || _staStarted) {
            stop();
            _ecoStandbySuspended = true;
        }
        setStatus("Eco standby");
        return;
    }

    if (_ecoStandbySuspended) {
        _ecoStandbySuspended = false;
        if (_config.enabled) {
            start();
        } else {
            setStatus("Disabled");
        }
    }

    if (!_config.enabled) return;

    if (_apActive) {
        // Captive DNS only runs while the setup AP is active.
        _dnsServer.processNextRequest();
    }

    uint32_t now = millis();
    bool wantsStation = _config.mode == NETWORK_MODE_STA || _config.mode == NETWORK_MODE_STA_AP;
    if (wantsStation && _staStarted && WiFi.status() == WL_CONNECTED) {
        setStatus(_apActive ? "Connected + setup AP" : "Connected");
        return;
    }

    if (wantsStation && _staStarted && WiFi.status() != WL_CONNECTED) {
        // If station connection takes too long, keep the device reachable through the setup AP when fallback is enabled.
        if (_config.apFallback && !_apActive && now - _connectStartMs > 15000) {
            startAccessPoint();
        }

        // Retry station connection periodically without blocking Core 0.
        if (now - _lastReconnectMs > 30000) {
            startStation();
        }
    }
#endif
}

void NetworkManager::restart() {
    // Used after config changes from serial/web. Clear Eco standby state so the next start decision reflects the new config.
    stop();
    _ecoStandbySuspended = false;
#if NETWORK_ENABLE
    if (_config.enabled) {
        if (shouldSuspendForStandby()) {
            _ecoStandbySuspended = true;
            setStatus("Eco standby");
        } else {
            start();
        }
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
    // Normalize access-control defaults before writing. A locked/read-only web UI must always have a PIN.
    _config.magic = NETWORK_CONFIG_MAGIC;
    _config.version = NETWORK_CONFIG_VERSION;
    if (_config.webPin[0] != 0) {
        size_t pinLen = strlen(_config.webPin);
        if (pinLen < 4 || pinLen > NETWORK_WEB_PIN_MAX) {
            copyBounded(_config.webPin, sizeof(_config.webPin), NETWORK_DEFAULT_WEB_PIN);
        }
    }
    if ((_config.readOnlyMode || _config.deviceLockEnabled) && _config.webPin[0] == 0) {
        copyBounded(_config.webPin, sizeof(_config.webPin), NETWORK_DEFAULT_WEB_PIN);
    }

    writeNetworkConfigBlob(NETWORK_CONFIG_FILE, _config);
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

bool NetworkManager::isEcoStandbySuspended() const {
    return _ecoStandbySuspended;
}

bool NetworkManager::hasStationCredentials() const {
    return _config.ssid[0] != 0;
}

bool NetworkManager::isDeviceLockEnabled() const {
    return _config.deviceLockEnabled && _config.webPin[0] != 0;
}

bool NetworkManager::isDeviceLocked() const {
    return isDeviceLockEnabled() && !_deviceUnlocked;
}

bool NetworkManager::isWebAccessLocked() const {
    // readOnlyMode protects write APIs; deviceLockEnabled also locks the local control surface until the PIN is entered.
    return (_config.readOnlyMode || _config.deviceLockEnabled) && _config.webPin[0] != 0;
}

bool NetworkManager::verifyWebPin(const char* pin) const {
    if (!isWebAccessLocked()) return true;
    if (!_config.webPin[0]) return true;
    if (!pin) return false;
    if (strlen(pin) > NETWORK_WEB_PIN_MAX) return false;
    return strcmp(pin, _config.webPin) == 0;
}

bool NetworkManager::setWebPin(const char* pin) {
    if (!pin) return false;
    size_t pinLen = strlen(pin);
    if (pinLen < 4 || pinLen > NETWORK_WEB_PIN_MAX) return false;
    copyBounded(_config.webPin, sizeof(_config.webPin), pin);
    if (_config.deviceLockEnabled) _deviceUnlocked = false;
    return true;
}

void NetworkManager::setDeviceLockEnabled(bool enabled) {
    // Enabling the device lock immediately locks access and installs a default PIN if the user has not set one.
    _config.deviceLockEnabled = enabled;
    if (enabled && _config.webPin[0] == 0) {
        copyBounded(_config.webPin, sizeof(_config.webPin), NETWORK_DEFAULT_WEB_PIN);
    }
    _deviceUnlocked = !enabled;
}

bool NetworkManager::unlockDevice(const char* pin) {
    if (!isDeviceLockEnabled()) {
        _deviceUnlocked = true;
        return true;
    }
    if (!verifyWebPin(pin)) return false;
    _deviceUnlocked = true;
    return true;
}

void NetworkManager::lockDevice() {
    if (isDeviceLockEnabled()) _deviceUnlocked = false;
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
    // Default to setup AP mode so a fresh board is reachable without existing station credentials.
    memset(&_config, 0, sizeof(_config));
    _config.magic = NETWORK_CONFIG_MAGIC;
    _config.version = NETWORK_CONFIG_VERSION;
    _config.enabled = true;
    _config.mode = NETWORK_MODE_AP;
    _config.dhcp = true;
    _config.apFallback = true;
    _config.apChannel = NETWORK_DEFAULT_AP_CHANNEL;
    _config.readOnlyMode = true;
    _config.webHomePage = WEB_HOME_DASHBOARD;
    _config.hiddenSsid = false;
    _config.standbyMode = NETWORK_STANDBY_NETWORK;
    _config.deviceLockEnabled = true;
    _deviceUnlocked = false;
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
    NetworkConfig loaded;
    size_t readSize = 0;
    bool wrapped = false;
    if (!loadNetworkConfigBlob(NETWORK_CONFIG_FILE, loaded, readSize, wrapped)) {
        setDefaults();
        return;
    }

    if (loaded.magic != NETWORK_CONFIG_MAGIC) {
        setDefaults();
        return;
    }

    // Older versions were prefixes of the current struct. Migrate by filling the fields that did not exist in that version, then rewrite the file.
    bool migrated = !wrapped;
    if (loaded.version == 1 && readSize >= offsetof(NetworkConfig, readOnlyMode)) {
        loaded.readOnlyMode = false;
        copyBounded(loaded.webPin, sizeof(loaded.webPin), NETWORK_DEFAULT_WEB_PIN);
        loaded.webHomePage = WEB_HOME_DASHBOARD;
        loaded.hiddenSsid = false;
        loaded.standbyMode = NETWORK_STANDBY_NETWORK;
        loaded.deviceLockEnabled = false;
        loaded.version = NETWORK_CONFIG_VERSION;
        migrated = true;
    } else if (loaded.version == 2 && readSize >= offsetof(NetworkConfig, webHomePage)) {
        loaded.webHomePage = WEB_HOME_DASHBOARD;
        loaded.hiddenSsid = false;
        loaded.standbyMode = NETWORK_STANDBY_NETWORK;
        loaded.deviceLockEnabled = false;
        loaded.version = NETWORK_CONFIG_VERSION;
        migrated = true;
    } else if (loaded.version == 3 && readSize >= offsetof(NetworkConfig, hiddenSsid)) {
        loaded.hiddenSsid = false;
        loaded.standbyMode = NETWORK_STANDBY_NETWORK;
        loaded.deviceLockEnabled = false;
        loaded.version = NETWORK_CONFIG_VERSION;
        migrated = true;
    } else if (loaded.version == 4 && readSize >= offsetof(NetworkConfig, standbyMode)) {
        loaded.deviceLockEnabled = false;
        loaded.version = NETWORK_CONFIG_VERSION;
        migrated = true;
    } else if (loaded.version == NETWORK_CONFIG_VERSION && readSize >= offsetof(NetworkConfig, deviceLockEnabled)) {
        if (readSize < offsetof(NetworkConfig, deviceLockEnabled) + sizeof(loaded.deviceLockEnabled)) {
            loaded.deviceLockEnabled = false;
            migrated = true;
        }
    } else if (loaded.version != NETWORK_CONFIG_VERSION || readSize != sizeof(NetworkConfig)) {
        setDefaults();
        return;
    }

    _config = loaded;
    // Force terminators on every persisted string before any UI/API reads it.
    _config.hostname[NETWORK_HOSTNAME_MAX] = 0;
    _config.ssid[NETWORK_SSID_MAX] = 0;
    _config.password[NETWORK_PASSWORD_MAX] = 0;
    _config.apSsid[NETWORK_SSID_MAX] = 0;
    _config.apPassword[NETWORK_PASSWORD_MAX] = 0;
    _config.webPin[NETWORK_WEB_PIN_MAX] = 0;

    if (_config.mode > NETWORK_MODE_STA_AP) _config.mode = NETWORK_MODE_AP;
    if (_config.standbyMode > NETWORK_STANDBY_ECO) _config.standbyMode = NETWORK_STANDBY_NETWORK;
    if (_config.apChannel < 1 || _config.apChannel > 13) _config.apChannel = NETWORK_DEFAULT_AP_CHANNEL;
    if (_config.webHomePage >= WEB_HOME_PAGE_COUNT) _config.webHomePage = WEB_HOME_DASHBOARD;
    if (_config.hostname[0] == 0) copyBounded(_config.hostname, sizeof(_config.hostname), NETWORK_DEFAULT_HOSTNAME);
    if (_config.apSsid[0] == 0) copyBounded(_config.apSsid, sizeof(_config.apSsid), NETWORK_DEFAULT_AP_SSID);
    if (_config.webPin[0] == 0) copyBounded(_config.webPin, sizeof(_config.webPin), NETWORK_DEFAULT_WEB_PIN);
    _deviceUnlocked = !_config.deviceLockEnabled;
    if (migrated) save();
}

void NetworkManager::start() {
#if NETWORK_ENABLE
    if (!_config.enabled) {
        setStatus("Disabled");
        return;
    }

    if (_config.mode == NETWORK_MODE_AP) {
        // AP-only mode is the setup/configuration fallback.
        WiFi.mode(WIFI_AP);
        startAccessPoint();
        return;
    }

    if (_config.mode == NETWORK_MODE_STA_AP) {
        // Keep setup AP available while also attempting station connection.
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
        // No station credentials: optionally expose the setup AP instead of leaving the device unreachable.
        setStatus("No station SSID");
        if (_config.apFallback && !_apActive) {
            startAccessPoint();
        }
        return;
    }

    WiFi.setHostname(_config.hostname);
    if (!_config.dhcp) {
        // Arduino-Pico WiFi.config order is local IP, DNS, gateway, subnet.
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
    // The setup AP always uses a fixed private address so captive DNS can point all names to the web UI.
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

bool NetworkManager::shouldSuspendForStandby() const {
    // Eco standby only cares about the high-level motor state mirror; it avoids reaching into MotorController from the network module.
    return _config.standbyMode == NETWORK_STANDBY_ECO &&
           currentMotorState == STATE_STANDBY;
}

void NetworkManager::setStatus(const char* text) {
    copyBounded(_statusText, sizeof(_statusText), text);
}
