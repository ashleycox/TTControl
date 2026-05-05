/*
 * TT Control, advanced sinusoidal control of multi-phase turntable motors
 * Created by Ashley Cox at The Blind Man’s Workshop
 * https://theblindmansworkshop.com
 * No part of this code may be used or reproduced for commercial purposes without written permission and contractual agreement
 * All external libraries and frameworks are the property of their respective authors and governed by their respective licenses
 */

#include "serial_cmd.h"
#include "ui.h"
#include "error_handler.h"
#include "hal.h"
#include "amp_monitor.h"
#include "network_manager.h"

extern UserInterface ui;

// --- CLI Registry ---
struct SettingItem {
    String name;
    std::function<String()> get;
    std::function<void(String)> set;
};

std::vector<SettingItem> registry;
bool cliInitialized = false;

static const char* speedName(SpeedMode speed) {
    if (speed == SPEED_33) return "33 RPM";
    if (speed == SPEED_45) return "45 RPM";
    return "78 RPM";
}

static const char* motorStateName() {
    if (motor.isRelayTestMode()) return "RELAY TEST";
    switch (motor.getState()) {
        case STATE_STANDBY: return "STANDBY";
        case STATE_STOPPED: return "STOPPED";
        case STATE_STARTING: return "STARTING";
        case STATE_RUNNING: return motor.isSpeedRamping() ? "RAMPING" : "RUNNING";
        case STATE_STOPPING: return "BRAKING";
    }
    return "UNKNOWN";
}

static const char* brakeModeName(uint8_t mode) {
    switch (mode) {
        case BRAKE_OFF: return "Off";
        case BRAKE_PULSE: return "Pulse";
        case BRAKE_RAMP: return "Ramp";
        case BRAKE_SOFT_STOP: return "Soft Stop";
    }
    return "Invalid";
}

static const char* filterName(uint8_t filter) {
    switch (filter) {
        case FILTER_NONE: return "None";
        case FILTER_IIR: return "IIR";
        case FILTER_FIR: return "FIR";
    }
    return "Invalid";
}

static const char* rampTypeName(uint8_t ramp) {
    return ramp == RAMP_SCURVE ? "S-Curve" : "Linear";
}

static void printPresetList();
static void printSettingsDump();
static void handlePresetCommand(const String& input);
static void handleRelayTestCommand(const String& input);
static void handleWifiCommand(const String& input);
static void updateWifiSerialTasks();

static int clampInt(int value, int minValue, int maxValue) {
    if (value < minValue) return minValue;
    if (value > maxValue) return maxValue;
    return value;
}

static float clampFloat(float value, float minValue, float maxValue) {
    if (value < minValue) return minValue;
    if (value > maxValue) return maxValue;
    return value;
}

static const char* onOffText(bool value) {
    return value ? "ON" : "OFF";
}

static const char* yesNoText(bool value) {
    return value ? "YES" : "NO";
}

static const char* networkModeName(uint8_t mode) {
    if (mode == NETWORK_MODE_STA) return "Station";
    if (mode == NETWORK_MODE_STA_AP) return "Station + setup AP";
    return "Setup AP";
}

static String ipBytesToString(const uint8_t bytes[4]) {
    String out;
    out.reserve(15);
    out += String(bytes[0]);
    out += ".";
    out += String(bytes[1]);
    out += ".";
    out += String(bytes[2]);
    out += ".";
    out += String(bytes[3]);
    return out;
}

static bool parseBoolValue(String value, bool& out) {
    value.trim();
    value.toLowerCase();
    if (value == "1" || value == "on" || value == "yes" || value == "true" || value == "y") {
        out = true;
        return true;
    }
    if (value == "0" || value == "off" || value == "no" || value == "false" || value == "n") {
        out = false;
        return true;
    }
    return false;
}

static bool parseNetworkMode(String value, uint8_t& mode) {
    value.trim();
    value.toLowerCase();
    value.replace("-", "_");
    if (value == "0" || value == "ap" || value == "setup" || value == "setup_ap") {
        mode = NETWORK_MODE_AP;
        return true;
    }
    if (value == "1" || value == "sta" || value == "station" || value == "client") {
        mode = NETWORK_MODE_STA;
        return true;
    }
    if (value == "2" || value == "sta_ap" || value == "station_ap" || value == "station_setup" ||
        value == "both") {
        mode = NETWORK_MODE_STA_AP;
        return true;
    }
    return false;
}

static bool parseIpv4(const String& value, uint8_t out[4]) {
    int start = 0;
    for (int part = 0; part < 4; part++) {
        int dot = value.indexOf('.', start);
        if (part < 3 && dot < 0) return false;
        if (part == 3 && dot >= 0) return false;

        String piece = part < 3 ? value.substring(start, dot) : value.substring(start);
        if (piece.length() == 0 || piece.length() > 3) return false;
        for (size_t i = 0; i < piece.length(); i++) {
            char c = piece.charAt(i);
            if (c < '0' || c > '9') return false;
        }

        int parsed = piece.toInt();
        if (parsed < 0 || parsed > 255) return false;
        out[part] = (uint8_t)parsed;
        start = dot + 1;
    }
    return true;
}

static bool copyConfigString(char* target, size_t targetSize, const String& value, const char* label) {
    if (!target || targetSize == 0) return false;
    if (value.length() >= targetSize) {
        Serial.print(label);
        Serial.print(" is too long. Maximum characters: ");
        Serial.println(targetSize - 1);
        return false;
    }

    strncpy(target, value.c_str(), targetSize - 1);
    target[targetSize - 1] = 0;
    return true;
}

static void parseCommandArgs(const String& text, std::vector<String>& args) {
    args.clear();

    String current;
    bool tokenActive = false;
    bool inQuote = false;
    char quoteChar = 0;

    for (size_t i = 0; i < text.length(); i++) {
        char c = text.charAt(i);

        if (inQuote) {
            if (c == quoteChar) {
                inQuote = false;
            } else if (c == '\\' && i + 1 < text.length()) {
                i++;
                current += text.charAt(i);
                tokenActive = true;
            } else {
                current += c;
                tokenActive = true;
            }
            continue;
        }

        if (c == '"' || c == '\'') {
            inQuote = true;
            quoteChar = c;
            tokenActive = true;
        } else if (c == ' ' || c == '\t') {
            if (tokenActive) {
                args.push_back(current);
                current = "";
                tokenActive = false;
            }
        } else {
            current += c;
            tokenActive = true;
        }
    }

    if (tokenActive) {
        args.push_back(current);
    }
}

enum WifiWizardStep : uint8_t {
    WIFI_WIZARD_IDLE = 0,
    WIFI_WIZARD_MODE,
    WIFI_WIZARD_SCAN_CHOICE,
    WIFI_WIZARD_SCAN_WAIT,
    WIFI_WIZARD_SSID,
    WIFI_WIZARD_PASSWORD,
    WIFI_WIZARD_DHCP,
    WIFI_WIZARD_STATIC_IP,
    WIFI_WIZARD_GATEWAY,
    WIFI_WIZARD_SUBNET,
    WIFI_WIZARD_DNS,
    WIFI_WIZARD_HOSTNAME,
    WIFI_WIZARD_AP_FALLBACK,
    WIFI_WIZARD_AP_SSID,
    WIFI_WIZARD_AP_PASSWORD,
    WIFI_WIZARD_AP_CHANNEL,
    WIFI_WIZARD_CONFIRM
};

static const uint8_t WIFI_WIZARD_MAX_SCAN_RESULTS = 12;
static bool wifiWizardActive = false;
static WifiWizardStep wifiWizardStep = WIFI_WIZARD_IDLE;
static NetworkConfig wifiWizardConfig;
static String wifiWizardScanSsids[WIFI_WIZARD_MAX_SCAN_RESULTS];
static uint8_t wifiWizardScanCount = 0;
static bool wifiScanActive = false;
static bool wifiScanForWizard = false;
static uint32_t wifiScanStartedMs = 0;

static bool wizardNeedsStation() {
    return wifiWizardConfig.mode == NETWORK_MODE_STA || wifiWizardConfig.mode == NETWORK_MODE_STA_AP;
}

static bool wizardNeedsApConfig() {
    return wifiWizardConfig.mode == NETWORK_MODE_AP ||
           wifiWizardConfig.mode == NETWORK_MODE_STA_AP ||
           (wifiWizardConfig.mode == NETWORK_MODE_STA && wifiWizardConfig.apFallback);
}

static void printWifiConfigSummary(const NetworkConfig& cfg) {
    Serial.print("Enabled: ");
    Serial.println(onOffText(cfg.enabled));
    Serial.print("Mode: ");
    Serial.println(networkModeName(cfg.mode));
    Serial.print("Hostname: ");
    Serial.println(cfg.hostname);
    Serial.print("Station SSID: ");
    Serial.println(cfg.ssid[0] ? cfg.ssid : "(not set)");
    Serial.print("Station password: ");
    Serial.println(cfg.password[0] ? "(saved)" : "(open/none)");
    Serial.print("DHCP: ");
    Serial.println(onOffText(cfg.dhcp));
    if (!cfg.dhcp) {
        Serial.print("Static IP: ");
        Serial.println(ipBytesToString(cfg.staticIp));
        Serial.print("Gateway: ");
        Serial.println(ipBytesToString(cfg.gateway));
        Serial.print("Subnet: ");
        Serial.println(ipBytesToString(cfg.subnet));
        Serial.print("DNS: ");
        Serial.println(ipBytesToString(cfg.dns));
    }
    Serial.print("AP fallback: ");
    Serial.println(onOffText(cfg.apFallback));
    Serial.print("Setup AP SSID: ");
    Serial.println(cfg.apSsid[0] ? cfg.apSsid : "(not set)");
    Serial.print("Setup AP password: ");
    Serial.println(cfg.apPassword[0] ? "(saved)" : "(open)");
    Serial.print("Setup AP channel: ");
    Serial.println(cfg.apChannel);
}

static void promptWifiWizardStep();

static void setWifiWizardStep(WifiWizardStep step) {
    wifiWizardStep = step;
    promptWifiWizardStep();
}

static void setWizardStepAfterStationConfig() {
    if (wizardNeedsStation()) {
        setWifiWizardStep(WIFI_WIZARD_SCAN_CHOICE);
    } else {
        setWifiWizardStep(WIFI_WIZARD_HOSTNAME);
    }
}

static void setWizardStepAfterDhcpChoice() {
    if (wifiWizardConfig.dhcp) {
        setWifiWizardStep(WIFI_WIZARD_HOSTNAME);
    } else {
        setWifiWizardStep(WIFI_WIZARD_STATIC_IP);
    }
}

static void setWizardStepAfterHostname() {
    if (wifiWizardConfig.mode == NETWORK_MODE_STA) {
        setWifiWizardStep(WIFI_WIZARD_AP_FALLBACK);
    } else if (wizardNeedsApConfig()) {
        setWifiWizardStep(WIFI_WIZARD_AP_SSID);
    } else {
        setWifiWizardStep(WIFI_WIZARD_CONFIRM);
    }
}

static void setWizardStepAfterFallback() {
    if (wizardNeedsApConfig()) {
        setWifiWizardStep(WIFI_WIZARD_AP_SSID);
    } else {
        setWifiWizardStep(WIFI_WIZARD_CONFIRM);
    }
}

static void promptWifiWizardStep() {
    if (!wifiWizardActive) return;

    switch (wifiWizardStep) {
        case WIFI_WIZARD_MODE:
            Serial.println();
            Serial.println("Wi-Fi setup wizard. Type 'cancel' at any prompt to exit.");
            Serial.println("Choose network mode:");
            Serial.println("  1 = Station + setup AP");
            Serial.println("  2 = Station only");
            Serial.println("  3 = Setup AP only");
            Serial.print("Mode [");
            Serial.print(networkModeName(wifiWizardConfig.mode));
            Serial.println("]:");
            break;
        case WIFI_WIZARD_SCAN_CHOICE:
            Serial.print("Scan for nearby Wi-Fi networks? [");
            Serial.print(wifiWizardScanCount ? "n" : "y");
            Serial.println("]:");
            break;
        case WIFI_WIZARD_SCAN_WAIT:
            Serial.println("Scanning. Results will print when ready.");
            break;
        case WIFI_WIZARD_SSID:
            if (wifiWizardScanCount > 0) {
                Serial.println("Enter a network number from the scan list, or type the SSID manually.");
            }
            Serial.print("Station SSID [");
            Serial.print(wifiWizardConfig.ssid[0] ? wifiWizardConfig.ssid : "required");
            Serial.println("]:");
            break;
        case WIFI_WIZARD_PASSWORD:
            Serial.println("Station password. Leave blank for an open network, or enter '-' to keep the saved password.");
            Serial.print("Password [");
            Serial.print(wifiWizardConfig.password[0] ? "saved" : "open");
            Serial.println("]:");
            break;
        case WIFI_WIZARD_DHCP:
            Serial.print("Use DHCP? [");
            Serial.print(wifiWizardConfig.dhcp ? "y" : "n");
            Serial.println("]:");
            break;
        case WIFI_WIZARD_STATIC_IP:
            Serial.print("Static IP [");
            Serial.print(ipBytesToString(wifiWizardConfig.staticIp));
            Serial.println("]:");
            break;
        case WIFI_WIZARD_GATEWAY:
            Serial.print("Gateway [");
            Serial.print(ipBytesToString(wifiWizardConfig.gateway));
            Serial.println("]:");
            break;
        case WIFI_WIZARD_SUBNET:
            Serial.print("Subnet [");
            Serial.print(ipBytesToString(wifiWizardConfig.subnet));
            Serial.println("]:");
            break;
        case WIFI_WIZARD_DNS:
            Serial.print("DNS [");
            Serial.print(ipBytesToString(wifiWizardConfig.dns));
            Serial.println("]:");
            break;
        case WIFI_WIZARD_HOSTNAME:
            Serial.print("Hostname [");
            Serial.print(wifiWizardConfig.hostname[0] ? wifiWizardConfig.hostname : NETWORK_DEFAULT_HOSTNAME);
            Serial.println("]:");
            break;
        case WIFI_WIZARD_AP_FALLBACK:
            Serial.print("Start setup AP if station connection fails? [");
            Serial.print(wifiWizardConfig.apFallback ? "y" : "n");
            Serial.println("]:");
            break;
        case WIFI_WIZARD_AP_SSID:
            Serial.print("Setup AP SSID [");
            Serial.print(wifiWizardConfig.apSsid[0] ? wifiWizardConfig.apSsid : NETWORK_DEFAULT_AP_SSID);
            Serial.println("]:");
            break;
        case WIFI_WIZARD_AP_PASSWORD:
            Serial.println("Setup AP password. Leave blank for an open setup AP, or enter '-' to keep the saved password.");
            Serial.print("Setup AP password [");
            Serial.print(wifiWizardConfig.apPassword[0] ? "saved" : "open");
            Serial.println("]:");
            break;
        case WIFI_WIZARD_AP_CHANNEL:
            Serial.print("Setup AP channel 1-13 [");
            Serial.print(wifiWizardConfig.apChannel);
            Serial.println("]:");
            break;
        case WIFI_WIZARD_CONFIRM:
            Serial.println();
            Serial.println("--- Wi-Fi Wizard Summary ---");
            printWifiConfigSummary(wifiWizardConfig);
            Serial.println("----------------------------");
            Serial.println("Save these settings and reconnect now? [y/n]:");
            break;
        default:
            break;
    }
}

static void startWifiWizard() {
    if (!networkManager.isAvailable()) {
        Serial.println("Network support is not enabled in this build.");
        return;
    }

    wifiWizardConfig = networkManager.getConfig();
    wifiWizardConfig.enabled = true;
    if (wifiWizardConfig.mode == NETWORK_MODE_AP && wifiWizardConfig.ssid[0] == 0) {
        wifiWizardConfig.mode = NETWORK_MODE_STA_AP;
    }
    if (wifiWizardConfig.mode > NETWORK_MODE_STA_AP) wifiWizardConfig.mode = NETWORK_MODE_STA_AP;
    if (wifiWizardConfig.apChannel < 1 || wifiWizardConfig.apChannel > 13) {
        wifiWizardConfig.apChannel = NETWORK_DEFAULT_AP_CHANNEL;
    }
    if (wifiWizardConfig.hostname[0] == 0) {
        copyConfigString(wifiWizardConfig.hostname, sizeof(wifiWizardConfig.hostname), NETWORK_DEFAULT_HOSTNAME, "Hostname");
    }
    if (wifiWizardConfig.apSsid[0] == 0) {
        copyConfigString(wifiWizardConfig.apSsid, sizeof(wifiWizardConfig.apSsid), NETWORK_DEFAULT_AP_SSID, "Setup AP SSID");
    }

    wifiWizardScanCount = 0;
    wifiWizardActive = true;
    setWifiWizardStep(WIFI_WIZARD_MODE);
}

static bool startWifiScan(bool forWizard) {
#if NETWORK_ENABLE
    if (!networkManager.isAvailable()) {
        Serial.println("Network support is not enabled in this build.");
        return false;
    }
    if (wifiScanActive) {
        Serial.println("A Wi-Fi scan is already in progress.");
        return false;
    }

    wifiScanForWizard = forWizard;
    wifiScanActive = true;
    wifiScanStartedMs = millis();
    wifiWizardScanCount = 0;
    WiFi.scanDelete();
    int result = WiFi.scanNetworks(true);
    if (result == -2) {
        wifiScanActive = false;
        Serial.println("Unable to start Wi-Fi scan.");
        return false;
    }

    Serial.println("Scanning for Wi-Fi networks...");
    return true;
#else
    (void)forWizard;
    Serial.println("Network support is not enabled in this build.");
    return false;
#endif
}

static void finishWifiScan(int count) {
#if NETWORK_ENABLE
    wifiScanActive = false;
    wifiWizardScanCount = 0;

    Serial.println("--- Wi-Fi Scan Results ---");
    if (count <= 0) {
        Serial.println("No networks found.");
    } else {
        for (int i = 0; i < count && wifiWizardScanCount < WIFI_WIZARD_MAX_SCAN_RESULTS; i++) {
            String ssid = WiFi.SSID(i);
            if (ssid.length() == 0) continue;

            bool duplicate = false;
            for (uint8_t j = 0; j < wifiWizardScanCount; j++) {
                if (wifiWizardScanSsids[j] == ssid) {
                    duplicate = true;
                    break;
                }
            }
            if (duplicate) continue;

            wifiWizardScanSsids[wifiWizardScanCount] = ssid;
            Serial.print(wifiWizardScanCount + 1);
            Serial.print(": ");
            Serial.print(ssid);
            Serial.print(" (");
            Serial.print(WiFi.RSSI(i));
            Serial.print(" dBm, channel ");
            Serial.print(WiFi.channel(i));
            Serial.println(")");
            wifiWizardScanCount++;
        }
        if (wifiWizardScanCount == 0) {
            Serial.println("No named networks found.");
        }
    }
    Serial.println("-------------------------");
    WiFi.scanDelete();

    if (wifiScanForWizard && wifiWizardActive && wifiWizardStep == WIFI_WIZARD_SCAN_WAIT) {
        setWifiWizardStep(WIFI_WIZARD_SSID);
    }
    wifiScanForWizard = false;
#else
    (void)count;
#endif
}

static void updateWifiSerialTasks() {
#if NETWORK_ENABLE
    if (!wifiScanActive) return;

    int count = WiFi.scanComplete();
    if (count >= 0) {
        finishWifiScan(count);
        return;
    }

    if (millis() - wifiScanStartedMs > 15000) {
        wifiScanActive = false;
        WiFi.scanDelete();
        Serial.println("Wi-Fi scan timed out.");
        if (wifiScanForWizard && wifiWizardActive && wifiWizardStep == WIFI_WIZARD_SCAN_WAIT) {
            wifiScanForWizard = false;
            setWifiWizardStep(WIFI_WIZARD_SSID);
        }
    }
#endif
}

static void handleWifiWizardInput(String input) {
    input.trim();

    String lowered = input;
    lowered.toLowerCase();
    if (lowered == "cancel" || lowered == "quit" || lowered == "exit") {
        wifiWizardActive = false;
        wifiWizardStep = WIFI_WIZARD_IDLE;
        Serial.println("Wi-Fi setup wizard cancelled.");
        return;
    }

    bool parsedBool = false;
    uint8_t parsedMode = NETWORK_MODE_STA_AP;
    uint8_t parsedIp[4];

    switch (wifiWizardStep) {
        case WIFI_WIZARD_MODE:
            if (input.length() > 0) {
                if (input == "1") {
                    parsedMode = NETWORK_MODE_STA_AP;
                } else if (input == "2") {
                    parsedMode = NETWORK_MODE_STA;
                } else if (input == "3") {
                    parsedMode = NETWORK_MODE_AP;
                } else if (!parseNetworkMode(input, parsedMode)) {
                    Serial.println("Enter 1, 2, 3, or a mode name.");
                    promptWifiWizardStep();
                    return;
                }
                wifiWizardConfig.mode = parsedMode;
            }
            wifiWizardConfig.enabled = true;
            if (wifiWizardConfig.mode == NETWORK_MODE_STA_AP) {
                wifiWizardConfig.apFallback = true;
            }
            setWizardStepAfterStationConfig();
            break;
        case WIFI_WIZARD_SCAN_CHOICE:
            if (input.length() == 0) {
                parsedBool = wifiWizardScanCount == 0;
            } else if (!parseBoolValue(input, parsedBool)) {
                Serial.println("Enter y or n.");
                promptWifiWizardStep();
                return;
            }
            if (parsedBool) {
                wifiWizardStep = WIFI_WIZARD_SCAN_WAIT;
                if (!startWifiScan(true)) {
                    setWifiWizardStep(WIFI_WIZARD_SSID);
                } else {
                    promptWifiWizardStep();
                }
            } else {
                setWifiWizardStep(WIFI_WIZARD_SSID);
            }
            break;
        case WIFI_WIZARD_SCAN_WAIT:
            Serial.println("Scan is still running. Type 'cancel' to exit, or wait for results.");
            break;
        case WIFI_WIZARD_SSID:
            if (input.length() == 0) {
                if (wifiWizardConfig.ssid[0] == 0) {
                    Serial.println("Station SSID is required for station mode.");
                    promptWifiWizardStep();
                    return;
                }
            } else {
                int selected = input.toInt();
                bool isNumber = selected > 0 && selected <= wifiWizardScanCount && input == String(selected);
                String ssid = isNumber ? wifiWizardScanSsids[selected - 1] : input;
                if (!copyConfigString(wifiWizardConfig.ssid, sizeof(wifiWizardConfig.ssid), ssid, "Station SSID")) {
                    promptWifiWizardStep();
                    return;
                }
            }
            setWifiWizardStep(WIFI_WIZARD_PASSWORD);
            break;
        case WIFI_WIZARD_PASSWORD:
            if (input != "-") {
                if (!copyConfigString(wifiWizardConfig.password, sizeof(wifiWizardConfig.password), input, "Station password")) {
                    promptWifiWizardStep();
                    return;
                }
            }
            setWifiWizardStep(WIFI_WIZARD_DHCP);
            break;
        case WIFI_WIZARD_DHCP:
            if (input.length() > 0) {
                if (!parseBoolValue(input, parsedBool)) {
                    Serial.println("Enter y or n.");
                    promptWifiWizardStep();
                    return;
                }
                wifiWizardConfig.dhcp = parsedBool;
            }
            setWizardStepAfterDhcpChoice();
            break;
        case WIFI_WIZARD_STATIC_IP:
            if (input.length() > 0) {
                if (!parseIpv4(input, parsedIp)) {
                    Serial.println("Enter a valid IPv4 address.");
                    promptWifiWizardStep();
                    return;
                }
                memcpy(wifiWizardConfig.staticIp, parsedIp, sizeof(wifiWizardConfig.staticIp));
            }
            setWifiWizardStep(WIFI_WIZARD_GATEWAY);
            break;
        case WIFI_WIZARD_GATEWAY:
            if (input.length() > 0) {
                if (!parseIpv4(input, parsedIp)) {
                    Serial.println("Enter a valid IPv4 address.");
                    promptWifiWizardStep();
                    return;
                }
                memcpy(wifiWizardConfig.gateway, parsedIp, sizeof(wifiWizardConfig.gateway));
            }
            setWifiWizardStep(WIFI_WIZARD_SUBNET);
            break;
        case WIFI_WIZARD_SUBNET:
            if (input.length() > 0) {
                if (!parseIpv4(input, parsedIp)) {
                    Serial.println("Enter a valid IPv4 address.");
                    promptWifiWizardStep();
                    return;
                }
                memcpy(wifiWizardConfig.subnet, parsedIp, sizeof(wifiWizardConfig.subnet));
            }
            setWifiWizardStep(WIFI_WIZARD_DNS);
            break;
        case WIFI_WIZARD_DNS:
            if (input.length() > 0) {
                if (!parseIpv4(input, parsedIp)) {
                    Serial.println("Enter a valid IPv4 address.");
                    promptWifiWizardStep();
                    return;
                }
                memcpy(wifiWizardConfig.dns, parsedIp, sizeof(wifiWizardConfig.dns));
            }
            setWifiWizardStep(WIFI_WIZARD_HOSTNAME);
            break;
        case WIFI_WIZARD_HOSTNAME:
            if (input.length() > 0) {
                if (!copyConfigString(wifiWizardConfig.hostname, sizeof(wifiWizardConfig.hostname), input, "Hostname")) {
                    promptWifiWizardStep();
                    return;
                }
            } else if (wifiWizardConfig.hostname[0] == 0) {
                copyConfigString(wifiWizardConfig.hostname, sizeof(wifiWizardConfig.hostname), NETWORK_DEFAULT_HOSTNAME, "Hostname");
            }
            setWizardStepAfterHostname();
            break;
        case WIFI_WIZARD_AP_FALLBACK:
            if (input.length() > 0) {
                if (!parseBoolValue(input, parsedBool)) {
                    Serial.println("Enter y or n.");
                    promptWifiWizardStep();
                    return;
                }
                wifiWizardConfig.apFallback = parsedBool;
            }
            setWizardStepAfterFallback();
            break;
        case WIFI_WIZARD_AP_SSID:
            if (input.length() > 0) {
                if (!copyConfigString(wifiWizardConfig.apSsid, sizeof(wifiWizardConfig.apSsid), input, "Setup AP SSID")) {
                    promptWifiWizardStep();
                    return;
                }
            } else if (wifiWizardConfig.apSsid[0] == 0) {
                copyConfigString(wifiWizardConfig.apSsid, sizeof(wifiWizardConfig.apSsid), NETWORK_DEFAULT_AP_SSID, "Setup AP SSID");
            }
            setWifiWizardStep(WIFI_WIZARD_AP_PASSWORD);
            break;
        case WIFI_WIZARD_AP_PASSWORD:
            if (input != "-") {
                if (input.length() > 0 && input.length() < 8) {
                    Serial.println("Setup AP password must be at least 8 characters, or blank for open setup AP.");
                    promptWifiWizardStep();
                    return;
                }
                if (!copyConfigString(wifiWizardConfig.apPassword, sizeof(wifiWizardConfig.apPassword), input, "Setup AP password")) {
                    promptWifiWizardStep();
                    return;
                }
            }
            setWifiWizardStep(WIFI_WIZARD_AP_CHANNEL);
            break;
        case WIFI_WIZARD_AP_CHANNEL:
            if (input.length() > 0) {
                int channel = input.toInt();
                if (channel < 1 || channel > 13) {
                    Serial.println("AP channel must be 1 through 13.");
                    promptWifiWizardStep();
                    return;
                }
                wifiWizardConfig.apChannel = (uint8_t)channel;
            }
            setWifiWizardStep(WIFI_WIZARD_CONFIRM);
            break;
        case WIFI_WIZARD_CONFIRM:
            if (!parseBoolValue(input, parsedBool)) {
                Serial.println("Enter y to save, or n to cancel.");
                promptWifiWizardStep();
                return;
            }
            if (parsedBool) {
                NetworkConfig& cfg = networkManager.getConfig();
                cfg = wifiWizardConfig;
                networkManager.save();
                networkManager.restart();
                wifiWizardActive = false;
                wifiWizardStep = WIFI_WIZARD_IDLE;
                Serial.println("Network settings saved. Reconnecting with the new configuration.");
            } else {
                wifiWizardActive = false;
                wifiWizardStep = WIFI_WIZARD_IDLE;
                Serial.println("Wi-Fi setup wizard cancelled. No changes saved.");
            }
            break;
        default:
            wifiWizardActive = false;
            wifiWizardStep = WIFI_WIZARD_IDLE;
            break;
    }
}

void initCLI() {
    if (cliInitialized) return;
    
    // --- Global Settings ---
    registry.push_back({ "brightness", 
        []() { return String(settings.get().displayBrightness); },
        [](String v) { settings.get().displayBrightness = (uint8_t)clampInt(v.toInt(), 0, 255); }
    });
    
    registry.push_back({ "ramp", 
        []() { return String(settings.get().rampType); },
        [](String v) { settings.get().rampType = (uint8_t)clampInt(v.toInt(), 0, 1); }
    });
    
    registry.push_back({ "pitch_step", 
        []() { return String(settings.get().pitchStepSize); },
        [](String v) { settings.get().pitchStepSize = clampFloat(v.toFloat(), 0.01, 1.0); }
    });
    
    registry.push_back({ "rev_enc", 
        []() { return String(settings.get().reverseEncoder); },
        [](String v) { settings.get().reverseEncoder = (v == "1" || v == "true"); }
    });
    
    registry.push_back({ "saver_mode", 
        []() { return String(settings.get().screensaverMode); },
        [](String v) { settings.get().screensaverMode = (uint8_t)clampInt(v.toInt(), 0, 2); }
    });

    registry.push_back({ "phase_mode",
        []() { return String(settings.get().phaseMode); },
        [](String v) {
            settings.get().phaseMode = (uint8_t)clampInt(v.toInt(), 1, MAX_PHASE_MODE);
            motor.applySettings();
        }
    });

    registry.push_back({ "max_amp",
        []() { return String(settings.get().maxAmplitude); },
        [](String v) { settings.get().maxAmplitude = (uint8_t)clampInt(v.toInt(), 0, 100); }
    });

    registry.push_back({ "smooth_switch",
        []() { return String(settings.get().smoothSwitching); },
        [](String v) { settings.get().smoothSwitching = (v == "1" || v == "true"); }
    });

    registry.push_back({ "switch_ramp",
        []() { return String(settings.get().switchRampDuration); },
        [](String v) { settings.get().switchRampDuration = (uint8_t)clampInt(v.toInt(), 1, 5); }
    });

    registry.push_back({ "brake_mode",
        []() { return String(settings.get().brakeMode); },
        [](String v) { settings.get().brakeMode = (uint8_t)clampInt(v.toInt(), 0, 3); }
    });

    registry.push_back({ "brake_duration",
        []() { return String(settings.get().brakeDuration); },
        [](String v) { settings.get().brakeDuration = clampFloat(v.toFloat(), 0.0, 10.0); }
    });

    registry.push_back({ "brake_pulse_gap",
        []() { return String(settings.get().brakePulseGap); },
        [](String v) { settings.get().brakePulseGap = clampFloat(v.toFloat(), 0.1, 2.0); }
    });

    registry.push_back({ "brake_start_freq",
        []() { return String(settings.get().brakeStartFreq); },
        [](String v) { settings.get().brakeStartFreq = clampFloat(v.toFloat(), 10.0, 200.0); }
    });

    registry.push_back({ "brake_stop_freq",
        []() { return String(settings.get().brakeStopFreq); },
        [](String v) { settings.get().brakeStopFreq = clampFloat(v.toFloat(), 0.0, 50.0); }
    });

    registry.push_back({ "brake_cutoff",
        []() { return String(settings.get().softStopCutoff); },
        [](String v) { settings.get().softStopCutoff = clampFloat(v.toFloat(), 0.0, 50.0); }
    });

    registry.push_back({ "relay_active_high",
        []() { return String(settings.get().relayActiveHigh); },
        [](String v) { settings.get().relayActiveHigh = (v == "1" || v == "true"); }
    });

    registry.push_back({ "relay_delay",
        []() { return String(settings.get().powerOnRelayDelay); },
        [](String v) { settings.get().powerOnRelayDelay = (uint8_t)clampInt(v.toInt(), 0, 10); }
    });
    
    // --- Current Speed Settings ---
    // Note: These access the *currently active* speed settings
    
    registry.push_back({ "freq", 
        []() { return String(settings.getCurrentSpeedSettings().frequency); },
        [](String v) {
            SpeedSettings& speed = settings.getCurrentSpeedSettings();
            speed.frequency = clampFloat(v.toFloat(), speed.minFrequency, speed.maxFrequency);
            speed.frequency = clampFloat(speed.frequency, MIN_OUTPUT_FREQUENCY_HZ, MAX_OUTPUT_FREQUENCY_HZ);
            motor.applySettings();
        }
    });
    
    registry.push_back({ "phase1", 
        []() { return String(settings.getCurrentSpeedSettings().phaseOffset[0]); },
        [](String v) {
            settings.getCurrentSpeedSettings().phaseOffset[0] = clampFloat(v.toFloat(), -360.0, 360.0);
            motor.applySettings();
        }
    });
    
    registry.push_back({ "phase2", 
        []() { return String(settings.getCurrentSpeedSettings().phaseOffset[1]); },
        [](String v) {
            settings.getCurrentSpeedSettings().phaseOffset[1] = clampFloat(v.toFloat(), -360.0, 360.0);
            motor.applySettings();
        }
    });
    
    registry.push_back({ "phase3", 
        []() { return String(settings.getCurrentSpeedSettings().phaseOffset[2]); },
        [](String v) {
            settings.getCurrentSpeedSettings().phaseOffset[2] = clampFloat(v.toFloat(), -360.0, 360.0);
            motor.applySettings();
        }
    });
    
#if ENABLE_4_CHANNEL_SUPPORT
    registry.push_back({ "phase4",
        []() { return String(settings.getCurrentSpeedSettings().phaseOffset[3]); },
        [](String v) {
            settings.getCurrentSpeedSettings().phaseOffset[3] = clampFloat(v.toFloat(), -360.0, 360.0);
            motor.applySettings();
        }
    });
#endif
    
    registry.push_back({ "soft_start", 
        []() { return String(settings.getCurrentSpeedSettings().softStartDuration); },
        [](String v) { settings.getCurrentSpeedSettings().softStartDuration = clampFloat(v.toFloat(), 0.0, 10.0); }
    });
    
    registry.push_back({ "kick", 
        []() { return String(settings.getCurrentSpeedSettings().startupKick); },
        [](String v) { settings.getCurrentSpeedSettings().startupKick = (uint8_t)clampInt(v.toInt(), 1, 4); }
    });
    
    registry.push_back({ "kick_dur", 
        []() { return String(settings.getCurrentSpeedSettings().startupKickDuration); },
        [](String v) { settings.getCurrentSpeedSettings().startupKickDuration = (uint8_t)clampInt(v.toInt(), 0, 15); }
    });

    registry.push_back({ "filter",
        []() { return String(settings.getCurrentSpeedSettings().filterType); },
        [](String v) {
            settings.getCurrentSpeedSettings().filterType = (uint8_t)clampInt(v.toInt(), 0, 2);
            motor.applySettings();
        }
    });

    registry.push_back({ "reduced_amp",
        []() { return String(settings.getCurrentSpeedSettings().reducedAmplitude); },
        [](String v) { settings.getCurrentSpeedSettings().reducedAmplitude = (uint8_t)clampInt(v.toInt(), 10, 100); }
    });

    registry.push_back({ "amp_delay",
        []() { return String(settings.getCurrentSpeedSettings().amplitudeDelay); },
        [](String v) { settings.getCurrentSpeedSettings().amplitudeDelay = (uint8_t)clampInt(v.toInt(), 0, 60); }
    });
    
    // --- Live Motor State ---
    registry.push_back({ "pitch", 
        []() { return String(motor.getPitchPercent()); },
        [](String v) { motor.setPitch(v.toFloat()); }
    });
    
    cliInitialized = true;
}

void handleSerialCommands() {
    if (!cliInitialized) initCLI();
    updateWifiSerialTasks();
    
    if (Serial.available() > 0) {
        String input = Serial.readStringUntil('\n');
        input.trim();

        if (wifiWizardActive) {
            handleWifiWizardInput(input);
            return;
        }
        
        if (input.length() == 0) return;
        
        // --- Standard Commands ---
        if (input == "start") {
            if (motor.isRelayTestMode()) {
                Serial.println("Exit relay test before starting.");
            } else {
                motor.start();
                Serial.println("Motor Started");
            }
        }
        else if (input == "stop") {
            if (motor.isRelayTestMode()) {
                Serial.println("Relay test is active. Use 'relay test off'.");
            } else {
                motor.stop();
                Serial.println("Motor Stopped");
            }
        }
        else if (input.startsWith("speed ")) {
            int s = input.substring(6).toInt();
            if (s >= 0 && s <= 2) {
                if (s == SPEED_78 && !settings.get().enable78rpm) {
                    Serial.println("78 RPM is disabled. Enable it before selecting speed 2.");
                    return;
                }
                motor.setSpeed((SpeedMode)s);
                Serial.print("Speed set to ");
                Serial.println(speedName(motor.getSpeed()));
            } else {
                Serial.println("Invalid speed index (0-2)");
            }
        }
        else if (input == "s") {
            motor.cycleSpeed();
            Serial.println("Speed Cycled");
        }
        else if (input == "status" || input == "i") {
            printStatus();
        }
        else if (input == "t") {
            motor.toggleStandby();
            Serial.println("Standby Toggled");
        }
        else if (input == "p") {
            motor.resetPitch();
            Serial.println("Pitch Reset");
        }
        else if (input == "f") {
            Serial.println("Factory Resetting...");
            settings.factoryReset();
            settings.load();
            motor.endRelayTest();
            motor.applySettings();
            Serial.println("Factory reset complete.");
        }
        else if (input == "help") {
            printHelp();
        }
        else if (input == "save") {
            settings.save(true);
        }
        else if (input == "reboot") {
            Serial.println("Rebooting...");
            Serial.flush();
            hal.watchdogReboot();
        }
        else if (input == "dump settings") {
            printSettingsDump();
        }
        else if (input.startsWith("preset ")) {
            handlePresetCommand(input);
        }
        else if (input.startsWith("relay test")) {
            handleRelayTestCommand(input);
        }
        else if (input == "wifi" || input.startsWith("wifi ")) {
            handleWifiCommand(input);
        }
        else if (input == "brake test start") {
            if (motor.isRelayTestMode()) {
                Serial.println("Exit relay test before starting.");
            } else {
                if (motor.isStandby()) motor.toggleStandby();
                motor.start();
                Serial.println("Brake test motor start requested.");
            }
        }
        else if (input == "brake test stop") {
            if (motor.isRunning()) {
                motor.stop();
                Serial.println("Brake stop requested.");
            } else {
                Serial.println("Motor is not running.");
            }
        }
        else if (input == "error dump") {
            errorHandler.dumpLog(Serial);
        }
        else if (input == "error clear") {
            errorHandler.clearLogs();
            Serial.println("Error Log Cleared");
        }
        
        // --- Registry Commands ---
        else if (input == "list") {
            Serial.println("--- Settings List ---");
            for (const auto& item : registry) {
                Serial.print(item.name);
                Serial.print(" = ");
                Serial.println(item.get());
            }
            Serial.println("---------------------");
        }
        else if (input.startsWith("set ")) {
            int firstSpace = input.indexOf(' ');
            int secondSpace = input.indexOf(' ', firstSpace + 1);
            
            if (secondSpace > 0) {
                String key = input.substring(firstSpace + 1, secondSpace);
                String valStr = input.substring(secondSpace + 1);
                
                bool found = false;
                for (const auto& item : registry) {
                    if (item.name == key) {
                        item.set(valStr);
                        Serial.print("Set "); Serial.print(key); Serial.print(" = "); Serial.println(valStr);
                        found = true;
                        break;
                    }
                }
                if (!found) Serial.println("Unknown setting key");
            } else {
                Serial.println("Usage: set <key> <value>");
            }
        }
        else if (input.startsWith("get ")) {
            String key = input.substring(4);
            bool found = false;
            for (const auto& item : registry) {
                if (item.name == key) {
                    Serial.println(item.get());
                    found = true;
                    break;
                }
            }
            if (!found) Serial.println("Unknown setting key");
        }
        
        // --- Preset Import/Export ---
        else if (input.startsWith("export preset ")) {
            int slot = input.substring(14).toInt() - 1; // 1-based to 0-based
            if (slot >= 0 && slot < MAX_PRESET_SLOTS) {
                String out;
                if (settings.exportPresetToJSON(slot, out)) {
                    Serial.println(out);
                } else {
                    Serial.println("Error exporting preset.");
                }
            } else {
                Serial.println("Invalid preset slot (1-5)");
            }
        }
        else if (input.startsWith("import preset ")) {
            int firstSpace = input.indexOf(' ', 14);
            if (firstSpace > 14) {
                int slot = input.substring(14, firstSpace).toInt() - 1;
                String jsonStr = input.substring(firstSpace + 1);
                
                if (slot >= 0 && slot < MAX_PRESET_SLOTS) {
                    if (settings.importPresetFromJSON(slot, jsonStr)) {
                        Serial.println("Preset imported successfully.");
                    } else {
                        Serial.println("Failed to import preset.");
                    }
                } else {
                    Serial.println("Invalid preset slot (1-5)");
                }
            } else {
                Serial.println("Usage: import preset <1-5> <json_string>");
            }
        }
        
        // --- UI Injection ---
        else if (input == "j") ui.injectInput(-1, false);
        else if (input == "l") ui.injectInput(1, false);
        else if (input == "k") ui.injectInput(0, true);
        else if (input == "m") ui.enterMenu();
        
        else {
            Serial.println("Unknown command. Type 'help' for list.");
        }
    }
}

void printStatus() {
    Serial.println("--- TT Control Status ---");
    Serial.print("State: ");
    Serial.println(motorStateName());
    
    Serial.print("Speed Mode: ");
    Serial.println(speedName(motor.getSpeed()));
    
    Serial.print("Frequency: ");
    Serial.print(motor.getCurrentFrequency());
    Serial.println(" Hz");
    
    Serial.print("Pitch: ");
    Serial.print(currentPitchPercent);
    Serial.println("%");

    Serial.print("Brake: ");
    Serial.println(brakeModeName(settings.get().brakeMode));

#if AMP_MONITOR_ENABLE
    Serial.print("Amp Temp: ");
    Serial.print(ampMonitor.getTemperatureC(), 1);
    Serial.println(" C");

    Serial.print("Amp Thermal: ");
    Serial.println(ampMonitor.isThermalOk() ? "OK" : "TRIPPED");
#endif

    if (networkManager.isAvailable()) {
        Serial.print("Network: ");
        Serial.print(networkManager.statusText());
        String ip = networkManager.ipText();
        if (ip.length() > 0) {
            Serial.print(" ");
            Serial.print(ip);
        }
        Serial.println();
    }
    
    Serial.println("-------------------------");
}

void printHelp() {
    if (!cliInitialized) initCLI();
    
    Serial.println("Available Commands:");
    Serial.println("start, stop, t (standby)");
    Serial.println("speed <0-2>, s (cycle)");
    Serial.println("status, p (reset pitch)");
    Serial.println("list - List all settings");
    Serial.println("set <key> <val> - Set setting");
    Serial.println("get <key> - Get setting");
    Serial.println("save, reboot, dump settings");
    Serial.println("preset list|load <1-5>|save <1-5>");
    Serial.println("export preset <1-5> - Dump JSON");
    Serial.println("import preset <1-5> <json> - Load JSON");
    Serial.println("brake test start|stop");
    Serial.println("relay test <0-N|off>");
    Serial.println("wifi help|status|wizard|scan|connect");
    Serial.println("error dump, error clear");
    Serial.println("f - Factory Reset");
}

static void printWifiHelp() {
    Serial.println("Wi-Fi Commands:");
    Serial.println("wifi status - Show current network state");
    Serial.println("wifi config - Show saved network configuration");
    Serial.println("wifi wizard - Start guided serial setup");
    Serial.println("wifi scan - List nearby networks");
    Serial.println("wifi connect <ssid> [password] - Save station credentials and reconnect");
    Serial.println("wifi set enabled <on|off>");
    Serial.println("wifi set mode <ap|sta|sta_ap>");
    Serial.println("wifi set ssid <ssid>");
    Serial.println("wifi set password <password>");
    Serial.println("wifi set hostname <name>");
    Serial.println("wifi set dhcp <on|off>");
    Serial.println("wifi set static <ip> <gateway> <subnet> <dns>");
    Serial.println("wifi set fallback <on|off>");
    Serial.println("wifi set ap_ssid <ssid>");
    Serial.println("wifi set ap_password <password>");
    Serial.println("wifi set ap_channel <1-13>");
    Serial.println("wifi clear password|ap_password|ssid");
    Serial.println("wifi apply - Save staged settings and reconnect");
    Serial.println("wifi reset - Restore network defaults and reconnect");
    Serial.println("Use quotes for names or passwords with spaces.");
}

static void printWifiStatus() {
    Serial.println("--- Network Status ---");
    Serial.print("Available: ");
    Serial.println(yesNoText(networkManager.isAvailable()));
    if (!networkManager.isAvailable()) {
        Serial.println("Network support is not enabled in this build.");
        Serial.println("----------------------");
        return;
    }

    const NetworkConfig& cfg = networkManager.getConfig();
    Serial.print("Enabled: ");
    Serial.println(onOffText(cfg.enabled));
    Serial.print("Mode: ");
    Serial.println(networkManager.modeText());
    Serial.print("Status: ");
    Serial.println(networkManager.statusText());
    Serial.print("Station configured: ");
    Serial.println(cfg.ssid[0] ? cfg.ssid : "(none)");
    Serial.print("Active SSID: ");
    Serial.println(networkManager.ssidText().length() ? networkManager.ssidText() : "(none)");
    Serial.print("Connected: ");
    Serial.println(yesNoText(networkManager.isConnected()));
    Serial.print("Station IP: ");
    Serial.println(networkManager.stationIpText().length() ? networkManager.stationIpText() : "(none)");
    Serial.print("Setup AP active: ");
    Serial.println(yesNoText(networkManager.isApActive()));
    Serial.print("Setup AP IP: ");
    Serial.println(networkManager.apIpText().length() ? networkManager.apIpText() : "(none)");
    Serial.print("Setup AP clients: ");
    Serial.println(networkManager.connectedClientCount());
    Serial.print("MAC: ");
    Serial.println(networkManager.macText().length() ? networkManager.macText() : "(unknown)");
    if (networkManager.isConnected()) {
        Serial.print("RSSI: ");
        Serial.print(networkManager.rssi());
        Serial.println(" dBm");
    }
    Serial.println("----------------------");
}

static void markWifiConfigUpdated() {
    Serial.println("Network configuration updated. Use 'wifi apply' to save and reconnect.");
}

static bool requireWifiAvailable() {
    if (networkManager.isAvailable()) return true;
    Serial.println("Network support is not enabled in this build.");
    return false;
}

static void handleWifiSetCommand(const std::vector<String>& args) {
    if (args.size() < 3) {
        Serial.println("Usage: wifi set <key> <value>");
        return;
    }

    String key = args[1];
    key.toLowerCase();
    NetworkConfig& cfg = networkManager.getConfig();
    bool parsedBool = false;
    uint8_t parsedMode = NETWORK_MODE_AP;
    uint8_t parsedIp[4];
    uint8_t parsedGateway[4];
    uint8_t parsedSubnet[4];
    uint8_t parsedDns[4];

    if (key == "enabled") {
        if (!parseBoolValue(args[2], parsedBool)) {
            Serial.println("Use on/off, yes/no, or true/false.");
            return;
        }
        cfg.enabled = parsedBool;
        markWifiConfigUpdated();
    } else if (key == "mode") {
        if (!parseNetworkMode(args[2], parsedMode)) {
            Serial.println("Mode must be ap, sta, or sta_ap.");
            return;
        }
        cfg.mode = parsedMode;
        if (cfg.mode == NETWORK_MODE_STA_AP) cfg.apFallback = true;
        markWifiConfigUpdated();
    } else if (key == "ssid") {
        if (!copyConfigString(cfg.ssid, sizeof(cfg.ssid), args[2], "Station SSID")) return;
        markWifiConfigUpdated();
    } else if (key == "password") {
        if (!copyConfigString(cfg.password, sizeof(cfg.password), args[2], "Station password")) return;
        markWifiConfigUpdated();
    } else if (key == "hostname" || key == "host") {
        if (!copyConfigString(cfg.hostname, sizeof(cfg.hostname), args[2], "Hostname")) return;
        markWifiConfigUpdated();
    } else if (key == "dhcp") {
        if (!parseBoolValue(args[2], parsedBool)) {
            Serial.println("Use on/off, yes/no, or true/false.");
            return;
        }
        cfg.dhcp = parsedBool;
        markWifiConfigUpdated();
    } else if (key == "static") {
        if (args.size() < 6) {
            Serial.println("Usage: wifi set static <ip> <gateway> <subnet> <dns>");
            return;
        }
        if (!parseIpv4(args[2], parsedIp) ||
            !parseIpv4(args[3], parsedGateway) ||
            !parseIpv4(args[4], parsedSubnet) ||
            !parseIpv4(args[5], parsedDns)) {
            Serial.println("Enter valid IPv4 addresses.");
            return;
        }
        memcpy(cfg.staticIp, parsedIp, sizeof(cfg.staticIp));
        memcpy(cfg.gateway, parsedGateway, sizeof(cfg.gateway));
        memcpy(cfg.subnet, parsedSubnet, sizeof(cfg.subnet));
        memcpy(cfg.dns, parsedDns, sizeof(cfg.dns));
        cfg.dhcp = false;
        markWifiConfigUpdated();
    } else if (key == "ip" || key == "static_ip") {
        if (!parseIpv4(args[2], parsedIp)) {
            Serial.println("Enter a valid IPv4 address.");
            return;
        }
        memcpy(cfg.staticIp, parsedIp, sizeof(cfg.staticIp));
        cfg.dhcp = false;
        markWifiConfigUpdated();
    } else if (key == "gateway") {
        if (!parseIpv4(args[2], parsedIp)) {
            Serial.println("Enter a valid IPv4 address.");
            return;
        }
        memcpy(cfg.gateway, parsedIp, sizeof(cfg.gateway));
        cfg.dhcp = false;
        markWifiConfigUpdated();
    } else if (key == "subnet") {
        if (!parseIpv4(args[2], parsedIp)) {
            Serial.println("Enter a valid IPv4 address.");
            return;
        }
        memcpy(cfg.subnet, parsedIp, sizeof(cfg.subnet));
        cfg.dhcp = false;
        markWifiConfigUpdated();
    } else if (key == "dns") {
        if (!parseIpv4(args[2], parsedIp)) {
            Serial.println("Enter a valid IPv4 address.");
            return;
        }
        memcpy(cfg.dns, parsedIp, sizeof(cfg.dns));
        cfg.dhcp = false;
        markWifiConfigUpdated();
    } else if (key == "fallback" || key == "ap_fallback") {
        if (!parseBoolValue(args[2], parsedBool)) {
            Serial.println("Use on/off, yes/no, or true/false.");
            return;
        }
        cfg.apFallback = parsedBool;
        markWifiConfigUpdated();
    } else if (key == "ap_ssid") {
        if (!copyConfigString(cfg.apSsid, sizeof(cfg.apSsid), args[2], "Setup AP SSID")) return;
        markWifiConfigUpdated();
    } else if (key == "ap_password") {
        if (args[2].length() > 0 && args[2].length() < 8) {
            Serial.println("Setup AP password must be at least 8 characters, or blank for open setup AP.");
            return;
        }
        if (!copyConfigString(cfg.apPassword, sizeof(cfg.apPassword), args[2], "Setup AP password")) return;
        markWifiConfigUpdated();
    } else if (key == "ap_channel") {
        int channel = args[2].toInt();
        if (channel < 1 || channel > 13) {
            Serial.println("AP channel must be 1 through 13.");
            return;
        }
        cfg.apChannel = (uint8_t)channel;
        markWifiConfigUpdated();
    } else if (key == "read_only" || key == "readonly") {
        if (!parseBoolValue(args[2], parsedBool)) {
            Serial.println("Use on/off, yes/no, or true/false.");
            return;
        }
        cfg.readOnlyMode = parsedBool;
        markWifiConfigUpdated();
    } else if (key == "web_pin") {
        if (args[2].length() > 0 && (args[2].length() < 4 || args[2].length() > NETWORK_WEB_PIN_MAX)) {
            Serial.print("Web PIN must be 4-");
            Serial.print(NETWORK_WEB_PIN_MAX);
            Serial.println(" characters, or blank to clear.");
            return;
        }
        if (!copyConfigString(cfg.webPin, sizeof(cfg.webPin), args[2], "Web PIN")) return;
        markWifiConfigUpdated();
    } else {
        Serial.println("Unknown Wi-Fi setting. Type 'wifi help'.");
    }
}

static void handleWifiClearCommand(const std::vector<String>& args) {
    if (args.size() < 2) {
        Serial.println("Usage: wifi clear password|ap_password|ssid");
        return;
    }

    String key = args[1];
    key.toLowerCase();
    NetworkConfig& cfg = networkManager.getConfig();

    if (key == "password") {
        cfg.password[0] = 0;
        markWifiConfigUpdated();
    } else if (key == "ap_password") {
        cfg.apPassword[0] = 0;
        markWifiConfigUpdated();
    } else if (key == "ssid") {
        cfg.ssid[0] = 0;
        markWifiConfigUpdated();
    } else {
        Serial.println("Usage: wifi clear password|ap_password|ssid");
    }
}

static void handleWifiConnectCommand(const std::vector<String>& args) {
    if (args.size() < 2) {
        Serial.println("Usage: wifi connect <ssid> [password]");
        return;
    }

    NetworkConfig& cfg = networkManager.getConfig();
    if (!copyConfigString(cfg.ssid, sizeof(cfg.ssid), args[1], "Station SSID")) return;
    String password = args.size() >= 3 ? args[2] : "";
    if (!copyConfigString(cfg.password, sizeof(cfg.password), password, "Station password")) return;

    cfg.enabled = true;
    cfg.mode = NETWORK_MODE_STA_AP;
    cfg.dhcp = true;
    cfg.apFallback = true;
    if (cfg.hostname[0] == 0) {
        copyConfigString(cfg.hostname, sizeof(cfg.hostname), NETWORK_DEFAULT_HOSTNAME, "Hostname");
    }
    if (cfg.apSsid[0] == 0) {
        copyConfigString(cfg.apSsid, sizeof(cfg.apSsid), NETWORK_DEFAULT_AP_SSID, "Setup AP SSID");
    }
    if (cfg.apChannel < 1 || cfg.apChannel > 13) {
        cfg.apChannel = NETWORK_DEFAULT_AP_CHANNEL;
    }

    networkManager.save();
    networkManager.restart();
    Serial.println("Station credentials saved. Reconnecting in Station + setup AP mode.");
}

static void handleWifiCommand(const String& input) {
    String rest = input.length() > 4 ? input.substring(5) : "";
    rest.trim();

    std::vector<String> args;
    parseCommandArgs(rest, args);

    if (args.empty()) {
        printWifiHelp();
        return;
    }

    String command = args[0];
    command.toLowerCase();

    if (command == "help") {
        printWifiHelp();
        return;
    }

    if (command == "status") {
        printWifiStatus();
        return;
    }

    if (!requireWifiAvailable()) return;

    if (command == "config") {
        Serial.println("--- Network Configuration ---");
        printWifiConfigSummary(networkManager.getConfig());
        Serial.println("-----------------------------");
    } else if (command == "wizard" || command == "setup") {
        startWifiWizard();
    } else if (command == "scan") {
        startWifiScan(false);
    } else if (command == "connect") {
        handleWifiConnectCommand(args);
    } else if (command == "set") {
        handleWifiSetCommand(args);
    } else if (command == "clear") {
        handleWifiClearCommand(args);
    } else if (command == "apply" || command == "save") {
        networkManager.save();
        networkManager.restart();
        Serial.println("Network settings saved. Reconnecting.");
    } else if (command == "restart" || command == "reconnect") {
        networkManager.restart();
        Serial.println("Network services restarted.");
    } else if (command == "reset" || command == "defaults") {
        networkManager.resetDefaults();
        networkManager.restart();
        Serial.println("Network defaults restored. Network services restarted.");
    } else {
        Serial.println("Unknown Wi-Fi command. Type 'wifi help'.");
    }
}

static void printPresetList() {
    Serial.println("--- Presets ---");
    for (int i = 0; i < MAX_PRESET_SLOTS; i++) {
        Serial.print(i + 1);
        Serial.print(": ");
        Serial.println(settings.getPresetName(i));
    }
}

static void printSettingsDump() {
    GlobalSettings& g = settings.get();

    Serial.println("--- Settings Dump ---");
    Serial.print("Schema: "); Serial.println(g.schemaVersion);
    Serial.print("Phase Mode: "); Serial.println(g.phaseMode);
    Serial.print("Max Amp: "); Serial.print(g.maxAmplitude); Serial.println("%");
    Serial.print("Ramp Type: "); Serial.println(rampTypeName(g.rampType));
    Serial.print("Smooth Switch: "); Serial.println(g.smoothSwitching ? "ON" : "OFF");
    Serial.print("Brake Mode: "); Serial.println(brakeModeName(g.brakeMode));
    Serial.print("Brake Duration: "); Serial.print(g.brakeDuration); Serial.println("s");
    Serial.print("Relay Active High: "); Serial.println(g.relayActiveHigh ? "YES" : "NO");
    Serial.print("Boot Speed: "); Serial.println(g.bootSpeed);
    Serial.print("Runtime: "); Serial.print(settings.getTotalRuntime()); Serial.println("s");

    for (int i = 0; i < 3; i++) {
        SpeedSettings& s = g.speeds[i];
        Serial.print("Speed ");
        Serial.print(i);
        Serial.print(" ");
        Serial.print(speedName((SpeedMode)i));
        Serial.print(": ");
        Serial.print(s.frequency);
        Serial.print("Hz, ");
        Serial.print(s.reducedAmplitude);
        Serial.print("% amp, filter ");
        Serial.println(filterName(s.filterType));
    }
    Serial.println("---------------------");
}

static void handlePresetCommand(const String& input) {
    if (input == "preset list") {
        printPresetList();
        return;
    }

    if (input.startsWith("preset load ")) {
        int slot = input.substring(12).toInt() - 1;
        if (slot >= 0 && slot < MAX_PRESET_SLOTS) {
            if (settings.loadPreset(slot)) {
                motor.applySettings();
                Serial.println("Preset loaded. Use 'save' to make it the boot settings.");
            } else {
                Serial.println("Preset slot is empty.");
            }
        } else {
            Serial.println("Invalid preset slot (1-5).");
        }
        return;
    }

    if (input.startsWith("preset save ")) {
        int slot = input.substring(12).toInt() - 1;
        if (slot >= 0 && slot < MAX_PRESET_SLOTS) {
            settings.savePreset(slot);
            Serial.println("Preset saved.");
        } else {
            Serial.println("Invalid preset slot (1-5).");
        }
        return;
    }

    Serial.println("Usage: preset list|load <1-5>|save <1-5>");
}

static void handleRelayTestCommand(const String& input) {
    String arg = "";
    if (input.length() > 10) {
        arg = input.substring(10);
        arg.trim();
    }

    if (arg == "off") {
        motor.endRelayTest();
        Serial.println("Relay test off.");
        return;
    }

    int stage = -1;
    bool numeric = arg.length() > 0;
    for (size_t i = 0; i < arg.length(); i++) {
        char c = arg.charAt(i);
        if (c < '0' || c > '9') {
            numeric = false;
            break;
        }
    }

    if (numeric) {
        stage = arg.toInt();
    }

    if (stage < 0 || stage >= motor.getRelayTestStageCount()) {
        Serial.print("Usage: relay test <0-");
        Serial.print(motor.getRelayTestStageCount() - 1);
        Serial.println("|off>");
        return;
    }

    if (!motor.isRelayTestMode() && !motor.beginRelayTest()) {
        Serial.println("Stop motor before relay test.");
        return;
    }

    motor.setRelayTestStage((uint8_t)stage);
    Serial.print("Relay test stage ");
    Serial.println(stage);
}
