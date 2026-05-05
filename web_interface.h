/*
 * TT Control, advanced sinusoidal control of multi-phase turntable motors
 * Created by Ashley Cox at The Blind Man's Workshop
 * https://theblindmansworkshop.com
 * No part of this code may be used or reproduced for commercial purposes without written permission and contractual agreement
 * All external libraries and frameworks are the property of their respective authors and governed by their respective licenses
 */

#ifndef WEB_INTERFACE_H
#define WEB_INTERFACE_H

#include <Arduino.h>
#include "config.h"

#if NETWORK_ENABLE
#include <WebServer.h>
#include <ArduinoJson.h>
#endif

class WebInterface {
public:
    WebInterface();

    void begin();
    void update();
    bool isStarted() const;

private:
#if NETWORK_ENABLE
    WebServer _server;
    bool _started;
    char _authToken[17];
    uint32_t _authExpiresMs;

    void setupRoutes();
    void sendJson(int code, JsonDocument& doc);
    void sendError(int code, const char* message);
    bool parseBody(JsonDocument& doc);
    bool isOpenSetupRequest();
    bool rejectOpenSetupAccess();
    bool hasWriteAccess();
    bool requireWriteAccess();
    void clearAuthToken();
    void issueAuthToken();
    void populateStatus(JsonDocument& doc);

    void handleRoot();
    void handleSetupRoot();
    void handleAuthGet();
    void handleAuthPost();
    void handlePreferencesGet();
    void handlePreferencesPost();
    void handleSchemaGet();
    void handleStatus();
    void handleEventsGet();
    void handleSettingsGet();
    void handleSettingsPost();
    void handleControl();
    void handleNetworkGet();
    void handleNetworkPost();
    void handleNetworkScan();
    void handleDiagnosticsGet();
    void handlePresetsGet();
    void handlePresetPost();
    void handleErrorsGet();
    void handleErrorsPost();
    void handleNotFound();
#else
    bool _started;
#endif
};

extern WebInterface webInterface;

#endif // WEB_INTERFACE_H
