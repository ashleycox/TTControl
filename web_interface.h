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

// Browser-facing control surface. The class serves the embedded HTML app, setup page, JSON APIs, status event stream, and PIN-gated write actions.
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

    // Single-session write token issued after a valid web PIN. It is intentionally short-lived and stored only in RAM.
    char _authToken[17];
    uint32_t _authExpiresMs;

    // WebServer's raw-body callback arrives in chunks. POST handlers parse this buffer once the body is complete, then release it immediately.
    char* _rawBody;
    size_t _rawBodyLength;
    size_t _rawBodyCapacity;
    bool _rawBodyOverflow;
    bool _rawBodyComplete;

    void setupRoutes();
    void sendJson(int code, JsonDocument& doc);
    void sendStaticHtml(PGM_P content, size_t contentLength);
    void sendError(int code, const char* message);
    bool parseBody(JsonDocument& doc);
    void handleRawBody();
    void releaseRawBody();
    bool isOpenSetupRequest();
    bool rejectOpenSetupAccess();
    bool hasWriteAccess();
    bool requireWriteAccess();
    void clearAuthToken();
    void issueAuthToken();
    void streamStatus(Print& out);
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
