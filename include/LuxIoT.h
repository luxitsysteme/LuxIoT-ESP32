#pragma once

#include <Arduino.h>
#include <HTTPClient.h>

#include <stdint.h>

#include "ArduinoJson.h"

#define LUX_PROVISION_FILE      "/provision.json"
#define LUX_FIRMWARE_FILE       "/firmware.bin"

#define LUX_ETAG_SUFFIX         ".etag"

class LuxIoT{
public:
    LuxIoT(const String &baseUrl, const String &provisionSuffix, const String &provisionToken, const uint32_t fallbackSleeptime, const uint32_t watchdogTime, HTTPClient &httpClient);
    uint8_t begin();
    uint8_t beginWifi(const String &wifiSSID, const String &wifiPass);

    uint8_t beginGlobalCA(const uint8_t* beginCert, const uint8_t* endCert);

    uint8_t ensureProvision();
    uint8_t ensureProvision(const String &identifier);

    uint8_t ensureOtaFirmware();

    uint8_t spiffsRead(const String &fileName, String &output);
    uint8_t spiffsWrite(const String &fileName, const String &input);
    uint8_t spiffsReadJson(const String &filename, JsonDocument &jsonDocument);
    uint8_t spiffsWriteJson(const String &filename, const JsonDocument &JsonDocument);

    uint8_t apiGet(const String &prefix, const String &suffix, String &output, uint8_t useDeviceId);
    uint8_t apiGet(const String &prefix, const String &suffix, String &output, const String &compareEtag, String &outputEtag, uint8_t useDeviceId);
    uint8_t apiHead(const String &prefix, const String &suffix, String &outputEtag, uint8_t useDeviceId);
    uint8_t apiPost(const String &prefix, const String &suffix, const String &contentType, const String &input, String &output, uint8_t useDeviceId);

    uint8_t apiLog(const String& format, ...);

    uint8_t beginTelemetry(uint8_t includeStd);
    JsonDocument &telemetry();
    uint8_t endTelemetry();

    uint8_t apiTelemetry(const JsonDocument &jsonDocument);

    void regularSleep(uint64_t sleepS);
    void fallbackSleep();
private:

    uint8_t _beginWifiInternalFast(const String &wifiSSID, const String &wifiPass, const uint8_t wifiChannel, const uint8_t* wifiBSSID);
    uint8_t _beginWifiInternalNormal(const String &wifiSSID, const String &wifiPass);
    uint8_t _waitWifiConnection();



    String mBaseURL;
    String mProvisionSuffix;

    int32_t mDeviceId;
    String mProvisionToken;
    String mApiToken;

    uint32_t mFallbackSleeptime;
    uint32_t mWatchdogTime;

    uint32_t mWifiConnectionMillis;
    uint64_t mMillisStart;

    HTTPClient &mHttpClient;

    uint8_t* mCertStart;
    uint8_t* mCertEnd;
};
