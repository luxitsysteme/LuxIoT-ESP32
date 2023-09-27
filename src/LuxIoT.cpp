#include "LuxIoT.h"

#include "WiFi.h"
#include "SPIFFS.h"

#include <esp_task_wdt.h>
#include <esp_pm.h>
#include <driver/rtc_io.h>

#include "esp32-hal-log.h"
#include "esp_https_ota.h"
#include "esp_tls.h"

#if CONFIG_IDF_TARGET_ESP32 // ESP32/PICO-D4
#include "esp32/rom/rtc.h"
#elif CONFIG_IDF_TARGET_ESP32S2
#include "esp32s2/rom/rtc.h"
#elif CONFIG_IDF_TARGET_ESP32C3
#include "esp32c3/rom/rtc.h"
#elif CONFIG_IDF_TARGET_ESP32S3
#include "esp32s3/rom/rtc.h"
#else 
#error Target CONFIG_IDF_TARGET is not supported
#endif

static const char* TAG = "LuxIoT";

RTC_DATA_ATTR  uint8_t lux_wifi_channel;        // used for faster WiFi connection
RTC_DATA_ATTR  uint8_t lux_wifi_bssid[6];       // used for faster WiFi connection
RTC_DATA_ATTR  uint8_t lux_wifi_valid = false;  // used for faster WiFi connection

RTC_DATA_ATTR  uint64_t lux_last_runtime = 0; // used for measuring the complete runtime

DynamicJsonDocument lux_json_doc(1024);   // temporary json document for parsing

/**
 * @brief Construct a new Lux IoT::LuxIoT object
 * 
 * This object is used to communicate with the Lux IoT Cloud. Use it to connect your application to the cloud.
 * It offers:
 *      - WiFi Connection handling - done (new creds from config not implemented)
 *      - OTA Update functionality - done (rollback feature not implemented)
 *      - Variable Access - not done
 *      - Basic API Requests - done (write into buffer methods not implemented)
 *      - Telemetry posting - done
 *      - Logging functionality - done
 *      
 * To get more information about these features, take a look at the specific functions.
 * 
 * @param baseUrl               the API base url. 
 *                              usually https://api.cloud.lux-it-systeme.de/v1/
 * @param provisionSuffix       the suffix used for provisioning devices. 
 *                              usually projects/<projectid>/provision
 * @param provisionToken        the token used for provisioning. 
 *                              Look in your project and add a provision token if there is none available
 * @param fallbackSleeptime     the sleeptime (in seconds) used if some temporary error occurs.
 *                              choose a time that does not drain the battery very much (~15 minutes is a reasonable value)
 * @param watchdogTime          the watchdog timeout (in seconds) used. It should be long enough that your application usually terminates.
 *                              if feeding the watchdog while your application is running you can lower this value.
 * @param beforeSleepCallback   a pointer to a function (signature: void function()) to call before going to sleep. You can use it to configure
 *                              the device for minimum sleep current (e.g. isolate gpio pins; setting analog modes, ...)
 */
LuxIoT::LuxIoT(
    const String &baseUrl,
    const String &provisionSuffix,
    const String &provisionToken,
    const uint32_t fallbackSleeptime,
    const uint32_t watchdogTime,
    HTTPClient &httpClient,
    void (*beforeSleepCallback)(uint64_t sleepTime) ) : 
        mBaseURL(baseUrl), 
        mProvisionSuffix(provisionSuffix), 
        mProvisionToken(provisionToken), 
        mFallbackSleeptime(fallbackSleeptime),
        mWatchdogTime(watchdogTime),
        mHttpClient(httpClient),
        mBeforeSleepCallback(beforeSleepCallback){
    
    // TODO: Implement sanity checks for given values.
    mMillisStart = millis();
}

/**
 * @brief Begin of LuxIoT, should be one of the very first commands of application because of wdt watchdog settings
 * 
 * @return uint8_t returns whether the initialization was successful
 */
uint8_t LuxIoT::begin(){
    //if(rtc_get_reset_reason(0) == RESET_REASON_CHIP_POWER_ON || rtc_get_reset_reason(1) == RESET_REASON_CHIP_POWER_ON){
    //    ESP.restart();
    //}

    // initialize watchdog
    esp_err_t wdt_init_res = esp_task_wdt_init(mWatchdogTime, true);
    if (wdt_init_res != ESP_OK){
        // This is usually called befor any wifi connection, therefore logging to serial and sleep. Should not be happening in the wild!
        ESP_LOGE(TAG, "Failed to initialize wdt.");
        fallbackSleep(); 
    }

    wdt_init_res = esp_task_wdt_add(NULL); //add current thread to WDT watch
    if (wdt_init_res != ESP_OK){
        // This is usually called befor any wifi connection, therefore logging to serial and sleep. Should not be happening in the wild!
        ESP_LOGE(TAG, "Failed to initialize wdt task.");
        fallbackSleep(); 
    }

    lux_json_doc.clear();

    uint8_t spiffres = SPIFFS.begin(true);
    if (!spiffres){
        ESP_LOGE(TAG, "Failed to open SPIFFS, continuing anyway because FW update could help.");
    }

    return true;

}

/**
 * @brief   Begins Wifi access, after calling this the module should be connected to the wifi network.
 *          Otherwise a fallback sleep will be performed in order to make future requests possible.
 *          Currently only WPA2 PSK is supported!
 * 
 * @param wifiSSID      The Wifi SSID to connect to
 * @param wifiPass      The Wifi Password used to connect
 * @return uint8_t      If the Wifi connection was successful
 */
uint8_t LuxIoT::beginWifi(const String &wifiSSID, const String &wifiPass){
    uint32_t startMillis = millis();
    if( lux_wifi_valid ) {
        ESP_LOGI(TAG, "There are wifi connection parameters cached, using it for fast wifi connectivity");
        lux_wifi_valid = false; // invalidate in case something goes bad
        if (!_beginWifiInternalFast(wifiSSID, wifiPass, lux_wifi_channel, lux_wifi_bssid)){
            ESP_LOGE(TAG, "Fast connection not possible, remove valid flag and sleep");
            lux_wifi_valid = false;
            fallbackSleep();
        }
        lux_wifi_valid = true;
	}
	else {
        ESP_LOGI(TAG, "There are no wifi connection parameters cached, using normal method.");
		if(!_beginWifiInternalNormal(wifiSSID, wifiPass)){
            ESP_LOGE(TAG, "Normal connection not possible");
            fallbackSleep();
        }
        lux_wifi_channel = WiFi.channel();
        memcpy(lux_wifi_bssid, WiFi.BSSID(), 6);
        lux_wifi_valid = true;
	}

    uint32_t endMillis = millis();
    mWifiConnectionMillis = endMillis - startMillis;
    return true;
}

/**
 * @brief Ensures the provisioning of the device with the Wifi MAC as identifier.
 *        Note: it is possible that local provisioning details are saved on SPIFFS. If that is the case
 *        no provisioning process will be started but deviceid and token will be fetched from SPIFFS
 * 
 * @return uint8_t returns whether the provisioning was successful
 */
uint8_t LuxIoT::ensureProvision(){
    return ensureProvision(WiFi.macAddress());
}

/**
 * @brief Ensures the provisioning of the device with a user given identifier.
 *        Note: it is possible that local provisioning details are saved on SPIFFS. If that is the case
 *        no provisioning process will be started but deviceid and token will be fetched from SPIFFS
 * 
 * @param identifier    The identifier used to provision the device.
 * @return uint8_t      returns whether the provisioning was successful
 */
uint8_t LuxIoT::ensureProvision(const String &identifier){
    uint8_t spiffRes = spiffsReadJson(LUX_PROVISION_FILE, lux_json_doc);
    if(spiffRes){
        mDeviceId = lux_json_doc["scope_id"];
        mApiToken = (const char*)lux_json_doc["token"];

        return true;
    }

    lux_json_doc.clear();
    lux_json_doc["token"] = mProvisionToken;
    lux_json_doc["identifier"] = identifier;

    String jsonStr;
    String tmp;
    serializeJson(lux_json_doc, jsonStr);

    uint8_t postRes = apiPost("", mProvisionSuffix, "application/json", jsonStr, tmp, false);

    if(!postRes){
        ESP_LOGE("TAG", "Error provisioning device...");
        fallbackSleep();
    }

    lux_json_doc.clear();
    if(deserializeJson(lux_json_doc, tmp) != DeserializationError::Ok){
        ESP_LOGE(TAG, "provisioning not deserializable: %s", tmp.c_str());
        fallbackSleep();
        return false;
    }

    mDeviceId = lux_json_doc["scope_id"];
    mApiToken = (const char*)lux_json_doc["token"];

    spiffsWriteJson(LUX_PROVISION_FILE, lux_json_doc);
    return true;    
}

/**
 * @brief Ensures that the devices makes an Ota Request. In the background following things happen:
 *        1. Firmware Etag is read from SPIFFS
 *        2. A Head request on the Firmware variable will be made, returning the latest Etag
 *        3. If the request was not successful, return false
 *        4. If Etags mismatch a firmware update process is started. If the update was successful, 
 *           Etag is written to SPIFFS and reboot. Otherwise a Fallback Sleep is performed
 * 
 * @return uint8_t 
 */
uint8_t LuxIoT::ensureOtaFirmware(){
    String url = mBaseURL + "devices/" + String(mDeviceId) + "/vars/firmware/value";
    String oldEtag;
    spiffsRead(LUX_FIRMWARE_FILE LUX_ETAG_SUFFIX, oldEtag);
    String requestEtag;
    uint8_t reqRes = apiHead("devices/", "vars/firmware/value", requestEtag, true);
    if(!reqRes){
        apiLog("No successful HEAD request on Firmware");
        return false;
    }

    if(requestEtag != oldEtag){

        apiLog("Begin Firmware update because Etag mismatch...");
        esp_http_client_config_t config = {
            .url = url.c_str(),
            .use_global_ca_store = true
        };
        esp_https_ota_config_t ota_config = {
            .http_config = &config,
            .partial_http_download = false
        };

        esp_err_t ret = esp_https_ota(&config);
        if (ret == ESP_OK) {
            apiLog("Firmware update successful, rebooting!");
            spiffsWrite(LUX_FIRMWARE_FILE LUX_ETAG_SUFFIX, requestEtag);
            lux_wifi_valid = false; // invalidate Wifi Credentials
            esp_restart();
        } else {
            apiLog("Firmware update was not successfull. Fallback Sleep!");
            fallbackSleep();
        }
    }

    return true;
}

/**
 * @brief Read a value from SPIFFS
 * 
 * @param fileName  the filename to read from
 * @param output    the String object to save the contents to
 * @return uint8_t  if the operation was successful
 */
uint8_t LuxIoT::spiffsRead(const String &fileName, String &output){
    if(!SPIFFS.exists(fileName)){
        return false;
    }

    File file = SPIFFS.open(fileName, "r");
 
    if (!file) {
        ESP_LOGE(TAG ,"Failed to open file for reading %s", fileName.c_str());
        return false;
    }

    output = file.readStringUntil('\0');

    file.close();
    return true;
}

/**
 * @brief Write a value to SPIFFS
 * 
 * @param fileName  the filename to write to
 * @param input     the String object containing the file contents
 * @return uint8_t  if the operation was successful
 */
uint8_t LuxIoT::spiffsWrite(const String &fileName, const String &input){
    File file = SPIFFS.open(fileName, "w");
 
    if (!file) {
        ESP_LOGE(TAG ,"Failed to open file for writing %s", fileName.c_str());
        return false;
    }
   
    file.write((uint8_t *)input.c_str(), input.length() + 1); // +1 for \0 terminator
    
    file.close();
    return true;
}

/**
 * @brief Read a Json Document from SPIFFS
 * 
 * @param filename  the filename to read from
 * @param jsonDocument  the Json document to save the contents into
 * @return uint8_t  if the operation was successful
 */
uint8_t LuxIoT::spiffsReadJson(const String &filename, JsonDocument &jsonDocument){
    String tmp;
    uint8_t res = spiffsRead(filename, tmp);

    if(!res){
        return false;
    }
    
    if(deserializeJson(jsonDocument, tmp) != DeserializationError::Ok){
        ESP_LOGE(TAG, "File not empty but not deserializable, something is bad, removing the file %s and sleep", filename.c_str());
        SPIFFS.remove(filename); 
        fallbackSleep();
        return false;
    }

    return true;
}

/**
 * @brief Write a Json Document to SPIFFS
 * 
 * @param filename  the filename to write to
 * @param jsonDocument  the Json document containing the file contents
 * @return uint8_t  if the operation was successful
 */
uint8_t LuxIoT::spiffsWriteJson(const String &filename, const JsonDocument &jsonDocument){
    String tmp;

    serializeJson(jsonDocument, tmp);

    uint8_t res = spiffsWrite(filename, tmp);
    if(!res){
        ESP_LOGE(TAG, "Could not write json to file, something is bad, removing the file %s and sleep", filename.c_str());
        fallbackSleep();
        return false;
    }

    return true;
}

/**
 * @brief Simple GET Request on the API. Returns the request content into a String reference.
 *        will assemble the URL:
 *        if useDeviceId:
 *          mBaseURL + prefix + mDeviceId + "/" + suffix;
 *        else:
 *          mBaseUrl + prefix + suffx
 * 
 * @param prefix    the prefix for the request
 * @param suffix    the suffix for the request
 * @param output    the output to write the request data to
 * @param useDeviceId   whether to use the device id inbetween prefix and suffix
 * @return uint8_t  if the request was successful (return code == 200)
 */
uint8_t LuxIoT::apiGet(const String &prefix, const String &suffix, String &output, uint8_t useDeviceId){
    String url = mBaseURL + prefix;
    if(useDeviceId){
        url += String(mDeviceId) + "/";
    }
    url += suffix;
    ESP_LOGI(TAG, "Simple GET Request to %s", url.c_str());
    mHttpClient.begin(url, (const char*)mCertStart);
    if (mApiToken.length() > 0)
        mHttpClient.addHeader("Authorization", "Bearer " + mApiToken);
    uint16_t status = mHttpClient.GET();
    uint8_t state = true;
    if(status != 200){
        //apiLog("Request GET %s returned %i", url, status);
        ESP_LOGE(TAG, "Got %i from request...", status);
        state = false;
    }
    output = mHttpClient.getString();
    mHttpClient.end();
    return state;
}

/**
 * @brief Not IMPLEMENTED!
 * 
 * @param prefix 
 * @param suffix 
 * @param output 
 * @param compareEtag 
 * @param outputEtag 
 * @param useDeviceId 
 * @return uint8_t 
 */
uint8_t LuxIoT::apiGet(const String &prefix, const String &suffix, String &output, const String &compareEtag, String &outputEtag, uint8_t useDeviceId){
    return true;
}

uint8_t LuxIoT::apiGetBuffer(const String &prefix, const String &suffix, uint8_t* output, uint32_t outputMaxSize, uint32_t &outputSize, uint8_t useDeviceId){
    String url = mBaseURL + prefix;
    if(useDeviceId){
        url += String(mDeviceId) + "/";
    }
    url += suffix;
    ESP_LOGI(TAG, "Simple GET Request to %s", url.c_str());
    mHttpClient.begin(url, (const char*)mCertStart);
    if (mApiToken.length() > 0)
        mHttpClient.addHeader("Authorization", "Bearer " + mApiToken);
    uint16_t status = mHttpClient.GET();
    uint8_t state = true;
    if(status != 200){
        //apiLog("Request GET %s returned %i", url, status);
        ESP_LOGE(TAG, "Got %i from request...", status);
        state = false;
    }

    if(state){
        WiFiClient *stream = mHttpClient.getStreamPtr();
        stream->setTimeout(1);
        size_t length = mHttpClient.getSize();
        outputSize = length;

        size_t currentPos = 0;
        size_t remainingBuffer = outputMaxSize;

        while (mHttpClient.connected()){
            size_t streamSize = stream->available();
            if(streamSize) {
                size_t bytesRead = stream->readBytes(output, remainingBuffer);

                ESP_LOGI("TEST", "bytesRead: %i, remainingBuffer: %i, length: %i", bytesRead, remainingBuffer, length);

                output += bytesRead;
                remainingBuffer -= bytesRead;
                length -= bytesRead;

                if(length <= 0 || remainingBuffer <= 0){ // should not be < 0, but just in case
                    break;
                }
            }
            delay(5);
        }
    }else{
        outputSize = 0;
    }
    
    mHttpClient.end();
    return state;
}

uint8_t LuxIoT::apiGetBuffer(const String &prefix, const String &suffix, uint8_t* output, uint32_t outputMaxSize, uint32_t &outputSize, const String &compareEtag, String &outputEtag, uint8_t useDeviceId){
    String url = mBaseURL + prefix;
    if(useDeviceId){
        url += String(mDeviceId) + "/";
    }
    url += suffix;
    ESP_LOGI(TAG, "Simple GET Request to %s", url.c_str());
    mHttpClient.begin(url, (const char*)mCertStart);
    const char* headerReq[1] = {"Etag"};
    mHttpClient.collectHeaders(headerReq, 1);
    mHttpClient.addHeader("If-None-Match", compareEtag);
    if (mApiToken.length() > 0)
        mHttpClient.addHeader("Authorization", "Bearer " + mApiToken);
    uint16_t status = mHttpClient.GET();
    uint8_t state = true;
    if(status != 200){
        //apiLog("Request GET %s returned %i", url, status);
        ESP_LOGE(TAG, "Got %i from request...", status);
        state = false;
    }

    if(mHttpClient.hasHeader("Etag")){
        outputEtag = mHttpClient.header("Etag");
        state = true;
    }else{
        state = false;
    }

    if(state){
        WiFiClient *stream = mHttpClient.getStreamPtr();
        stream->setTimeout(1);
        size_t length = mHttpClient.getSize();
        outputSize = length;

        size_t currentPos = 0;
        size_t remainingBuffer = outputMaxSize;

        while (mHttpClient.connected()){
            size_t streamSize = stream->available();
            if(streamSize) {
                size_t bytesRead = stream->readBytes(output, remainingBuffer);

                output += bytesRead;
                remainingBuffer -= bytesRead;
                length -= bytesRead;

                if(length <= 0 || remainingBuffer <= 0){ // should not be < 0, but just in case
                    break;
                }
            }
            delay(5);
        }
    }else{
        outputSize = 0;
        outputEtag = compareEtag;
    }
    
    mHttpClient.end();
    return state;
}

/**
 * @brief Simple HEAD Request on the API. Used to fetch Etag of a given ressource
 *        will assemble the URL:
 *        if useDeviceId:
 *          mBaseURL + prefix + mDeviceId + "/" + suffix;
 *        else:
 *          mBaseUrl + prefix + suffx
 * 
 * @param prefix    the prefix for the request
 * @param suffix    the suffix for the request
 * @param outputEtag    the output to write the Etag to
 * @param useDeviceId   whether to use the device id inbetween prefix and suffix
 * @return uint8_t  if the request was successful (return code == 200)
 */
uint8_t LuxIoT::apiHead(const String &prefix, const String &suffix, String &outputEtag, uint8_t useDeviceId){
    String url = mBaseURL + prefix;
    if(useDeviceId){
        url += String(mDeviceId) + "/";
    }
    url += suffix;
    ESP_LOGI(TAG, "Simple HEAD Request to %s", url.c_str());

    mHttpClient.begin(url, (const char*)mCertStart);
    const char* headerReq[1] = {"Etag"};
    mHttpClient.collectHeaders(headerReq, 1);
    if (mApiToken.length() > 0)
        mHttpClient.addHeader("Authorization", "Bearer " + mApiToken);
    uint16_t status = mHttpClient.sendRequest("HEAD");

    uint8_t state = true;

    if(status != 200){
        //apiLog("Request POST %s returned %i", url, status);
        ESP_LOGE(TAG, "Got %i from request...", status);
        state = false;
    }

    if(mHttpClient.hasHeader("Etag")){
        outputEtag = mHttpClient.header("Etag");
        state = true;
    }else{
        state = false;
    }

    mHttpClient.end();
    return state;
}


/**
 * @brief Simple POST request. 
 *        will assemble the URL:
 *        if useDeviceId:
 *          mBaseURL + prefix + mDeviceId + "/" + suffix;
 *        else:
 *          mBaseUrl + prefix + suffx
 * 
 * @param prefix    the prefix for the request
 * @param suffix    the suffix for the request
 * @param contentType  the request data content-type
 * @param input     the request data
 * @param output    where to store the response data into
 * @param useDeviceId   whether to use the device id inbetween prefix and suffix
 * @return uint8_t  if the request was successful (return code == 200)
 */
uint8_t LuxIoT::apiPost(const String &prefix, const String &suffix, const String &contentType, const String &input, String &output, uint8_t useDeviceId){
    String url = mBaseURL + prefix;
    if(useDeviceId){
        url += String(mDeviceId) + "/";
    }
    url += suffix;
    ESP_LOGI(TAG, "Simple POST Request to %s", url.c_str());

    mHttpClient.begin(url, (const char*)mCertStart);
    mHttpClient.addHeader("Content-Type", contentType);
    if (mApiToken.length() > 0)
        mHttpClient.addHeader("Authorization", "Bearer " + mApiToken);
    uint16_t status = mHttpClient.POST(input);
    uint8_t state = true;

    if(status != 200){
        //apiLog("Request POST %s returned %i", url, status);
        ESP_LOGE(TAG, "Got %i from request...", status);
        state = false;
    }

    if(state){
        output = mHttpClient.getString();
    }

    mHttpClient.end();
    return state;
}

/**
 * @brief A function to log into the API Console. Use only after provisioning, otherwise it won't work.
 * 
 * @param format    the format string
 * @param ...       printf style vars
 * @return uint8_t  if the request was successful
 */
uint8_t LuxIoT::apiLog(const String &format, ...){
    const char * const zcFormat = format.c_str();

    va_list vaArgs;
    va_start(vaArgs, format);

    va_list vaCopy;
    va_copy(vaCopy, vaArgs);
    const int iLen = std::vsnprintf(NULL, 0, zcFormat, vaCopy);
    va_end(vaCopy);

    std::vector<char> zc(iLen + 1);
    std::vsnprintf(zc.data(), zc.size(), zcFormat, vaArgs);
    va_end(vaArgs);

    String buff = String(zc.data(), zc.size());

    String tmp;
    return apiPost("devices/", "log", "text/plain", buff, tmp, true);
}

/**
 * @brief begin telemetry section. clears the internal telemetry buffer.
 * 
 * @return uint8_t 
 */
uint8_t LuxIoT::beginTelemetry(uint8_t includeStd){
    lux_json_doc.clear();
    if(includeStd){
        lux_json_doc["times"]["wifi"] = mWifiConnectionMillis;
        lux_json_doc["times"]["overall"] = lux_last_runtime;
        lux_json_doc["wifi"]["ssid"] = WiFi.SSID();
        lux_json_doc["wifi"]["rssi"] = WiFi.RSSI();
        lux_json_doc["wifi"]["mac"] = WiFi.macAddress();
        lux_json_doc["wifi"]["channel"] = WiFi.channel();
        lux_json_doc["wifi"]["bssid"] = WiFi.BSSIDstr();
        lux_json_doc["sw"]["sdk"] = ESP.getSdkVersion();
        lux_json_doc["sw"]["rst_reason0"] = rtc_get_reset_reason(0);
        lux_json_doc["sw"]["rst_reason1"] = rtc_get_reset_reason(1);
        lux_json_doc["hw"]["model"] = ESP.getChipModel();
        lux_json_doc["hw"]["revision"] = ESP.getChipRevision();
        lux_json_doc["memory"]["flash_size"] = ESP.getFlashChipSize();
        lux_json_doc["memory"]["psram"]["size"] = ESP.getPsramSize();
        lux_json_doc["memory"]["psram"]["min_free"] = ESP.getMinFreePsram();
        lux_json_doc["memory"]["heap"]["size"] = ESP.getHeapSize();
        lux_json_doc["memory"]["heap"]["min_free"] = ESP.getMinFreeHeap();
    }
    return true;
}

/**
 * @brief return a reference to the internal json containing telemetry data.
 * 
 * @return JsonDocument&    the internal json document
 */
JsonDocument &LuxIoT::telemetry(){
    return lux_json_doc;
}

/**
 * @brief end telemetry section. Post the telemetry data to api console.
 * 
 * @return uint8_t  if the request was successful
 */
uint8_t LuxIoT::endTelemetry(){
    return apiTelemetry(lux_json_doc);
}

/**
 * @brief Post a json document as telemetry data to api console
 * 
 * @param jsonDocument  the json document to post
 * @return uint8_t  if the request was successful
 */
uint8_t LuxIoT::apiTelemetry(const JsonDocument &jsonDocument){
    String telemetry;
    String tmp;
    serializeJson(jsonDocument, telemetry);

    return apiPost("devices/", "telemetry", "application/json", telemetry, tmp, true);
}

uint8_t LuxIoT::_beginWifiInternalFast(const String &wifiSSID, const String &wifiPass, const uint8_t wifiChannel, const uint8_t* wifiBSSID){
    WiFi.begin(wifiSSID.c_str(), wifiPass.c_str(), wifiChannel, wifiBSSID, true);
    return _waitWifiConnection();
}

uint8_t LuxIoT::_beginWifiInternalNormal(const String &wifiSSID, const String &wifiPass){
    WiFi.begin(wifiSSID.c_str(), wifiPass.c_str());
    return _waitWifiConnection();
}

uint8_t LuxIoT::_waitWifiConnection(){
    uint16_t retries = 0;
	wl_status_t wifiStatus = WiFi.status();
	while( wifiStatus != WL_CONNECTED && retries < 1000) {
		delay( 10 );
        retries++;
		wifiStatus = WiFi.status();
	}

    return wifiStatus == WL_CONNECTED;
}

/**
 * @brief performs a regular sleep. 
 *        Performs some GPIO related energy saving stuff. Perhaps this needs to be outsourced to the application
 * 
 * @param sleepS    how many seconds to sleep
 */
void LuxIoT::regularSleep(uint64_t sleepS){
    ESP_LOGI(TAG, "Going to sleep for %d seconds", sleepS);
	SPIFFS.end();
	WiFi.disconnect(true);
	WiFi.mode(WIFI_OFF);

    if(this->mBeforeSleepCallback != nullptr){
        ((void(*)(uint64_t))this->mBeforeSleepCallback)(sleepS); // function pointer magic
    }

	lux_last_runtime = millis() - mMillisStart;

    esp_sleep_enable_timer_wakeup(sleepS * 1000000L);
	esp_deep_sleep_start();
}

/**
 * @brief performs a fallback sleep. Used in case a sleeptime can not be determined 
 *        because ther is no config in this device present
 * 
 */
void LuxIoT::fallbackSleep(){
    regularSleep(mFallbackSleeptime);
}

uint8_t LuxIoT::beginGlobalCA(const uint8_t* beginCert, const uint8_t* endCert){
    esp_tls_init_global_ca_store();
    esp_tls_set_global_ca_store(beginCert, endCert - beginCert);

    mCertStart = (uint8_t*)beginCert;
    mCertEnd = (uint8_t*)endCert;

    mHttpClient.setReuse(true);

    return true;
}