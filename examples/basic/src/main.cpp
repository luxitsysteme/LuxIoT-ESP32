#include "Arduino.h"

#include "HTTPClient.h"
#include "LuxIoT.h"

#define WIFI_SSID                "<Your SSID>"
#define WIFI_PW                  "<Your Wifi Password>"

#define API_BASE_URL             "https://api.cloud.lux-it-systeme.de/v1/"
#define API_PROVISION_SUFFIX     "projects/<Your Project ID>/provision"
#define API_PROVISION_TOKEN      "<Your Provision Token>" 

extern const uint8_t cert_pem_start[] asm("_binary_cert_root_pem_start");
extern const uint8_t cert_pem_end[]   asm("_binary_cert_root_pem_end");

HTTPClient httpClient;

// Set the values here according to the documentation. It is recommended to keep the watchdog timeout > 60s in order for the OTA functionality to function
// Upcoming revisions should handle this better but for now just set it to a value > 60s.
LuxIoT luxIoT(API_BASE_URL,
              API_PROVISION_SUFFIX,
              API_PROVISION_TOKEN,
              300, 30, httpClient, nullptr);

void setup(){
    // Initialize Wwatchdog and other settings related to the reliability of your device
    luxIoT.begin();
    // Begin Wifi connection, highly optimized for quick connects
    luxIoT.beginWifi(WIFI_SSID, WIFI_PW);

    // setup a global certificate to use as a trust chain root. 
    // Note: Don't use your short-lived public certificate here, use the root cert above instead.
    luxIoT.beginGlobalCA(cert_pem_start, cert_pem_end);

    // Ensure that the device is provisioned
    luxIoT.ensureProvision();

    // Ensure that the device has the latest firmware installed
    luxIoT.ensureOtaFirmware();

    // Insert your device logic here. By doing so you minimize the risk of bricking your device because of broken logic.
    // An OTA update can still be performed.

    // Begin telemetry block. Allows you to use the internal json document for memory efficiency
    luxIoT.beginTelemetry(true);

    // Set items in the telemetry like this.
    // Note: It is highly recommended to only set variables here and 
    // calculate them between the ensureOtaFirmware() and beginTelemetry() function call.
    luxIoT.telemetry()["time"]["current"] = millis();

    // End telemetry block. Sends the telemetry data to the cloud
    luxIoT.endTelemetry();

    // going to deep sleep for a battery operated device. You could omit this but then you must make sure to call ensureOtaFirmware() regularly.
    luxIoT.regularSleep(60);
}

// Not used because of deep sleep
void loop(){}