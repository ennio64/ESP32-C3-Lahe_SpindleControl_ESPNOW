#ifndef OTA_MANAGER_H
#define OTA_MANAGER_H

#include <string>
#include <esp_http_server.h>
#include "led_strip.h"

class OTAManager {
private:
    static httpd_handle_t server;
    static bool otaEnabled;
    static unsigned long otaStartTime;
    static led_strip_handle_t stripHandle;

    static esp_err_t handleRoot(httpd_req_t* req);
    static esp_err_t handleUpdate(httpd_req_t* req);
    static esp_err_t handleUpdatePost(httpd_req_t* req);

public:
    static void setup(led_strip_handle_t strip);
    static bool waitForOTARequest(led_strip_handle_t strip, unsigned long timeoutMs);
    static void showPromptPattern();   // pattern giallo/blu statico
    static void setStripHandle(led_strip_handle_t strip);
    static void update();
    static bool isOTAEnabled();
    static void disableOTA();
    static std::string getConnectionInfo();
    static unsigned long getOTAUptime();
};

#endif