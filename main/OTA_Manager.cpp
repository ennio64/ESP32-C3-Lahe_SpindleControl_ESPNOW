#include "OTA_Manager.h"
#include <esp_wifi.h>
#include <esp_netif.h>
#include <esp_log.h>
#include <esp_ota_ops.h>
#include <esp_http_server.h>
#include <esp_system.h>
#include <esp_timer.h>
#include <driver/gpio.h>
#include <lwip/ip4_addr.h>
#include <cstring>
#include <cstdlib>
#include "Config.h"

static const char *TAG = "OTA_Manager";

httpd_handle_t OTAManager::server = nullptr;
bool OTAManager::otaEnabled = false;
unsigned long OTAManager::otaStartTime = 0;
led_strip_handle_t OTAManager::stripHandle = nullptr;

static const char OTA_INDEX_HTML[] =
    "<html><body>"
    "<h2>ESP32-C3 OTA Update</h2>"
    "<p>Invia il firmware come raw binary usando curl:</p>"
    "<pre>curl -X POST --data-binary @firmware.bin http://%s/update</pre>"
    "</body></html>";

static esp_err_t ota_root_get(httpd_req_t *req) {
    char buffer[256];
    snprintf(buffer, sizeof(buffer), OTA_INDEX_HTML, "192.168.5.1");
    httpd_resp_send(req, buffer, HTTPD_RESP_USE_STRLEN);
    return ESP_OK;
}

static esp_err_t ota_update_post(httpd_req_t *req) {
    if (req->content_len <= 0) {
        httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST, "Missing payload");
        return ESP_FAIL;
    }

    const esp_partition_t *update_partition = esp_ota_get_next_update_partition(NULL);
    if (!update_partition) {
        httpd_resp_send_err(req, HTTPD_500_INTERNAL_SERVER_ERROR, "No OTA partition");
        return ESP_FAIL;
    }

    esp_ota_handle_t ota_handle;
    esp_err_t err = esp_ota_begin(update_partition, OTA_SIZE_UNKNOWN, &ota_handle);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "esp_ota_begin failed (%d)", err);
        httpd_resp_send_err(req, HTTPD_500_INTERNAL_SERVER_ERROR, "OTA begin failed");
        return ESP_FAIL;
    }

    char buffer[1024];
    int remaining = req->content_len;
    while (remaining > 0) {
        int to_read = remaining;
        if (to_read > sizeof(buffer)) to_read = sizeof(buffer);
        int ret = httpd_req_recv(req, buffer, to_read);
        if (ret <= 0) {
            ESP_LOGE(TAG, "httpd_req_recv failed: %d", ret);
            esp_ota_end(ota_handle);
            httpd_resp_send_err(req, HTTPD_500_INTERNAL_SERVER_ERROR, "Download failed");
            return ESP_FAIL;
        }
        err = esp_ota_write(ota_handle, buffer, ret);
        if (err != ESP_OK) {
            ESP_LOGE(TAG, "esp_ota_write failed (%d)", err);
            esp_ota_end(ota_handle);
            httpd_resp_send_err(req, HTTPD_500_INTERNAL_SERVER_ERROR, "OTA write failed");
            return ESP_FAIL;
        }
        remaining -= ret;
    }

    err = esp_ota_end(ota_handle);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "esp_ota_end failed (%d)", err);
        httpd_resp_send_err(req, HTTPD_500_INTERNAL_SERVER_ERROR, "OTA end failed");
        return ESP_FAIL;
    }

    err = esp_ota_set_boot_partition(update_partition);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "esp_ota_set_boot_partition failed (%d)", err);
        httpd_resp_send_err(req, HTTPD_500_INTERNAL_SERVER_ERROR, "Boot partition failed");
        return ESP_FAIL;
    }

    httpd_resp_sendstr(req, "Firmware aggiornato, riavvio in corso...");
    vTaskDelay(pdMS_TO_TICKS(100));
    esp_restart();
    return ESP_OK;
}

void OTAManager::setup(led_strip_handle_t strip) {
    stripHandle = strip;
    ESP_LOGI(TAG, "Configuring OTA Access Point...");

    esp_netif_t* ap_netif = esp_netif_create_default_wifi_ap();
    if (!ap_netif) {
        ESP_LOGE(TAG, "Failed to create AP netif");
        return;
    }

    wifi_config_t ap_config = {};
    strncpy((char*)ap_config.ap.ssid, config.ota.ssid, sizeof(ap_config.ap.ssid) - 1);
    ap_config.ap.ssid_len = strlen(config.ota.ssid);
    strncpy((char*)ap_config.ap.password, config.ota.password, sizeof(ap_config.ap.password) - 1);
    ap_config.ap.channel = config.espNowChannel ? config.espNowChannel : 6;
    ap_config.ap.max_connection = 4;
    ap_config.ap.authmode = strlen(config.ota.password) > 0 ? WIFI_AUTH_WPA_WPA2_PSK : WIFI_AUTH_OPEN;

    esp_err_t err = esp_wifi_stop();
    if (err != ESP_OK && err != ESP_ERR_WIFI_NOT_STARTED) {
        ESP_LOGW(TAG, "esp_wifi_stop fallito: %d", err);
    }

    err = esp_wifi_set_mode(WIFI_MODE_AP);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "esp_wifi_set_mode failed: %d", err);
        return;
    }
    err = esp_wifi_set_config(WIFI_IF_AP, &ap_config);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "esp_wifi_set_config failed: %d", err);
        return;
    }
    err = esp_wifi_start();
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "esp_wifi_start failed: %d", err);
        return;
    }

    esp_netif_ip_info_t ip_info = {};
    ip_info.ip.addr = esp_ip4addr_aton(config.ota.localIP);
    ip_info.gw.addr = esp_ip4addr_aton(config.ota.gateway);
    ip_info.netmask.addr = esp_ip4addr_aton(config.ota.subnet);
    esp_netif_set_ip_info(ap_netif, &ip_info);

    httpd_config_t server_config = HTTPD_DEFAULT_CONFIG();
    server_config.server_port = config.ota.serverPort;

    err = httpd_start(&server, &server_config);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "httpd_start failed: %d", err);
        return;
    }

    httpd_uri_t root_uri = {
        .uri = "/",
        .method = HTTP_GET,
        .handler = ota_root_get,
        .user_ctx = NULL
    };
    httpd_register_uri_handler(server, &root_uri);

    httpd_uri_t update_uri = {
        .uri = "/update",
        .method = HTTP_POST,
        .handler = ota_update_post,
        .user_ctx = NULL
    };
    httpd_register_uri_handler(server, &update_uri);

    otaEnabled = true;
    otaStartTime = esp_timer_get_time() / 1000;
    ESP_LOGI(TAG, "OTA Update Ready: http://%s:%d/update", config.ota.localIP, config.ota.serverPort);
}

bool OTAManager::waitForOTARequest(led_strip_handle_t strip, unsigned long timeoutMs) {
    if (!strip) return false;
    setStripHandle(strip);               // imposta lo strip handle interno
    showPromptPattern();                 // pattern giallo/blu statico

    unsigned long start = esp_timer_get_time() / 1000;
    while ((esp_timer_get_time() / 1000) - start < timeoutMs) {
        if (gpio_get_level(static_cast<gpio_num_t>(config.spindleKey)) == 0) {
            return true;                 // tasto premuto
        }
        vTaskDelay(pdMS_TO_TICKS(10));
    }
    return false;                        // timeout scaduto senza pressione
}

void OTAManager::showPromptPattern() {
    if (!stripHandle) return;
    static const uint8_t patternBase[ledCount] = {
        0,0,0,1,1,1, 0,0,0,1,1,1, 0,0,0,1,1,1
    };
    for (int i = 0; i < ledCount; i++) {
        if (patternBase[i] == 0) {
            led_strip_set_pixel(stripHandle, i, 0, 0, 40);   // blu
        } else {
            led_strip_set_pixel(stripHandle, i, 40, 40, 0); // giallo
        }
    }
    led_strip_refresh(stripHandle);
}

void OTAManager::setStripHandle(led_strip_handle_t strip) {
    stripHandle = strip;
}

void OTAManager::update() {
    if (!otaEnabled) return;

    // Animazione LED: pattern 3 blu + 3 gialli rotante
    if (stripHandle) {
        static int offset = 0;
        static unsigned long lastUpdate = 0;
        const unsigned long speed = 120; // ms per step
        unsigned long now = esp_timer_get_time() / 1000;
        if (now - lastUpdate >= speed) {
            lastUpdate = now;
            //offset = (offset + 1) % ledCount; // scorrimento circolare antiorario
            offset = (offset - 1 + ledCount) % ledCount; // scorrimento circolare orario

            // Pattern base per 18 LED: tre blu (0), tre gialli (1), ripetuto 3 volte
            static const uint8_t patternBase[ledCount] = {
                0,0,0,1,1,1, 0,0,0,1,1,1, 0,0,0,1,1,1
            };

            led_strip_clear(stripHandle);
            for (int i = 0; i < ledCount; i++) {
                int patternIdx = (i + offset) % ledCount;
                uint8_t colorType = patternBase[patternIdx];
                uint8_t r, g, b;
                if (colorType == 0) {
                    r = 0; g = 0; b = 40;   // blu
                } else {
                    r = 40; g = 40; b = 0;   // giallo
                }
                led_strip_set_pixel(stripHandle, i, r, g, b);
            }
            led_strip_refresh(stripHandle);
        }
    }

    // Gestione uscita da OTA con pressione lunga del tasto spindleKey
    static unsigned long pressStart = 0;
    static bool wasPressed = false;
    bool pressed = (gpio_get_level(static_cast<gpio_num_t>(config.spindleKey)) == 0);

    if (pressed && !wasPressed) {
        pressStart = esp_timer_get_time() / 1000;
        wasPressed = true;
    }
    if (!pressed && wasPressed) {
        wasPressed = false;
    }
    if (pressed && wasPressed && ((esp_timer_get_time() / 1000) - pressStart > 2000)) {
        ESP_LOGI(TAG, "🔄 Uscita da OTA tramite pressione lunga...");
        disableOTA();
        vTaskDelay(pdMS_TO_TICKS(200));
        esp_restart();
    }

    // Lampeggio LED di reset (indicatore attività OTA)
    static unsigned long lastBlink = 0;
    static bool ledState = false;
    unsigned long now = esp_timer_get_time() / 1000;
    if (now - lastBlink > 500) {
        ledState = !ledState;
        gpio_set_level(static_cast<gpio_num_t>(config.ledPinReset), ledState ? 1 : 0);
        lastBlink = now;
    }
}

/*void OTAManager::update() {
    if (!otaEnabled) return;

    if (stripHandle) {
        static int idx = 0;
        static unsigned long lastUpdate = 0;
        const unsigned long speed = 120;
        unsigned long now = esp_timer_get_time() / 1000;
        if (now - lastUpdate >= speed) {
            lastUpdate = now;
            led_strip_clear(stripHandle);
            uint8_t strong_r = 0, strong_g = 0, strong_b = 50;
            uint8_t weak_r = 0, weak_g = 0, weak_b = 15;
            int count = ledCount;
            int i0 = idx % count;
            int iL = (idx + count - 1) % count;
            int iR = (idx + 1) % count;
            led_strip_set_pixel(stripHandle, i0, strong_r, strong_g, strong_b);
            led_strip_set_pixel(stripHandle, iL, weak_r, weak_g, weak_b);
            led_strip_set_pixel(stripHandle, iR, weak_r, weak_g, weak_b);
            led_strip_refresh(stripHandle);
            idx = (idx + 1) % count;
        }
    }

    static unsigned long pressStart = 0;
    static bool wasPressed = false;
    bool pressed = (gpio_get_level(static_cast<gpio_num_t>(config.spindleKey)) == 0);

    if (pressed && !wasPressed) {
        pressStart = esp_timer_get_time() / 1000;
        wasPressed = true;
    }
    if (!pressed && wasPressed) {
        wasPressed = false;
    }
    if (pressed && wasPressed && ((esp_timer_get_time() / 1000) - pressStart > 2000)) {
        ESP_LOGI(TAG, "🔄 Uscita da OTA tramite pressione lunga...");
        disableOTA();
        vTaskDelay(pdMS_TO_TICKS(200));
        esp_restart();
    }

    static unsigned long lastBlink = 0;
    static bool ledState = false;
    unsigned long now = esp_timer_get_time() / 1000;
    if (now - lastBlink > 500) {
        ledState = !ledState;
        gpio_set_level(static_cast<gpio_num_t>(config.ledPinReset), ledState ? 1 : 0);
        lastBlink = now;
    }
}*/

bool OTAManager::isOTAEnabled() {
    return otaEnabled;
}

void OTAManager::disableOTA() {
    if (!otaEnabled) return;
    if (server) {
        httpd_stop(server);
        server = nullptr;
    }
    esp_wifi_stop();
    otaEnabled = false;
    ESP_LOGI(TAG, "🔒 OTA disabled - AP turned off");
}

std::string OTAManager::getConnectionInfo() {
    if (!otaEnabled) return "OTA Disabled";
    char buffer[128];
    snprintf(buffer, sizeof(buffer), "SSID: %s\nIP: %s\nPass: %s", config.ota.ssid, config.ota.localIP, config.ota.password);
    return std::string(buffer);
}

unsigned long OTAManager::getOTAUptime() {
    return otaEnabled ? ((esp_timer_get_time() / 1000) - otaStartTime) : 0;
}