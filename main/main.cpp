#include "Headers.h"
#include "SpindleControl.h"
#include "utils.h"
#include <string>
#include <vector>
#include <algorithm>
#include <cctype>
#include <cstring>
#include <esp_wifi.h>
#include <esp_now.h>
#include <esp_netif.h>
#include <esp_event.h>
#include <esp_log.h>
#include <nvs_flash.h>
#include <esp_system.h>
#include <esp_timer.h>
#include <esp_err.h>
#include <driver/gpio.h>
#include <atomic>

static const char *TAG = "MAIN";

Config config;
MachineStatus machine;
led_strip_handle_t global_led_strip = NULL;
QueueHandle_t gcodeQueue = NULL;
TaskHandle_t TaskMainHandle = NULL;
bool grblReady = false;

esp_now_peer_info_t peerInfoFrontend = {};
esp_now_peer_info_t peerInfoBridge = {};
uint8_t bridgeMac[6] = {0};
std::atomic<bool> paired{false};          // <-- ATOMICO
bool bridgePeerAdded = false;
uint16_t sequence = 0;
volatile uint16_t lastAckSeq = 0xFFFF;
unsigned long lastDataTime = 0;
std::atomic<unsigned long> packetCounter{0};
unsigned long pairStartTime = 0;
const unsigned long PAIR_TIMEOUT = 10000;
const unsigned long HEARTBEAT_TIMEOUT_MS = 5000;   // 5 secondi
const unsigned long ACK_TIMEOUT_MS = 3000;
unsigned long lastPairAttempt = 0;
const unsigned long PAIR_RETRY_INTERVAL_MS = 5000;   // riprova ogni 5 secondi

int maxSpindleRPM = 0;
bool waitingForRPMResponse = false;

bool waitingForBrakeAck = false;
unsigned long brakeAckTimeout = 0;
int brakeAckRetries = 0;
std::string lastBrakeMsg;
const int MAX_BRAKE_ACK_RETRIES = 5;
const unsigned long BRAKE_ACK_TIMEOUT_MS = 1000;

bool waitingForAck = false;
unsigned long ackTimeout = 0;
std::string lastSentCommand;
int ackRetryCount = 0;
const int MAX_ACK_RETRIES = 5;
const unsigned long GLOBAL_ACK_TIMEOUT_MS = 3000;

bool channelFound = false;
int currentScanChannel = 0;
unsigned long lastChannelSwitch = 0;
bool channelRequestSent = false;
unsigned long channelRequestTime = 0;

bool rpmRequestSent = false;
unsigned long rpmRequestTime = 0;

// Forward declarations
bool sendWithAck(const uint8_t *data, size_t len, uint16_t seq);
bool sendCommand(const std::string &cmd);
bool sendRealtime(uint8_t cmd);
void resetPairing();
void sendBrakeStatus();
bool checkOTARequest();
void initializeOTAMode();
void initializeNormalMode();
void initializeSerial();
void initializeGPIO();
void forceBootPinsHigh();
void onEspNowRecv(const esp_now_recv_info *info, const uint8_t *incomingData, int len);
void sendToFrontend(const std::string &cmd);
void sendGCode(const std::string &cmd);
void diagnoseNetwork();
void checkBrakeAckTimeout();

static std::string toUpper(std::string s)
{
    std::transform(s.begin(), s.end(), s.begin(), [](unsigned char c)
                   { return std::toupper(c); });
    return s;
}

bool sendWithAck(const uint8_t *data, size_t len, uint16_t seq)
{
    esp_err_t err = esp_now_send(bridgeMac, data, len);
    if (err != ESP_OK)
        return false;

    unsigned long t0 = getMillis();
    while (lastAckSeq != seq)
    {
        if (getMillis() - t0 > ACK_TIMEOUT_MS)
        {
            ESP_LOGW(TAG, "Timeout ACK");
            return false;
        }
        vTaskDelay(pdMS_TO_TICKS(1));
    }
    return true;
}

bool sendCommand(const std::string &cmd)
{
    if (!paired.load() || !bridgePeerAdded)
    {
        ESP_LOGW(TAG, "Non ancora associato al bridge");
        return false;
    }
    std::string command = cmd;
    if (command.empty() || command.back() != '\n')
    {
        command.push_back('\n');
    }
    command = toUpper(command);

    size_t len = command.size();
    if (len > 246)
        len = 246;

    std::vector<uint8_t> packet(len + 4);
    packet[0] = sequence & 0xFF;
    packet[1] = (sequence >> 8) & 0xFF;
    packet[2] = len & 0xFF;
    packet[3] = (len >> 8) & 0xFF;
    memcpy(packet.data() + 4, command.c_str(), len);

    bool ok = sendWithAck(packet.data(), len + 4, sequence);
    if (ok)
    {
        sequence++;
        ESP_LOGI(TAG, "Inviato al bridge: %s", command.c_str());
    }
    else
    {
        ESP_LOGE(TAG, "Invio fallito (ACK timeout): %s", command.c_str());
    }
    return ok;
}

bool sendRealtime(uint8_t cmd)
{
    if (!paired.load() || !bridgePeerAdded)
        return false;
    uint8_t packet[5] = {static_cast<uint8_t>(sequence & 0xFF), static_cast<uint8_t>((sequence >> 8) & 0xFF), 1, 0, cmd};
    bool ok = sendWithAck(packet, sizeof(packet), sequence);
    if (ok)
        sequence++;
    return ok;
}

void resetPairing()
{
    ESP_LOGI(TAG, "Reset pairing");
    paired.store(false);
    bridgePeerAdded = false;

    uint8_t zeroMac[6] = {0};
    if (memcmp(bridgeMac, zeroMac, sizeof(bridgeMac)) != 0)
    {
        esp_now_del_peer(bridgeMac);
    }

    memset(bridgeMac, 0, sizeof(bridgeMac));
    sequence = 0;
    lastAckSeq = 0xFFFF;
    // Invia un primo PAIR immediato
    uint8_t broadcastMac[] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF};
    esp_now_send(broadcastMac, reinterpret_cast<const uint8_t *>("PAIR"), 4);
    pairStartTime = getMillis();
    lastPairAttempt = getMillis();   // per evitare che il task principale invii subito un altro PAIR
}

void sendToFrontend(const std::string &cmd)
{
    esp_err_t result = esp_now_send(config.frontendMac, reinterpret_cast<const uint8_t *>(cmd.c_str()), cmd.size());
    if (result == ESP_OK)
    {
        ESP_LOGI(TAG, "Inviato al frontend: %s", cmd.c_str());
    }
    else
    {
        ESP_LOGE(TAG, "Invio al frontend fallito: %s", cmd.c_str());
    }
}

void sendBrakeStatus()
{
    bool brakeActive = (gpio_get_level(static_cast<gpio_num_t>(config.brakePin)) == 1);
    std::string brakeMsg = "BRAKE:" + std::to_string(brakeActive ? 1 : 0);
    if (waitingForBrakeAck && brakeMsg == lastBrakeMsg)
    {
        return;
    }
    lastBrakeMsg = brakeMsg;
    esp_err_t result = esp_now_send(config.frontendMac, reinterpret_cast<const uint8_t *>(brakeMsg.c_str()), brakeMsg.size());
    if (result == ESP_OK)
    {
        waitingForBrakeAck = true;
        brakeAckTimeout = getMillis() + BRAKE_ACK_TIMEOUT_MS;
        brakeAckRetries = 0;
        ESP_LOGI(TAG, "Inviato al frontend (in attesa ACK): %s", brakeMsg.c_str());
    }
    else
    {
        ESP_LOGE(TAG, "Invio al frontend fallito: %s", brakeMsg.c_str());
    }
}

void checkBrakeAckTimeout()
{
    if (!waitingForBrakeAck)
        return;
    if (getMillis() > brakeAckTimeout)
    {
        brakeAckRetries++;
        if (brakeAckRetries < MAX_BRAKE_ACK_RETRIES)
        {
            esp_err_t result = esp_now_send(config.frontendMac, reinterpret_cast<const uint8_t *>(lastBrakeMsg.c_str()), lastBrakeMsg.size());
            if (result == ESP_OK)
            {
                brakeAckTimeout = getMillis() + BRAKE_ACK_TIMEOUT_MS;
                ESP_LOGW(TAG, "Ritrasmissione freno (%d/%d)", brakeAckRetries, MAX_BRAKE_ACK_RETRIES);
            }
            else
            {
                waitingForBrakeAck = false;
                ESP_LOGE(TAG, "Invio ritrasmissione fallito");
            }
        }
        else
        {
            waitingForBrakeAck = false;
            ESP_LOGE(TAG, "ACK freno non ricevuto dopo ritrasmissioni max");
            sendCommand("M5");
        }
    }
}

void sendGCode(const std::string &cmd)
{
    if (!paired.load() || !bridgePeerAdded)
    {
        ESP_LOGW(TAG, "Non connesso al bridge, comando ignorato: %s", cmd.c_str());
        return;
    }
    sendCommand(cmd);
}

void onEspNowRecv(const esp_now_recv_info *info, const uint8_t *data, int len)
{
    const uint8_t *mac = info->src_addr;

    // Incrementa contatore pacchetti (per heartbeat passivo)
    unsigned long newVal = packetCounter.load() + 1;
    if (newVal >= 100000000)
        newVal = 0;
    packetCounter.store(newVal);

    // Risposta al ping
    if (len == 4 && memcmp(data, "PING", 4) == 0)
    {
        esp_now_send(info->src_addr, (const uint8_t *)"PONG", 4);
        return;
    }

    // ACK per i comandi inviati
    if (len == 3 && data[2] == 0x01)
    {
        uint16_t seq = data[0] | (data[1] << 8);
        lastAckSeq = seq;
        return;
    }

    // ACK dal frontend per il freno
    if (len == 9 && memcmp(data, "BRAKE_ACK", 9) == 0)
    {
        waitingForBrakeAck = false;
        ESP_LOGI(TAG, "ACK freno ricevuto dal frontend");
        return;
    }

    // Conferma pairing dal bridge
    if (len == 7 && memcmp(data, "PAIR_OK", 7) == 0)
    {
        memcpy(bridgeMac, mac, sizeof(bridgeMac));
        memcpy(peerInfoBridge.peer_addr, bridgeMac, sizeof(bridgeMac));
        peerInfoBridge.channel = config.espNowChannel;
        peerInfoBridge.encrypt = false;
        if (esp_now_add_peer(&peerInfoBridge) == ESP_OK)
        {
            bridgePeerAdded = true;
            paired.store(true);
            lastPairAttempt = 0;               // resetta il contatore dei tentativi
            ESP_LOGI(TAG, "Bridge trovato e aggiunto come peer");
            char macStr[18];
            snprintf(macStr, sizeof(macStr), "%02X:%02X:%02X:%02X:%02X:%02X",
                     bridgeMac[0], bridgeMac[1], bridgeMac[2],
                     bridgeMac[3], bridgeMac[4], bridgeMac[5]);
            ESP_LOGI(TAG, "MAC Bridge: %s", macStr);
        }
        return;
    }

    // Risposta al comando $30 (max RPM)
    if (len > 4 && memcmp(data, "$30=", 4) == 0)
    {
        std::string reply(reinterpret_cast<const char *>(data), len);
        int rpm = atoi(reply.substr(4).c_str());
        if (rpm > 0)
        {
            maxSpindleRPM = rpm;
            machine.maxFeedrateA = maxSpindleRPM;
            ESP_LOGI(TAG, "Max RPM ricevuto dal bridge: %d", maxSpindleRPM);
            waitingForRPMResponse = false;
        }
        return;
    }

    // Pacchetto di stato (inizia con '<')
    if (len > 0 && data[0] == '<')
    {
        std::string status(reinterpret_cast<const char *>(data), len);
        updateMachineFromStatus(machine, status);
        return;
    }
}

void taskCNC(void *parameter)
{
    vTaskDelay(pdMS_TO_TICKS(100));
    ESP_LOGI(TAG, "TaskCNC avviato");
    CNCCommand cmd;
    while (true)
    {
        if (gcodeQueue != NULL && xQueueReceive(gcodeQueue, &cmd, 0) == pdPASS)
        {
            sendCommand(std::string(cmd.gcode));
        }
        vTaskDelay(pdMS_TO_TICKS(10));
    }
}

void TaskMain(void *pvParameters)
{
    static unsigned long lastPacketCount = 0;
    static unsigned long lastPacketTime = 0;

    for (;;)
    {
        if (!grblReady)
        {
            SpindleControl::showBeforeReadyLEDs();
            vTaskDelay(pdMS_TO_TICKS(50));
            continue;
        }

        updateHoldStartLEDs();
        updateBrakeLED();
        updateResetLED();
        SpindleControl::showAlarmStateLEDs();
        SpindleControl::updateMultiplier();
        SpindleControl::updateFromEncoder();
        SpindleControl::updateCommand();
        SpindleControl::updateFreezeState();
        SpindleControl::showSpindleRotationLEDs(SpindleControl::isCW());
        checkBrakeAckTimeout();

        unsigned long now = getMillis();
        unsigned long currentCount = packetCounter.load();

        // Heartbeat passivo: rileva se i pacchetti sono fermi
        if (currentCount != lastPacketCount)
        {
            lastPacketCount = currentCount;
            lastPacketTime = now;
        }
        else
        {
            if (paired.load() && lastPacketTime != 0 && (now - lastPacketTime) > HEARTBEAT_TIMEOUT_MS)
            {
                ESP_LOGW(TAG, "Bridge lost (no packets for %lu ms), reset pairing", HEARTBEAT_TIMEOUT_MS);
                resetPairing();               // imposta paired=false, invia un PAIR
                lastPacketTime = 0;
                // Il tentativo di pairing verrà gestito anche dal blocco successivo
            }
        }

        // Ritentativi periodici di pairing se non siamo accoppiati
        if (!paired.load())
        {
            if (lastPairAttempt == 0 || (now - lastPairAttempt) >= PAIR_RETRY_INTERVAL_MS)
            {
                ESP_LOGI(TAG, "Retrying pairing...");
                // Pulisci il vecchio peer se esiste
                uint8_t zeroMac[6] = {0};
                if (memcmp(bridgeMac, zeroMac, sizeof(bridgeMac)) != 0)
                {
                    esp_now_del_peer(bridgeMac);
                    memset(bridgeMac, 0, sizeof(bridgeMac));
                }
                uint8_t broadcastMac[] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF};
                esp_now_send(broadcastMac, reinterpret_cast<const uint8_t *>("PAIR"), 4);
                lastPairAttempt = now;
            }
        }

        vTaskDelay(pdMS_TO_TICKS(1));
    }
}

void diagnoseNetwork()
{
    uint8_t primaryChan = 0;
    wifi_second_chan_t secondChan;
    esp_wifi_get_channel(&primaryChan, &secondChan);
    uint8_t mac[6];
    esp_wifi_get_mac(WIFI_IF_STA, mac);
    ESP_LOGI(TAG, "Configurazione ESP-NOW attuale:");
    ESP_LOGI(TAG, "Canale: %d", primaryChan);
    ESP_LOGI(TAG, "MAC Pendant: %02X:%02X:%02X:%02X:%02X:%02X", mac[0], mac[1], mac[2], mac[3], mac[4], mac[5]);
    char frontendMacStr[18];
    snprintf(frontendMacStr, sizeof(frontendMacStr), "%02X:%02X:%02X:%02X:%02X:%02X",
             config.frontendMac[0], config.frontendMac[1], config.frontendMac[2],
             config.frontendMac[3], config.frontendMac[4], config.frontendMac[5]);
    ESP_LOGI(TAG, "Frontend MAC statico: %s", frontendMacStr);
    if (paired.load())
    {
        ESP_LOGI(TAG, "Bridge MAC (pairing): %02X:%02X:%02X:%02X:%02X:%02X",
                 bridgeMac[0], bridgeMac[1], bridgeMac[2],
                 bridgeMac[3], bridgeMac[4], bridgeMac[5]);
    }
}

void initializeSerial()
{
    esp_log_level_set("*", ESP_LOG_INFO);
    ESP_LOGI(TAG, "CNC Pendant for GrblHAL - ESP32-C3 (protocollo frontend)");
}

void initializeGPIO()
{
    gpio_config_t io_conf = {};
    io_conf.mode = GPIO_MODE_INPUT;
    io_conf.pin_bit_mask = (1ULL << config.spindleDirPin) |
                           (1ULL << config.resetPin) |
                           (1ULL << config.spindleKey) |
                           (1ULL << config.brakePin);
    io_conf.pull_up_en = GPIO_PULLUP_ENABLE;
    gpio_config(&io_conf);

    io_conf.mode = GPIO_MODE_OUTPUT;
    io_conf.pin_bit_mask = (1ULL << config.ledPinReset) |
                           (1ULL << config.ledPinHold) |
                           (1ULL << config.ledPinStart) |
                           (1ULL << config.ledPinBrake);
    io_conf.pull_up_en = GPIO_PULLUP_DISABLE;
    gpio_config(&io_conf);

    gpio_set_level(static_cast<gpio_num_t>(config.ledPinReset), 0);
    gpio_set_level(static_cast<gpio_num_t>(config.ledPinHold), 0);
    gpio_set_level(static_cast<gpio_num_t>(config.ledPinStart), 0);
    gpio_set_level(static_cast<gpio_num_t>(config.ledPinBrake), 0);
    ESP_LOGI(TAG, "GPIO Initialized");
}

void forceBootPinsHigh()
{
    gpio_set_direction(GPIO_NUM_2, GPIO_MODE_OUTPUT);
    gpio_set_level(GPIO_NUM_2, 1);
    gpio_set_direction(GPIO_NUM_8, GPIO_MODE_OUTPUT);
    gpio_set_level(GPIO_NUM_8, 1);
    gpio_set_direction(GPIO_NUM_9, GPIO_MODE_OUTPUT);
    gpio_set_level(GPIO_NUM_9, 1);
    vTaskDelay(pdMS_TO_TICKS(80));
}

bool checkOTARequest()
{
    gpio_set_direction(static_cast<gpio_num_t>(config.spindleKey), GPIO_MODE_INPUT);
    gpio_set_pull_mode(static_cast<gpio_num_t>(config.spindleKey), GPIO_PULLUP_ONLY);
    vTaskDelay(pdMS_TO_TICKS(10));
    bool otaRequested = (gpio_get_level(static_cast<gpio_num_t>(config.spindleKey)) == 0);
    ESP_LOGI(TAG, "OTA Check → %s", otaRequested ? "YES" : "NO");
    return otaRequested;
}

void initializeOTAMode()
{
    ESP_LOGI(TAG, "Starting in OTA Mode (Access Point)...");
    OTAManager::setup(global_led_strip);
    while (true)
    {
        OTAManager::update();
        if (getMillis() > config.ota.timeout)
        {
            ESP_LOGI(TAG, "OTA Timeout - Restarting...");
            esp_restart();
        }
        vTaskDelay(pdMS_TO_TICKS(10));
    }
}

void initializeNormalMode()
{
    ESP_LOGI(TAG, "Starting in Normal Mode...");
    gcodeQueue = xQueueCreate(20, sizeof(CNCCommand));
    if (!gcodeQueue)
    {
        ESP_LOGE(TAG, "Errore: impossibile creare gcodeQueue");
        while (true) vTaskDelay(pdMS_TO_TICKS(1000));
    }
    SpindleControl::begin();
    machine.cncConnected = true;
    grblReady = true;
    xTaskCreatePinnedToCore(TaskMain, "TaskMain", 12288, NULL, 1, &TaskMainHandle, 0);
    xTaskCreatePinnedToCore(taskCNC, "TaskCNC", 8192, NULL, 2, NULL, 0);
    ESP_LOGI(TAG, "CNC Pendant Ready - Protocollo identico al frontend");
}

extern "C" void app_main(void)
{
    esp_err_t err = nvs_flash_init();
    if (err == ESP_ERR_NVS_NO_FREE_PAGES || err == ESP_ERR_NVS_NEW_VERSION_FOUND)
    {
        ESP_ERROR_CHECK(nvs_flash_erase());
        err = nvs_flash_init();
    }
    ESP_ERROR_CHECK(err);

    ESP_ERROR_CHECK(esp_netif_init());
    ESP_ERROR_CHECK(esp_event_loop_create_default());
    esp_netif_create_default_wifi_sta();

    wifi_init_config_t wifi_cfg = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_wifi_init(&wifi_cfg));
    ESP_ERROR_CHECK(esp_wifi_set_storage(WIFI_STORAGE_RAM));
    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_STA));
    ESP_ERROR_CHECK(esp_wifi_start());

    gpio_install_isr_service(0);

    forceBootPinsHigh();
    initializeSerial();
    initializeGPIO();
    vTaskDelay(pdMS_TO_TICKS(100));

    // Inizializza striscia LED
    led_strip_config_t strip_config = {
        .strip_gpio_num = ledPin,
        .max_leds = ledCount,
        .led_model = LED_MODEL_WS2812,
        .color_component_format = LED_STRIP_COLOR_COMPONENT_FMT_GRB,
        .flags = {.invert_out = false},
    };
    led_strip_rmt_config_t rmt_config = {
        .clk_src = RMT_CLK_SRC_DEFAULT,
        .resolution_hz = 10 * 1000 * 1000,
        .mem_block_symbols = 64,
        .flags = {.with_dma = false},
    };
    err = led_strip_new_rmt_device(&strip_config, &rmt_config, &global_led_strip);
    if (err != ESP_OK)
        ESP_LOGE(TAG, "led_strip_new_rmt_device failed: %d", err);
    else
        ESP_LOGI(TAG, "LED strip initialized on GPIO%d", ledPin);

    ESP_LOGI(TAG, "========== DIAGNOSTICA PULSANTI ==========");
    int spindleDir = gpio_get_level(static_cast<gpio_num_t>(config.spindleDirPin));
    int resetPin = gpio_get_level(static_cast<gpio_num_t>(config.resetPin));
    int spindleKey = gpio_get_level(static_cast<gpio_num_t>(config.spindleKey));
    int brakePin = gpio_get_level(static_cast<gpio_num_t>(config.brakePin));
    ESP_LOGI(TAG, "spindleDirPin (GPIO%d): %s", config.spindleDirPin, spindleDir ? "HIGH (CW)" : "LOW (CCW)");
    ESP_LOGI(TAG, "resetPin (GPIO%d): %s", config.resetPin, resetPin ? "HIGH (premuto)" : "LOW (rilasciato)");
    ESP_LOGI(TAG, "encoderKey (GPIO%d): %s", config.spindleKey, spindleKey ? "HIGH (rilasciato)" : "LOW (premuto)");
    ESP_LOGI(TAG, "brakePin (GPIO%d): %s", config.brakePin, brakePin ? "HIGH (freno bloccato)" : "LOW (freno disattivo)");
    ESP_LOGI(TAG, "==========================================");

    SpindleControl::earlyInitLEDs();

    if (OTAManager::waitForOTARequest(global_led_strip, 5000) && config.ota.enabled)
    {
        initializeOTAMode();
    }

    // LED blu fissi per modalità normale
    led_strip_clear(global_led_strip);
    for (int i = 0; i < ledCount; i++)
        led_strip_set_pixel(global_led_strip, i, 0, 0, 40);
    led_strip_refresh(global_led_strip);
    ESP_LOGI(TAG, "Nessun tasto premuto, procedo con modalità normale (LED blu)");

    // Scansione canale WiFi
    ESP_LOGI(TAG, "Ricerca canale ESP-NOW tramite reti WiFi conosciute...");
    const char *knownNetworks[] = {"TISCALI-5311", "TISCALI-07EE7E", "HUAWEI P30 lite"};
    const int knownCount = 3;
    int foundChannel = -1;
    std::string foundNetwork;
    int bestRSSI = -127;

    wifi_scan_config_t scan_config = {};
    scan_config.show_hidden = false;
    scan_config.scan_type = WIFI_SCAN_TYPE_ACTIVE;
    scan_config.scan_time.active.min = 100;
    scan_config.scan_time.active.max = 150;

    ESP_ERROR_CHECK(esp_wifi_scan_start(&scan_config, true));
    uint16_t apCount = 0;
    ESP_ERROR_CHECK(esp_wifi_scan_get_ap_num(&apCount));
    if (apCount > 0)
    {
        std::vector<wifi_ap_record_t> ap_records(apCount);
        ESP_ERROR_CHECK(esp_wifi_scan_get_ap_records(&apCount, ap_records.data()));
        ESP_LOGI(TAG, "Trovate %d reti WiFi", apCount);
        for (int i = 0; i < apCount; i++)
        {
            std::string ssid(reinterpret_cast<const char *>(ap_records[i].ssid));
            int channel = ap_records[i].primary;
            int rssi = ap_records[i].rssi;
            ESP_LOGI(TAG, "%s (Canale: %d, RSSI: %d dBm)", ssid.c_str(), channel, rssi);
            for (int j = 0; j < knownCount; j++)
            {
                if (ssid == knownNetworks[j] && rssi > bestRSSI)
                {
                    bestRSSI = rssi;
                    foundChannel = channel;
                    foundNetwork = ssid;
                }
            }
        }
    }

    ESP_ERROR_CHECK(esp_wifi_stop());

    if (foundChannel > 0)
    {
        ESP_LOGI(TAG, "Rete trovata: %s (Canale: %d, RSSI: %d dBm)", foundNetwork.c_str(), foundChannel, bestRSSI);
        config.espNowChannel = foundChannel;
    }
    else
    {
        ESP_LOGW(TAG, "Nessuna rete conosciuta, uso canale 6");
        config.espNowChannel = 6;
    }

    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_STA));
    ESP_ERROR_CHECK(esp_wifi_start());
    if (esp_now_init() != ESP_OK)
    {
        ESP_LOGE(TAG, "ESP-NOW init failed");
        esp_restart();
    }
    esp_now_register_recv_cb(onEspNowRecv);
    ESP_ERROR_CHECK(esp_wifi_set_channel(config.espNowChannel, WIFI_SECOND_CHAN_NONE));

    memcpy(peerInfoFrontend.peer_addr, config.frontendMac, sizeof(config.frontendMac));
    peerInfoFrontend.channel = config.espNowChannel;
    peerInfoFrontend.encrypt = false;
    if (esp_now_add_peer(&peerInfoFrontend) != ESP_OK)
        ESP_LOGE(TAG, "Frontend peer add failed");
    else
        ESP_LOGI(TAG, "Frontend peer aggiunto (MAC statico)");

    ESP_LOGI(TAG, "Avvio pairing con il bridge...");
    uint8_t broadcastMac[] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF};
    esp_now_peer_info_t broadcastPeer = {};
    memcpy(broadcastPeer.peer_addr, broadcastMac, sizeof(broadcastMac));
    broadcastPeer.channel = config.espNowChannel;
    broadcastPeer.encrypt = false;
    esp_now_add_peer(&broadcastPeer);
    esp_now_send(broadcastMac, reinterpret_cast<const uint8_t *>("PAIR"), 4);
    pairStartTime = getMillis();
    while (!paired.load() && getMillis() - pairStartTime < PAIR_TIMEOUT)
    {
        vTaskDelay(pdMS_TO_TICKS(10));
    }
    if (!paired.load())
    {
        ESP_LOGE(TAG, "Bridge non trovato entro il timeout, riavvio...");
        esp_restart();
    }

    ESP_LOGI(TAG, "Bridge associato");
    diagnoseNetwork();

    waitingForRPMResponse = true;
    ESP_LOGI(TAG, "Richiedo max RPM al bridge ($30)...");
    sendCommand("$30");
    unsigned long rpmStart = getMillis();
    while (waitingForRPMResponse && (getMillis() - rpmStart < 5000))
    {
        vTaskDelay(pdMS_TO_TICKS(10));
    }
    if (maxSpindleRPM == 0)
    {
        maxSpindleRPM = 1000;
        ESP_LOGW(TAG, "Max RPM non ricevuto, uso default 1000");
    }
    else
    {
        machine.maxFeedrateA = maxSpindleRPM;
    }

    sendBrakeStatus();
    initializeNormalMode();
}