#include "Headers.h"
#include "SpindleControl.h"
#include <cstring>
#include <string>
#include <algorithm>
#include <esp_log.h>
#include <driver/gpio.h>
#include <esp_timer.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>

extern led_strip_handle_t global_led_strip;
extern MachineStatus machine;
extern Config config;

namespace SpindleControl {

void checkDirectionChange();

static const int8_t ENC_TABLE[16] = {
  0, -1, +1, 0,
  +1, 0, 0, -1,
  -1, 0, 0, +1,
  0, +1, -1, 0
};

volatile int8_t spindleAccum = 0;
volatile int8_t spindleSteps = 0;
volatile uint8_t spindleOldState = 0;
static portMUX_TYPE spindleMux = portMUX_INITIALIZER_UNLOCKED;

static int staticEncPinA = 0;
static int staticEncPinB = 0;

int pendingDelta = 0;
unsigned long lastMoveTime = 0;
int spindleMultiplier = 1;
unsigned long lastMultTime = 0;
const unsigned long multDebounce = 250;
unsigned long multiplierTimerStart = 0;
const unsigned long multiplierTimeout = 5000;
bool freezeRotation = false;
unsigned long freezeStart = 0;
const unsigned long freezeDuration = 5000;
bool lastPressWasLong = false;
int spinIndex = 0;
unsigned long lastSpinUpdate = 0;
const unsigned long spinInterval = 100;
static unsigned long lastButtonPressTime = 0;
static bool readyBlinkState = false;
static unsigned long lastBlinkTime = 0;
static bool lastCW = true;
static unsigned long lastDirCheck = 0;
const unsigned long dirDebounce = 80;

void showMultiplierLEDs() {
    uint8_t r=0, g=0, b=0;
    if (spindleMultiplier == 0) {
        r=0; g=10; b=0;
    } else if (spindleMultiplier == 1) {
        r=0; g=0; b=40;
    } else if (spindleMultiplier == 10) {
        r=40; g=40; b=0;
    } else if (spindleMultiplier == 100) {
        r=40; g=0; b=40;
    } else {
        r=0; g=20; b=0;
    }
    for (int i = 0; i < ledCount; i++) {
        led_strip_set_pixel(global_led_strip, i, r, g, b);
    }
    led_strip_refresh(global_led_strip);
}

/*void showReadyFeedback() {
    unsigned long now = esp_timer_get_time() / 1000;
    int rpm = static_cast<int>(machine.rpm);
    if (rpm <= 0) {
        if (now - lastButtonPressTime > 5000 && spindleMultiplier != 0 && spindleMultiplier != 1) {
            spindleMultiplier = 1;
            ESP_LOGI("SpindleControl", "Reset automatico: moltiplicatore riportato a 1");
        }
        if (now - lastButtonPressTime < 3000) {
            showMultiplierLEDs();
        } else {
            if (spindleMultiplier != 0) {
                spindleMultiplier = 0;
            }
            if (now - lastBlinkTime > 800) {
                lastBlinkTime = now;
                readyBlinkState = !readyBlinkState;
            }
            led_strip_clear(global_led_strip);
            uint8_t r = 0;
            uint8_t g = readyBlinkState ? 10 : 3;
            uint8_t b = 0;
            for (int i = 0; i < ledCount; i++) {
                led_strip_set_pixel(global_led_strip, i, r, g, b);
            }
            led_strip_refresh(global_led_strip);
        }
    } else {
        showMultiplierLEDs();
    }
}*/

void showReadyFeedback() {
    static bool lastBlinkState = false;
    static bool lastMultiplierVisible = false;
    unsigned long now = esp_timer_get_time() / 1000;
    int rpm = static_cast<int>(machine.rpm);

    if (rpm <= 0) {
        if (now - lastButtonPressTime > 5000 && spindleMultiplier != 0 && spindleMultiplier != 1) {
            spindleMultiplier = 1;
            ESP_LOGI("SpindleControl", "Reset automatico: moltiplicatore riportato a 1");
        }

        bool showMultiplier = (now - lastButtonPressTime < 3000);
        if (showMultiplier) {
            if (!lastMultiplierVisible) {
                showMultiplierLEDs();  // aggiorna solo all'ingresso della modalità
                lastMultiplierVisible = true;
                lastBlinkState = !readyBlinkState; // forza refresh allo switch
            }
            return;
        } else {
            if (lastMultiplierVisible) {
                lastMultiplierVisible = false;
                // reset dello stato per forzare il prossimo aggiornamento
                lastBlinkState = !readyBlinkState;
            }
            if (spindleMultiplier != 0) {
                spindleMultiplier = 0;
            }

            // Gestione lampeggio: cambia stato ogni 800ms
            if (now - lastBlinkTime > 800) {
                lastBlinkTime = now;
                readyBlinkState = !readyBlinkState;
            }

            // Aggiorna la striscia solo se lo stato di lampeggio è cambiato
            if (readyBlinkState != lastBlinkState) {
                lastBlinkState = readyBlinkState;
                uint8_t g = readyBlinkState ? 10 : 3;
                for (int i = 0; i < ledCount; i++) {
                    led_strip_set_pixel(global_led_strip, i, 0, g, 0);
                }
                led_strip_refresh(global_led_strip);
            }
        }
    } else {
        // mandrino in movimento: assicuriamoci di uscire dalla modalità ready
        if (lastMultiplierVisible || lastBlinkState) {
            lastMultiplierVisible = false;
            lastBlinkState = false;
        }
        showMultiplierLEDs();
    }
}

void showSpindleRotationLEDs(bool cw) {
    if (machine.state == STATE_ALARM) {
        return;
    }
    if (freezeRotation && (esp_timer_get_time() / 1000) - freezeStart > freezeDuration) {
        freezeRotation = false;
    }
    if (machine.rpm > 0) {
        checkDirectionChange();
    }
    if (machine.rpm <= 0) {
        showReadyFeedback();
        return;
    }
    if (freezeRotation) {
        showMultiplierLEDs();
        return;
    }
    unsigned long now = esp_timer_get_time() / 1000;
    if (now - lastSpinUpdate < spinInterval) {
        return;
    }
    lastSpinUpdate = now;
    if (cw) {
        spinIndex = (spinIndex + 1) % ledCount;
    } else {
        spinIndex = (spinIndex - 1 + ledCount) % ledCount;
    }
    led_strip_clear(global_led_strip);
    uint8_t cOn_r = 60, cOn_g = 0, cOn_b = 0;
    uint8_t cOff_r = 0, cOff_g = 0, cOff_b = 0;
    for (int i = 0; i < ledCount; i++) {
        int pos = (i - spinIndex + ledCount) % ledCount;
        bool isOn = (pos / 3) % 2 == 0;
        if (isOn) {
            led_strip_set_pixel(global_led_strip, i, cOn_r, cOn_g, cOn_b);
        } else {
            led_strip_set_pixel(global_led_strip, i, cOff_r, cOff_g, cOff_b);
        }
    }
    led_strip_refresh(global_led_strip);
}

void checkDirectionChange() {
    if (machine.rpm <= 0) return;
    unsigned long now = esp_timer_get_time() / 1000;
    if (now - lastDirCheck < dirDebounce) return;
    lastDirCheck = now;
    bool cw = isCW();
    if (cw != lastCW) {
        lastCW = cw;
        std::string base = cw ? "M3" : "M4";
        std::string cmd = base + " S" + std::to_string(static_cast<int>(machine.rpm));
        ESP_LOGI("SpindleControl", "Cambio direzione in moto → %s", cmd.c_str());
        sendGCode(cmd);
        vTaskDelay(pdMS_TO_TICKS(200));
    }
}

void IRAM_ATTR spindleAB_ISR(void *arg) {
    portENTER_CRITICAL_ISR(&spindleMux);
    uint8_t a = gpio_get_level(static_cast<gpio_num_t>(staticEncPinA));
    uint8_t b = gpio_get_level(static_cast<gpio_num_t>(staticEncPinB));
    uint8_t newState = (spindleOldState << 2) | ((a << 1) | b);
    spindleAccum = spindleAccum + ENC_TABLE[newState & 0x0F];
    if (spindleAccum > 3) {
        spindleSteps = spindleSteps + 1;
        spindleAccum = 0;
    } else if (spindleAccum < -3) {
        spindleSteps = spindleSteps - 1;
        spindleAccum = 0;
    }
    spindleOldState = newState;
    portEXIT_CRITICAL_ISR(&spindleMux);
}

void earlyInitLEDs() {
    led_strip_clear(global_led_strip);
    led_strip_refresh(global_led_strip);
    vTaskDelay(pdMS_TO_TICKS(250));
    for (int i = 0; i < ledCount; i++) {
        led_strip_set_pixel(global_led_strip, i, 0, 0, 40);
    }
    led_strip_refresh(global_led_strip);
}

void begin() {
    gpio_config_t io_conf = {};
    io_conf.intr_type = GPIO_INTR_ANYEDGE;
    io_conf.mode = GPIO_MODE_INPUT;
    io_conf.pin_bit_mask = (1ULL << config.spindleEncA) | (1ULL << config.spindleEncB);
    io_conf.pull_down_en = GPIO_PULLDOWN_DISABLE;
    io_conf.pull_up_en = GPIO_PULLUP_ENABLE;
    gpio_config(&io_conf);

    gpio_config_t io_conf_key = {};
    io_conf_key.intr_type = GPIO_INTR_DISABLE;
    io_conf_key.mode = GPIO_MODE_INPUT;
    io_conf_key.pin_bit_mask = (1ULL << config.spindleKey);
    io_conf_key.pull_down_en = GPIO_PULLDOWN_DISABLE;
    io_conf_key.pull_up_en = GPIO_PULLUP_ENABLE;
    gpio_config(&io_conf_key);

    staticEncPinA = config.spindleEncA;
    staticEncPinB = config.spindleEncB;

    spindleOldState = ((gpio_get_level(static_cast<gpio_num_t>(config.spindleEncA)) << 1) |
                       gpio_get_level(static_cast<gpio_num_t>(config.spindleEncB)));

    gpio_isr_handler_add(static_cast<gpio_num_t>(config.spindleEncA), spindleAB_ISR, NULL);
    gpio_isr_handler_add(static_cast<gpio_num_t>(config.spindleEncB), spindleAB_ISR, NULL);

    ESP_LOGI("SpindleControl", "SpindleControl encoder attivo");

    lastBlinkTime = esp_timer_get_time() / 1000;
    showMultiplierLEDs();
}

void updateFromEncoder() {
    if (gpio_get_level(static_cast<gpio_num_t>(config.brakePin)) == 1) {
        portENTER_CRITICAL(&spindleMux);
        spindleSteps = 0;
        portEXIT_CRITICAL(&spindleMux);
        pendingDelta = 0;
        return;
    }

    portENTER_CRITICAL(&spindleMux);
    int8_t steps = spindleSteps;
    spindleSteps = 0;
    portEXIT_CRITICAL(&spindleMux);

    if (steps != 0) {
        multiplierTimerStart = esp_timer_get_time() / 1000;
        pendingDelta += steps * spindleMultiplier;
        lastMoveTime = esp_timer_get_time() / 1000;
    }
}

bool isCW() {
    return gpio_get_level(static_cast<gpio_num_t>(config.spindleDirPin)) == 1;
}

bool isRotationFrozen() {
    return freezeRotation;
}

void updateFreezeState() {
    if (freezeRotation) {
        if ((esp_timer_get_time() / 1000) - freezeStart > freezeDuration) {
            freezeRotation = false;
        }
    }
}

void updateCommand() {
    if (pendingDelta == 0) return;

    unsigned long now = esp_timer_get_time() / 1000;

    if (now - lastMoveTime < 500)
        return;

    int currentRPM = machine.rpm;
    int maxRPM = machine.maxFeedrateA;

    int newRPM = currentRPM + pendingDelta;

    if (newRPM < 0) newRPM = 0;
    if (newRPM > maxRPM) newRPM = maxRPM;

    bool cw = isCW();
    std::string base = cw ? "M3" : "M4";

    if (newRPM <= 0) {
        sendGCode(base + " S0");
        sendGCode("M5");
        pendingDelta = 0;
        showMultiplierLEDs();
        return;
    }

    std::string cmd = base + " S" + std::to_string(newRPM);
    sendGCode(cmd);
    vTaskDelay(pdMS_TO_TICKS(200));
    pendingDelta = 0;
}

enum ButtonState {
    IDLE,
    DEBOUNCE_PRESS,
    PRESSED,
    LONG_PRESS,
    DEBOUNCE_RELEASE
};

ButtonState btnState = IDLE;
unsigned long btnTimer = 0;
const unsigned long debounceTime = 50;
const unsigned long longPressTime = 800;

void handleLongPress() {
    lastPressWasLong = true;
    sendGCode("M5");
    pendingDelta = 0;
    spindleMultiplier = 0;
    freezeRotation = true;
    freezeStart = esp_timer_get_time() / 1000;
    lastButtonPressTime = esp_timer_get_time() / 1000;
    ESP_LOGI("SpindleControl", "PRESSIONE LUNGA → STOP mandrino");
    showMultiplierLEDs();
}

void handleShortPress() {
    if (machine.state == STATE_ALARM) { return; }
    if (gpio_get_level(static_cast<gpio_num_t>(config.brakePin)) == 1) {
        ESP_LOGI("SpindleControl", "Freno attivo → ignorato short press");
        return;
    }

    unsigned long now = esp_timer_get_time() / 1000;
    int rpm = machine.rpm;
    lastButtonPressTime = now;

    if (rpm > 0) {
        if (!freezeRotation) {
            spindleMultiplier = 1;
            freezeRotation = true;
            freezeStart = now;
            multiplierTimerStart = now;
            ESP_LOGI("SpindleControl", "Pressione breve: freeze attivato, moltiplicatore = 1");
            showMultiplierLEDs();
            return;
        }
        if (spindleMultiplier == 1)
            spindleMultiplier = 10;
        else if (spindleMultiplier == 10)
            spindleMultiplier = 100;
        else
            spindleMultiplier = 1;
        freezeStart = now;
        multiplierTimerStart = now;
        ESP_LOGI("SpindleControl", "Moltiplicatore impostato: %d", spindleMultiplier);
        showMultiplierLEDs();
        return;
    }

    // Mandrino fermo
    if (now - lastButtonPressTime > 1000) {
        spindleMultiplier = 1;
        ESP_LOGI("SpindleControl", "Primo click mandrino fermo → moltiplicatore = 1");
    } else {
        if (spindleMultiplier == 1)
            spindleMultiplier = 10;
        else if (spindleMultiplier == 10)
            spindleMultiplier = 100;
        else
            spindleMultiplier = 1;
        ESP_LOGI("SpindleControl", "Ciclo moltiplicatore: %d", spindleMultiplier);
    }
    multiplierTimerStart = now;
    showMultiplierLEDs();
}

void updateMultiplier() {
    unsigned long now = esp_timer_get_time() / 1000;
    bool pressed = (gpio_get_level(static_cast<gpio_num_t>(config.spindleKey)) == 0);

    if (spindleMultiplier != 1 && spindleMultiplier != 0 && (now - multiplierTimerStart > multiplierTimeout)) {
        spindleMultiplier = 1;
        ESP_LOGI("SpindleControl", "Timeout: moltiplicatore riportato a 1");
        showMultiplierLEDs();
    }

    switch (btnState) {
        case IDLE:
            if (pressed) {
                btnState = DEBOUNCE_PRESS;
                btnTimer = now;
            }
            break;
        case DEBOUNCE_PRESS:
            if (!pressed) {
                btnState = IDLE;
            } else if (now - btnTimer >= debounceTime) {
                btnState = PRESSED;
                btnTimer = now;
            }
            break;
        case PRESSED:
            if (!pressed) {
                btnState = DEBOUNCE_RELEASE;
                btnTimer = now;
            } else if (now - btnTimer >= longPressTime) {
                handleLongPress();
                btnState = LONG_PRESS;
            }
            break;
        case LONG_PRESS:
            if (!pressed) {
                btnState = DEBOUNCE_RELEASE;
                btnTimer = now;
            }
            break;
        case DEBOUNCE_RELEASE:
            if (pressed) {
                btnState = PRESSED;
            } else if (now - btnTimer >= debounceTime) {
                if (lastPressWasLong) {
                    ESP_LOGI("SpindleControl", "Rilascio ignorato dopo pressione lunga");
                    lastPressWasLong = false;
                } else {
                    handleShortPress();
                }
                btnState = IDLE;
            }
            break;
    }
}

void showBeforeReadyLEDs() {
    static bool toggle = false;
    static unsigned long lastBlink = 0;
    const unsigned long blinkInterval = 300;

    unsigned long now = esp_timer_get_time() / 1000;
    if (now - lastBlink > blinkInterval) {
        lastBlink = now;
        toggle = !toggle;
    }

    led_strip_clear(global_led_strip);
    uint8_t c = toggle ? 20 : 0;
    for (int i = 0; i < ledCount; i++) {
        led_strip_set_pixel(global_led_strip, i, c, 0, 0);
    }
    led_strip_refresh(global_led_strip);
}

void showAlarmStateLEDs() {
    if (machine.state != STATE_ALARM) return;

    led_strip_clear(global_led_strip);
    for (int i = 0; i < ledCount; i++) {
        led_strip_set_pixel(global_led_strip, i, 60, 0, 0);
    }
    led_strip_refresh(global_led_strip);
}

} // namespace SpindleControl