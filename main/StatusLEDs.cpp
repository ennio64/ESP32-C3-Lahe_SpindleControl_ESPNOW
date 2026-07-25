#include "Headers.h"
#include "StatusLEDs.h"
#include <esp_log.h>
#include <driver/gpio.h>
#include <esp_timer.h>

extern Config config;
extern MachineStatus machine;
extern void sendToFrontend(const std::string &cmd);

static unsigned long lastBlink = 0;
static bool blinkState = false;
static const unsigned long blinkInterval = 300;

void updateHoldStartLEDs() {
    if (machine.state == STATE_HOLD) {
        gpio_set_level(static_cast<gpio_num_t>(config.ledPinHold), 1);
        unsigned long now = esp_timer_get_time() / 1000;
        if (now - lastBlink > blinkInterval) {
            lastBlink = now;
            blinkState = !blinkState;
            gpio_set_level(static_cast<gpio_num_t>(config.ledPinStart), blinkState ? 1 : 0);
        }
    } else {
        gpio_set_level(static_cast<gpio_num_t>(config.ledPinHold), 0);
        gpio_set_level(static_cast<gpio_num_t>(config.ledPinStart), 0);
    }
}

void updateBrakeLED() {
    static unsigned long lastBrakeStopTime = 0;
    static bool lastBrakeState = false;
    const unsigned long brakeStopInterval = 200;
    bool brakeState = gpio_get_level(static_cast<gpio_num_t>(config.brakePin));

    if (brakeState != lastBrakeState) {
        lastBrakeState = brakeState;
        std::string brakeMsg = "BRAKE:" + std::to_string(brakeState ? 1 : 0);
        sendToFrontend(brakeMsg);
        ESP_LOGI("StatusLEDs", "🔧 Stato freno inviato: %s (%s)",
                 brakeMsg.c_str(), brakeState ? "ATTIVO" : "DISATTIVO");
    }

    if (brakeState == 1) {
        if (machine.rpm > 0) {
            unsigned long now = esp_timer_get_time() / 1000;
            if (now - lastBrakeStopTime >= brakeStopInterval) {
                lastBrakeStopTime = now;
                sendGCode("M5");
                ESP_LOGI("StatusLEDs", "Freno bloccato → STOP mandrino");
            }
        }

        static unsigned long localLastBlink = 0;
        static bool localBlinkState = false;
        unsigned long now = esp_timer_get_time() / 1000;
        if (now - localLastBlink > blinkInterval) {
            localLastBlink = now;
            localBlinkState = !localBlinkState;
            gpio_set_level(static_cast<gpio_num_t>(config.ledPinBrake), localBlinkState ? 1 : 0);
        }
    } else {
        gpio_set_level(static_cast<gpio_num_t>(config.ledPinBrake), 1);
    }
}

void updateResetLED() {
    static unsigned long lastBlinkReset = 0;
    static bool blinkStateReset = false;
    static bool alreadyHandled = false;
    const unsigned long blinkIntervalReset = 300;
    bool resetPressed = (gpio_get_level(static_cast<gpio_num_t>(config.resetPin)) == 1);

    if (machine.state == STATE_ALARM) {
        unsigned long now = esp_timer_get_time() / 1000;
        if (now - lastBlinkReset > blinkIntervalReset) {
            lastBlinkReset = now;
            blinkStateReset = !blinkStateReset;
        }
        gpio_set_level(static_cast<gpio_num_t>(config.ledPinReset), blinkStateReset ? 1 : 0);
    } else {
        gpio_set_level(static_cast<gpio_num_t>(config.ledPinReset), 0);
    }

    if (resetPressed && !alreadyHandled) {
        alreadyHandled = true;
        if (machine.state == STATE_ALARM) {
            ESP_LOGI("StatusLEDs", "🔧 RESET: ALARM → invio $X");
            sendGCode("$X");
        } else {
            ESP_LOGI("StatusLEDs", "🔧 RESET: soft reset");
            sendGCode("\x18");
        }
    } else if (!resetPressed) {
        alreadyHandled = false;
    }
}
