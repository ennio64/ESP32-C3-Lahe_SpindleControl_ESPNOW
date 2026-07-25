#ifndef GLOBALS_H
#define GLOBALS_H

#include <string>
#include <cstring>
#include "Config.h"
#include "Machine_State.h"
#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"

extern Config config;
extern MachineStatus machine;
extern bool grblReady;

extern bool waitingForAck;
extern unsigned long ackTimeout;
extern std::string lastSentCommand;
extern int ackRetryCount;
extern const int MAX_ACK_RETRIES;
extern const unsigned long ACK_TIMEOUT_MS;

extern bool channelFound;
extern int currentScanChannel;
extern unsigned long lastChannelSwitch;
extern bool channelRequestSent;
extern unsigned long channelRequestTime;

extern bool rpmRequestSent;
extern unsigned long rpmRequestTime;
extern int maxSpindleRPM;

#define MAX_CMD_LEN 64

extern QueueHandle_t gcodeQueue;

struct CNCCommand {
    char gcode[MAX_CMD_LEN];
};

inline bool enqueueGCode(const std::string& g) {
    if (gcodeQueue == NULL) {
        printf("❌ enqueueGCode fallito: coda non pronta\n");
        return false;
    }
    CNCCommand cmd;
    strncpy(cmd.gcode, g.c_str(), MAX_CMD_LEN);
    cmd.gcode[MAX_CMD_LEN - 1] = '\0';
    return (xQueueSend(gcodeQueue, &cmd, portMAX_DELAY) == pdPASS);
}

void sendGCode(const std::string& cmd);

#endif // GLOBALS_H
