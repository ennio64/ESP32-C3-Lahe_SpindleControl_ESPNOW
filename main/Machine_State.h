#ifndef MACHINE_STATE_H
#define MACHINE_STATE_H

#include <string>
#include <cstdlib>

enum MachineState {
    STATE_IDLE,
    STATE_RUN,
    STATE_HOLD,
    STATE_ALARM,
    STATE_UNKNOWN
};

struct Position {
    float x = 0;
    float y = 0;
    float z = 0;
    float a = 0;
};

struct MachineStatus {
    float x = 0.0;
    float y = 0.0;
    float z = 0.0;
    float a = 0.0;
    float rpm = 0.0;
    int maxFeedrateA = 2000;
    MachineState state = STATE_UNKNOWN;
    bool cncConnected = false;
    int lastRssi = -127;
    unsigned long lastRssiTime = 0;
    int rssiHistory[5] = {-127, -127, -127, -127, -127};
    int historyIndex = 0;
};

inline bool startsWith(const std::string& text, const std::string& prefix) {
    return text.size() >= prefix.size() && text.compare(0, prefix.size(), prefix) == 0;
}

inline MachineState parseMachineState(const std::string& status) {
    if (startsWith(status, "<Idle")) return STATE_IDLE;
    if (startsWith(status, "<Run")) return STATE_RUN;
    if (startsWith(status, "<Hold")) return STATE_HOLD;
    if (startsWith(status, "<Alarm")) return STATE_ALARM;
    return STATE_UNKNOWN;
}

inline float parseRPM(const std::string& status) {
    size_t fsIndex = status.find("FS:");
    if (fsIndex == std::string::npos) return 0;
    std::string fs = status.substr(fsIndex + 3);
    size_t end = fs.find('|');
    if (end != std::string::npos) fs = fs.substr(0, end);
    size_t comma = fs.find(',');
    if (comma == std::string::npos) return 0;
    return std::stof(fs.substr(comma + 1));
}

inline Position parsePosition(const std::string& status) {
    Position pos;
    size_t mposIndex = status.find("MPos:");
    if (mposIndex == std::string::npos) return pos;
    std::string coords = status.substr(mposIndex + 5);
    size_t end = coords.find('|');
    if (end != std::string::npos) coords = coords.substr(0, end);
    size_t idx1 = coords.find(',');
    size_t idx2 = coords.find(',', idx1 + 1);
    size_t idx3 = coords.find(',', idx2 + 1);
    if (idx1 != std::string::npos) pos.x = std::stof(coords.substr(0, idx1));
    if (idx2 != std::string::npos) pos.y = std::stof(coords.substr(idx1 + 1, idx2 - idx1 - 1));
    if (idx3 != std::string::npos) pos.z = std::stof(coords.substr(idx2 + 1, idx3 - idx2 - 1));
    if (idx3 != std::string::npos) pos.a = std::stof(coords.substr(idx3 + 1));
    return pos;
}

inline void updateMachineFromStatus(MachineStatus& machine, const std::string& status) {
    if (status.empty() || status[0] != '<') return;
    machine.state = parseMachineState(status);
    machine.rpm = parseRPM(status);
    Position pos = parsePosition(status);
    machine.x = pos.x;
    machine.y = pos.y;
    machine.z = pos.z;
    machine.a = pos.a;
}

#endif // MACHINE_STATE_H
