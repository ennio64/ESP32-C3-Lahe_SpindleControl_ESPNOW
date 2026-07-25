#ifndef UTILS_H
#define UTILS_H

#include <esp_timer.h>

/**
 * @brief Centralizzata funzione per ottenere millisecondi dal boot
 * @return unsigned long millisecondi dal boot del sistema
 */
inline unsigned long getMillis() {
    return esp_timer_get_time() / 1000;
}

#endif // UTILS_H
