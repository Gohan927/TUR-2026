#include "Safety.h"

bool Safety::isFrontObstacleTooClose(const SensorData &sensors) const {
    if (!sensors.frontValid) return false; // incerteza -- nao afirma perigo
    return (sensors.front < FRONT_SAFETY_DISTANCE_CM);
}

void Safety::startMovementTimeout() {
    movementStartMillis = millis();
}

bool Safety::isMovementTimeoutExceeded() const {
    return (millis() - movementStartMillis) > MOVEMENT_TIMEOUT_MS;
}
