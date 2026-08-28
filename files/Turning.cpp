#include "Turning.h"

void Turning::begin(Gyro *gyroRef, Motors *motorsRef) {
    gyro = gyroRef;
    motors = motorsRef;
}

void Turning::startTurn(float targetDegrees, int direction) {
    targetAngle = targetDegrees;
    turnDirection = direction;
    gyro->resetAngle(); // referencia local: "graus girados desde o inicio DESTA curva"
    turnStartMillis = millis();
}

TurnResult Turning::update() {
    gyro->update();

    float currentAngle = fabs(gyro->getAngle());
    float remaining = targetAngle - currentAngle;

    // --- Timeout de seguranca ---
    if (millis() - turnStartMillis > TURN_TIMEOUT_MS) {
        motors->stopMotors();
        return TURN_TIMEOUT_ERROR;
    }

    // --- Curva concluida dentro da tolerancia ---
    if (fabs(remaining) <= TURN_TOLERANCE_DEG) {
        motors->stopMotors();
        return TURN_COMPLETED;
    }

    // --- Define velocidade: desacelera perto do alvo ---
    int speed;
    if (remaining <= TURN_SLOWDOWN_ANGLE_DEG) {
        speed = TURN_SPEED_SLOW;
    } else {
        speed = TURN_SPEED;
    }

    // --- Aplica giro no proprio eixo (motores em sentidos opostos) ---
    if (turnDirection > 0) {
        // Curva para a DIREITA (sentido horario visto de cima):
        // motor esquerdo avanca, motor direito recua.
        motors->setMotors(speed, -speed);
    } else {
        // Curva para a ESQUERDA (sentido anti-horario):
        // motor esquerdo recua, motor direito avanca.
        motors->setMotors(-speed, speed);
    }

    return TURN_IN_PROGRESS;
}
