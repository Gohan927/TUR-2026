#include "Motors.h"

void Motors::begin() {
    pinMode(PIN_MOTOR_LEFT_IN1, OUTPUT);
    pinMode(PIN_MOTOR_LEFT_IN2, OUTPUT);
    pinMode(PIN_MOTOR_LEFT_ENA, OUTPUT);

    pinMode(PIN_MOTOR_RIGHT_IN3, OUTPUT);
    pinMode(PIN_MOTOR_RIGHT_IN4, OUTPUT);
    pinMode(PIN_MOTOR_RIGHT_ENB, OUTPUT);

    stopMotors();
}

// Garante que o valor final nunca ultrapasse MOTOR_MAX_PWM, evitando
// PWM invalido (>255) ou estresse eletrico desnecessario nos motores.
int Motors::clampSpeed(int speed) {
    if (speed > MOTOR_MAX_PWM) return MOTOR_MAX_PWM;
    if (speed < -MOTOR_MAX_PWM) return -MOTOR_MAX_PWM;
    return speed;
}

// Aplica velocidade e sentido a um motor especifico. Se speed != 0, o piso
// MOTOR_MIN_EFFECTIVE_PWM e respeitado (ver explicacao no header).
void Motors::applyMotor(int speed, uint8_t pinIn1, uint8_t pinIn2, uint8_t pinEnable) {
    speed = clampSpeed(speed);

    if (speed == 0) {
        digitalWrite(pinIn1, LOW);
        digitalWrite(pinIn2, LOW);
        analogWrite(pinEnable, 0);
        return;
    }

    bool forward = (speed > 0);
    int magnitude = abs(speed);

    // Aplica o piso minimo efetivo, preservando o sentido.
    if (magnitude < MOTOR_MIN_EFFECTIVE_PWM) {
        magnitude = MOTOR_MIN_EFFECTIVE_PWM;
    }

    digitalWrite(pinIn1, forward ? HIGH : LOW);
    digitalWrite(pinIn2, forward ? LOW : HIGH);
    analogWrite(pinEnable, magnitude);
}

void Motors::setMotorLeft(int speed) {
    applyMotor(speed, PIN_MOTOR_LEFT_IN1, PIN_MOTOR_LEFT_IN2, PIN_MOTOR_LEFT_ENA);
}

void Motors::setMotorRight(int speed) {
    applyMotor(speed, PIN_MOTOR_RIGHT_IN3, PIN_MOTOR_RIGHT_IN4, PIN_MOTOR_RIGHT_ENB);
}

void Motors::setMotors(int leftSpeed, int rightSpeed) {
    setMotorLeft(leftSpeed);
    setMotorRight(rightSpeed);
}

void Motors::stopMotors() {
    digitalWrite(PIN_MOTOR_LEFT_IN1, LOW);
    digitalWrite(PIN_MOTOR_LEFT_IN2, LOW);
    analogWrite(PIN_MOTOR_LEFT_ENA, 0);

    digitalWrite(PIN_MOTOR_RIGHT_IN3, LOW);
    digitalWrite(PIN_MOTOR_RIGHT_IN4, LOW);
    analogWrite(PIN_MOTOR_RIGHT_ENB, 0);
}
