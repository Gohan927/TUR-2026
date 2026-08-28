/*
 * ============================================================================
 * Motors.h / .cpp
 * ============================================================================
 * Camada de abstracao dos motores via L298N. Nenhum outro modulo do
 * projeto deve escrever diretamente em pinos de motor -- toda a logica de
 * labirinto, navegacao e curvas fala apenas com esta interface.
 *
 * IMPORTANTE - L298N + motores N20:
 *   O L298N e baseado em transistores bipolares (nao MOSFET), o que causa
 *   uma queda de tensao interna de aproximadamente 2V por canal. Para
 *   motores N20 (tipicamente 3-6V, baixa corrente), essa queda e
 *   proporcionalmente significativa e reduz a faixa util de PWM.
 *   Por isso existe MOTOR_MIN_EFFECTIVE_PWM em Config.h: valores de PWM
 *   abaixo desse piso podem ser insuficientes para vencer o atrito estatico
 *   do motor+redutor, fazendo o motor simplesmente nao girar mesmo com PWM
 *   maior que zero. Esta classe aplica esse piso automaticamente sempre que
 *   a velocidade solicitada for diferente de zero.
 * ============================================================================
 */

#ifndef MOTORS_H
#define MOTORS_H

#include <Arduino.h>
#include "Config.h"

class Motors {
public:
    void begin();

    // speed: -255 a +255. Sinal define o sentido (positivo = frente).
    // O piso MOTOR_MIN_EFFECTIVE_PWM e aplicado automaticamente.
    void setMotorLeft(int speed);
    void setMotorRight(int speed);

    // Define os dois motores de uma vez (uso mais comum).
    void setMotors(int leftSpeed, int rightSpeed);

    void stopMotors();

private:
    int clampSpeed(int speed);
    void applyMotor(int speed, uint8_t pinIn1, uint8_t pinIn2, uint8_t pinEnable);
};

#endif // MOTORS_H
