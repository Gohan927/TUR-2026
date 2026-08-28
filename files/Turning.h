/*
 * ============================================================================
 * Turning.h / .cpp
 * ============================================================================
 * Controle de curvas (90 graus esquerda/direita, 180 graus) usando o
 * giroscopio como feedback -- NUNCA delay() para definir duracao de curva.
 *
 * Por que nao usar delay() fixo?
 *   O tempo real de uma curva varia com: carga da bateria, atrito das
 *   rodas/redutor, peso do robo, tipo de piso, tensao momentanea dos
 *   motores. Um delay(500) fixo funciona hoje e falha amanha com a
 *   bateria mais fraca. O giroscopio mede o angulo real girado, entao a
 *   curva termina quando o angulo real atinge o alvo -- independente de
 *   quanto tempo isso levou.
 *
 * ESTRATEGIA:
 *   1. Zera o angulo acumulado do giroscopio (referencia local a esta curva).
 *   2. Gira os motores em sentidos opostos (giro no proprio eixo).
 *   3. A cada ciclo, atualiza o giroscopio e verifica o angulo ja percorrido.
 *   4. Quando falta pouco para o alvo (TURN_SLOWDOWN_ANGLE_DEG), reduz a
 *      velocidade (TURN_SPEED_SLOW) para nao ultrapassar o alvo por inercia.
 *   5. Para quando |anguloAtual - anguloAlvo| <= TURN_TOLERANCE_DEG.
 *   6. Timeout de seguranca (TURN_TIMEOUT_MS) caso o giroscopio falhe ou
 *      o robo esteja fisicamente preso -- evita loop infinito.
 * ============================================================================
 */

#ifndef TURNING_H
#define TURNING_H

#include <Arduino.h>
#include <math.h>
#include "Config.h"
#include "Gyro.h"
#include "Motors.h"

enum TurnResult {
    TURN_IN_PROGRESS,
    TURN_COMPLETED,
    TURN_TIMEOUT_ERROR
};

class Turning {
public:
    void begin(Gyro *gyroRef, Motors *motorsRef);

    // Inicia uma nova curva (zera o angulo de referencia do giroscopio).
    // direction: +1 para direita (sentido horario), -1 para esquerda.
    void startTurn(float targetDegrees, int direction);

    // Deve ser chamado repetidamente (nao bloqueante) ate retornar
    // TURN_COMPLETED ou TURN_TIMEOUT_ERROR.
    TurnResult update();

private:
    Gyro *gyro;
    Motors *motors;

    float targetAngle;
    int turnDirection; // +1 ou -1
    unsigned long turnStartMillis;
};

#endif // TURNING_H
