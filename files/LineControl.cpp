#include "LineControl.h"

void LineControl::begin() {
    reset();
}

void LineControl::reset() {
    integral = 0.0;
    lastError = 0.0;
    firstUpdate = true;
    lastUpdateMillis = millis();
}

float LineControl::computeCorrection(const SensorData &sensors) {
    float error;

    if (sensors.leftValid && sensors.rightValid) {
        // Ambas paredes disponiveis: erro classico left - right.
        // Se error > 0, o robo esta mais proximo da parede direita
        // (mais espaco a esquerda) -- convencao: erro positivo empurra
        // o robo para a esquerda.
        error = sensors.left - sensors.right;
    } else if (sensors.leftValid && !sensors.rightValid) {
        // Somente parede esquerda: manter distancia-alvo fixa dela.
        error = sensors.left - SINGLE_WALL_TARGET_CM;
    } else if (!sensors.leftValid && sensors.rightValid) {
        // Somente parede direita: manter distancia-alvo fixa dela
        // (sinal invertido pois "afastar da direita" = error negativo
        // na mesma convencao do caso com as duas paredes).
        error = -(sensors.right - SINGLE_WALL_TARGET_CM);
    } else {
        // Nenhuma referencia lateral valida -- nao corrigir.
        // Reseta o integrador para nao acumular erro de um contexto
        // anterior que nao se aplica mais.
        integral = 0.0;
        lastError = 0.0;
        return 0.0;
    }

    unsigned long nowMillis = millis();
    float dt;

    if (firstUpdate) {
        dt = 0.0;
        firstUpdate = false;
    } else {
        dt = (nowMillis - lastUpdateMillis) / 1000.0;
    }
    lastUpdateMillis = nowMillis;

    // Termo proporcional
    float pTerm = PID_KP * error;

    // Termo integral, com protecao anti-windup: so acumula se dt > 0
    // e limita o integrador diretamente (nao apenas a saida final),
    // que e a forma mais eficaz de evitar windup.
    if (dt > 0.0) {
        integral += error * dt;
    }
    float iTermRaw = PID_KI * integral;
    // Anti-windup: satura o termo integral dentro do limite de saida
    if (iTermRaw > PID_OUTPUT_LIMIT) {
        iTermRaw = PID_OUTPUT_LIMIT;
        integral = iTermRaw / (PID_KI != 0.0 ? PID_KI : 1.0);
    } else if (iTermRaw < -PID_OUTPUT_LIMIT) {
        iTermRaw = -PID_OUTPUT_LIMIT;
        integral = iTermRaw / (PID_KI != 0.0 ? PID_KI : 1.0);
    }

    // Termo derivativo, usando dt REAL (nao um valor fixo assumido)
    float dTerm = 0.0;
    if (dt > 0.0) {
        dTerm = PID_KD * (error - lastError) / dt;
    }
    lastError = error;

    float output = pTerm + iTermRaw + dTerm;

    // Limite final de saida
    if (output > PID_OUTPUT_LIMIT) output = PID_OUTPUT_LIMIT;
    if (output < -PID_OUTPUT_LIMIT) output = -PID_OUTPUT_LIMIT;

    return output;
}
