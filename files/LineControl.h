/*
 * ============================================================================
 * LineControl.h / .cpp
 * ============================================================================
 * PID de correcao lateral durante o movimento em linha reta, usando a
 * diferenca entre as distancias lidas pelos sensores ultrassonicos
 * esquerdo e direito.
 *
 * DECISAO DE PROJETO: por que um PID aqui, mas simples (Ki=0 por padrao)?
 *   - Kp (proporcional): faz o essencial -- corrige proporcionalmente ao
 *     quao desalinhado o robo esta no corredor. E o termo mais importante.
 *   - Kd (derivativo): amortece a correcao, evitando que o robo "zigue-
 *     zague" de um lado para o outro do corredor (oscilacao).
 *   - Ki (integral) comeca em 0: nao ha razao estrutural para um erro
 *     residual permanente em linha reta (ao contrario de um Line Follower
 *     seguindo fita, aqui o "erro zero" e simplesmente "meio do corredor").
 *     Um Ki desnecessario so adiciona risco de windup e complexidade de
 *     calibracao sem beneficio real. Deixado configuravel em Config.h caso
 *     os testes fisicos mostrem um viés sistematico (ex: um motor mais
 *     fraco que o outro).
 *
 * QUANDO NAO HA PAREDES LATERAIS SUFICIENTES:
 *   Se apenas uma parede lateral estiver disponivel, usamos so ela como
 *   referencia (mantendo uma distancia-alvo fixa dessa parede).
 *   Se NENHUMA parede lateral estiver disponivel, NAO tentamos corrigir --
 *   o PID e mantido em repouso (saida 0) e o robo anda reto usando apenas
 *   a velocidade base, confiando no giroscopio para nao desviar angularmente
 *   (ver Gyro.h). Corrigir com base em valores inexistentes seria pior do
 *   que nao corrigir.
 * ============================================================================
 */

#ifndef LINE_CONTROL_H
#define LINE_CONTROL_H

#include <Arduino.h>
#include "Config.h"
#include "Types.h"

class LineControl {
public:
    void begin();
    void reset(); // zera integrador e erro anterior (chamar ao iniciar cada trecho reto)

    // Calcula a correcao (-PID_OUTPUT_LIMIT a +PID_OUTPUT_LIMIT) a ser
    // somada/subtraida da velocidade base de cada motor.
    // Retorna 0.0 se nao houver referencia lateral suficiente.
    float computeCorrection(const SensorData &sensors);

private:
    float integral;
    float lastError;
    unsigned long lastUpdateMillis;
    bool firstUpdate;
};

// Distancia-alvo assumida quando apenas UM lado tem parede detectada
// (metade da largura da pista, ja que o robo deveria estar centralizado).
// Definida fora da classe (em vez de "static constexpr" membro) por
// compatibilidade maxima com o avr-gcc usado pelo Arduino IDE.
#define SINGLE_WALL_TARGET_CM 20.0

#endif // LINE_CONTROL_H
