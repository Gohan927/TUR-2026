/*
 * ============================================================================
 * Gyro.h / .cpp
 * ============================================================================
 * Responsavel por: comunicacao com o MPU6050, calibracao do offset do
 * giroscopio, e integracao da velocidade angular em angulo (eixo Z / Yaw).
 *
 * BIBLIOTECA ESCOLHIDA: MPU6050 de Jeff Rowberg (+ I2Cdev), SEM o DMP
 * (Digital Motion Processor).
 *
 * Por que sem DMP?
 *   - O DMP entrega um quaternion pronto e ja filtrado internamente pelo
 *     firmware do proprio chip -- e mais preciso a longo prazo, mas e uma
 *     "caixa preta": exige codigo de inicializacao bem mais complexo,
 *     manuseio de FIFO, interrupcoes dedicadas, e e mais dificil de
 *     depurar quando algo da errado.
 *   - Para curvas de 90/180 graus controladas em tempo real, o que
 *     realmente importa e a VELOCIDADE ANGULAR instantanea (graus/s) e sua
 *     integracao simples ao longo do tempo -- exatamente o que a leitura
 *     crua do giroscopio (getRotationZ) fornece.
 *   - Mantendo a mesma filosofia do Line Follower: simplicidade e
 *     confiabilidade antes de sofisticacao. O DMP seria uma otimizacao
 *     prematura aqui.
 *
 * IMPORTANTE (conforme o documento de especificacao):
 *   O giroscopio mede APENAS orientacao/velocidade angular, NUNCA posicao.
 *   Nao existe, em nenhum lugar deste modulo, integracao dupla do
 *   acelerometro para tentar obter posicao -- isso acumularia erro
 *   rapidamente e nao e confiavel. A posicao do robo no labirinto vem
 *   exclusivamente da logica de celulas (Maze.cpp), nunca do MPU6050.
 *
 * EIXO USADO: Z (Yaw). Isso pressupoe que o GY-521 esta montado com a
 * placa PARALELA AO CHAO do robo (deitada na horizontal), com os
 * componentes (chip MPU6050) voltados para cima ou para baixo -- nessa
 * montagem, uma rotacao do robo no plano horizontal (as curvas do
 * labirinto) corresponde exatamente a uma rotacao no eixo Z do sensor.
 * *** Se o GY-521 for montado em outra orientacao fisica (ex: na vertical,
 * encostado numa parede lateral do chassi), o eixo correto MUDA para X ou Y
 * -- isso PRECISA ser verificado fisicamente antes dos testes, girando o
 * robo manualmente 90 graus e observando qual eixo (getRotationX/Y/Z)
 * responde. ***
 * ============================================================================
 */

#ifndef GYRO_H
#define GYRO_H

#include <Arduino.h>
#include <Wire.h>
#include <MPU6050.h>
#include "Config.h"
#include "Types.h"

class Gyro {
public:
    void begin();

    // Executa a rotina de calibracao (robo DEVE estar parado e imovel).
    // Preenche gyroZOffset com a media de GYRO_CALIBRATION_SAMPLES leituras.
    void calibrate();

    // Deve ser chamado a cada ciclo do loop principal. Le a velocidade
    // angular atual, subtrai o offset de calibracao, e integra no tempo
    // (usando dt real, nao um valor fixo assumido) para atualizar o
    // angulo acumulado.
    void update();

    // Angulo acumulado atual (graus). Pode crescer/decrescer sem limite
    // (nao e normalizado automaticamente em 0-360) -- quem consome este
    // valor (Turning.cpp) e responsavel por calcular diferencas relativas.
    float getAngle() const;

    // Zera o angulo acumulado (usado no INICIO de cada curva, para que o
    // controle de curva sempre trabalhe com "quantos graus girei desde que
    // esta curva comecou", em vez de um angulo absoluto acumulado desde o
    // ligar do robo).
    void resetAngle();

    bool isCalibrated() const;

private:
    MPU6050 mpu;
    GyroState state;
    unsigned long lastUpdateMicros;
    bool firstUpdate;
};

#endif // GYRO_H
