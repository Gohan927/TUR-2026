/*
 * ============================================================================
 * Types.h
 * ============================================================================
 * Tipos, structs e enums compartilhados entre os modulos do projeto.
 * Mantidos em um unico lugar para evitar definicoes duplicadas ou
 * inconsistentes entre arquivos.
 * ============================================================================
 */

#ifndef TYPES_H
#define TYPES_H

#include <Arduino.h>

// ----------------------------------------------------------------------------
// Leitura filtrada dos sensores ultrassonicos + deteccao de parede
// ----------------------------------------------------------------------------
struct SensorData {
    float left;
    float front;
    float right;

    bool leftValid;
    bool frontValid;
    bool rightValid;

    bool wallLeft;
    bool wallFront;
    bool wallRight;
};

// ----------------------------------------------------------------------------
// Orientacao FISICA do robo (medida pelo giroscopio, eixo Z / Yaw).
// Este e um conceito DIFERENTE da orientacao LOGICA (Direction, abaixo).
// O MPU6050 sabe "quantos graus o chassi girou desde a calibracao".
// O enum Direction sabe "para onde o robo aponta DENTRO DO MAPA".
// Sao sincronizados a cada curva, mas nunca confundidos no codigo.
// ----------------------------------------------------------------------------
struct GyroState {
    float angleZ;          // angulo acumulado (graus), referencia arbitraria
    float gyroZOffset;     // offset de calibracao do eixo Z
    bool calibrated;
};

// ----------------------------------------------------------------------------
// Orientacao LOGICA do robo dentro do labirinto.
// ----------------------------------------------------------------------------
enum Direction {
    NORTH = 0,
    EAST  = 1,
    SOUTH = 2,
    WEST  = 3
};

// ----------------------------------------------------------------------------
// Representacao de uma celula do labirinto.
// As paredes sao armazenadas em referencia ABSOLUTA (Norte/Sul/Leste/Oeste
// do mapa), nao relativa ao robo -- isso evita confusao ao converter
// leituras "esquerda/frente/direita" (relativas ao robo) para o mapa.
// ----------------------------------------------------------------------------
struct Cell {
    bool wallNorth;
    bool wallEast;
    bool wallSouth;
    bool wallWest;

    uint8_t floodValue;   // distancia (em celulas) ate o objetivo, via Flood Fill
    bool visited;
};

// ----------------------------------------------------------------------------
// Maquina de estados principal do robo.
// ----------------------------------------------------------------------------
enum RobotState {
    STATE_STARTUP,
    STATE_CALIBRATING,
    STATE_WAIT_FOR_START,
    STATE_READ_SENSORS,
    STATE_UPDATE_MAP,
    STATE_CHOOSE_DIRECTION,
    STATE_MOVE_FORWARD,
    STATE_TURN_LEFT,
    STATE_TURN_RIGHT,
    STATE_TURN_BACK,
    STATE_GOAL_REACHED,
    STATE_EMERGENCY_STOP,
    STATE_ERROR
};

// ----------------------------------------------------------------------------
// Proxima acao decidida pelo algoritmo de navegacao.
// ----------------------------------------------------------------------------
enum NavAction {
    ACTION_FORWARD,
    ACTION_TURN_LEFT,
    ACTION_TURN_RIGHT,
    ACTION_TURN_BACK,
    ACTION_GOAL_REACHED,
    ACTION_NONE  // nenhuma acao possivel (ex.: cercado por paredes -- erro de mapa)
};

#endif // TYPES_H
