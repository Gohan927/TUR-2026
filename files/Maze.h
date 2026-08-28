/*
 * ============================================================================
 * Maze.h / .cpp
 * ============================================================================
 * Representacao do labirinto em memoria + algoritmo Flood Fill para decidir
 * a proxima direcao de movimento.
 *
 * REPRESENTACAO ESCOLHIDA:
 *   Uma matriz MAZE_SIZE x MAZE_SIZE de structs Cell (ver Types.h), cada
 *   uma guardando 4 flags de parede (Norte/Leste/Sul/Oeste, em referencia
 *   ABSOLUTA ao mapa) + um valor de "distancia Flood Fill" ate o objetivo.
 *   Por que paredes em referencia absoluta (nao relativa ao robo)?
 *     Se guardassemos "parede na frente/esquerda/direita", o significado
 *     mudaria toda vez que o robo girasse -- e teriamos que reescrever o
 *     mapa a cada curva. Guardando em absoluto (N/E/S/O fixos), o mapa e
 *     estavel e so precisamos de UMA conversao (relativa->absoluta) no
 *     momento de gravar uma leitura de sensor, e outra (absoluta->relativa)
 *     no momento de decidir a proxima curva. Essa conversao usa a
 *     orientacao logica atual do robo (enum Direction).
 *
 * ALGORITMO: Flood Fill classico (BFS a partir do objetivo).
 *   1. Todas as celulas comecam com floodValue = "infinito" (255).
 *   2. A celula objetivo recebe floodValue = 0.
 *   3. Propaga-se em ondas: uma celula vizinha (sem parede entre elas)
 *      recebe floodValue = floodValue_da_celula_atual + 1.
 *   4. Repete-se ate todas as celulas alcancaveis terem um valor.
 *   5. Para decidir a proxima celula, o robo escolhe, entre suas vizinhas
 *      SEM parede, aquela com o MENOR floodValue.
 *   Flood Fill e escolhido por ser simples, confiavel, facil de depurar e
 *   suficiente para labirintos do tamanho tipico de competicao -- alinhado
 *   com "confiabilidade e simplicidade antes de sofisticacao".
 *
 * IMPORTANTE - orientacao FISICA vs LOGICA:
 *   Maze.cpp so trabalha com a orientacao LOGICA (enum Direction: para
 *   onde o robo aponta DENTRO DO MAPA). O angulo fisico medido pelo
 *   giroscopio (Gyro.h) e um conceito totalmente separado, usado apenas
 *   durante a execucao de UMA curva para saber quando parar de girar.
 *   Apos cada curva bem-sucedida, a orientacao logica e atualizada
 *   (ver Navigation.cpp) -- esse e o UNICO ponto de sincronizacao entre
 *   os dois conceitos.
 * ============================================================================
 */

#ifndef MAZE_H
#define MAZE_H

#include <Arduino.h>
#include "Config.h"
#include "Types.h"

#define FLOOD_INFINITY 255

class Maze {
public:
    void begin();

    // Posicao atual do robo no mapa (atualizada pela Navigation apos
    // cada movimento bem-sucedido).
    uint8_t robotX;
    uint8_t robotY;
    Direction robotDirection;

    // Registra as paredes detectadas nesta celula, convertendo de
    // referencia RELATIVA (esquerda/frente/direita, o que os sensores
    // realmente medem) para ABSOLUTA (N/E/S/O), usando robotDirection.
    //
    // REGRA DE SINCRONIZACAO (ver secao 17 do documento de especificacao):
    //   Uma leitura de sensor so pode ADICIONAR uma parede que ainda nao
    //   era conhecida. Nunca REMOVE uma parede ja registrada -- isso
    //   evita que uma leitura ruidosa isolada apague informacao valida
    //   do mapa. (Falsos positivos de parede sao tratados pela filtragem
    //   de mediana em UltrasonicSensors.cpp, upstream deste modulo.)
    void updateWallsAtCurrentCell(bool wallLeft, bool wallFront, bool wallRight);

    // Recalcula o Flood Fill a partir da celula objetivo (GOAL_X, GOAL_Y).
    void recalculateFloodFill();

    // Decide a proxima acao com base no floodValue das celulas vizinhas
    // alcancaveis (sem parede) a partir da posicao/orientacao atuais.
    NavAction decideNextAction();

    // Atualiza a posicao logica apos um movimento para frente bem-sucedido.
    void advancePosition();

    // Atualiza a orientacao logica apos uma curva bem-sucedida.
    void turnLogicalDirection(NavAction turnAction);

    bool isAtGoal() const;

    Cell& cellAt(uint8_t x, uint8_t y);

private:
    Cell grid[MAZE_SIZE][MAZE_SIZE];

    bool isValidCell(int x, int y) const;
    bool hasWallBetween(uint8_t x, uint8_t y, Direction dir) const;

    // Converte uma direcao RELATIVA ao robo (esquerda/frente/direita) para
    // uma Direction ABSOLUTA, dado para onde o robo aponta atualmente.
    Direction relativeLeftOf(Direction facing) const;
    Direction relativeRightOf(Direction facing) const;
};

#endif // MAZE_H
