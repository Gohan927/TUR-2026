/*
 * ============================================================================
 * StartButton.h / .cpp
 * ============================================================================
 * Botao fisico de Start / Parada de Emergencia, usando interrupcao externa
 * (INT0, pino D2) combinada com medicao de "segurar por X segundos"
 * (BUTTON_HOLD_TIME_MS = 2000ms, definido em Config.h).
 *
 * POR QUE "SEGURAR POR 2 SEGUNDOS" EM VEZ DE UM CLIQUE SIMPLES?
 *   Evita que um toque acidental (vibracao, encostar sem querer) dispare
 *   o inicio da corrida ou -- pior -- uma parada de emergencia no meio da
 *   navegacao. Segurar por 2s e uma acao deliberada.
 *
 * POR QUE A ISR NAO FAZ A CONTAGEM DE TEMPO DIRETAMENTE?
 *   Rotinas de interrupcao (ISR) devem ser CURTAS. Medir 2 segundos dentro
 *   de uma ISR bloquearia todas as outras interrupcoes (incluindo as
 *   usadas internamente pelo I2C/Wire para o MPU6050) durante esse tempo,
 *   o que e uma pessima pratica e pode causar falhas sutis em outros
 *   modulos. Por isso a ISR apenas sinaliza uma mudanca de estado do botao
 *   (via flag volatile), e a CONTAGEM do tempo de pressao e feita no loop
 *   principal, usando millis().
 *
 * DEBOUNCE:
 *   Botoes mecanicos "ricocheteiam" eletricamente por alguns milissegundos
 *   ao serem pressionados/soltos, gerando multiplas interrupcoes espurias.
 *   BUTTON_DEBOUNCE_MS filtra isso, ignorando mudancas de estado que
 *   ocorram rapido demais para serem um toque humano real.
 * ============================================================================
 */

#ifndef START_BUTTON_H
#define START_BUTTON_H

#include <Arduino.h>
#include "Config.h"

enum ButtonEvent {
    BUTTON_NONE,           // nada aconteceu ainda
    BUTTON_HOLD_CONFIRMED  // botao foi seguro pressionado por >= BUTTON_HOLD_TIME_MS
};

class StartButton {
public:
    void begin();

    // Deve ser chamado a cada ciclo do loop principal (nao bloqueante).
    // Retorna BUTTON_HOLD_CONFIRMED exatamente UMA vez quando a condicao
    // de "segurar 2s" e satisfeita; caso contrario BUTTON_NONE.
    ButtonEvent update();

    // Attach externo da ISR precisa de uma funcao estatica/global --
    // este metodo e chamado por essa funcao livre (ver StartButton.cpp).
    static void isrHandler();

private:
    static volatile bool pinChanged;   // sinalizado pela ISR

    unsigned long pressStartMillis;
    bool isPressed;
    bool holdEventFired; // evita disparar BUTTON_HOLD_CONFIRMED repetidamente
    unsigned long lastDebounceMillis;
    bool lastRawState;
};

#endif // START_BUTTON_H
