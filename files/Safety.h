/*
 * ============================================================================
 * Safety.h / .cpp
 * ============================================================================
 * Verificacoes de seguranca centralizadas. Qualquer parte do codigo que
 * precise decidir "e seguro continuar?" consulta este modulo, em vez de
 * espalhar checagens de seguranca por toda a maquina de estados.
 * ============================================================================
 */

#ifndef SAFETY_H
#define SAFETY_H

#include <Arduino.h>
#include "Config.h"
#include "Types.h"

class Safety {
public:
    // Verifica se ha obstaculo perigosamente proximo a frente.
    // Leitura invalida NAO conta como "seguro" nem como "perigo" --
    // e tratada como incerteza (retorna false = nao ha perigo confirmado,
    // mas o chamador deve saber que a leitura nao e confiavel via
    // sensors.frontValid).
    bool isFrontObstacleTooClose(const SensorData &sensors) const;

    // Inicia contagem de timeout para um movimento (frente ou curva).
    void startMovementTimeout();

    // Verifica se o timeout do movimento atual foi excedido.
    bool isMovementTimeoutExceeded() const;

private:
    unsigned long movementStartMillis;
};

#endif // SAFETY_H
