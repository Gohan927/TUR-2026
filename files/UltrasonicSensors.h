/*
 * ============================================================================
 * UltrasonicSensors.h / .cpp
 * ============================================================================
 * Responsavel por: inicializacao, leitura e filtragem dos 3 sensores HC-SR04.
 *
 * ESTRATEGIA DE FILTRAGEM ESCOLHIDA: mediana de N amostras (N=3 por padrao).
 *
 * Por que mediana e nao media movel?
 *   - Sensores ultrassonicos ocasionalmente retornam leituras absurdas
 *     (eco perdido, reflexao cruzada entre os 3 sensores, ruido eletrico).
 *   - A MEDIA e sensivel a outliers: uma leitura de 400cm no meio de leituras
 *     de 10cm distorce a media para cima.
 *   - A MEDIANA ignora outliers isolados: com 3 amostras, o valor do meio
 *     "vence" mesmo que uma das 3 leituras seja um erro grosseiro.
 *   - Com apenas 3 amostras lidas sequencialmente (~orcamento de tempo
 *     pequeno), o atraso adicional é mínimo -- não compromete a resposta do
 *     robo a tempo real, ao contrario de uma media movel com janela maior.
 *
 * Cada sensor e lido de forma SEQUENCIAL (nunca simultanea), para evitar que
 * o pulso ultrassonico de um sensor seja captado pelo echo de outro
 * (interferencia cruzada), problema comum quando ha 3 sensores proximos.
 * ============================================================================
 */

#ifndef ULTRASONIC_SENSORS_H
#define ULTRASONIC_SENSORS_H

#include <Arduino.h>
#include "Config.h"
#include "Types.h"

class UltrasonicSensors {
public:
    void begin();

    // Le os 3 sensores (sequencialmente, com filtro de mediana) e
    // preenche a struct SensorData, incluindo deteccao de parede.
    SensorData readAll();

private:
    float readSingleRaw(uint8_t trigPin, uint8_t echoPin);
    float readMedianFiltered(uint8_t trigPin, uint8_t echoPin);
    float median3(float a, float b, float c);
    bool isValidReading(float distanceCm);
};

#endif // ULTRASONIC_SENSORS_H
