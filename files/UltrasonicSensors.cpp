#include "UltrasonicSensors.h"

void UltrasonicSensors::begin() {
    pinMode(PIN_US_LEFT_TRIG, OUTPUT);
    pinMode(PIN_US_LEFT_ECHO, INPUT);

    pinMode(PIN_US_FRONT_TRIG, OUTPUT);
    pinMode(PIN_US_FRONT_ECHO, INPUT);

    pinMode(PIN_US_RIGHT_TRIG, OUTPUT);
    pinMode(PIN_US_RIGHT_ECHO, INPUT);

    digitalWrite(PIN_US_LEFT_TRIG, LOW);
    digitalWrite(PIN_US_FRONT_TRIG, LOW);
    digitalWrite(PIN_US_RIGHT_TRIG, LOW);
}

// Dispara um pulso de trigger e mede o tempo de retorno do echo,
// convertendo para distancia em cm. Retorna -1.0 se o timeout for atingido
// (echo nunca retornou -- sensor desconectado, sem obstaculo no alcance
// maximo, ou falha de hardware).
float UltrasonicSensors::readSingleRaw(uint8_t trigPin, uint8_t echoPin) {
    digitalWrite(trigPin, LOW);
    delayMicroseconds(2);
    digitalWrite(trigPin, HIGH);
    delayMicroseconds(10);
    digitalWrite(trigPin, LOW);

    unsigned long durationUs = pulseIn(echoPin, HIGH, US_TIMEOUT_US);

    if (durationUs == 0) {
        return -1.0; // timeout -- leitura invalida
    }

    // Distancia (cm) = tempo * velocidade do som (~0.0343 cm/us) / 2 (ida e volta)
    float distanceCm = (durationUs * 0.0343) / 2.0;
    return distanceCm;
}

bool UltrasonicSensors::isValidReading(float distanceCm) {
    if (distanceCm < 0) return false; // timeout do pulseIn
    if (distanceCm < MIN_VALID_DISTANCE_CM) return false;
    if (distanceCm > MAX_VALID_DISTANCE_CM) return false;
    return true;
}

float UltrasonicSensors::median3(float a, float b, float c) {
    if (a > b) { float t = a; a = b; b = t; }
    if (b > c) { float t = b; b = c; c = t; }
    if (a > b) { float t = a; a = b; b = t; }
    return b; // valor do meio apos ordenacao
}

// Le N amostras (US_MEDIAN_SAMPLES) e retorna a mediana das leituras VALIDAS.
// Se todas as amostras forem invalidas, retorna -1.0 (sinaliza leitura falha
// ao chamador, que deve tratar isso como "sensor indisponivel neste ciclo",
// nunca como "sem parede").
float UltrasonicSensors::readMedianFiltered(uint8_t trigPin, uint8_t echoPin) {
    float samples[US_MEDIAN_SAMPLES];
    uint8_t validCount = 0;

    for (uint8_t i = 0; i < US_MEDIAN_SAMPLES; i++) {
        float reading = readSingleRaw(trigPin, echoPin);
        if (isValidReading(reading)) {
            samples[validCount] = reading;
            validCount++;
        }
        // pequeno espacamento entre disparos do mesmo sensor, para deixar
        // o eco anterior se dissipar e nao contaminar a proxima leitura.
        delayMicroseconds(500);
    }

    if (validCount == 0) {
        return -1.0;
    }
    if (validCount == 1) {
        return samples[0];
    }
    if (validCount == 2) {
        // com apenas 2 validas, usar a media simples entre elas
        return (samples[0] + samples[1]) / 2.0;
    }

    // caso padrao: 3 amostras validas -> mediana verdadeira
    return median3(samples[0], samples[1], samples[2]);
}

SensorData UltrasonicSensors::readAll() {
    SensorData data;

    data.left  = readMedianFiltered(PIN_US_LEFT_TRIG, PIN_US_LEFT_ECHO);
    data.front = readMedianFiltered(PIN_US_FRONT_TRIG, PIN_US_FRONT_ECHO);
    data.right = readMedianFiltered(PIN_US_RIGHT_TRIG, PIN_US_RIGHT_ECHO);

    data.leftValid  = (data.left  >= 0);
    data.frontValid = (data.front >= 0);
    data.rightValid = (data.right >= 0);

    // Regra de seguranca: uma leitura INVALIDA nunca deve ser interpretada
    // como "sem parede". Tratamos leitura invalida como wall=false apenas
    // no sentido de "nao temos certeza" -- a camada de navegacao (Maze.cpp)
    // e quem decide como lidar com incerteza (nunca sobrescrever uma parede
    // ja conhecida com base em uma leitura invalida).
    data.wallLeft  = data.leftValid  && (data.left  < WALL_DISTANCE_CM);
    data.wallFront = data.frontValid && (data.front < WALL_DISTANCE_CM);
    data.wallRight = data.rightValid && (data.right < WALL_DISTANCE_CM);

    return data;
}
