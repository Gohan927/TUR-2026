#include "Gyro.h"

void Gyro::begin() {
    Wire.begin();
#if defined(TWBR)
    Wire.setClock(400000); // I2C a 400kHz -- reduz tempo gasto lendo o MPU6050
#endif

    mpu.initialize();

    state.angleZ = 0.0;
    state.gyroZOffset = 0.0;
    state.calibrated = false;

    firstUpdate = true;
    lastUpdateMicros = micros();

#if DEBUG
    Serial.print(F("MPU6050 conexao: "));
    Serial.println(mpu.testConnection() ? F("OK") : F("FALHOU"));
#endif
}

// Calibracao: robo DEVE estar parado e imovel sobre uma superficie estavel.
// Coletamos GYRO_CALIBRATION_SAMPLES leituras cruas do eixo Z e calculamos
// a media -- esse valor e o "offset" (erro sistematico) que sera subtraido
// de toda leitura futura. Sem isso, o giroscopio "deriva" (drift) mesmo com
// o robo parado, pois nenhum MEMS e perfeitamente calibrado de fabrica.
void Gyro::calibrate() {
    long sumGz = 0;

    for (uint16_t i = 0; i < GYRO_CALIBRATION_SAMPLES; i++) {
        int16_t gz = mpu.getRotationZ();
        sumGz += gz;
        delay(3); // pequeno espacamento entre leituras durante a calibracao
    }

    // offset em LSB bruto (mesma unidade da leitura crua)
    float offsetLsb = (float)sumGz / (float)GYRO_CALIBRATION_SAMPLES;

    // convertido para graus/segundo, para ser subtraido diretamente do
    // valor ja convertido em update()
    state.gyroZOffset = offsetLsb / GYRO_SENSITIVITY_LSB_PER_DPS;
    state.calibrated = true;
    state.angleZ = 0.0;

    lastUpdateMicros = micros();
    firstUpdate = false;

#if DEBUG
    Serial.print(F("Calibracao giroscopio concluida. Offset Z (dps): "));
    Serial.println(state.gyroZOffset, 4);
#endif
}

void Gyro::update() {
    if (!state.calibrated) return; // seguranca: nao integra antes de calibrar

    unsigned long nowMicros = micros();

    if (firstUpdate) {
        lastUpdateMicros = nowMicros;
        firstUpdate = false;
        return;
    }

    // dt REAL entre esta leitura e a anterior, em segundos.
    // Usar micros() (nao millis()) da mais resolucao para integracao angular.
    unsigned long deltaMicros = nowMicros - lastUpdateMicros; // seguro mesmo com overflow, por ser unsigned
    float dtSeconds = deltaMicros / 1000000.0;
    lastUpdateMicros = nowMicros;

    int16_t rawGz = mpu.getRotationZ();
    float gyroZDps = (rawGz / GYRO_SENSITIVITY_LSB_PER_DPS) - state.gyroZOffset;

    // Integracao: angulo += velocidade_angular * tempo_decorrido
    state.angleZ += gyroZDps * dtSeconds;
}

float Gyro::getAngle() const {
    return state.angleZ;
}

void Gyro::resetAngle() {
    state.angleZ = 0.0;
}

bool Gyro::isCalibrated() const {
    return state.calibrated;
}
