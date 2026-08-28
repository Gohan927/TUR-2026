/*
 * ============================================================================
 * MICROMOUSE - TUR Competition
 * ============================================================================
 * Hardware:
 *   - Arduino Uno/Nano (ATmega328)
 *   - 3x Sensor Ultrassonico HC-SR04 (esquerdo, frontal, direito)
 *   - 1x GY-521 (MPU6050) via I2C
 *   - 1x Driver Ponte H L298N
 *   - 2x Motor DC N20
 *   - 1x Botao de Start/Emergencia (segurar 2s para confirmar)
 *   - Sem encoders (posicao estimada por tempo, ver CELL_TRAVEL_TIME_MS)
 *
 * Filosofia: Confiabilidade -> Controle -> Velocidade
 * Estrategia: 1a passada = mapeamento confiavel e lento.
 *             2a passada = execucao otimizada (a implementar apos validar
 *             a 1a passada em hardware real).
 *
 * BIBLIOTECAS NECESSARIAS (instalar via Arduino Library Manager):
 *   - "MPU6050" by Electronic Cats / jrowberg (I2Cdev based)
 *   - "I2Cdevlib-MPU6050" (dependencia da acima)
 *   - Wire.h (ja incluida no core do Arduino)
 * ============================================================================
 */

#include <Wire.h>
#include <MPU6050.h>

#include "Config.h"
#include "Types.h"
#include "UltrasonicSensors.h"
#include "Gyro.h"
#include "Motors.h"
#include "LineControl.h"
#include "Turning.h"
#include "StartButton.h"
#include "Maze.h"
#include "Safety.h"

// ============================================================================
// INSTANCIAS GLOBAIS DOS MODULOS
// ============================================================================
UltrasonicSensors sensors;
Gyro gyro;
Motors motors;
LineControl lineControl;
Turning turning;
StartButton startButton;
Maze maze;
Safety safety;

// ============================================================================
// ESTADO GLOBAL DA MAQUINA DE ESTADOS
// ============================================================================
RobotState currentState = STATE_STARTUP;
SensorData lastSensorReading;
NavAction pendingAction = ACTION_NONE;

unsigned long forwardMoveStartMillis = 0;
unsigned long lastDebugPrintMillis = 0;

// Guarda se a emergencia foi disparada pelo BOTAO (permite retomada manual)
// ou por um ERRO interno (nao deveria ser retomado sem investigacao).
bool emergencyFromButton = false;

// Flags de inicializacao de movimento/curva. Sao variaveis GLOBAIS (nao
// "static" dentro do case do switch) de proposito: se uma emergencia
// interromper o robo no meio de um STATE_MOVE_FORWARD ou de uma curva,
// essas flags precisam ser resetadas explicitamente ao entrar em
// EMERGENCY_STOP -- caso contrario, uma variavel static local ficaria
// "presa" em true, e a proxima vez que a maquina de estados entrasse
// naquele state ela puularia a reinicializacao (lineControl.reset(),
// timeout, etc.), um bug sutil que so apareceria em campo.
bool forwardMoveInitialized = false;
bool turnInitialized = false;

// Impede reimpressao repetida das mensagens de estado terminal (GOAL/ERROR).
bool goalMessagePrinted = false;
bool errorMessagePrinted = false;

// Controla o ciclo de "armar" a retomada apos emergencia por botao:
// exige soltar o botao e dar um NOVO hold de 2s, nunca retoma no mesmo
// evento que causou a parada.
bool waitingButtonReleaseAfterEmergency = false;

// Reseta todas as flags de inicializacao de movimento -- chamado sempre
// que o robo entra em um estado que interrompe um movimento em curso.
void resetMovementFlags() {
    forwardMoveInitialized = false;
    turnInitialized = false;
}

// ============================================================================
// SETUP
// ============================================================================
void setup() {
#if DEBUG
    Serial.begin(115200);
    Serial.println(F("=== Micromouse TUR - Inicializando ==="));
#endif

    sensors.begin();
    motors.begin();
    gyro.begin();
    lineControl.begin();
    turning.begin(&gyro, &motors);
    startButton.begin();
    maze.begin();

    currentState = STATE_CALIBRATING;
}

// ============================================================================
// LOOP PRINCIPAL
// ============================================================================
void loop() {
    // O botao e verificado em QUALQUER estado -- permite parada de
    // emergencia a qualquer momento durante a navegacao.
    ButtonEvent buttonEvent = startButton.update();

    if (buttonEvent == BUTTON_HOLD_CONFIRMED) {
        if (currentState == STATE_WAIT_FOR_START) {
            motors.stopMotors();
            lineControl.reset();
            currentState = STATE_READ_SENSORS;
#if DEBUG
            Serial.println(F(">>> START confirmado. Iniciando navegacao. <<<"));
#endif
        } else if (currentState == STATE_EMERGENCY_STOP && emergencyFromButton) {
            // Segundo hold do botao, agora DENTRO do estado de emergencia:
            // isso ARMA a retomada. A retomada real so ocorre depois que o
            // botao for solto (ver bloco abaixo), nunca no mesmo instante
            // do hold que a confirmou -- evita retomar acidentalmente
            // encadeado com o proprio evento que causou a parada.
            waitingButtonReleaseAfterEmergency = true;
#if DEBUG
            Serial.println(F(">>> Retomada armada. Solte o botao para continuar. <<<"));
#endif
        } else if (currentState != STATE_CALIBRATING &&
                   currentState != STATE_STARTUP &&
                   currentState != STATE_GOAL_REACHED &&
                   currentState != STATE_EMERGENCY_STOP &&
                   currentState != STATE_ERROR) {
            // Segurar o botao durante a navegacao = parada de emergencia.
            motors.stopMotors();
            resetMovementFlags();
            emergencyFromButton = true;
            currentState = STATE_EMERGENCY_STOP;
#if DEBUG
            Serial.println(F(">>> PARADA DE EMERGENCIA (botao) <<<"));
#endif
        }
    }

    // Retomada efetiva: so acontece depois que a retomada foi ARMADA
    // (bloco acima) E o botao ja foi solto fisicamente (nivel HIGH,
    // com INPUT_PULLUP). Isso garante uma acao humana deliberada em
    // dois passos: segurar 2s para armar, soltar para confirmar.
    if (waitingButtonReleaseAfterEmergency &&
        currentState == STATE_EMERGENCY_STOP &&
        digitalRead(PIN_START_BUTTON) == HIGH) {

        waitingButtonReleaseAfterEmergency = false;
        emergencyFromButton = false;
        resetMovementFlags();
        lineControl.reset();
        currentState = STATE_READ_SENSORS;
#if DEBUG
        Serial.println(F(">>> Retomando navegacao apos emergencia. <<<"));
#endif
    }

    switch (currentState) {

        // --------------------------------------------------------------
        case STATE_STARTUP: {
            // Nao deveria persistir aqui -- setup() ja transiciona para
            // CALIBRATING. Presente por completude da maquina de estados.
            currentState = STATE_CALIBRATING;
            break;
        }

        // --------------------------------------------------------------
        case STATE_CALIBRATING: {
            // O robo DEVE estar parado e imovel neste momento.
#if DEBUG
            Serial.println(F("Calibrando giroscopio... NAO MOVA O ROBO."));
#endif
            gyro.calibrate();

            if (gyro.isCalibrated()) {
                currentState = STATE_WAIT_FOR_START;
#if DEBUG
                Serial.println(F("Calibracao concluida. Segure o botao por 2s para iniciar."));
#endif
            } else {
                // calibrate() e bloqueante e so retorna apos concluir;
                // isCalibrated() deveria ser sempre true aqui. Se nao for,
                // algo grave ocorreu com o sensor -- vai para erro.
                currentState = STATE_ERROR;
            }
            break;
        }

        // --------------------------------------------------------------
        case STATE_WAIT_FOR_START: {
            // Nao faz nada alem de aguardar o evento de botao (tratado
            // no topo do loop). Mantem os motores parados por seguranca.
            motors.stopMotors();
            break;
        }

        // --------------------------------------------------------------
        case STATE_READ_SENSORS: {
            lastSensorReading = sensors.readAll();

            // Seguranca: obstaculo frontal perigosamente proximo detectado
            // ja na leitura -- aborta antes mesmo de decidir a proxima acao.
            if (safety.isFrontObstacleTooClose(lastSensorReading)) {
                motors.stopMotors();
                currentState = STATE_EMERGENCY_STOP;
                emergencyFromButton = false;
#if DEBUG
                Serial.println(F("EMERGENCIA: obstaculo frontal muito proximo."));
#endif
                break;
            }

            currentState = STATE_UPDATE_MAP;
            break;
        }

        // --------------------------------------------------------------
        case STATE_UPDATE_MAP: {
            maze.updateWallsAtCurrentCell(
                lastSensorReading.wallLeft,
                lastSensorReading.wallFront,
                lastSensorReading.wallRight
            );
            maze.recalculateFloodFill();
            currentState = STATE_CHOOSE_DIRECTION;
            break;
        }

        // --------------------------------------------------------------
        case STATE_CHOOSE_DIRECTION: {
            pendingAction = maze.decideNextAction();

            switch (pendingAction) {
                case ACTION_GOAL_REACHED:
                    currentState = STATE_GOAL_REACHED;
                    break;
                case ACTION_FORWARD:
                    currentState = STATE_MOVE_FORWARD;
                    break;
                case ACTION_TURN_LEFT:
                    currentState = STATE_TURN_LEFT;
                    break;
                case ACTION_TURN_RIGHT:
                    currentState = STATE_TURN_RIGHT;
                    break;
                case ACTION_TURN_BACK:
                    currentState = STATE_TURN_BACK;
                    break;
                case ACTION_NONE:
                default:
                    // Mapa inconsistente ou celula isolada -- nao ha
                    // movimento seguro possivel. Nunca avancar as cegas.
                    currentState = STATE_ERROR;
#if DEBUG
                    Serial.println(F("ERRO: nenhuma acao de navegacao valida encontrada."));
#endif
                    break;
            }
            break;
        }

        // --------------------------------------------------------------
        case STATE_MOVE_FORWARD: {
            if (!forwardMoveInitialized) {
                lineControl.reset();
                safety.startMovementTimeout();
                forwardMoveStartMillis = millis();
                forwardMoveInitialized = true;
            }

            // Re-le sensores continuamente durante o movimento, para
            // seguranca (obstaculo surgindo no meio do trajeto) e para
            // o PID de correcao lateral.
            SensorData liveSensors = sensors.readAll();

            if (safety.isFrontObstacleTooClose(liveSensors)) {
                motors.stopMotors();
                resetMovementFlags();
                currentState = STATE_EMERGENCY_STOP;
                emergencyFromButton = false;
#if DEBUG
                Serial.println(F("EMERGENCIA: obstaculo detectado durante avanco."));
#endif
                break;
            }

            if (safety.isMovementTimeoutExceeded()) {
                motors.stopMotors();
                resetMovementFlags();
                currentState = STATE_ERROR;
#if DEBUG
                Serial.println(F("ERRO: timeout de movimento excedido."));
#endif
                break;
            }

            gyro.update();
            float correction = lineControl.computeCorrection(liveSensors);

            // Correcao positiva (mais espaco a esquerda) -> reduz motor
            // esquerdo e aumenta o direito, para "puxar" o robo de volta
            // ao centro do corredor.
            int leftSpeed  = BASE_SPEED - (int)correction;
            int rightSpeed = BASE_SPEED + (int)correction;
            motors.setMotors(leftSpeed, rightSpeed);

            // Avanca por CELL_TRAVEL_TIME_MS (unica referencia disponivel
            // sem encoders -- ver nota em Config.h).
            if (millis() - forwardMoveStartMillis >= CELL_TRAVEL_TIME_MS) {
                motors.stopMotors();
                resetMovementFlags();
                maze.advancePosition();
                currentState = STATE_READ_SENSORS;
            }
            break;
        }

        // --------------------------------------------------------------
        case STATE_TURN_LEFT: {
            if (!turnInitialized) {
                turning.startTurn(TURN_90_TARGET_DEG, -1);
                safety.startMovementTimeout();
                turnInitialized = true;
            }

            TurnResult result = turning.update();

            if (result == TURN_COMPLETED) {
                maze.turnLogicalDirection(ACTION_TURN_LEFT);
                resetMovementFlags();
                currentState = STATE_READ_SENSORS;
            } else if (result == TURN_TIMEOUT_ERROR) {
                resetMovementFlags();
                currentState = STATE_ERROR;
#if DEBUG
                Serial.println(F("ERRO: timeout na curva esquerda."));
#endif
            }
            break;
        }

        // --------------------------------------------------------------
        case STATE_TURN_RIGHT: {
            if (!turnInitialized) {
                turning.startTurn(TURN_90_TARGET_DEG, +1);
                safety.startMovementTimeout();
                turnInitialized = true;
            }

            TurnResult result = turning.update();

            if (result == TURN_COMPLETED) {
                maze.turnLogicalDirection(ACTION_TURN_RIGHT);
                resetMovementFlags();
                currentState = STATE_READ_SENSORS;
            } else if (result == TURN_TIMEOUT_ERROR) {
                resetMovementFlags();
                currentState = STATE_ERROR;
#if DEBUG
                Serial.println(F("ERRO: timeout na curva direita."));
#endif
            }
            break;
        }

        // --------------------------------------------------------------
        case STATE_TURN_BACK: {
            if (!turnInitialized) {
                // Convencao: 180 graus sempre gira para a direita.
                turning.startTurn(TURN_180_TARGET_DEG, +1);
                safety.startMovementTimeout();
                turnInitialized = true;
            }

            TurnResult result = turning.update();

            if (result == TURN_COMPLETED) {
                maze.turnLogicalDirection(ACTION_TURN_BACK);
                resetMovementFlags();
                currentState = STATE_READ_SENSORS;
            } else if (result == TURN_TIMEOUT_ERROR) {
                resetMovementFlags();
                currentState = STATE_ERROR;
#if DEBUG
                Serial.println(F("ERRO: timeout na curva de 180."));
#endif
            }
            break;
        }

        // --------------------------------------------------------------
        case STATE_GOAL_REACHED: {
            motors.stopMotors();
#if DEBUG
            if (!goalMessagePrinted) {
                Serial.println(F(">>> OBJETIVO ALCANCADO! <<<"));
                goalMessagePrinted = true;
            }
#endif
            // Permanece aqui. 2a passada otimizada e um modulo futuro,
            // a implementar apos validar esta 1a passada em hardware real.
            break;
        }

        // --------------------------------------------------------------
        case STATE_EMERGENCY_STOP: {
            // A logica de "armar" e "confirmar" a retomada (segurar 2s +
            // soltar o botao) fica centralizada no topo do loop(), pois
            // precisa reagir tanto a eventos de hold quanto ao estado
            // "solto" do botao a cada ciclo -- manter tudo em um so lugar
            // evita logica de retomada duplicada/inconsistente.
            motors.stopMotors();
            break;
        }

        // --------------------------------------------------------------
        case STATE_ERROR: {
            motors.stopMotors();
#if DEBUG
            if (!errorMessagePrinted) {
                Serial.println(F("*** ESTADO DE ERRO - ROBO PARADO ***"));
                Serial.println(F("Verifique: sensores, mapa, giroscopio, motores."));
                errorMessagePrinted = true;
            }
#endif
            // Estado terminal -- requer reset fisico do Arduino para sair.
            break;
        }
    }

#if DEBUG
    if (millis() - lastDebugPrintMillis >= DEBUG_PRINT_INTERVAL_MS) {
        lastDebugPrintMillis = millis();
        Serial.print(F("Estado: "));
        Serial.print((int)currentState);
        Serial.print(F(" | X:")); Serial.print(maze.robotX);
        Serial.print(F(" Y:")); Serial.print(maze.robotY);
        Serial.print(F(" Dir:")); Serial.print((int)maze.robotDirection);
        Serial.print(F(" | Ang:")); Serial.print(gyro.getAngle());
        Serial.print(F(" | L:")); Serial.print(lastSensorReading.left);
        Serial.print(F(" F:")); Serial.print(lastSensorReading.front);
        Serial.print(F(" R:")); Serial.println(lastSensorReading.right);
    }
#endif
}
