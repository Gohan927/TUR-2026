/*
 * ============================================================================
 * Config.h
 * ============================================================================
 * CONFIGURACOES CENTRALIZADAS DO ROBO MICROMOUSE
 *
 * Todos os parametros que provavelmente precisarao de ajuste apos os testes
 * fisicos ficam aqui. Nao espalhe "numeros magicos" pelo resto do codigo --
 * se precisar de uma constante nova, adicione aqui.
 *
 * Filosofia do projeto (a mesma do Line Follower):
 *   Confiabilidade -> Controle -> Velocidade
 * Portanto os valores padrao aqui sao conservadores. So aumente velocidade
 * depois que o robo estiver navegando o labirinto de forma confiavel.
 * ============================================================================
 */

#ifndef CONFIG_H
#define CONFIG_H

#include <Arduino.h>

// ============================================================================
// HARDWARE - PINAGEM
// ============================================================================

// --- MPU6050 (GY-521) ---
// SDA = A4, SCL = A5 : pinos fixos de hardware I2C no Uno/Nano (ATmega328).
// Nao sao configuraveis via software.

// --- Botao de Start / Parada de Emergencia ---
#define PIN_START_BUTTON        2   // INT0 - interrupcao externa

// --- Sensor Ultrassonico ESQUERDO ---
#define PIN_US_LEFT_TRIG        4
#define PIN_US_LEFT_ECHO        7

// --- Sensor Ultrassonico FRONTAL ---
#define PIN_US_FRONT_TRIG       8
#define PIN_US_FRONT_ECHO       12

// --- Sensor Ultrassonico DIREITO ---
#define PIN_US_RIGHT_TRIG       13
#define PIN_US_RIGHT_ECHO       A0  // usado como pino digital

// --- Motor ESQUERDO (L298N) ---
#define PIN_MOTOR_LEFT_IN1      A1
#define PIN_MOTOR_LEFT_IN2      A2
#define PIN_MOTOR_LEFT_ENA      9   // PWM (Timer1)

// --- Motor DIREITO (L298N) ---
#define PIN_MOTOR_RIGHT_IN3     A3
#define PIN_MOTOR_RIGHT_IN4     11
#define PIN_MOTOR_RIGHT_ENB     10  // PWM (Timer1)

// ============================================================================
// BOTAO DE START / EMERGENCIA
// ============================================================================

// Tempo que o botao precisa ficar pressionado para confirmar uma acao.
// Isso evita que um toque acidental inicie o robo ou pare a competicao.
#define BUTTON_HOLD_TIME_MS         2000UL

// Tempo de debounce (ignora mudancas de estado mais rapidas que isso).
#define BUTTON_DEBOUNCE_MS          30UL

// ============================================================================
// MOTORES
// ============================================================================

// PWM maximo permitido (0-255). Nao usar 255 puro reduz estresse eletrico
// e da margem para o PID/controle de curva "acelerar" se precisar.
#define MOTOR_MAX_PWM               220

// PWM minimo efetivo. Por causa da queda de tensao interna do L298N
// (~2V por canal) combinada com o baixo torque dos motores N20, valores
// de PWM abaixo deste limite podem nao ser suficientes para vencer o
// atrito estatico do redutor -- o motor simplesmente nao gira.
// *** ESTE VALOR PRECISA SER CALIBRADO FISICAMENTE (ver secao de calibracao). ***
#define MOTOR_MIN_EFFECTIVE_PWM     60

// Velocidade base conservadora para movimento em linha reta (0-255).
// Comeca baixo, de acordo com a filosofia Confiabilidade > Velocidade.
#define BASE_SPEED                  100

// Velocidade durante curvas (antes da desaceleracao final).
#define TURN_SPEED                  90

// Velocidade minima durante a fase de desaceleracao da curva (perto do alvo).
#define TURN_SPEED_SLOW             MOTOR_MIN_EFFECTIVE_PWM + 10

// ============================================================================
// SENSORES ULTRASSONICOS (HC-SR04)
// ============================================================================

// Distancia (cm) abaixo da qual consideramos que ha uma parede.
#define WALL_DISTANCE_CM            12.0

// Faixa valida de leitura do HC-SR04. Fora disso, a leitura e descartada.
#define MIN_VALID_DISTANCE_CM       2.0
#define MAX_VALID_DISTANCE_CM       400.0

// Distancia de seguranca frontal: abaixo disso, parada de emergencia imediata,
// independente do que a maquina de estados estiver fazendo.
#define FRONT_SAFETY_DISTANCE_CM    5.0

// Timeout do pulseIn (microssegundos). ~30ms cobre a faixa maxima do HC-SR04
// (400cm ida e volta) com folga, sem travar o programa indefinidamente
// caso o eco nunca retorne (sensor desconectado/com defeito).
#define US_TIMEOUT_US               30000UL

// Numero de amostras usadas no filtro de mediana de cada sensor.
// 3 amostras: bom equilibrio entre rejeitar outliers e nao atrasar a leitura.
#define US_MEDIAN_SAMPLES           3

// ============================================================================
// MPU6050 / GIROSCOPIO
// ============================================================================

// Numero de amostras coletadas na calibracao do giroscopio (robo parado).
// Mais amostras = offset mais preciso, porem calibracao mais demorada.
#define GYRO_CALIBRATION_SAMPLES    500

// Sensibilidade padrao do MPU6050 no range +-250 graus/s:
// 131 LSB por (graus/segundo). Se o range for alterado na inicializacao,
// este divisor DEVE ser alterado tambem.
#define GYRO_SENSITIVITY_LSB_PER_DPS 131.0

// ============================================================================
// CONTROLE DE CURVAS (baseado no giroscopio, eixo Z / Yaw)
// ============================================================================

#define TURN_90_TARGET_DEG          90.0
#define TURN_180_TARGET_DEG         180.0

// Tolerancia aceitavel de erro angular ao finalizar uma curva.
#define TURN_TOLERANCE_DEG          2.0

// A partir de quantos graus antes do alvo comecamos a desacelerar.
#define TURN_SLOWDOWN_ANGLE_DEG     20.0

// Timeout de seguranca para uma curva (evita loop infinito caso o
// giroscopio falhe ou a curva nunca atinja o angulo alvo).
#define TURN_TIMEOUT_MS             3000UL

// ============================================================================
// PID - CORRECAO DE TRAJETORIA EM LINHA RETA (usando paredes laterais)
// ============================================================================

#define PID_KP                      2.5
#define PID_KI                      0.0
#define PID_KD                      1.2

// Limite de saida do PID (soma/subtrai da velocidade base dos motores).
#define PID_OUTPUT_LIMIT            60.0

// ============================================================================
// LABIRINTO
// ============================================================================

// Tamanho do labirinto (ajustar conforme a competicao - padrao classico 16x16,
// mas comecamos conservador / configuravel).
#define MAZE_SIZE                   16

// Celula objetivo (centro, padrao IEEE Micromouse). Ajustar se a regra local
// definir outro alvo.
#define GOAL_X                      (MAZE_SIZE / 2)
#define GOAL_Y                      (MAZE_SIZE / 2)

// Tempo estimado (ms) para percorrer uma celula em BASE_SPEED.
// *** Sem encoders, esta e a UNICA referencia de "quanto andei".            ***
// *** DEVE ser calibrado fisicamente medindo o tempo real de uma celula.    ***
#define CELL_TRAVEL_TIME_MS         700UL

// ============================================================================
// SEGURANCA
// ============================================================================

// Timeout generico para qualquer movimento (frente, curva) -- protege
// contra travamentos de motor, roda presa, etc.
#define MOVEMENT_TIMEOUT_MS         4000UL

// ============================================================================
// DEBUG
// ============================================================================

#define DEBUG                       true

// Intervalo minimo (ms) entre impressoes de debug, para nao sobrecarregar
// a porta serial com milhares de mensagens por segundo.
#define DEBUG_PRINT_INTERVAL_MS     200UL

#endif // CONFIG_H
