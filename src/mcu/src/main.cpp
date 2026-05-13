#include <Arduino.h>
#include "main.h"
#include <FastLED.h>
#include <Wire.h>

#define LED_STRIP_PIN 13       // Пин подключения ленты к ESP32
#define NUM_LEDS    32          // Количество светодиодов в ленте
#define BRIGHTNESS  255          // Яркость (0-255)
#define LED_TYPE    WS2812B     // Тип светодиодной ленты
#define COLOR_ORDER GRB         // Порядок цветов (для WS2812B обычно GRB)

#define LEFT_MOTOR_A 14     // Цифровой выход (левый мотор). Если 0 - едем вперед
#define LEFT_MOTOR_B 27     // Цифровой выход (левый мотор). Если 0 - едем назад
#define RIGHT_MOTOR_A 25    // Цифровой выход (правый мотор). Если 0 - едем вперед
#define RIGHT_MOTOR_B 26    // Цифровой выход (правый мотор). Если 0 - едем назад

#define LED_BUILTIN_PIN 2       // Встроенный светодиод на ESP32 Dev Board

#define LEFT_ENCODER_A 33    // Цифровой вход, канал A энкодера левого колеса
#define LEFT_ENCODER_B 32    // Цифровой вход, канал B энкодера левого колеса 
#define RIGHT_ENCODER_A 34   // Цифровой вход, канал A энкодера правого колеса
#define RIGHT_ENCODER_B 35   // Цифровой вход, канал B энкодера правого колеса

#define JETSON_UART_RX 16     // RX2: прием данных от Jetson TX
#define JETSON_UART_TX 17     // TX2: передача данных на Jetson RX
#define JETSON_UART_BAUD 115200

#define INA226_I2C_SDA 21
#define INA226_I2C_SCL 22
#define INA226_I2C_ADDRESS 0x40
#define INA226_SHUNT_OHMS 0.01f

#define INA226_REG_CONFIG 0x00
#define INA226_REG_SHUNT_VOLTAGE 0x01
#define INA226_REG_BUS_VOLTAGE 0x02
#define INA226_REG_MANUFACTURER_ID 0xFE
#define INA226_REG_DIE_ID 0xFF

/**
 * Глобальные переменные для хранения текущего счёта тиков (импульсов) энкодеров.
 * Используется тип long, т.к. значения могут быть достаточно большими при длительной работе.
 * Ключевое слово volatile указывает, что данные могут изменяться асинхронно (в прерываниях).
 */
volatile long left_encoder_value = 0; 
volatile long right_encoder_value = 0;

/**
 * Переменные для харения состоания скорости колёс и одометрии
 */
long last_left_encoder = 0;
long last_right_encoder = 0;

long last_speed_left_encoder = 0, last_speed_right_encoder = 0;
float leftWheelSpeed = 0.0, rightWheelSpeed = 0.0;
float vlSpeedFiltered = 0.0, vrSpeedFiltered = 0.0;
uint32_t lastTimeL = 0, lastTimeR = 0;

float targetLeftWheelSpeed = 0, targetRightWheelSpeed = 0;

float xPos = 0.0, yPos = 0.0, theta = 0.0;

const float WHEEL_DIAMETER = 68.0; // Укажите ваш диаметр колёс
const float WHEEL_BASE = 212.94; // Укажите ваще расстояние между колёсами
const float ENCODER_RESOLUTION = 330.0;
const float TICKS_TO_MM = (PI * WHEEL_DIAMETER) / ENCODER_RESOLUTION;
const float ALPHA = 0.2; // Коэффициент фильтрации скорости

// Глобальные коэффициенты PID – их можно обновлять через UART
float pidKp = 1.1;
float pidKi = 1.3;
float pidKd = 0.01;
float pidKff = 0.25;

/**
 * Переменные для настройки моргания светодиодом на плате
 */
bool led_state = false;
unsigned long previous_led_millis = 0;
const unsigned long LED_BLINK_INTERVAL_MS = 500;


/**
 * Обработчики прерываний для энкодеров:
 * - left_interrupt() изменяет счётчик левого энкодера в зависимости от состояния второго канала (LEFT_ENCODER_B).
 * - right_interrupt() аналогично обрабатывает импульсы правого энкодера, используя RIGHT_ENCODER_B.
 *
 * Логика:
 *   Если на втором канале энкодера HIGH — увеличиваем счётчик, иначе уменьшаем.
 *   Это классический способ определения направления вращения по двум каналам энкодера.
 */
void left_interrupt() {digitalRead(LEFT_ENCODER_B)?left_encoder_value++:left_encoder_value--;}
void right_interrupt() {digitalRead(RIGHT_ENCODER_B)?right_encoder_value++:right_encoder_value--;}

/*
 * Для светодиодной ленты FastLED, мы создаём 
 * глобальный массив типа CRGB, который будет хранить цвет каждого светодиода.
*/
CRGB leds[NUM_LEDS];
uint8_t currentLedBrightness = BRIGHTNESS;
CRGB currentLedColor = CRGB::White;
bool ledsNeedUpdate = true;

bool ina226Available = false;
uint8_t ina226Address = INA226_I2C_ADDRESS;
float ina226BusVoltage = 0.0f;
float ina226Current = 0.0f;
bool motorDriverEnabled = true;


/**
 * SETUP
 */
void setup() {
  Serial2.begin(JETSON_UART_BAUD, SERIAL_8N1, JETSON_UART_RX, JETSON_UART_TX);
  delay(1000);
  Serial2.println("System started");

  pinMode(INA226_I2C_SDA, INPUT_PULLUP);
  pinMode(INA226_I2C_SCL, INPUT_PULLUP);
  Wire.begin(INA226_I2C_SDA, INA226_I2C_SCL);
  Wire.setClock(100000);
  Wire.setTimeOut(50);
  ina226Available = initIna226();
  if (ina226Available) {
    Serial2.printf("INA226 started addr=0x%02X\r\n", ina226Address);
  } else {
    Serial2.println("INA226 not found");
  }

  // Инициализация ленты с указанием пина, типа и массива светодиодов
  FastLED.addLeds<LED_TYPE, LED_STRIP_PIN, COLOR_ORDER>(leds, NUM_LEDS).setCorrection(TypicalLEDStrip);
  FastLED.setBrightness(currentLedBrightness);
  fill_solid(leds, NUM_LEDS, currentLedColor);
  FastLED.show();

  /**
   * Подключение функций-прерываний (interrupt service routines, ISR) к выводам энкодеров.
   * digitalPinToInterrupt(ПИН) преобразует номер пина в номер прерывания, которое мы хотим отлавливать.
   * Событие RISING означает вызов прерывания при переходе сигнала с LOW на HIGH.
   */
  attachInterrupt(digitalPinToInterrupt(LEFT_ENCODER_A), left_interrupt, RISING);
  attachInterrupt(digitalPinToInterrupt(RIGHT_ENCODER_A), right_interrupt, RISING);
  
  // Настройка выводов для управления моторами как выходы (OUTPUT).
  pinMode(LEFT_MOTOR_A,OUTPUT);
  pinMode(LEFT_MOTOR_B,OUTPUT);
  pinMode(RIGHT_MOTOR_A,OUTPUT);
  pinMode(RIGHT_MOTOR_B,OUTPUT);
  pinMode(LED_BUILTIN_PIN, OUTPUT);
  
  // Настройка выводов энкодеров как входы (INPUT).
  pinMode(LEFT_ENCODER_A, INPUT);
  pinMode(LEFT_ENCODER_B, INPUT);
  pinMode(RIGHT_ENCODER_A, INPUT);
  pinMode(RIGHT_ENCODER_B, INPUT);
  
  /**
   * Настройка сигналов для управления скоростью и направлением вращения:
   *   - analogWrite(LEFT_MOTOR_A, 0) и analogWrite(LEFT_MOTOR_B, 100) задают движение левого мотора "вперед"
   *     (поскольку один вывод на 0, второй на некий уровень ШИМ).
   *   - analogWrite(RIGHT_MOTOR_A, 0) и analogWrite(RIGHT_MOTOR_B, 100) аналогично для правого мотора.
   *
   * В зависимости от используемой библиотеки и типа контроллера, analogWrite может включать ШИМ (PWM).
   * Значение 0 = 0% скважности, 255 = 100% (может зависеть от конкретной платформы).
   */
  // analogWrite(LEFT_MOTOR_A,0);
  // analogWrite(LEFT_MOTOR_B,255); 
  // analogWrite(RIGHT_MOTOR_A,0);
  // analogWrite(RIGHT_MOTOR_B,255);
}

/**
 * LOOP
 */
void loop() {
  // Блок для моргания встроенным светодиодом на плате ESP32 каждые 500 мс, чтобы показать, что система работает.
  unsigned long current_millis = millis();
  if (current_millis - previous_led_millis >= LED_BLINK_INTERVAL_MS) {
    previous_led_millis = current_millis;
    led_state = !led_state;
    digitalWrite(LED_BUILTIN_PIN, led_state ? HIGH : LOW);
  }
  if (ledsNeedUpdate) {
    FastLED.setBrightness(currentLedBrightness);
    fill_solid(leds, NUM_LEDS, currentLedColor);
    FastLED.show();
    ledsNeedUpdate = false;
  }

  if (Serial2.available()) {
    String command = Serial2.readStringUntil('\n');
    processCommand(command);
  }
  static uint32_t PIDTimer = 0;
  if (millis() - PIDTimer > 50) {
    computeSpeed();
    updateOdometry();
    computeWheelsPID();
    updateIna226();
    PIDTimer = millis();
  }

  static uint32_t printTimer = 0;
  if (millis() - printTimer > 100){
    Serial2.printf("POS X=%.2f Y=%.2f Th=%.2f ", xPos, yPos, theta);
    Serial2.printf("ENC L=%ld R=%ld ", left_encoder_value, right_encoder_value);
    Serial2.printf("SPD L=%.2f mm/s R=%.2f mm/s ", vlSpeedFiltered, vrSpeedFiltered);
    if (ina226Available) {
      Serial2.printf("PWR V=%.3f I=%.3f A\r\n", ina226BusVoltage, ina226Current);
    } else {
      Serial2.print("PWR V=nan I=nan A\r\n");
    }
    printTimer = millis();
  }

  static uint32_t ina226RetryTimer = 0;
  if (!ina226Available && millis() - ina226RetryTimer > 1000) {
    ina226Available = initIna226();
    if (ina226Available) {
      Serial2.printf("INA226 started addr=0x%02X\r\n", ina226Address);
    }
    ina226RetryTimer = millis();
  }
}

void processCommand(String command) {
  command.trim();
  if (command.startsWith("SET_PWM")) {
    int leftA_PWM = 0, leftB_PWM = 0, rightA_PWM =0, rightB_PWM = 0;
    if (parseSetPWM(command, &leftA_PWM, &leftB_PWM, &rightA_PWM, &rightB_PWM)) {
      setMotorsPWM(leftA_PWM, leftB_PWM, rightA_PWM, rightB_PWM);
      Serial2.print("OK: Set PWM");
    }
    return;
  } else if (command.startsWith("SET_POSE")) {
    float x, y, th;
    if (parseSetPose(command, &x, &y, &th)) {
      xPos = x;
      yPos = y;
      theta = th;
      Serial2.println("OK: Pose set");
    }
  } else if (command.startsWith("SET_WHEELS_SPEED")) {
    int leftSpeed = 0, rightSpeed = 0;
    if (parseSetSpeed(command, &leftSpeed, &rightSpeed)) {
      targetLeftWheelSpeed = leftSpeed;
      targetRightWheelSpeed = rightSpeed;
      Serial2.println("OK: Wheel speed set");
    }
  } else if (command.startsWith("SET_COEFF")) {
    float newKp, newKi, newKd, newKff;
    if (parseSetCoeff(command, &newKp, &newKi, &newKd, &newKff)) {
      pidKp = newKp;
      pidKi = newKi;
      pidKd = newKd;
      pidKff = newKff;
      Serial2.println("OK: Coefficients updated");
    } else {
      Serial2.println("ERROR: Invalid coefficients");
    }
  } else if (command.startsWith("SET_LED")) {
    int brightness = 0;
    String colorName;
    if (parseSetLed(command, &brightness, &colorName)) {
      CRGB parsedColor;
      if (parseColorName(colorName, &parsedColor)) {
        currentLedBrightness = constrain(brightness, 0, 255);
        currentLedColor = parsedColor;
        ledsNeedUpdate = true;
        Serial2.println("OK: LED updated");
      } else {
        Serial2.println("ERROR: Unknown color");
      }
    } else {
      Serial2.println("ERROR: Invalid SET_LED format");
    }
  } else if (command == "SHUTDOWN") {
    shutdownSystem();
    Serial2.println("OK: Shutdown");
  } else if (command == "I2C_SCAN") {
    scanI2cBus();
  } else if (command == "INA226_DEBUG") {
    printIna226Debug();
  } else {
    Serial2.println("ERROR: Unknown command");
  }
}

bool parseSetPose(const String& command, float* x, float* y, float* th) {
  int index1 = command.indexOf(' ');
  if (index1 == -1) return false;
  int index2 = command.indexOf(' ', index1 + 1);
  if (index2 == -1) return false;
  int index3 = command.indexOf(' ', index2 + 1);
  if (index3 == -1) return false;
  
  *x = command.substring(index1 + 1, index2).toFloat();
  *y = command.substring(index2 + 1, index3).toFloat();
  *th = command.substring(index3 + 1).toFloat();
  return true;
}

bool parseSetPWM(const String& command, int* leftA_PWM, int* leftB_PWM, int* rightA_PWM, int* rightB_PWM) {
  int index1 = command.indexOf(' ');
  if (index1 == -1) return false;
  int index2 = command.indexOf(' ', index1 + 1);
  if (index2 == -1) return false;
  int index3 = command.indexOf(' ', index2 + 1);
  if (index3 == -1) return false;
  int index4 = command.indexOf(' ', index3 + 1);
  if (index4 == -1) return false;

  *leftA_PWM = command.substring(index1 + 1, index2).toInt();
  *leftB_PWM = command.substring(index2 + 1, index3).toInt();
  *rightA_PWM = command.substring(index3 + 1, index4).toInt();
  *rightB_PWM = command.substring(index4 + 1).toInt();
  return true;
}


void setMotorsPWM(int leftA, int leftB, int rightA, int rightB) {
  if (!motorDriverEnabled) {
    analogWrite(LEFT_MOTOR_A, 0);
    analogWrite(LEFT_MOTOR_B, 0);
    analogWrite(RIGHT_MOTOR_A, 0);
    analogWrite(RIGHT_MOTOR_B, 0);
    return;
  }

  leftA  = constrain(leftA, 0, 255);
  leftB  = constrain(leftB, 0, 255);
  rightA = constrain(rightA, 0, 255);
  rightB = constrain(rightB, 0, 255);

  analogWrite(LEFT_MOTOR_A,   leftA);
  analogWrite(LEFT_MOTOR_B,   leftB);
  analogWrite(RIGHT_MOTOR_A,  rightA);
  analogWrite(RIGHT_MOTOR_B,  rightB);
}

void shutdownSystem() {
  targetLeftWheelSpeed = 0;
  targetRightWheelSpeed = 0;
  motorDriverEnabled = false;
  setMotorsPWM(0, 0, 0, 0);

  currentLedBrightness = 0;
  currentLedColor = CRGB::Black;
  fill_solid(leds, NUM_LEDS, CRGB::Black);
  FastLED.setBrightness(0);
  FastLED.show();
  ledsNeedUpdate = false;
}

void updateOdometry() {
  long deltaLeft = left_encoder_value - last_left_encoder;
  long deltaRight = right_encoder_value - last_right_encoder;

  last_left_encoder = left_encoder_value;
  last_right_encoder = right_encoder_value;

  float distLeft = deltaLeft * TICKS_TO_MM;
  float distRight = deltaRight * TICKS_TO_MM;

  float deltaS = (distLeft + distRight) / 2.0;
  float deltaTheta = (distRight - distLeft) / WHEEL_BASE;

  xPos += deltaS * cos(theta + deltaTheta / 2.0);
  yPos += deltaS * sin(theta + deltaTheta / 2.0);

  theta += deltaTheta;
}

void computeSpeed() {
  uint32_t now = micros();

  if (lastTimeL == 0 || lastTimeR == 0) { // Первая инициализация таймера
    lastTimeL = now;
    lastTimeR = now;
    last_speed_left_encoder = left_encoder_value;
    last_speed_right_encoder = right_encoder_value;
    return;
  }

  float dtL = (now - lastTimeL) * 0.000001;
  float dtR = (now - lastTimeR) * 0.000001;

  if (dtL > 0 && left_encoder_value != last_speed_left_encoder) {
    long deltaLeft = left_encoder_value - last_speed_left_encoder;
    float wlSpeed = ((float)deltaLeft / ENCODER_RESOLUTION) * 360.0f / dtL;
    float vlSpeed = (wlSpeed / 360.0f) * (PI * WHEEL_DIAMETER);
    vlSpeedFiltered = ALPHA * vlSpeed + (1.0f - ALPHA) * vlSpeedFiltered;
    last_speed_left_encoder = left_encoder_value;
    lastTimeL = now;

    //Serial.printf("Left: ΔEnc=%ld, dt=%.6f, Speed=%.2f mm/s\n", deltaLeft, dtL, vlSpeedFiltered);
  } else if (dtL > 0.005 && left_encoder_value == last_speed_left_encoder){
    vlSpeedFiltered = 0;
  }

  if (dtR > 0 && right_encoder_value != last_speed_right_encoder) {
    long deltaRight = right_encoder_value - last_speed_right_encoder;
    float wrSpeed = ((float)deltaRight / ENCODER_RESOLUTION) * 360.0f / dtR;
    float vrSpeed = (wrSpeed / 360.0f) * (PI * WHEEL_DIAMETER);
    vrSpeedFiltered = ALPHA * vrSpeed + (1.0f - ALPHA) * vrSpeedFiltered;
    last_speed_right_encoder = right_encoder_value;
    lastTimeR = now;

    //Serial.printf("Right: ΔEnc=%ld, dt=%.6f, Speed=%.2f mm/s\n", deltaRight, dtR, vrSpeedFiltered);
  } else if (dtR > 0.005 && right_encoder_value == last_speed_right_encoder){
    vrSpeedFiltered = 0;
  }
}

void computeWheelsPID(){
  if (!motorDriverEnabled) {
    setMotorsPWM(0, 0, 0, 0);
    return;
  }

  // Ограничение интегральной составляющей (anti-windup)
  const float integralLimit = 100.0;

  // Статические переменные для состояния PID для каждого колеса
  static float errorLeftIntegral = 0;
  static float errorRightIntegral = 0;
  static float prevErrorLeft = 0;
  static float prevErrorRight = 0;
  static uint32_t lastPIDTime = millis();
  // Для сброса интегральной составляющей при смене целевой скорости
  static float lastTargetLeft = 0.0;
  static float lastTargetRight = 0.0;

  // Сброс интеграла, если целевая скорость изменилась
  if(targetLeftWheelSpeed != lastTargetLeft){
    errorLeftIntegral = 0;
    prevErrorLeft = 0;
    lastTargetLeft = targetLeftWheelSpeed;
  }
  if(targetRightWheelSpeed != lastTargetRight){
    errorRightIntegral = 0;
    prevErrorRight = 0;
    lastTargetRight = targetRightWheelSpeed;
  }

  // Вычисляем интервал dt (в секундах)
  uint32_t now = millis();
  float dt = (now - lastPIDTime) / 1000.0f;
  if(dt < 0.001f) dt = 0.001f;
  lastPIDTime = now;

  // Расчет ошибок
  float errorLeft  = targetLeftWheelSpeed  - vlSpeedFiltered;
  float errorRight = targetRightWheelSpeed - vrSpeedFiltered;

  // Интегральная составляющая
  errorLeftIntegral  += errorLeft  * dt;
  errorRightIntegral += errorRight * dt;
  if(errorLeftIntegral > integralLimit)  errorLeftIntegral  = integralLimit;
  if(errorLeftIntegral < -integralLimit) errorLeftIntegral  = -integralLimit;
  if(errorRightIntegral > integralLimit)  errorRightIntegral = integralLimit;
  if(errorRightIntegral < -integralLimit) errorRightIntegral = -integralLimit;

  // Производная ошибки
  float derivativeLeft  = (errorLeft  - prevErrorLeft)  / dt;
  float derivativeRight = (errorRight - prevErrorRight) / dt;
  prevErrorLeft  = errorLeft;
  prevErrorRight = errorRight;

  // Вычисление PID-выхода с использованием глобальных коэффициентов
  float pidLeft  = pidKp * errorLeft  + pidKi * errorLeftIntegral  + pidKd * derivativeLeft;
  float pidRight = pidKp * errorRight + pidKi * errorRightIntegral + pidKd * derivativeRight;

  // Добавляем feedforward
  float outputLeft  = pidLeft  + pidKff * targetLeftWheelSpeed;
  float outputRight = pidRight + pidKff * targetRightWheelSpeed;

  // Преобразование в значения ШИМ
  int leftA_PWM = 0, leftB_PWM = 0;
  int rightA_PWM = 0, rightB_PWM = 0;

  if(outputLeft >= 0){
    leftA_PWM = 0;
    leftB_PWM = constrain((int)outputLeft, 0, 255);
  } else {
    leftA_PWM = constrain((int)(-outputLeft), 0, 255);
    leftB_PWM = 0;
  }
  if(outputRight >= 0){
    rightA_PWM = 0;
    rightB_PWM = constrain((int)outputRight, 0, 255);
  } else {
    rightA_PWM = constrain((int)(-outputRight), 0, 255);
    rightB_PWM = 0;
  }

  setMotorsPWM(leftA_PWM, leftB_PWM, rightA_PWM, rightB_PWM);
}

bool initIna226() {
  if (!findIna226()) {
    return false;
  }

  // AVG=16, VBUSCT=1.1 ms, VSHCT=1.1 ms, continuous shunt+bus mode.
  return writeIna226Register(INA226_REG_CONFIG, 0x4527);
}

bool findIna226() {
  uint8_t firstRespondingAddress = 0;

  for (uint8_t address = 0x40; address <= 0x4F; address++) {
    Wire.beginTransmission(address);
    if (Wire.endTransmission() != 0) {
      continue;
    }

    if (firstRespondingAddress == 0) {
      firstRespondingAddress = address;
    }

    ina226Address = address;
    uint16_t manufacturerId = 0;
    uint16_t dieId = 0;
    if (readIna226Register(INA226_REG_MANUFACTURER_ID, &manufacturerId) &&
        readIna226Register(INA226_REG_DIE_ID, &dieId) &&
        manufacturerId == 0x5449 &&
        (dieId & 0xFFF0) == 0x2260) {
      return true;
    }
  }

  if (firstRespondingAddress != 0) {
    ina226Address = firstRespondingAddress;
    return true;
  }

  ina226Address = INA226_I2C_ADDRESS;
  return false;
}

void updateIna226() {
  if (!ina226Available) {
    return;
  }

  uint16_t busRaw = 0;
  uint16_t shuntRaw = 0;
  if (!readIna226Register(INA226_REG_BUS_VOLTAGE, &busRaw) ||
      !readIna226Register(INA226_REG_SHUNT_VOLTAGE, &shuntRaw)) {
    ina226Available = false;
    return;
  }

  ina226BusVoltage = busRaw * 0.00125f;
  const float shuntVoltage = (int16_t)shuntRaw * 0.0000025f;
  ina226Current = shuntVoltage / INA226_SHUNT_OHMS;
}

bool readIna226Register(uint8_t reg, uint16_t* value) {
  Wire.beginTransmission(ina226Address);
  Wire.write(reg);
  if (Wire.endTransmission(false) != 0) {
    return false;
  }

  if (Wire.requestFrom(ina226Address, (uint8_t)2) != 2) {
    return false;
  }

  *value = ((uint16_t)Wire.read() << 8) | Wire.read();
  return true;
}

bool writeIna226Register(uint8_t reg, uint16_t value) {
  Wire.beginTransmission(ina226Address);
  Wire.write(reg);
  Wire.write((uint8_t)(value >> 8));
  Wire.write((uint8_t)(value & 0xFF));
  return Wire.endTransmission() == 0;
}

void scanI2cBus() {
  Serial2.print("I2C_SCAN");
  bool foundAny = false;
  for (uint8_t address = 1; address < 127; address++) {
    Wire.beginTransmission(address);
    if (Wire.endTransmission() == 0) {
      Serial2.printf(" 0x%02X", address);
      foundAny = true;
    }
  }

  if (!foundAny) {
    Serial2.print(" none");
  }
  Serial2.println();
}

void printIna226Debug() {
  Serial2.printf("INA226_DEBUG sda=%d scl=%d addr=0x%02X available=%d ",
                 INA226_I2C_SDA,
                 INA226_I2C_SCL,
                 ina226Address,
                 ina226Available ? 1 : 0);

  Wire.beginTransmission(INA226_I2C_ADDRESS);
  uint8_t probeError = Wire.endTransmission();
  Serial2.printf("probe_0x40=%u ", probeError);

  ina226Address = INA226_I2C_ADDRESS;
  uint16_t config = 0;
  uint16_t busRaw = 0;
  uint16_t shuntRaw = 0;
  uint16_t manufacturerId = 0;
  uint16_t dieId = 0;
  bool okConfig = readIna226Register(INA226_REG_CONFIG, &config);
  bool okBus = readIna226Register(INA226_REG_BUS_VOLTAGE, &busRaw);
  bool okShunt = readIna226Register(INA226_REG_SHUNT_VOLTAGE, &shuntRaw);
  bool okManufacturer = readIna226Register(INA226_REG_MANUFACTURER_ID, &manufacturerId);
  bool okDie = readIna226Register(INA226_REG_DIE_ID, &dieId);

  Serial2.printf("cfg=%d:0x%04X bus=%d:%u shunt=%d:%d mfg=%d:0x%04X die=%d:0x%04X\r\n",
                 okConfig ? 1 : 0,
                 config,
                 okBus ? 1 : 0,
                 busRaw,
                 okShunt ? 1 : 0,
                 (int16_t)shuntRaw,
                 okManufacturer ? 1 : 0,
                 manufacturerId,
                 okDie ? 1 : 0,
                 dieId);
}

// Функция для парсинга команды установки коэффициентов: "SET_COEFF Kp Ki Kd Kff"
bool parseSetCoeff(const String& command, float* Kp, float* Ki, float* Kd, float* Kff) {
  int index1 = command.indexOf(' ');
  if(index1 == -1) return false;
  int index2 = command.indexOf(' ', index1 + 1);
  if(index2 == -1) return false;
  int index3 = command.indexOf(' ', index2 + 1);
  if(index3 == -1) return false;
  int index4 = command.indexOf(' ', index3 + 1);
  if(index4 == -1) return false;

  *Kp = command.substring(index1 + 1, index2).toFloat();
  *Ki = command.substring(index2 + 1, index3).toFloat();
  *Kd = command.substring(index3 + 1, index4).toFloat();
  *Kff = command.substring(index4 + 1).toFloat();
  return true;
}

bool parseSetSpeed(const String& command, int* speedLeft, int* speedRight) {
  int index1 = command.indexOf(' ');
  if (index1 == -1) return false;
  int index2 = command.indexOf(' ', index1 + 1);
  if (index2 == -1) return false;

  *speedLeft = command.substring(index1 + 1, index2).toInt();
  *speedRight = command.substring(index2 + 1).toInt();
  return true;
}

bool parseSetLed(const String& command, int* brightness, String* colorName) {
  int index1 = command.indexOf(' ');
  if (index1 == -1) return false;
  int index2 = command.indexOf(' ', index1 + 1);
  if (index2 == -1) return false;

  *brightness = command.substring(index1 + 1, index2).toInt();
  *colorName = command.substring(index2 + 1);
  colorName->trim();
  return true;
}

bool parseColorName(const String& name, CRGB* color) {
  String lowerName = name;
  lowerName.toLowerCase();

  if (lowerName == "white") {
    *color = CRGB::White;
  } else if (lowerName == "red") {
    *color = CRGB::Red;
  } else if (lowerName == "green") {
    *color = CRGB::Green;
  } else if (lowerName == "blue") {
    *color = CRGB::Blue;
  } else if (lowerName == "yellow") {
    *color = CRGB::Yellow;
  } else if (lowerName == "cyan") {
    *color = CRGB::Cyan;
  } else if (lowerName == "magenta") {
    *color = CRGB::Magenta;
  } else if (lowerName == "orange") {
    *color = CRGB::Orange;
  } else if (lowerName == "purple") {
    *color = CRGB::Purple;
  } else if (lowerName == "black" || lowerName == "off") {
    *color = CRGB::Black;
  } else {
    return false;
  }
  return true;
}
