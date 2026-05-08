#include <Arduino.h>

#define LEFT_MOTOR_A 14     // Цифровой выход (левый мотор). Если 0 - едем вперед
#define LEFT_MOTOR_B 27     // Цифровой выход (левый мотор). Если 0 - едем назад
#define RIGHT_MOTOR_A 25    // Цифровой выход (правый мотор). Если 0 - едем вперед
#define RIGHT_MOTOR_B 26    // Цифровой выход (правый мотор). Если 0 - едем назад

#define LED_PIN 2           // Встроенный светодиод на ESP32 Dev Board

#define LEFT_ENCODER_A 33    // Цифровой вход, канал A энкодера левого колеса
#define LEFT_ENCODER_B 32    // Цифровой вход, канал B энкодера левого колеса 
#define RIGHT_ENCODER_A 34   // Цифровой вход, канал A энкодера правого колеса
#define RIGHT_ENCODER_B 35   // Цифровой вход, канал B энкодера правого колеса

#define JETSON_UART_RX 16     // RX2: прием данных от Jetson TX
#define JETSON_UART_TX 17     // TX2: передача данных на Jetson RX
#define JETSON_UART_BAUD 115200

/**
 * Глобальные переменные для хранения текущего счёта тиков (импульсов) энкодеров.
 * Используется тип long, т.к. значения могут быть достаточно большими при длительной работе.
 * Ключевое слово volatile указывает, что данные могут изменяться асинхронно (в прерываниях).
 */
volatile long left_encoder_value = 0; 
volatile long right_encoder_value = 0;

bool led_state = false;
unsigned long previous_led_millis = 0;
const unsigned long LED_BLINK_INTERVAL_MS = 500;

void processCommand(String command);
bool parseSetPWM(const String& command, int* leftA_PWM, int* leftB_PWM, int* rightA_PWM, int* rightB_PWM);
void setMotorsPWM(int leftA, int leftB, int rightA, int rightB);


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

void setup() {
  Serial2.begin(JETSON_UART_BAUD, SERIAL_8N1, JETSON_UART_RX, JETSON_UART_TX);
  delay(1000);
  Serial2.println("System started");

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
  pinMode(LED_PIN, OUTPUT);
  
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

void loop() {
  unsigned long current_millis = millis();
  if (current_millis - previous_led_millis >= LED_BLINK_INTERVAL_MS) {
    previous_led_millis = current_millis;
    led_state = !led_state;
    digitalWrite(LED_PIN, led_state ? HIGH : LOW);
  }
  if (Serial2.available()) {
    String command = Serial2.readStringUntil('\n');
    processCommand(command);
  }
  static uint32_t printTimer = 0;
  if (millis() - printTimer > 100){
    Serial2.print(" Left_encoder: "); Serial2.print(left_encoder_value);
    Serial2.print(" Right_encoder: "); Serial2.print(right_encoder_value);
    Serial2.println();
    printTimer = millis();
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
  } else {
    Serial2.println("ERROR: Unknown command");
  }
}
bool parseSetPWM(const String& command, int* leftA_PWM, int* leftB_PWM, int* rightA_PWM, int* rightB_PWM) {
  // Ищем, где заканчивается "SET_PWM " и начинаются параметры
  int index = command.indexOf(' ');
  if (index == -1) {
    Serial2.println("ERROR: Invalid SET_PWM command (no parameters found)");
    return false;
  }
  // Извлекаем подстроку с параметрами
  String params = command.substring(index + 1);
  params.trim(); // Убираем пробелы по краям

  // Нам нужно выделить 4 числа (leftA, leftB, rightA, rightB)
  // Воспользуемся последовательным поиском пробелов
  int idx1 = params.indexOf(' ');
  if (idx1 == -1) {
    Serial2.println("ERROR: Missing parameters for SET_PWM");
    return false;
  }
  String leftAStr = params.substring(0, idx1);
  leftAStr.trim();

  int idx2 = params.indexOf(' ', idx1 + 1);
  if (idx2 == -1) {
    Serial2.println("ERROR: Not enough parameters for SET_PWM (need 4)");
    return false;
  }
  String leftBStr = params.substring(idx1 + 1, idx2);
  leftBStr.trim();

  int idx3 = params.indexOf(' ', idx2 + 1);
  if (idx3 == -1) {
    Serial2.println("ERROR: Not enough parameters for SET_PWM (need 4)");
    return false;
  }
  String rightAStr = params.substring(idx2 + 1, idx3);
  rightAStr.trim();

  // Оставшуюся часть строки считаем четвертым параметром
  String rightBStr = params.substring(idx3 + 1);
  rightBStr.trim();

  // Преобразуем строки в int
  int la = leftAStr.toInt();
  int lb = leftBStr.toInt();
  int ra = rightAStr.toInt();
  int rb = rightBStr.toInt();

  // Записываем в выходные параметры
  *leftA_PWM  = la;
  *leftB_PWM  = lb;
  *rightA_PWM = ra;
  *rightB_PWM = rb;

  return true;
}
void setMotorsPWM(int leftA, int leftB, int rightA, int rightB) {
  leftA  = constrain(leftA, 0, 255);
  leftB  = constrain(leftB, 0, 255);
  rightA = constrain(rightA, 0, 255);
  rightB = constrain(rightB, 0, 255);

  analogWrite(LEFT_MOTOR_A,   leftA);
  analogWrite(LEFT_MOTOR_B,   leftB);
  analogWrite(RIGHT_MOTOR_A,  rightA);
  analogWrite(RIGHT_MOTOR_B,  rightB);
}
