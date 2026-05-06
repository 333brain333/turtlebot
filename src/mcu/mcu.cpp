#define LEFT_MOTOR_A 14     // Цифровой выход (левый мотор). Если 0 - едем вперед
#define LEFT_MOTOR_B 27     // Цифровой выход (левый мотор). Если 0 - едем назад
#define RIGHT_MOTOR_A 26    // Цифровой выход (правый мотор). Если 0 - едем вперед
#define RIGHT_MOTOR_B 25    // Цифровой выход (правый мотор). Если 0 - едем назад

#define LEFT_ENCODER_A 33    // Цифровой вход, канал A энкодера левого колеса
#define LEFT_ENCODER_B 32    // Цифровой вход, канал B энкодера левого колеса 
#define RIGHT_ENCODER_A 35   // Цифровой вход, канал A энкодера правого колеса
#define RIGHT_ENCODER_B 34   // Цифровой вход, канал B энкодера правого колеса

/**
 * Глобальные переменные для хранения текущего счёта тиков (импульсов) энкодеров.
 * Используется тип long, т.к. значения могут быть достаточно большими при длительной работе.
 * Ключевое слово volatile указывает, что данные могут изменяться асинхронно (в прерываниях).
 */
volatile long left_encoder_value = 0; 
volatile long right_encoder_value = 0;


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
  Serial.begin(115200);
  delay(1000);
  Serial.println("System started");

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
  analogWrite(LEFT_MOTOR_A,0);
  analogWrite(LEFT_MOTOR_B,100); 
  analogWrite(RIGHT_MOTOR_A,0);
  analogWrite(RIGHT_MOTOR_B,100);
}

void loop() {
  Serial.print(" Left_encoder: "); Serial.print(left_encoder_value);
  Serial.print(" Right_encoder: "); Serial.print(right_encoder_value);
  Serial.println();
}
