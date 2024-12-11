#include <SimpleFOC.h>
extern "C" {
  #include "can2040.h"
  #include "canard.h" // Основной файл библиотеки DroneCAN
}



#define CanardTransferTypeMessage 0

// DroneCAN настройки
static CanardInstance canard;
static uint8_t canard_memory_pool[1024];
const uint8_t node_id = 10; // ID узла DroneCAN
const uint32_t TARGET_ANGLE_SUBJECT_ID = 1000; // ID темы для целевого угла
const uint32_t CURRENT_ANGLE_SUBJECT_ID = 1001; // ID темы для текущего угла

// Настройки CAN2040
static struct can2040 cbus;
static struct can2040_msg msg;

// Настройки мотора
const uint8_t endstopPin = 9;
BLDCMotor motor = BLDCMotor(7);
BLDCDriver3PWM driver = BLDCDriver3PWM(28, 27, 26, 15);
MagneticSensorI2C sensor = MagneticSensorI2C(AS5600_I2C);

double target_angle = 0;
double angleMin = 0;
double angleMax = 21;

// Функция для мапирования значений
double mapDouble(double x, double in_min, double in_max, double out_min, double out_max) {
  return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}

// Callback для обработки входящих сообщений
void onReception(CanardInstance* ins, CanardRxTransfer* transfer) {
  if (transfer->data_type_id == TARGET_ANGLE_SUBJECT_ID && 
      transfer->transfer_type == CanardTransferTypeMessage) {
    if (transfer->payload_len >= sizeof(float)) { // Проверяем размер данных
      float received_angle = 0;
      memcpy(&received_angle, transfer->payload_head, sizeof(float)); // Копируем данные
      target_angle = received_angle; // Обновляем целевой угол
      Serial.print("Received target angle: ");
      Serial.println(target_angle);
    } else {
      Serial.println("Received payload is too small to contain a float.");
    }
  }
}
// Обработчик для CAN2040
void can2040_cb(can2040 *cd, uint32_t notify, can2040_msg *msg) {
  // Копируем данные из сообщения в глобальную переменную для дальнейшей обработки
  uint32_t id = msg->id;       // Получаем ID сообщения
  uint8_t dlc = msg->dlc;      // Длина сообщения
  uint8_t data[8];             // Буфер для данных
  memcpy(data, msg->data, dlc); // Копируем данные

  // Отладочный вывод
  Serial.print("Received CAN message: ID = ");
  Serial.print(id, HEX);
  Serial.print(", Length = ");
  Serial.print(dlc);
  Serial.print(", Data = ");
  for (int i = 0; i < dlc; i++) {
    Serial.print(data[i], HEX);
    Serial.print(" ");
  }
  Serial.println();
}

// Фильтрация сообщений
bool shouldAccept(const CanardInstance* ins, uint64_t* out_data_type_signature, 
                  uint16_t data_type_id, CanardTransferType transfer_type, 
                  uint8_t source_node_id) {
  if (transfer_type == CanardTransferTypeMessage && data_type_id == TARGET_ANGLE_SUBJECT_ID) {
    *out_data_type_signature = 0;
    return true;
  }
  return false;
}

// Инициализация DroneCAN
void initDroneCAN() {
  canardInit(&canard, canard_memory_pool, sizeof(canard_memory_pool), onReception, shouldAccept, nullptr);
  canard.node_id = node_id;
  Serial.println("DroneCAN initialized");
}

// Инициализация CAN2040
void initCAN() {
  uint32_t sys_clock = 125000000;
  uint32_t bitrate = 500000;
  uint32_t gpio_rx = 8;
  uint32_t gpio_tx = 9;

  can2040_setup(&cbus, 0);
  can2040_callback_config(&cbus, can2040_cb);
  can2040_start(&cbus, sys_clock, bitrate, gpio_rx, gpio_tx);
  Serial.println("CAN2040 initialized");
}

// Реализация функции чтения кадров CAN
bool readCanFrame(CanardCANFrame* frame) {
  if (msg.dlc > 0) { // Проверяем, есть ли новое сообщение
    frame->id = msg.id;
    frame->data_len = msg.dlc;
    memcpy(frame->data, msg.data, msg.dlc);
    msg.dlc = 0; // Сбрасываем буфер после обработки
    return true;
  }
  return false;
}
// Обработка входящих сообщений
void processIncomingFrames() {
  CanardCANFrame frame;
  uint64_t timestamp_usec = micros();
  if (readCanFrame(&frame)) {
    int16_t result = canardHandleRxFrame(&canard, &frame, timestamp_usec);
    if (result < 0) {
      Serial.print("Error handling incoming frame: ");
      Serial.println(result);
    }
  }
}

// Передача текущего угла через CAN
void sendCurrentAngle() {
  float current_angle = motor.shaft_angle;
  uint8_t payload[4];
  memcpy(payload, &current_angle, sizeof(float));

  CanardTxTransfer transfer = {
      .transfer_type = CanardTransferTypeBroadcast,
      .data_type_signature = 0,
      .data_type_id = CURRENT_ANGLE_SUBJECT_ID,
      .inout_transfer_id = nullptr,
      .priority = CANARD_TRANSFER_PRIORITY_MEDIUM,
      .payload = payload,
      .payload_len = sizeof(payload)
  };

  int16_t result = canardBroadcastObj(&canard, &transfer);
  if (result < 0) {
    Serial.print("Error sending current angle: ");
    Serial.println(result);
  }
}

void setup() {
  Serial.begin(115200);
  delay(1000);

  pinMode(endstopPin, INPUT_PULLUP);

  initDroneCAN();
  initCAN();

  // Инициализация мотора
  sensor.init();
  motor.linkSensor(&sensor);

  driver.voltage_power_supply = 32;
  driver.voltage_limit = 12;
  driver.init();
  motor.linkDriver(&driver);

  motor.foc_modulation = FOCModulationType::SpaceVectorPWM;
  motor.torque_controller = TorqueControlType::voltage;
  motor.controller = MotionControlType::angle;

  motor.PID_velocity.P = 0.2;
  motor.PID_velocity.I = 0.2;
  motor.PID_velocity.D = 0.0001;
  motor.LPF_velocity.Tf = 0.12;
  motor.velocity_limit = 25;
  motor.current_limit = 1;
  motor.voltage_limit = 11;

  Serial.begin(115200);
  motor.useMonitoring(Serial);

  motor.init();
  motor.initFOC();

  float search_speed = 100;
  motor.move(search_speed);

  while (digitalRead(endstopPin) == HIGH) {
    motor.loopFOC();
    motor.move(search_speed);
    delay(10);
  }

  motor.move(0);
  sensor.update();
  float initial_position = -sensor.getAngle();
  angleMin = initial_position;
  angleMax = angleMin - 21;
  Serial.println(initial_position);
  Serial.println("Initial position found.");

  Serial.println("Motor ready.");
  delay(100);

  Serial.println("Setup complete");
}

void loop() {
  motor.loopFOC();
  motor.monitor();

  // Управляем мотором на основе целевого угла
  motor.move(target_angle);

  // Передаем текущий угол
  sendCurrentAngle();

  // Обрабатываем входящие сообщения
  processIncomingFrames();

  delay(10);
}
