#include "tclac.h"
#include "esphome/core/log.h"
#include <cstring>

namespace esphome {
namespace tclac {

static const char *const TAG = "tclac";

void TCLAC::setup() {
  ESP_LOGCONFIG(TAG, "Setting up TCL AC...");
  // Полная инициализация массивов нулями
  memset(dataTX, 0, sizeof(dataTX));
  memset(dataRX, 0, sizeof(dataRX));
  
  waiting_for_response = false;
  allow_take_control = false;
  is_call_control = false;
}

void TCLAC::loop() {
  // Неблокирующее чтение с таймаутом
  if (Serial.available() > 0) {
    delay(2); // Небольшая задержка для накопления пакета
    
    int len = Serial.available();
    if (len > 61) len = 61; // Защита от переполнения

    for (int i = 0; i < len; i++) {
      dataRX[i] = Serial.read();
    }

    // Проверка заголовка (55 AA)
    if (dataRX[0] == 0x55 && dataRX[1] == 0xAA) {
      ESP_LOGD(TAG, "RX Header OK, Length: %d", len);
      // Вывод первых байт для отладки
      ESP_LOGD(TAG, "RX Data: %s", getHex(dataRX, len).c_str());

      // Простая проверка контрольной суммы (если длина достаточна)
      // Реальная логика проверки зависит от структуры пакета TCL
      // Здесь мы предполагаем успешный прием для демонстрации сброса флага
      
      waiting_for_response = false;
      last_command_time = 0;
      
      // Разрешаем управление только после успешного получения ответа
      if (!allow_take_control) {
        ESP_LOGI(TAG, "First successful response received. Control allowed.");
        allow_take_control = true;
      }
      
      // TODO: Здесь должен быть парсинг температуры и состояния
      // Для примера просто публикуем текущее состояние без изменений
      publish_state();
    } else {
      ESP_LOGW(TAG, "Invalid header received: %02X %02X", dataRX[0], dataRX[1]);
    }
  }

  // Проверка таймаута ожидания ответа (2 секунды)
  if (waiting_for_response && (millis() - last_command_time > 2000)) {
    ESP_LOGW(TAG, "Timeout waiting for response from AC!");
    waiting_for_response = false;
    // Сбрасываем флаг управления, чтобы избежать ошибок при следующей попытке
    // Но не блокируем навсегда, даем шанс на повторный опрос
    allow_take_control = false; 
  }

  yield(); // Важно для предотвращения watchdog reset
}

void TCLAC::update() {
  if (waiting_for_response) {
    ESP_LOGD(TAG, "Update skipped: waiting for response");
    return;
  }
  
  ESP_LOGD(TAG, "Periodic update: requesting status");
  takeControl();
}

void TCLAC::control(const climate::ClimateCall &call) {
  // ГЛАВНОЕ ИСПРАВЛЕНИЕ: Запрет управления до первого успешного чтения
  if (!allow_take_control) {
    ESP_LOGW(TAG, "Control blocked: Waiting for initial AC response. Please turn on AC with remote first.");
    return;
  }

  if (waiting_for_response) {
    ESP_LOGW(TAG, "Control blocked: Busy waiting for previous response");
    return;
  }

  ESP_LOGD(TAG, "Processing control call...");
  
  // Сохраняем желаемые состояния в объект Climate
  if (call.get_mode().has_value()) this->mode = call.get_mode().value();
  if (call.get_target_temperature().has_value()) this->target_temperature = call.get_target_temperature().value();
  if (call.get_custom_fan_mode().has_value()) this->custom_fan_mode = call.get_custom_fan_mode().value();

  // Формируем команду
  takeControl(true); // true означает, что это команда управления, а не запрос статуса
}

void TCLAC::takeControl(bool is_command) {
  memset(dataTX, 0, sizeof(dataTX));

  // Формирование базового пакета TCL (пример структуры)
  dataTX[0] = 0x55;
  dataTX[1] = 0xAA;
  dataTX[2] = 0x01; // Address
  dataTX[3] = 0x01; 
  
  if (is_command) {
    dataTX[4] = 0x02; // Command type
    dataTX[5] = 0x0D; // Length
    // Здесь нужно вставить логику установки битов в зависимости от mode, temp, fan
    // Для краткости оставим заглушку, которую нужно дополнить вашей логикой битов
    ESP_LOGD(TAG, "Sending CONTROL command");
  } else {
    dataTX[4] = 0x02; // Request type
    dataTX[5] = 0x0B; // Length
    ESP_LOGD(TAG, "Sending STATUS REQUEST");
  }

  // Расчет контрольной суммы
  uint8_t sum = checkSum(dataTX, 37); // Сумма по всем байтам кроме последнего
  dataTX[37] = sum;

  ESP_LOGD(TAG, "TX Packet: %s", getHex(dataTX, 38).c_str());

  waiting_for_response = true;
  last_command_time = millis();
  sendData();
}

void TCLAC::sendData() {
  Serial.write(dataTX, 38);
  Serial.flush();
  ESP_LOGD(TAG, "Data sent via UART");
}

std::string TCLAC::getHex(uint8_t *data, int len) {
  char buffer[180];
  int offset = 0;
  for (int i = 0; i < len; i++) {
    offset += sprintf(buffer + offset, "%02X ", data[i]);
  }
  return std::string(buffer);
}

uint8_t TCLAC::checkSum(uint8_t *data, uint8_t len) {
  uint8_t sum = 0;
  for (int i = 0; i < len; i++) {
    sum += data[i];
  }
  return sum;
}

climate::ClimateTraits TCLAC::traits() {
  auto traits = climate::ClimateTraits();
  traits.set_supports_current_temperature(false);
  traits.set_visual_min_temperature(16);
  traits.set_visual_max_temperature(32);
  traits.set_visual_temperature_step(1);
  traits.add_supported_mode(climate::CLIMATE_MODE_OFF);
  traits.add_supported_mode(climate::CLIMATE_MODE_COOL);
  traits.add_supported_mode(climate::CLIMATE_MODE_HEAT);
  traits.add_supported_mode(climate::CLIMATE_MODE_FAN_ONLY);
  traits.add_supported_mode(climate::CLIMATE_MODE_DRY);
  traits.add_supported_custom_fan_mode("Low");
  traits.add_supported_custom_fan_mode("Medium");
  traits.add_supported_custom_fan_mode("High");
  traits.add_supported_custom_fan_mode("Auto");
  return traits;
}

} // namespace tclac
} // namespace esphome
