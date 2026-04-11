#pragma once

#include "esphome/core/component.h"
#include "esphome/components/climate/climate.h"
#include "esphome/core/log.h"

namespace esphome {
namespace tclac {

class TCLAC : public Component, public climate::Climate {
 public:
  void setup() override;
  void loop() override;
  void update() override;

 protected:
  uint8_t dataTX[38] = {0}; // Увеличен размер под полный пакет
  uint8_t dataRX[61] = {0}; // Увеличен размер под полный пакет
  
  bool waiting_for_response = false;
  unsigned long last_command_time = 0;
  bool allow_take_control = false;
  bool is_call_control = false;

  void control(const climate::ClimateCall &call) override;
  climate::ClimateTraits traits() override;
  void readData();
  void takeControl();
  void sendData();
  std::string getHex(uint8_t *data, int len);
  uint8_t checkSum(uint8_t *data, uint8_t len);
  
  // Вспомогательные переменные для парсинга
  float current_temp_ = NAN;
};

} // namespace tclac
} // namespace esphome
