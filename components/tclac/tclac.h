#pragma once

#include "esphome/core/component.h"
#include "esphome/components/climate/climate.h"
#include "esphome/components/uart/uart.h"
#include "esphome/core/gpio.h"
#include "esphome/core/helpers.h"

namespace esphome {
namespace tclac {

enum VerticalSwingDirection : uint8_t {
  UPDOWN = 0,
  UPSIDE = 1,
  DOWNSIDE = 2,
};

enum HorizontalSwingDirection : uint8_t {
  LEFT_RIGHT = 0,
  LEFTSIDE = 1,
  CENTER = 2,
  RIGHTSIDE = 3,
};

enum AirflowVerticalDirection : uint8_t {
  LAST = 0,
  MAX_UP = 1,
  UP = 2,
  CENTER_VERTICAL = 3,
  DOWN = 4,
  MAX_DOWN = 5,
};

enum AirflowHorizontalDirection : uint8_t {
  LAST_HORIZONTAL = 0,
  MAX_LEFT = 1,
  LEFT = 2,
  CENTER_HORIZONTAL = 3,
  RIGHT = 4,
  MAX_RIGHT = 5,
};

class tclacClimate : public climate::Climate, public PollingComponent, public uart::UARTDevice {
 public:
  tclacClimate() = default;
  explicit tclacClimate(uart::UARTComponent *parent) : uart::UARTDevice(parent) {}

  climate::ClimateTraits traits() override;
  void setup() override;
  void loop() override;
  void update() override;
  void control(const climate::ClimateCall &call) override;

  void set_beeper_state(bool state);
  void set_display_state(bool state);
  void set_force_mode_state(bool state);
  void set_module_display_state(bool state);
  void set_vertical_airflow(AirflowVerticalDirection direction);
  void set_horizontal_airflow(AirflowHorizontalDirection direction);
  void set_vertical_swing_direction(VerticalSwingDirection direction);
  void set_horizontal_swing_direction(HorizontalSwingDirection direction);
  void set_supported_modes(climate::ClimateModeMask modes);
  void set_supported_fan_modes(climate::ClimateFanModeMask modes);
  void set_supported_swing_modes(climate::ClimateSwingModeMask modes);
  void set_supported_presets(climate::ClimatePresetMask presets);

#ifdef CONF_RX_LED
  void set_rx_led_pin(GPIOPin *rx_led_pin);
#endif
#ifdef CONF_TX_LED
  void set_tx_led_pin(GPIOPin *tx_led_pin);
#endif

 protected:
  static const uint8_t MODE_POS = 7;
  static const uint8_t FAN_SPEED_POS = 10;
  static const uint8_t FAN_QUIET_POS = 8;
  static const uint8_t SWING_POS = 10;

  static const uint8_t MODE_MASK = 0x0F;
  static const uint8_t SET_TEMP_MASK = 0x0F;
  static const uint8_t FAN_SPEED_MASK = 0x07;
  static const uint8_t SWING_MODE_MASK = 0x38;

  static const uint8_t MODE_AUTO = 0x08;
  static const uint8_t MODE_COOL = 0x03;
  static const uint8_t MODE_DRY = 0x02;
  static const uint8_t MODE_FAN_ONLY = 0x07;
  static const uint8_t MODE_HEAT = 0x01;

  static const uint8_t FAN_AUTO = 0x00;
  static const uint8_t FAN_LOW = 0x01;
  static const uint8_t FAN_MIDDLE = 0x06;
  static const uint8_t FAN_MEDIUM = 0x03;
  static const uint8_t FAN_HIGH = 0x07;
  static const uint8_t FAN_FOCUS = 0x05;
  static const uint8_t FAN_QUIET = 0x80;
  static const uint8_t FAN_DIFFUSE = 0x40;

  static const uint8_t SWING_OFF = 0x00;
  static const uint8_t SWING_VERTICAL = 0x38;
  static const uint8_t SWING_HORIZONTAL = 0x08;
  static const uint8_t SWING_BOTH = 0x38;

  void readData();
  void takeControl();
  void sendData(uint8_t *message, uint8_t size);
  String getHex(uint8_t *message, uint8_t size);
  uint8_t getChecksum(const uint8_t *message, size_t size);
  void dataShow(bool flow, bool shine);

  bool initialized_{false};
  bool allow_take_control{false};
  bool is_call_control{false};
  uint32_t last_control_time_{0};

  bool beeper_status_{true};
  bool display_status_{true};
  bool force_mode_status_{true};
  bool module_display_status_{true};

  VerticalSwingDirection vertical_swing_direction_{VerticalSwingDirection::UPDOWN};
  HorizontalSwingDirection horizontal_swing_direction_{HorizontalSwingDirection::LEFT_RIGHT};
  AirflowVerticalDirection vertical_direction_{AirflowVerticalDirection::CENTER_VERTICAL};
  AirflowHorizontalDirection horizontal_direction_{AirflowHorizontalDirection::CENTER_HORIZONTAL};

  climate::ClimateMode switch_climate_mode{climate::CLIMATE_MODE_OFF};
  climate::ClimatePreset switch_preset{climate::CLIMATE_PRESET_NONE};
  climate::ClimateFanMode switch_fan_mode{climate::CLIMATE_FAN_AUTO};
  climate::ClimateSwingMode switch_swing_mode{climate::CLIMATE_SWING_OFF};

  climate::ClimateModeMask supported_modes_{};
  climate::ClimateFanModeMask supported_fan_modes_{};
  climate::ClimateSwingModeMask supported_swing_modes_{};
  climate::ClimatePresetMask supported_presets_{};

  uint8_t target_temperature_set{15};

  uint8_t dataRX[61]{};
  uint8_t dataTX[38]{};

  const uint8_t poll[30] = {
      0xBB, 0x00, 0x01, 0x04, 0x19, 0x03, 0x01, 0x00, 0x00, 0x00,
      0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
      0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xA6};

#ifdef CONF_RX_LED
  GPIOPin *rx_led_pin_{nullptr};
#endif
#ifdef CONF_TX_LED
  GPIOPin *tx_led_pin_{nullptr};
#endif
};

}  // namespace tclac
}  // namespace esphome
