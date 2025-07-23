#pragma once

#include "esphome/core/component.h"
#include "esphome/core/hal.h"

#ifdef USE_ESP32

// Vérification de disponibilité d'esp_camera
#if __has_include("esp_camera.h")
#include "esp_camera.h"
#include "esp_system.h"
#include "esp_log.h"
#define HAS_ESP_CAMERA
#endif

namespace esphome {
namespace tab5_camera {

class Tab5Camera : public Component {
 public:
  void setup() override;
  void dump_config() override;
  float get_setup_priority() const override;
  
  void set_name(const std::string &name) { this->name_ = name; }
  void set_external_clock_pin(uint8_t pin) { this->external_clock_pin_ = pin; }
  void set_external_clock_frequency(uint32_t frequency) { this->external_clock_frequency_ = frequency; }
  void set_reset_pin(GPIOPin *pin) { this->reset_pin_ = pin; }
  
  bool take_snapshot();

 protected:
#ifdef HAS_ESP_CAMERA
  bool init_camera_();
  void deinit_camera_();
  
  camera_config_t camera_config_;
  bool camera_initialized_{false};
#endif
  
  std::string name_;
  uint8_t external_clock_pin_{36};
  uint32_t external_clock_frequency_{20000000}; // 20MHz
  GPIOPin *reset_pin_{nullptr};
};

}  // namespace tab5_camera
}  // namespace esphome

#endif  // USE_ESP32



