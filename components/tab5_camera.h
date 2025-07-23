#pragma once

#include "esphome/core/component.h"
#include "esphome/core/hal.h"
#include "esphome/components/camera/camera.h"

#ifdef USE_ESP32

#include "esp_camera.h"
#include "esp_system.h"
#include "esp_log.h"

namespace esphome {
namespace tab5_camera {

class Tab5Camera : public camera::Camera, public Component {
 public:
  void setup() override;
  void dump_config() override;
  float get_setup_priority() const override;
  
  void set_external_clock_pin(uint8_t pin) { this->external_clock_pin_ = pin; }
  void set_external_clock_frequency(uint32_t frequency) { this->external_clock_frequency_ = frequency; }
  void set_reset_pin(GPIOPin *pin) { this->reset_pin_ = pin; }
  
  camera::CameraImage *snapshot() override;

 protected:
  bool init_camera_();
  void deinit_camera_();
  
  uint8_t external_clock_pin_{36};
  uint32_t external_clock_frequency_{20000000}; // 20MHz
  GPIOPin *reset_pin_{nullptr};
  
  camera_config_t camera_config_;
  bool camera_initialized_{false};
};

}  // namespace tab5_camera
}  // namespace esphome

#endif  // USE_ESP32
