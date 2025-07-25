#pragma once

#include "esphome/core/component.h"
#include "esphome/core/hal.h"

#ifdef USE_ESP32

// Nouvelle API ESP32-P4
#if __has_include("esp_cam_ctlr_csi.h")
#include "esp_cam_ctlr_csi.h"
#include "esp_cam_ctlr.h"
#include "driver/isp.h"
#include "esp_cache.h"
#define HAS_ESP32_P4_CAMERA
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
#ifdef HAS_ESP32_P4_CAMERA
  bool init_camera_();
  void deinit_camera_();
  
  static bool camera_get_new_vb_callback(esp_cam_ctlr_handle_t handle, esp_cam_ctlr_trans_t *trans, void *user_data);
  static bool camera_get_finished_trans_callback(esp_cam_ctlr_handle_t handle, esp_cam_ctlr_trans_t *trans, void *user_data);
  
  esp_cam_ctlr_handle_t cam_handle_{nullptr};
  isp_proc_handle_t isp_proc_{nullptr};
  void *frame_buffer_{nullptr};
  size_t frame_buffer_size_{0};
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




