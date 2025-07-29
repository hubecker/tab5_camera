#pragma once

#include "esphome/core/component.h"
#include "esphome/core/gpio.h"
#include "esphome/core/log.h"
#include "esphome/components/i2c/i2c.h"

namespace esphome {
namespace tab5_camera {

static constexpr uint8_t SC2356_I2C_ADDRESS = 0x43;

// Résolutions supportées
enum class SC2356Resolution {
  QVGA_320x240,
  VGA_640x480,
  SVGA_800x600,
  HD_1280x720,
  UXGA_1600x1200,
  FHD_1920x1080
};

// Formats de pixels
enum class SC2356PixelFormat {
  RAW10,
  YUV422,
  RGB565,
  JPEG
};

class Tab5Camera : public Component, public i2c::I2CDevice {
 public:
  void setup() override;
  void dump_config() override;
  float get_setup_priority() const override { return setup_priority::DATA; }

  void set_name(const std::string &name) { name_ = name; }
  void set_external_clock_pin(uint8_t pin) { external_clock_pin_ = pin; }
  void set_external_clock_frequency(uint32_t freq) { external_clock_frequency_ = freq; }
  void set_reset_pin(GPIOPin *pin) { reset_pin_ = pin; }

  void set_resolution(const std::string &resolution);
  void set_pixel_format(const std::string &format);
  void set_jpeg_quality(uint8_t quality) { jpeg_quality_ = quality; }
  void set_framerate(uint8_t framerate) { framerate_ = framerate; }
  void set_exposure_time(uint32_t time) { exposure_time_ = time; }
  void set_analog_gain(uint16_t gain) { analog_gain_ = gain; }
  void set_digital_gain(uint16_t gain) { digital_gain_ = gain; }
  void set_test_pattern(bool pattern) { test_pattern_ = pattern; }

  bool start_streaming();
  bool stop_streaming();
  bool is_streaming() const { return streaming_active_; }
  bool detect_sc2356();

 protected:
  void set_error(const std::string &error) { 
    error_state_ = true;
    last_error_ = error;
    ESP_LOGE(TAG, "Error: %s", error.c_str());
  }
  
  void clear_error() { 
    error_state_ = false; 
    last_error_.clear();
  }

 private:
  static const char *TAG;
  
  std::string name_;
  uint8_t external_clock_pin_{36};
  uint32_t external_clock_frequency_{24000000};
  GPIOPin *reset_pin_{nullptr};
  
  SC2356Resolution resolution_{SC2356Resolution::VGA_640x480};
  SC2356PixelFormat pixel_format_{SC2356PixelFormat::RGB565};
  uint8_t jpeg_quality_{10};
  uint8_t framerate_{15};
  uint32_t exposure_time_{10000};
  uint16_t analog_gain_{128};
  uint16_t digital_gain_{128};
  bool test_pattern_{false};
  
  bool error_state_{false};
  std::string last_error_;
  bool streaming_active_{false};
};

}  // namespace tab5_camera
}  // namespace esphome









