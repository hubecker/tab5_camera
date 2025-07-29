#pragma once

#include "esphome/core/component.h"
#include "esphome/core/gpio.h"
#include "esphome/components/i2c/i2c.h"

namespace esphome {
namespace tab5_camera {

// Configuration spécifique SC2356
static constexpr uint8_t SC2356_I2C_ADDRESS = 0x43;

// Résolutions supportées par SC2356 (noms correspondant au YAML)
enum class SC2356Resolution {
  RES_QVGA_320x240,   // "QVGA_320x240"
  RES_VGA_640x480,    // "VGA_640x480"
  RES_SVGA_800x600,   // "SVGA_800x600"
  RES_HD_1280x720,    // "HD_1280x720"
  RES_UXGA_1600x1200, // "UXGA_1600x1200"
  RES_FHD_1920x1080   // "FHD_1920x1080"
};

// Formats de pixels supportés
enum class SC2356PixelFormat {
  FORMAT_RAW10,   // "RAW10"
  FORMAT_YUV422,  // "YUV422"
  FORMAT_RGB565,  // "RGB565"
  FORMAT_JPEG     // "JPEG"
};

class Tab5Camera : public Component, public i2c::I2CDevice {
 public:
  void setup() override;
  void dump_config() override;

  // Configuration de base
  void set_name(const std::string &name) { name_ = name; }
  void set_external_clock_pin(uint8_t pin) { external_clock_pin_ = pin; }
  void set_external_clock_frequency(uint32_t freq) { external_clock_frequency_ = freq; }
  void set_reset_pin(GPIOPin *pin) { reset_pin_ = pin; }

  // Configuration via strings (pour YAML)
  void set_resolution(const std::string &resolution) {
    if (resolution == "QVGA_320x240") {
      resolution_ = SC2356Resolution::RES_QVGA_320x240;
    } else if (resolution == "VGA_640x480") {
      resolution_ = SC2356Resolution::RES_VGA_640x480;
    } else if (resolution == "SVGA_800x600") {
      resolution_ = SC2356Resolution::RES_SVGA_800x600;
    } else if (resolution == "HD_1280x720") {
      resolution_ = SC2356Resolution::RES_HD_1280x720;
    } else if (resolution == "UXGA_1600x1200") {
      resolution_ = SC2356Resolution::RES_UXGA_1600x1200;
    } else if (resolution == "FHD_1920x1080") {
      resolution_ = SC2356Resolution::RES_FHD_1920x1080;
    } else {
      ESP_LOGW("tab5_camera", "Résolution inconnue '%s', utilisation de VGA_640x480 par défaut", resolution.c_str());
      resolution_ = SC2356Resolution::RES_VGA_640x480;
    }
  }

  void set_pixel_format(const std::string &format) {
    if (format == "RGB565") {
      pixel_format_ = SC2356PixelFormat::FORMAT_RGB565;
    } else if (format == "YUV422") {
      pixel_format_ = SC2356PixelFormat::FORMAT_YUV422;
    } else if (format == "RAW10") {
      pixel_format_ = SC2356PixelFormat::FORMAT_RAW10;
    } else if (format == "JPEG") {
      pixel_format_ = SC2356PixelFormat::FORMAT_JPEG;
    } else {
      ESP_LOGW("tab5_camera", "Format inconnu '%s', utilisation de RGB565 par défaut", format.c_str());
      pixel_format_ = SC2356PixelFormat::FORMAT_RGB565;
    }
  }

  // Paramètres optionnels
  void set_jpeg_quality(uint8_t quality) { jpeg_quality_ = std::clamp(quality, static_cast<uint8_t>(1), static_cast<uint8_t>(63)); }
  void set_framerate(uint8_t framerate) { framerate_ = std::clamp(framerate, static_cast<uint8_t>(5), static_cast<uint8_t>(30))); }
  void set_exposure_time(uint32_t time) { exposure_time_ = time; }
  void set_analog_gain(uint16_t gain) { analog_gain_ = gain; }
  void set_digital_gain(uint16_t gain) { digital_gain_ = gain; }
  void set_test_pattern(bool pattern) { test_pattern_ = pattern; }

 protected:
  // Conversion résolution en dimensions
  std::pair<uint16_t, uint16_t> get_resolution_dimensions() const {
    switch (resolution_) {
      case SC2356Resolution::RES_QVGA_320x240: return {320, 240};
      case SC2356Resolution::RES_VGA_640x480: return {640, 480};
      case SC2356Resolution::RES_SVGA_800x600: return {800, 600};
      case SC2356Resolution::RES_HD_1280x720: return {1280, 720};
      case SC2356Resolution::RES_UXGA_1600x1200: return {1600, 1200};
      case SC2356Resolution::RES_FHD_1920x1080: return {1920, 1080};
      default: return {640, 480};
    }
  }

 private:
  // Configuration de base
  std::string name_{"Tab5 Camera"};
  uint8_t external_clock_pin_{36};
  uint32_t external_clock_frequency_{24000000};
  GPIOPin *reset_pin_{nullptr};

  // Paramètres caméra
  SC2356Resolution resolution_{SC2356Resolution::RES_VGA_640x480};
  SC2356PixelFormat pixel_format_{SC2356PixelFormat::FORMAT_RGB565};
  uint8_t jpeg_quality_{10};
  uint8_t framerate_{15};

  // Paramètres avancés
  uint32_t exposure_time_{10000};
  uint16_t analog_gain_{128};
  uint16_t digital_gain_{128};
  bool test_pattern_{false};
};

}  // namespace tab5_camera
}  // namespace esphome









