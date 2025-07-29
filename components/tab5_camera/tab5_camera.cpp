#include "tab5_camera.h"
#include "esphome/core/log.h"
#include "esphome/core/helpers.h"
#include "esphome/core/hal.h"

namespace esphome {
namespace tab5_camera {

static const char *const TAG = "tab5_camera";

void Tab5Camera::setup() {
  ESP_LOGCONFIG(TAG, "Setting up Tab5 Camera...");
  
  if (!detect_sc2356()) {
    set_error("Failed to detect SC2356 sensor");
    mark_failed();
    return;
  }
  
  ESP_LOGCONFIG(TAG, "Tab5 Camera setup complete");
}

void Tab5Camera::dump_config() {
  ESP_LOGCONFIG(TAG, "Tab5 Camera '%s':", name_.c_str());
  ESP_LOGCONFIG(TAG, "  I2C Address: 0x%02X", SC2356_I2C_ADDRESS);
  
  if (error_state_) {
    ESP_LOGCONFIG(TAG, "  Status: Error - %s", last_error_.c_str());
  } else {
    ESP_LOGCONFIG(TAG, "  Status: Initialized");
  }
}

void Tab5Camera::set_resolution(const std::string &resolution) {
  if (resolution == "QVGA_320x240") {
    resolution_ = SC2356Resolution::QVGA_320x240;
  } else if (resolution == "VGA_640x480") {
    resolution_ = SC2356Resolution::VGA_640x480;
  } else if (resolution == "SVGA_800x600") {
    resolution_ = SC2356Resolution::SVGA_800x600;
  } else if (resolution == "HD_1280x720") {
    resolution_ = SC2356Resolution::HD_1280x720;
  } else if (resolution == "UXGA_1600x1200") {
    resolution_ = SC2356Resolution::UXGA_1600x1200;
  } else if (resolution == "FHD_1920x1080") {
    resolution_ = SC2356Resolution::FHD_1920x1080;
  } else {
    ESP_LOGW(TAG, "Unknown resolution '%s', using VGA_640x480", resolution.c_str());
    resolution_ = SC2356Resolution::VGA_640x480;
  }
}

void Tab5Camera::set_pixel_format(const std::string &format) {
  if (format == "RGB565") {
    pixel_format_ = SC2356PixelFormat::RGB565;
  } else if (format == "YUV422") {
    pixel_format_ = SC2356PixelFormat::YUV422;
  } else if (format == "RAW10") {
    pixel_format_ = SC2356PixelFormat::RAW10;
  } else if (format == "JPEG") {
    pixel_format_ = SC2356PixelFormat::JPEG;
  } else {
    ESP_LOGW(TAG, "Unknown format '%s', using RGB565", format.c_str());
    pixel_format_ = SC2356PixelFormat::RGB565;
  }
}

bool Tab5Camera::detect_sc2356() {
  uint8_t chip_id_h, chip_id_l;
  
  if (!read_byte(SC2356_CHIP_ID_H_REG, &chip_id_h) ||
      !read_byte(SC2356_CHIP_ID_L_REG, &chip_id_l)) {
    ESP_LOGE(TAG, "Failed to read SC2356 chip ID");
    return false;
  }
  
  uint16_t chip_id = (chip_id_h << 8) | chip_id_l;
  if (chip_id != SC2356_CHIP_ID_VALUE) {
    ESP_LOGE(TAG, "Invalid SC2356 chip ID: 0x%04X (expected 0x%04X)", 
             chip_id, SC2356_CHIP_ID_VALUE);
    return false;
  }
  
  ESP_LOGI(TAG, "SC2356 detected successfully");
  return true;
}

bool Tab5Camera::start_streaming() {
  // Implémentation simplifiée pour l'exemple
  ESP_LOGI(TAG, "Starting SC2356 streaming");
  streaming_active_ = true;
  return true;
}

bool Tab5Camera::stop_streaming() {
  ESP_LOGI(TAG, "Stopping SC2356 streaming");
  streaming_active_ = false;
  return true;
}

}  // namespace tab5_camera
}  // namespace esphome










