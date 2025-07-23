#include "tab5_camera.h"
#include "esphome/core/log.h"
#include "esphome/core/helpers.h"

#ifdef USE_ESP32

static const char *const TAG = "tab5_camera";

namespace esphome {
namespace tab5_camera {

void Tab5Camera::setup() {
#ifdef HAS_ESP_CAMERA
  ESP_LOGCONFIG(TAG, "Setting up Tab5 Camera...");
  
  // Configuration de la caméra pour MIPI-CSI
  this->camera_config_.pin_pwdn = -1;  // power_down_pin (NO_PIN)
  this->camera_config_.pin_reset = this->reset_pin_ ? this->reset_pin_->get_pin() : -1;
  this->camera_config_.pin_xclk = this->external_clock_pin_;
  
  // Broches MIPI-CSI spécifiques à la Tab5
  this->camera_config_.pin_sscb_sda = -1;  // I2C géré séparément
  this->camera_config_.pin_sscb_scl = -1;  // I2C géré séparément
  
  // Interface MIPI-CSI (pas de broches parallèles)
  this->camera_config_.pin_d7 = -1;
  this->camera_config_.pin_d6 = -1;
  this->camera_config_.pin_d5 = -1;
  this->camera_config_.pin_d4 = -1;
  this->camera_config_.pin_d3 = -1;
  this->camera_config_.pin_d2 = -1;
  this->camera_config_.pin_d1 = -1;
  this->camera_config_.pin_d0 = -1;
  this->camera_config_.pin_vsync = -1;
  this->camera_config_.pin_href = -1;
  this->camera_config_.pin_pclk = -1;
  
  // Configuration MIPI-CSI
  this->camera_config_.xclk_freq_hz = this->external_clock_frequency_;
  this->camera_config_.ledc_timer = LEDC_TIMER_0;
  this->camera_config_.ledc_channel = LEDC_CHANNEL_0;
  
  // Format et résolution
  this->camera_config_.pixel_format = PIXFORMAT_JPEG;
  this->camera_config_.frame_size = FRAMESIZE_VGA;  // 640x480
  this->camera_config_.jpeg_quality = 12;
  this->camera_config_.fb_count = 2;
  this->camera_config_.fb_location = CAMERA_FB_IN_PSRAM;
  this->camera_config_.grab_mode = CAMERA_GRAB_WHEN_EMPTY;
  
  // Configuration spécifique ESP32-P4 MIPI
  this->camera_config_.conv_limit_en = false;
  this->camera_config_.conv_mode = YUV422_TO_YUV420;
  
  if (!this->init_camera_()) {
    this->mark_failed();
    return;
  }
  
  ESP_LOGCONFIG(TAG, "Tab5 Camera '%s' setup completed", this->name_.c_str());
#else
  ESP_LOGE(TAG, "esp_camera.h not available - Tab5 Camera component disabled");
  this->mark_failed();
#endif
}

void Tab5Camera::dump_config() {
  ESP_LOGCONFIG(TAG, "Tab5 Camera '%s':", this->name_.c_str());
#ifdef HAS_ESP_CAMERA
  ESP_LOGCONFIG(TAG, "  External Clock Pin: GPIO%u", this->external_clock_pin_);
  ESP_LOGCONFIG(TAG, "  External Clock Frequency: %u Hz", this->external_clock_frequency_);
  
  if (this->reset_pin_) {
    LOG_PIN("  Reset Pin: ", this->reset_pin_);
  }
#else
  ESP_LOGCONFIG(TAG, "  Status: esp_camera.h not available");
#endif
  
  if (this->is_failed()) {
    ESP_LOGCONFIG(TAG, "  Setup Failed");
  }
}

float Tab5Camera::get_setup_priority() const {
  return setup_priority::HARDWARE - 1.0f;
}

#ifdef HAS_ESP_CAMERA
bool Tab5Camera::init_camera_() {
  if (this->camera_initialized_) {
    return true;
  }
  
  // Reset de la caméra si pin disponible
  if (this->reset_pin_) {
    this->reset_pin_->setup();
    this->reset_pin_->digital_write(false);
    delay(10);
    this->reset_pin_->digital_write(true);
    delay(10);
  }
  
  // Initialisation de la caméra ESP32-P4 avec MIPI-CSI
  esp_err_t err = esp_camera_init(&this->camera_config_);
  if (err != ESP_OK) {
    ESP_LOGE(TAG, "Camera init failed with error 0x%x", err);
    return false;
  }
  
  // Configuration du capteur
  sensor_t *sensor = esp_camera_sensor_get();
  if (sensor) {
    // Réglages spécifiques pour la Tab5
    sensor->set_framesize(sensor, FRAMESIZE_VGA);
    sensor->set_quality(sensor, 12);
    sensor->set_colorbar(sensor, 0);
    sensor->set_whitebal(sensor, 1);
    sensor->set_gain_ctrl(sensor, 1);
    sensor->set_exposure_ctrl(sensor, 1);
    sensor->set_hmirror(sensor, 0);
    sensor->set_vflip(sensor, 0);
    sensor->set_awb_gain(sensor, 1);
    sensor->set_agc_gain(sensor, 0);
    sensor->set_aec_value(sensor, 300);
    sensor->set_aec2(sensor, 0);
    sensor->set_dcw(sensor, 1);
    sensor->set_bpc(sensor, 0);
    sensor->set_wpc(sensor, 1);
    sensor->set_raw_gma(sensor, 1);
    sensor->set_lenc(sensor, 1);
    sensor->set_special_effect(sensor, 0);
    sensor->set_wb_mode(sensor, 0);
    sensor->set_ae_level(sensor, 0);
  }
  
  this->camera_initialized_ = true;
  ESP_LOGD(TAG, "Camera '%s' initialized successfully", this->name_.c_str());
  return true;
}

void Tab5Camera::deinit_camera_() {
  if (this->camera_initialized_) {
    esp_camera_deinit();
    this->camera_initialized_ = false;
    ESP_LOGD(TAG, "Camera '%s' deinitialized", this->name_.c_str());
  }
}
#endif

bool Tab5Camera::take_snapshot() {
#ifdef HAS_ESP_CAMERA
  if (!this->camera_initialized_) {
    ESP_LOGE(TAG, "Camera '%s' not initialized", this->name_.c_str());
    return false;
  }
  
  camera_fb_t *fb = esp_camera_fb_get();
  if (!fb) {
    ESP_LOGW(TAG, "Camera '%s' capture failed", this->name_.c_str());
    return false;
  }
  
  ESP_LOGD(TAG, "Camera '%s' snapshot taken, size: %zu bytes", this->name_.c_str(), fb->len);
  
  // Ici tu peux traiter l'image ou l'envoyer via un service
  // Pour l'instant on libère juste le buffer
  esp_camera_fb_return(fb);
  
  return true;
#else
  ESP_LOGE(TAG, "Camera '%s' not available - esp_camera.h missing", this->name_.c_str());
  return false;
#endif
}

}  // namespace tab5_camera
}  // namespace esphome

#endif  // USE_ESP32


