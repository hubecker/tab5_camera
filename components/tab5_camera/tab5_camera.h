#pragma once

#include "esphome/core/component.h"
#include "esphome/core/gpio.h"
#include "esphome/core/preferences.h"
#include "esphome/components/i2c/i2c.h"

#ifdef USE_ESP32

// Vérification spécifique pour ESP32-P4
#if defined(CONFIG_IDF_TARGET_ESP32P4) || (defined(CONFIG_IDF_TARGET) && defined(CONFIG_IDF_TARGET_ESP32P4))
#define HAS_ESP32_P4_CAMERA 1

// Includes ESP32-P4 spécifiques
#include "esp_cam_ctlr_csi.h"
#include "esp_cam_ctlr.h"
#include "driver/isp.h"
#include "esp_cache.h"
#include "esp_heap_caps.h"
#include "esp_ldo_regulator.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
#include "freertos/queue.h"

// Vérification des versions IDF
#if ESP_IDF_VERSION >= ESP_IDF_VERSION_VAL(5, 1, 0)
#define ESP_P4_CAMERA_SUPPORTED 1
#else
#warning "ESP32-P4 camera support requires IDF 5.1 or later"
#undef HAS_ESP32_P4_CAMERA
#endif

#endif // ESP32-P4 check

namespace esphome {
namespace tab5_camera {

// Configuration spécifique SC2356
static constexpr uint8_t SC2356_I2C_ADDRESS = 0x43;
static constexpr uint16_t SC2356_CHIP_ID_REG = 0x3107;
static constexpr uint16_t SC2356_CHIP_ID_VALUE = 0x2356;

// Résolutions supportées par SC2356
enum class SC2356Resolution {
  QVGA_320x240,
  VGA_640x480,
  SVGA_800x600,
  HD_1280x720,
  UXGA_1600x1200,  // Mode natif SC2356
  FHD_1920x1080    // Mode crop/scale
};

// Formats de pixels supportés
enum class SC2356PixelFormat {
  RAW10,           // Format natif Bayer
  YUV422,          // Pour streaming
  RGB565,          // Pour affichage
  JPEG             // Pour stockage
};

class Tab5Camera : public Component, public i2c::I2CDevice {
 public:
  Tab5Camera();
  ~Tab5Camera();

  void setup() override;
  void dump_config() override;
  float get_setup_priority() const override;

  // Configuration I2C - Fixe pour SC2356
  void set_sensor_address(uint8_t address) { 
    if (address == SC2356_I2C_ADDRESS) {
      this->set_i2c_address(address); 
    } else {
      ESP_LOGW("tab5_camera", "SC2356 uses fixed I2C address 0x43, ignoring 0x%02X", address);
    }
  }

  // Configuration caméra
  void set_name(const std::string &name) { this->name_ = name; }
  void set_external_clock_pin(uint8_t pin) { this->external_clock_pin_ = pin; }
  void set_external_clock_frequency(uint32_t freq) { this->external_clock_frequency_ = freq; }
  void set_reset_pin(GPIOPin *pin) { this->reset_pin_ = pin; }

  // Configuration SC2356 spécifique
  void set_resolution(SC2356Resolution resolution) { this->resolution_ = resolution; }
  void set_pixel_format(SC2356PixelFormat format) { this->pixel_format_ = format; }
  void set_jpeg_quality(uint8_t quality) { 
    this->jpeg_quality_ = std::max(static_cast<uint8_t>(1), std::min(quality, static_cast<uint8_t>(63))); 
  }
  void set_framerate(uint8_t framerate) { 
    this->framerate_ = std::max(static_cast<uint8_t>(5), std::min(framerate, static_cast<uint8_t>(30))); 
  }

  // Paramètres avancés SC2356
  void set_exposure_time(uint32_t exposure_us) { this->exposure_time_ = exposure_us; }
  void set_analog_gain(uint16_t gain) { this->analog_gain_ = gain; }
  void set_digital_gain(uint16_t gain) { this->digital_gain_ = gain; }
  void set_test_pattern(bool enable) { this->test_pattern_enabled_ = enable; }

  // Getters
  const std::string &get_name() const { return this->name_; }
  SC2356Resolution get_resolution() const { return this->resolution_; }
  SC2356PixelFormat get_pixel_format() const { return this->pixel_format_; }
  uint8_t get_jpeg_quality() const { return this->jpeg_quality_; }
  uint8_t get_framerate() const { return this->framerate_; }

  // Fonctions principales
  bool take_snapshot();
  bool start_streaming();
  bool stop_streaming();

#ifdef HAS_ESP32_P4_CAMERA
  bool is_streaming() const { return this->streaming_active_; }
  uint8_t* get_frame_buffer() const { return static_cast<uint8_t*>(this->frame_buffer_); }
  size_t get_frame_buffer_size() const { return this->frame_buffer_size_; }
  
  // Fonctions ESP32-P4 spécifiques
  bool init_camera_controller();
  bool init_isp_processor();
  void cleanup_resources();
#else
  // Stubs pour autres ESP32
  bool is_streaming() const { return false; }
  uint8_t* get_frame_buffer() const { return nullptr; }
  size_t get_frame_buffer_size() const { return 0; }
  bool init_camera_controller() { return false; }
  bool init_isp_processor() { return false; }
  void cleanup_resources() {}
#endif

  // Callbacks
  void add_on_frame_callback(std::function<void(uint8_t*, size_t)> &&callback) {
    this->on_frame_callbacks_.add(std::move(callback));
  }

  // Gestion des erreurs
  bool has_error() const { return this->error_state_; }
  const std::string &get_last_error() const { return this->last_error_; }

  // Fonction de détection SC2356
  bool detect_sc2356();

 protected:
#ifdef HAS_ESP32_P4_CAMERA
  // Structure de frame
  struct FrameData {
    void* buffer;
    size_t size;
    uint32_t timestamp;
    bool valid;
  };

  // Initialisation
  bool init_camera_();
  bool init_sc2356_sensor_();
  bool init_ldo_();
  void deinit_camera_();
  bool setup_external_clock_();

  // Configuration SC2356
  bool configure_sc2356_();
  bool reset_sc2356_();
  bool set_sc2356_resolution_();
  bool set_sc2356_format_();
  bool set_sc2356_framerate_();
  
  // Communication I2C SC2356
  bool read_sc2356_register_(uint16_t reg, uint8_t *value);
  bool write_sc2356_register_(uint16_t reg, uint8_t value);
  bool write_sc2356_register_16_(uint16_t reg, uint16_t value);
  
  // Callbacks statiques
  static bool IRAM_ATTR camera_get_new_vb_callback(esp_cam_ctlr_handle_t handle, esp_cam_ctlr_trans_t *trans, void *user_data);
  static bool IRAM_ATTR camera_get_finished_trans_callback(esp_cam_ctlr_handle_t handle, esp_cam_ctlr_trans_t *trans, void *user_data);
  
  // Tâche de streaming
  static void streaming_task(void *parameter);
  void streaming_loop_();
  
  // Handles ESP32-P4
  esp_cam_ctlr_handle_t cam_handle_{nullptr};
  isp_proc_handle_t isp_proc_{nullptr};
  esp_ldo_channel_handle_t ldo_mipi_phy_{nullptr};
  void *frame_buffer_{nullptr};
  size_t frame_buffer_size_{0};
  
  // États
  bool camera_initialized_{false};
  bool sensor_initialized_{false};
  bool ldo_initialized_{false};
  
  // Streaming
  TaskHandle_t streaming_task_handle_{nullptr};
  SemaphoreHandle_t frame_ready_semaphore_{nullptr};
  QueueHandle_t frame_queue_{nullptr};
  bool streaming_active_{false};
  bool streaming_should_stop_{false};
  
  // Constantes
  static constexpr size_t FRAME_QUEUE_SIZE = 3;
  static constexpr size_t FRAME_BUFFER_COUNT = 1;
  static constexpr uint32_t STREAMING_TASK_STACK_SIZE = 8192;  // Augmenté pour SC2356
  static constexpr UBaseType_t STREAMING_TASK_PRIORITY = 5;
#endif

 private:
  // Configuration fixe
  std::string name_{"Tab5 Camera SC2356"};
  uint8_t external_clock_pin_{36};  // Pin par défaut Tab5
  uint32_t external_clock_frequency_{24000000};  // 24MHz pour SC2356
  GPIOPin *reset_pin_{nullptr};

  // Paramètres SC2356
  SC2356Resolution resolution_{SC2356Resolution::VGA_640x480};
  SC2356PixelFormat pixel_format_{SC2356PixelFormat::RGB565};
  uint8_t jpeg_quality_{10};
  uint8_t framerate_{15};

  // Paramètres avancés SC2356
  uint32_t exposure_time_{10000};  // 10ms par défaut
  uint16_t analog_gain_{128};      // Gain x1
  uint16_t digital_gain_{128};     // Gain x1
  bool test_pattern_enabled_{false};

  // Gestion des erreurs
  bool error_state_{false};
  std::string last_error_{""};
  
  // Callbacks
  CallbackManager<void(uint8_t*, size_t)> on_frame_callbacks_;
  
  // Utilitaires
  void set_error_(const std::string &error);
  void clear_error_();
  std::pair<uint16_t, uint16_t> get_resolution_dimensions_(SC2356Resolution resolution) const;
  size_t calculate_frame_size_() const;
};

}  // namespace tab5_camera
}  // namespace esphome

#endif  // USE_ESP32









