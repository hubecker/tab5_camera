#pragma once

#include "esphome/core/component.h"
#include "esphome/core/gpio.h"
#include "esphome/core/preferences.h"
#include "esphome/components/i2c/i2c.h"
#include "sdkconfig.h"

#ifdef USE_ESP32

// Vérification spécifique pour ESP32-P4
#if defined(CONFIG_IDF_TARGET_ESP32P4)
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

// Constantes basées sur l'exemple IDF
#define TAB5_RGB565_BITS_PER_PIXEL           16
#define TAB5_MIPI_IDI_CLOCK_RATE             (50000000)
#define TAB5_MIPI_CSI_LANE_BITRATE_MBPS      200
#define TAB5_MIPI_CSI_CAM_SCCB_SCL_IO        (8)
#define TAB5_MIPI_CSI_CAM_SCCB_SDA_IO        (7)

// Énumérations pour les formats et résolutions
enum class PixelFormat {
  RAW8,
  RAW10,
  YUV422,
  RGB565,
  JPEG
};

enum class CameraResolution {
  RES_800x640,
  RES_800x800,
  RES_800x1280,
  RES_1024x600,
  CUSTOM
};

// Structure pour les configurations de résolution
struct ResolutionConfig {
  uint16_t width;
  uint16_t height;
  uint8_t fps;
  const char* format_string;
};

class Tab5Camera : public Component, public i2c::I2CDevice {
 public:
  Tab5Camera() = default;
  ~Tab5Camera();

  void setup() override;
  void dump_config() override;
  float get_setup_priority() const override;

  // Configuration I2C
  void set_sensor_address(uint8_t address) { 
    this->sensor_address_ = address; 
    this->set_i2c_address(address); 
  }
  void set_sccb_scl_pin(uint8_t pin) { this->sccb_scl_pin_ = pin; }
  void set_sccb_sda_pin(uint8_t pin) { this->sccb_sda_pin_ = pin; }

  // Configuration générale
  void set_name(const std::string &name) { this->name_ = name; }
  void set_external_clock_pin(uint8_t pin) { this->external_clock_pin_ = pin; }
  void set_external_clock_frequency(uint32_t freq) { this->external_clock_frequency_ = freq; }
  void set_reset_pin(GPIOPin *pin) { this->reset_pin_ = pin; }
  void set_power_down_pin(GPIOPin *pin) { this->power_down_pin_ = pin; }

  // Configuration MIPI CSI
  void set_mipi_data_lanes(uint8_t lanes) { this->mipi_data_lanes_ = lanes; }
  void set_mipi_lane_bitrate(uint32_t bitrate) { this->mipi_lane_bitrate_ = bitrate; }
  void set_mipi_idi_clock_rate(uint32_t rate) { this->mipi_idi_clock_rate_ = rate; }

  // Paramètres de caméra
  void set_resolution(CameraResolution res) { this->resolution_ = res; }
  void set_custom_resolution(uint16_t width, uint16_t height) { 
    this->custom_width_ = width; 
    this->custom_height_ = height;
    this->resolution_ = CameraResolution::CUSTOM;
  }
  void set_pixel_format(PixelFormat format) { this->pixel_format_ = format; }
  void set_jpeg_quality(uint8_t quality) { 
    this->jpeg_quality_ = std::max(static_cast<uint8_t>(1), std::min(quality, static_cast<uint8_t>(63))); 
  }
  void set_framerate(uint8_t framerate) { 
    this->framerate_ = std::max(static_cast<uint8_t>(1), std::min(framerate, static_cast<uint8_t>(60))); 
  }

  // Getters
  const std::string &get_name() const { return this->name_; }
  uint16_t get_frame_width() const;
  uint16_t get_frame_height() const;
  PixelFormat get_pixel_format() const { return this->pixel_format_; }
  uint8_t get_jpeg_quality() const { return this->jpeg_quality_; }
  uint8_t get_framerate() const { return this->framerate_; }
  uint8_t get_sensor_address() const { return this->sensor_address_; }

  // Fonctions de capture
  bool take_snapshot();
  
  // Fonctions de streaming
  bool start_streaming();
  bool stop_streaming();

#ifdef HAS_ESP32_P4_CAMERA
  bool is_streaming() const { return this->streaming_active_; }
  uint8_t* get_frame_buffer() const { return static_cast<uint8_t*>(this->current_frame_buffer_); }
  size_t get_frame_buffer_size() const { return this->frame_buffer_size_; }
  
  // Fonctions spécifiques ESP32-P4
  bool init_camera_controller();
  bool init_isp_processor();
  bool init_ldo_regulator();
  void cleanup_resources();
#else
  bool is_streaming() const { return false; }
  uint8_t* get_frame_buffer() const { return nullptr; }
  size_t get_frame_buffer_size() const { return 0; }
  
  // Stubs pour compatibilité
  bool init_camera_controller() { return false; }
  bool init_isp_processor() { return false; }
  bool init_ldo_regulator() { return false; }
  void cleanup_resources() {}
#endif

  // Callbacks pour le streaming
  void add_on_frame_callback(std::function<void(uint8_t*, size_t)> &&callback) {
    this->on_frame_callbacks_.add(std::move(callback));
  }

  // Gestion des erreurs
  bool has_error() const { return this->error_state_; }
  const std::string &get_last_error() const { return this->last_error_; }

 protected:
#ifdef HAS_ESP32_P4_CAMERA
  // Structure pour les transactions de frame
  struct FrameTransaction {
    esp_cam_ctlr_trans_t trans;
    void* buffer;
    size_t size;
    bool in_use;
    uint32_t timestamp;
  };

  // Fonctions principales d'initialisation
  bool init_camera_();
  bool init_sensor_();
  void deinit_camera_();

  // Configuration hardware
  bool setup_external_clock_();
  bool setup_mipi_csi_();
  
  // Configuration du capteur
  bool configure_sensor_();
  bool reset_sensor_();
  bool power_on_sensor_();
  bool power_off_sensor_();
  
  // Communication I2C/SCCB avec le capteur
  bool read_sensor_register_(uint16_t reg, uint8_t *value);
  bool write_sensor_register_(uint16_t reg, uint8_t value);
  bool read_sensor_register_16_(uint16_t reg, uint16_t *value);
  bool write_sensor_register_16_(uint16_t reg, uint16_t value);
  
  // Gestion des buffers
  bool allocate_frame_buffers_();
  void deallocate_frame_buffers_();
  FrameTransaction* get_free_transaction_();
  void return_transaction_(FrameTransaction* trans);
  
  // Callbacks statiques pour le contrôleur de caméra
  static bool IRAM_ATTR on_trans_finished_callback_(esp_cam_ctlr_handle_t handle, 
                                                   esp_cam_ctlr_trans_t *trans, 
                                                   void *user_data);
  
  // Tâche de streaming
  static void streaming_task_(void *parameter);
  void streaming_loop_();
  
  // Utilitaires
  ResolutionConfig get_resolution_config_() const;
  size_t calculate_frame_size_() const;
  esp_cam_ctlr_color_t get_esp_color_format_() const;
  
  // Debug et diagnostic
  void debug_camera_status_();
  void log_camera_info_();
  
  // Handles ESP32-P4 spécifiques
  esp_cam_ctlr_handle_t cam_handle_{nullptr};
  isp_proc_handle_t isp_proc_{nullptr};
  esp_ldo_channel_handle_t ldo_mipi_phy_{nullptr};
  esp_ldo_channel_handle_t ldo_cam_{nullptr};
  
  // Buffers et transactions
  FrameTransaction* frame_transactions_{nullptr};
  void* current_frame_buffer_{nullptr};
  size_t frame_buffer_size_{0};
  uint8_t frame_buffer_count_{2};
  
  // États d'initialisation
  bool camera_initialized_{false};
  bool sensor_initialized_{false};
  bool ldo_initialized_{false};
  bool isp_initialized_{false};
  
  // Variables de streaming
  TaskHandle_t streaming_task_handle_{nullptr};
  SemaphoreHandle_t frame_ready_semaphore_{nullptr};
  QueueHandle_t frame_queue_{nullptr};
  bool streaming_active_{false};
  bool streaming_should_stop_{false};
  
  // Configuration des buffers et tâches
  static constexpr size_t FRAME_QUEUE_SIZE = 4;
  static constexpr size_t MIN_FRAME_BUFFER_COUNT = 2;
  static constexpr size_t MAX_FRAME_BUFFER_COUNT = 4;
  static constexpr uint32_t STREAMING_TASK_STACK_SIZE = 8192;
  static constexpr UBaseType_t STREAMING_TASK_PRIORITY = 5;
  static constexpr uint32_t FRAME_TIMEOUT_MS = 1000;
#endif

 private:
  // Configuration générale
  std::string name_{"Tab5 Camera"};
  uint8_t external_clock_pin_{0};
  uint32_t external_clock_frequency_{24000000};  // 24MHz par défaut
  GPIOPin *reset_pin_{nullptr};
  GPIOPin *power_down_pin_{nullptr};

  // Configuration I2C/SCCB
  uint8_t sensor_address_{0x43};  // Adresse I2C par défaut du capteur SC2356
  uint8_t sccb_scl_pin_{TAB5_MIPI_CSI_CAM_SCCB_SCL_IO};
  uint8_t sccb_sda_pin_{TAB5_MIPI_CSI_CAM_SCCB_SDA_IO};

  // Configuration MIPI CSI
  uint8_t mipi_data_lanes_{2};
  uint32_t mipi_lane_bitrate_{TAB5_MIPI_CSI_LANE_BITRATE_MBPS * 1000000};
  uint32_t mipi_idi_clock_rate_{TAB5_MIPI_IDI_CLOCK_RATE};

  // Paramètres de caméra
  CameraResolution resolution_{CameraResolution::RES_800x640};
  uint16_t custom_width_{800};
  uint16_t custom_height_{640};
  PixelFormat pixel_format_{PixelFormat::RAW8};
  uint8_t jpeg_quality_{10};
  uint8_t framerate_{30};

  // Gestion des erreurs
  bool error_state_{false};
  std::string last_error_{""};
  
  // Callbacks
  CallbackManager<void(uint8_t*, size_t)> on_frame_callbacks_;
  
  // Méthodes utilitaires privées
  void set_error_(const std::string &error);
  void clear_error_();
  
  // Table de configuration des résolutions
  static const ResolutionConfig resolution_configs_[];
};

}  // namespace tab5_camera
}  // namespace esphome

#endif  // USE_ESP32







