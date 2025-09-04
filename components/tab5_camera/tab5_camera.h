#pragma once

/* -------------------------------------------------------------
 *  Includes généraux ESPHome
 * ------------------------------------------------------------- */
#include "esphome/core/component.h"
#include "esphome/core/gpio.h"
#include "esphome/core/preferences.h"
#include "esphome/core/automation.h"
#include "esphome/components/i2c/i2c.h"

/* -------------------------------------------------------------
 *  Détection du chipset ESP32‑P4
 * ------------------------------------------------------------- */
#ifdef USE_ESP32

#if defined(CONFIG_IDF_TARGET_ESP32P4) || \
    (defined(CONFIG_IDF_TARGET) && defined(CONFIG_IDF_TARGET_ESP32P4))
#define HAS_ESP32_P4_CAMERA 1
#endif

/* -------------------------------------------------------------
 *  Headers spécifiques ESP32‑P4
 * ------------------------------------------------------------- */
#if defined(HAS_ESP32_P4_CAMERA)
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
#endif

/* -------------------------------------------------------------
 *  Vérification de la version IDF (>= 5.1)
 * ------------------------------------------------------------- */
#if ESP_IDF_VERSION >= ESP_IDF_VERSION_VAL(5, 1, 0)
#define ESP_P4_CAMERA_SUPPORTED 1
#else
#warning "ESP32-P4 camera support requires IDF 5.1 or later"
#undef HAS_ESP32_P4_CAMERA
#endif

/* -------------------------------------------------------------
 *  Fin de la section conditionnelle ESP32
 * ------------------------------------------------------------- */
// NOTE : **NE PAS** fermer le #ifdef ici ! Le reste du fichier (namespace,
 // classes, etc.) doit rester à l’intérieur du bloc USE_ESP32.
// Le #endif correspondant sera placé à la toute fin du fichier.

namespace esphome {
namespace tab5_camera {

/* -------------------------------------------------------------
 *  Forward declaration
 * ------------------------------------------------------------- */
class Tab5Camera;

/* -------------------------------------------------------------
 *  Trigger utilisé par les automatisations ESPHome (on_frame:)
 * ------------------------------------------------------------- */
class OnFrameTrigger : public Trigger<uint8_t *, size_t> {
 public:
  explicit OnFrameTrigger(Tab5Camera *parent) : parent_(parent) {}

 protected:
  Tab5Camera *parent_;
};

/* -------------------------------------------------------------
 *  Enumérations de configuration
 * ------------------------------------------------------------- */
enum class PixelFormat { RAW8, RAW10, YUV422, RGB565, JPEG };
enum class CameraResolution { QVGA_320x240, VGA_640x480, HD_720p, FHD_1080p };

/* -------------------------------------------------------------
 *  Classe principale du driver
 * ------------------------------------------------------------- */
class Tab5Camera : public Component, public i2c::I2CDevice {
 public:
  Tab5Camera() = default;
  ~Tab5Camera();

  /* ----- Cycle de vie ESPHome ----- */
  void setup() override;
  void dump_config() override;
  float get_setup_priority() const override;

  /* ----- Configuration I2C (héritée) ----- */
  void set_sensor_address(uint8_t address) {
    this->sensor_address_ = address;
    this->set_i2c_address(address);
  }

  /* ----- Configuration générale ----- */
  void set_name(const std::string &name) { this->name_ = name; }
  void set_external_clock_pin(uint8_t pin) { this->external_clock_pin_ = pin; }
  void set_external_clock_frequency(uint32_t freq) { this->external_clock_frequency_ = freq; }
  void set_reset_pin(GPIOPin *pin) { this->reset_pin_ = pin; }

  /* ----- Paramètres de la caméra ----- */
  void set_resolution(uint16_t width, uint16_t height) {
    this->frame_width_  = width;
    this->frame_height_ = height;
  }
  void set_pixel_format(const std::string &format) { this->pixel_format_ = format; }
  void set_jpeg_quality(uint8_t quality) {
    this->jpeg_quality_ = std::max(static_cast<uint8_t>(1),
                                  std::min(quality, static_cast<uint8_t>(63)));
  }
  void set_framerate(uint8_t framerate) {
    this->framerate_ = std::max(static_cast<uint8_t>(1),
                               std::min(framerate, static_cast<uint8_t>(60)));
  }

  /* ----- Getters ----- */
  const std::string &get_name() const { return this->name_; }
  uint16_t get_frame_width() const { return this->frame_width_; }
  uint16_t get_frame_height() const { return this->frame_height_; }
  const std::string &get_pixel_format() const { return this->pixel_format_; }
  uint8_t get_jpeg_quality() const { return this->jpeg_quality_; }
  uint8_t get_framerate() const { return this->framerate_; }
  uint8_t get_sensor_address() const { return this->sensor_address_; }

  /* ----- Capture / Streaming ----- */
  bool take_snapshot();
  bool start_streaming();
  bool stop_streaming();

  /* ----- Diagnostics ----- */
  bool is_ready() const;
  bool has_error() const { return this->error_state_; }
  const std::string &get_last_error() const { return this->last_error_; }

  /* ----- Triggers ESPHome ----- */
  void add_on_frame_trigger(OnFrameTrigger *trigger) {
    this->on_frame_triggers_.push_back(trigger);
  }

  /* ----- ESP32‑P4 specific API ----- */
#ifdef HAS_ESP32_P4_CAMERA
  bool is_streaming() const { return this->streaming_active_; }
  uint8_t *get_frame_buffer() const {
    return static_cast<uint8_t *>(this->frame_buffer_);
  }
  size_t get_frame_buffer_size() const { return this->frame_buffer_size_; }

  bool init_camera_controller();
  bool init_isp_processor();
  void cleanup_resources();
#else
  bool is_streaming() const { return false; }
  uint8_t *get_frame_buffer() const { return nullptr; }
  size_t get_frame_buffer_size() const { return 0; }

  /* Stubs pour compatibilité */
  bool init_camera_controller() { return false; }
  bool init_isp_processor() { return false; }
  void cleanup_resources() {}
#endif

  /* ----- Callback registration ----- */
  void add_on_frame_callback(
      std::function<void(uint8_t *, size_t)> &&callback) {
    this->on_frame_callbacks_.add(std::move(callback));
  }

 protected:
  /* ---------------------------------------------------------
   *  ESP32‑P4 specific structures & helpers
   * --------------------------------------------------------- */
#ifdef HAS_ESP32_P4_CAMERA
  struct FrameData {
    void *buffer;
    size_t size;
    uint32_t timestamp;
    bool valid;
  };

  /* Core init / deinit */
  bool init_camera_();
  bool init_sensor_();
  bool init_ldo_();
  void deinit_camera_();

  /* Reset & clock */
  bool reset_sensor_();
  bool setup_external_clock_();   // LEDC clock config

  /* SC2356 sensor configuration */
  bool configure_sc2356_();
  bool configure_sc2356_8bit_();
  bool configure_sc2356_16bit_();
  bool configure_sc2356_generic_();
  bool configure_sc2356_mipi_output_();

  /* Debug helpers */
  void debug_camera_status();

  /* I2C register access */
  bool read_sensor_register_(uint16_t reg, uint8_t *value);
  bool write_sensor_register_(uint16_t reg, uint8_t value);

  /* Camera controller callbacks (static) */
  static bool IRAM_ATTR camera_get_new_vb_callback(
      esp_cam_ctlr_handle_t handle, esp_cam_ctlr_trans_t *trans,
      void *user_data);
  static bool IRAM_ATTR camera_get_finished_trans_callback(
      esp_cam_ctlr_handle_t handle, esp_cam_ctlr_trans_t *trans,
      void *user_data);

  /* Streaming task */
  static void streaming_task(void *parameter);
  void streaming_loop_();

  /* Frame processing */
  void process_frame_(uint8_t *data, size_t len);
  void trigger_on_frame_callbacks_(uint8_t *data, size_t len);

  /* ESP‑IDF handles */
  esp_cam_ctlr_handle_t cam_handle_{nullptr};
  isp_proc_handle_t isp_proc_{nullptr};
  esp_ldo_channel_handle_t ldo_mipi_phy_{nullptr};

  void *frame_buffer_{nullptr};
  size_t frame_buffer_size_{0};

  /* Init flags */
  bool camera_initialized_{false};
  bool sensor_initialized_{false};
  bool ldo_initialized_{false};

  /* Streaming task primitives */
  TaskHandle_t streaming_task_handle_{nullptr};
  SemaphoreHandle_t frame_ready_semaphore_{nullptr};
  QueueHandle_t frame_queue_{nullptr};
  bool streaming_active_{false};
  bool streaming_should_stop_{false};

  /* Buffer / queue constants */
  static constexpr size_t FRAME_QUEUE_SIZE = 3;
  static constexpr size_t FRAME_BUFFER_COUNT = 1;
  static constexpr uint32_t STREAMING_TASK_STACK_SIZE = 4096;
  static constexpr UBaseType_t STREAMING_TASK_PRIORITY = 5;
#endif   // HAS_ESP32_P4_CAMERA

 private:
  /* ---------------------------------------------------------
   *  Configuration générale
   * --------------------------------------------------------- */
  std::string name_{"Tab5 Camera"};
  uint8_t external_clock_pin_{0};
  uint32_t external_clock_frequency_{24000000};   // 24 MHz par défaut
  uint8_t sensor_address_{0x36};                 // adresse I²C du capteur SC2356
  GPIOPin *reset_pin_{nullptr};

  /* ---------------------------------------------------------
   *  Paramètres de la caméra
   * --------------------------------------------------------- */
  uint16_t frame_width_{640};
  uint16_t frame_height_{480};
  std::string pixel_format_{"RGB565"};
  uint8_t jpeg_quality_{10};
  uint8_t framerate_{15};

  /* ---------------------------------------------------------
   *  Registres 16‑bit du capteur (SC2356)
   * --------------------------------------------------------- */
  bool write_register_16(uint16_t reg, uint8_t val);
  bool read_register_16(uint16_t reg, uint8_t *val);

  /* ---------------------------------------------------------
   *  Gestion d’erreurs
   * --------------------------------------------------------- */
  bool error_state_{false};
  std::string last_error_{""};

  /* ---------------------------------------------------------
   *  Callbacks & triggers internes
   * --------------------------------------------------------- */
  CallbackManager<void(uint8_t *, size_t)> on_frame_callbacks_;
  std::vector<OnFrameTrigger *> on_frame_triggers_;

  /* ---------------------------------------------------------
   *  Nouveau membre : trigger exposé à ESPHome
   * --------------------------------------------------------- */
  OnFrameTrigger *frame_trigger_{nullptr};

  /* ---------------------------------------------------------
   *  Méthodes utilitaires privées
   * --------------------------------------------------------- */
  void set_error_(const std::string &error);
  void clear_error_();
  PixelFormat parse_pixel_format_(const std::string &format) const;
  size_t calculate_frame_size_() const;

  /* ---------------------------------------------------------
   *  Statistiques de diagnostic
   * --------------------------------------------------------- */
  uint32_t frame_count_{0};
  uint32_t error_count_{0};
  uint32_t last_frame_timestamp_{0};
};

}  // namespace tab5_camera
}  // namespace esphome

/* -------------------------------------------------------------
 *  Fermeture du bloc conditionnel ESP32
 * ------------------------------------------------------------- */
#endif  // USE_ESP32





