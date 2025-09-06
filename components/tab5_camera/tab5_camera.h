#pragma once

#include "esphome/core/component.h"
#include "esphome/core/gpio.h"
#include "esphome/core/preferences.h"
#include "esphome/core/automation.h"
#include "esphome/components/i2c/i2c.h"

/* --------------------------------------------------------------
 *  ESP‑IDF – I²C master‑bus (nouvelle API introduite v5.0)
 * -------------------------------------------------------------- */
#include "driver/i2c.h"

/* --------------------------------------------------------------
 *  SCCB helper – fournit sccb_i2c_config_t, sccb_new_i2c_io(),
 *  sccb_handle_t, …  (le fichier peut s’appeler sccb.h ou
 *  esp_sccb.h selon le dépôt que vous avez importé)
 * -------------------------------------------------------------- */
#include "esp_sccb_intf.h"         // <-- changez le nom si votre header porte un autre nom

/* --------------------------------------------------------------
 *  ESP‑32 / ESP‑32‑P4 specific
 * -------------------------------------------------------------- */
#ifdef USE_ESP32

/* Detect ESP32‑P4 target – enables the full camera stack */
#if defined(CONFIG_IDF_TARGET_ESP32P4) || \
    (defined(CONFIG_IDF_TARGET) && defined(CONFIG_IDF_TARGET_ESP32P4))
#define HAS_ESP32_P4_CAMERA 1

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

#if ESP_IDF_VERSION >= ESP_IDF_VERSION_VAL(5, 1, 0)
#define ESP_P4_CAMERA_SUPPORTED 1
#else
#warning "ESP32‑P4 camera support requires IDF 5.1 or later"
#undef HAS_ESP32_P4_CAMERA
#endif

#endif   // ESP32‑P4 target check

namespace esphome {
namespace tab5_camera {

/* ----------------------------------------------------------------
 *  Forward declaration
 * ---------------------------------------------------------------- */
class Tab5Camera;

/* ----------------------------------------------------------------
 *  Trigger used for the “on_frame” automation
 * ---------------------------------------------------------------- */
class OnFrameTrigger : public Trigger<uint8_t *, size_t> {
 public:
  explicit OnFrameTrigger(Tab5Camera *parent) : parent_(parent) {}
 protected:
  Tab5Camera *parent_;
};

/* ----------------------------------------------------------------
 *  Enumerations
 * ---------------------------------------------------------------- */
enum class PixelFormat { RAW8, RAW10, YUV422, RGB565, JPEG };
enum class CameraResolution { QVGA_320x240, VGA_640x480, HD_720p, FHD_1080p };

/* ----------------------------------------------------------------
 *  Main component class
 * ---------------------------------------------------------------- */
class Tab5Camera : public Component,
                  public i2c::I2CDevice {
 public:
  Tab5Camera() = default;
  ~Tab5Camera();

  /* ----- Component lifecycle ----- */
  void setup() override;
  void dump_config() override;
  float get_setup_priority() const override;

  /* ----- Configuration setters ----- */
  void set_sensor_address(uint8_t address) {
    this->sensor_address_ = address;
    this->set_i2c_address(address);
  }
  void set_name(const std::string &name) { this->name_ = name; }
  void set_external_clock_pin(uint8_t pin) { this->external_clock_pin_ = pin; }
  void set_external_clock_frequency(uint32_t freq) { this->external_clock_frequency_ = freq; }
  void set_reset_pin(GPIOPin *pin) { this->reset_pin_ = pin; }

  void set_resolution(uint16_t width, uint16_t height) {
    this->frame_width_  = width;
    this->frame_height_ = height;
  }
  void set_pixel_format(const std::string &format) { this->pixel_format_ = format; }
  void set_jpeg_quality(uint8_t quality) {
    this->jpeg_quality_ = std::max<uint8_t>(1, std::min<uint8_t>(quality, 63));
  }
  void set_framerate(uint8_t framerate) {
    this->framerate_ = std::max<uint8_t>(1, std::min<uint8_t>(framerate, 60));
  }

  /* ----- Public getters ----- */
  const std::string &get_name() const { return this->name_; }
  uint16_t get_frame_width() const { return this->frame_width_; }
  uint16_t get_frame_height() const { return this->frame_height_; }
  const std::string &get_pixel_format() const { return this->pixel_format_; }
  uint8_t get_jpeg_quality() const { return this->jpeg_quality_; }
  uint8_t get_framerate() const { return this->framerate_; }
  uint8_t get_sensor_address() const { return this->sensor_address_; }

  /* ----- Core operations ----- */
  bool take_snapshot();
  bool start_streaming();
  bool stop_streaming();

  bool is_ready() const;
  bool has_error() const { return this->error_state_; }
  const std::string &get_last_error() const { return this->last_error_; }

  void add_on_frame_trigger(OnFrameTrigger *trigger) {
    this->on_frame_triggers_.push_back(trigger);
  }

  /* ----- P4‑specific API ----- */
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
  /* Stubs for non‑P4 builds */
  bool is_streaming() const { return false; }
  uint8_t *get_frame_buffer() const { return nullptr; }
  size_t get_frame_buffer_size() const { return 0; }
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
  /* ----- Internal structures ----- */
#ifdef HAS_ESP32_P4_CAMERA
  struct FrameData {
    void *buffer;
    size_t size;
    uint32_t timestamp;
    bool valid;
  };

  /* ----- Low‑level init / deinit ----- */
  bool init_camera_();
  bool init_sensor_();
  bool init_ldo_();
  void deinit_camera_();

  bool reset_sensor_();
  bool setup_external_clock_();

  /* SCCB / I²C helpers */
  bool init_sccb_();
  bool detect_sensor_with_sccb_();

  /* ----- Sensor configuration ----- */
  bool identify_sensor_();
  bool configure_minimal_sensor_();
  bool test_manual_capture_();
  bool start_continuous_capture_();

  void debug_camera_status_();
  void verify_external_clock_();

  bool read_sensor_register_(uint16_t reg, uint8_t *value);
  bool write_sensor_register_(uint16_t reg, uint8_t value);

  /* ----- Callbacks used by ESP‑CAM driver ----- */
  static bool IRAM_ATTR camera_get_new_vb_callback(
      esp_cam_ctlr_handle_t handle,
      esp_cam_ctlr_trans_t *trans,
      void *user_data);
  static bool IRAM_ATTR camera_get_finished_trans_callback(
      esp_cam_ctlr_handle_t handle,
      esp_cam_ctlr_trans_t *trans,
      void *user_data);

  /* ----- Streaming task ----- */
  static void streaming_task(void *parameter);
  void streaming_loop_();
  void process_frame_(uint8_t *data, size_t len);
  void trigger_on_frame_callbacks_(uint8_t *data, size_t len);

  /* ----- Handles & resources ----- */
  esp_cam_ctlr_handle_t cam_handle_{nullptr};
  isp_proc_handle_t isp_proc_{nullptr};
  esp_ldo_channel_handle_t ldo_mipi_phy_{nullptr};

  void *frame_buffer_{nullptr};
  size_t frame_buffer_size_{0};

  static constexpr size_t NUM_FRAME_BUFFERS = 3;
  void *frame_buffers_[NUM_FRAME_BUFFERS]{nullptr};
  size_t current_buffer_index_{0};

  bool camera_initialized_{false};
  bool sensor_initialized_{false};
  bool ldo_initialized_{false};

  TaskHandle_t streaming_task_handle_{nullptr};
  SemaphoreHandle_t frame_ready_semaphore_{nullptr};
  QueueHandle_t frame_queue_{nullptr};

  bool streaming_active_{false};
  bool streaming_should_stop_{false};
  bool continuous_capture_active_{false};

  static constexpr size_t FRAME_QUEUE_SIZE = 3;
  static constexpr uint32_t STREAMING_TASK_STACK_SIZE = 4096;
  static constexpr UBaseType_t STREAMING_TASK_PRIORITY = 5;

  /* ----- SCCB handle (new) ----- */
  sccb_handle_t sccb_handle_;   // <-- déclaré grâce à sccb.h
#endif   // HAS_ESP32_P4_CAMERA

 private:
  /* ----- User‑visible configuration ----- */
  std::string name_{"Tab5 Camera"};
  uint8_t external_clock_pin_{0};
  uint32_t external_clock_frequency_{24000000};
  uint8_t sensor_address_{0x36};
  GPIOPin *reset_pin_{nullptr};

  uint16_t frame_width_{640};
  uint16_t frame_height_{480};
  std::string pixel_format_{"RGB565"};
  uint8_t jpeg_quality_{10};
  uint8_t framerate_{15};

  /* ----- Helper methods ----- */
  bool write_register_16(uint16_t reg, uint8_t val);
  bool read_register_16(uint16_t reg, uint8_t *val);

  void set_error_(const std::string &error);
  void clear_error_();

  PixelFormat parse_pixel_format_(const std::string &format) const;
  size_t calculate_frame_size_() const;

  /* ----- State tracking ----- */
  bool error_state_{false};
  std::string last_error_{""};

  CallbackManager<void(uint8_t *, size_t)> on_frame_callbacks_;
  std::vector<OnFrameTrigger *> on_frame_triggers_;

  uint32_t frame_count_{0};
  uint32_t error_count_{0};
  uint32_t last_frame_timestamp_{0};
};

}  // namespace tab5_camera
}  // namespace esphome

#endif  // USE_ESP32






