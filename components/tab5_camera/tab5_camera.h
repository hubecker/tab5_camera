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

#endif  // ESP32-P4 check

namespace esphome {
namespace tab5_camera {

// ==== 📸 Définitions d'adresses SCCB et IDs capteurs ====
#define SC2356_SCCB_ADDR    0x43
#define SC2356_CHIP_ID      0x2356
#define SC2336_SCCB_ADDR    0x30
#define SC2336_CHIP_ID      0x2336
#define OV5645_SCCB_ADDR    0x78
#define OV5645_CHIP_ID      0x5645

// ==== ⚙️ Constantes techniques ====
#define TAB5_RGB565_BITS_PER_PIXEL           16
#define TAB5_MIPI_IDI_CLOCK_RATE             (50000000)
#define TAB5_MIPI_CSI_LANE_BITRATE_MBPS      200
#define TAB5_MIPI_CSI_CAM_SCCB_SCL_IO        (8)
#define TAB5_MIPI_CSI_CAM_SCCB_SDA_IO        (7)

enum class PixelFormat {
  RAW8,
  RAW10,
  YUV422,
  RGB565,
  JPEG
};

enum class CameraResolution {
  QVGA_320x240,
  VGA_640x480,
  HD_720p,
  FHD_1080p
};

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

  void set_sensor_address(uint8_t address) { this->sensor_address_ = address; this->set_i2c_address(address); }
  void set_name(const std::string &name) { this->name_ = name; }
  void set_external_clock_pin(uint8_t pin) { this->external_clock_pin_ = pin; }
  void set_external_clock_frequency(uint32_t freq) { this->external_clock_frequency_ = freq; }
  void set_reset_pin(GPIOPin *pin) { this->reset_pin_ = pin; }

  void set_resolution(uint16_t width, uint16_t height) { this->frame_width_ = width; this->frame_height_ = height; }
  void set_pixel_format(const std::string &format) { this->pixel_format_ = format; }
  void set_jpeg_quality(uint8_t quality) {
    this->jpeg_quality_ = std::max(static_cast<uint8_t>(1), std::min(quality, static_cast<uint8_t>(63)));
  }
  void set_framerate(uint8_t framerate) {
    this->framerate_ = std::max(static_cast<uint8_t>(1), std::min(framerate, static_cast<uint8_t>(60)));
  }

  const std::string &get_name() const { return this->name_; }
  uint16_t get_frame_width() const { return this->frame_width_; }
  uint16_t get_frame_height() const { return this->frame_height_; }
  const std::string &get_pixel_format() const { return this->pixel_format_; }
  uint8_t get_jpeg_quality() const { return this->jpeg_quality_; }
  uint8_t get_framerate() const { return this->framerate_; }
  uint8_t get_sensor_address() const { return this->sensor_address_; }

  bool take_snapshot();
  bool start_streaming();
  bool stop_streaming();

#ifdef HAS_ESP32_P4_CAMERA
  bool is_streaming() const { return this->streaming_active_; }
  uint8_t* get_frame_buffer() const { return static_cast<uint8_t*>(this->frame_buffer_); }
  size_t get_frame_buffer_size() const { return this->frame_buffer_size_; }

  bool init_camera_controller();
  bool init_isp_processor();
  void cleanup_resources();
#else
  bool is_streaming() const { return false; }
  uint8_t* get_frame_buffer() const { return nullptr; }
  size_t get_frame_buffer_size() const { return 0; }

  bool init_camera_controller() { return false; }
  bool init_isp_processor() { return false; }
  void cleanup_resources() {}
#endif

  void add_on_frame_callback(std::function<void(uint8_t*, size_t)> &&callback) {
    this->on_frame_callbacks_.add(std::move(callback));
  }

  bool has_error() const { return this->error_state_; }
  const std::string &get_last_error() const { return this->last_error_; }

 protected:
#ifdef HAS_ESP32_P4_CAMERA
  struct FrameData {
    void* buffer;
    size_t size;
    uint32_t timestamp;
    bool valid;
  };

  bool init_camera_();
  bool init_sensor_();
  bool init_ldo_();
  void deinit_camera_();

  bool setup_external_clock_();
  void debug_camera_status();

  // === Config capteurs ===
  bool configure_sc2356_();
  bool configure_ov5645_();        // 👈 AJOUTÉ
  bool configure_sc2336_();        // 👈 AJOUTÉ

  bool reset_sensor_();

  // === Communication I2C ===
  bool read_byte_16(uint16_t reg, uint8_t *data);
  bool write_byte_16(uint16_t reg, uint8_t data);
  bool read_sensor_register_(uint16_t reg, uint8_t *value);
  bool write_sensor_register_(uint16_t reg, uint8_t value);

  // === Détection capteurs ===
  uint16_t detect_sc2356_sensor_();
  uint16_t detect_sensor_id_();              // 👈 AJOUTÉ
  bool test_sc2356_communication_();
  bool test_sensor_communication_(uint8_t);  // 👈 AJOUTÉ

  static bool IRAM_ATTR camera_get_finished_trans_callback(esp_cam_ctlr_handle_t handle, esp_cam_ctlr_trans_t *trans, void *user_data);
  static void streaming_task(void *parameter);
  void streaming_loop_();

  esp_cam_ctlr_handle_t cam_handle_{nullptr};
  isp_proc_handle_t isp_proc_{nullptr};
  esp_ldo_channel_handle_t ldo_mipi_phy_{nullptr};
  void *frame_buffer_{nullptr};
  size_t frame_buffer_size_{0};

  bool camera_initialized_{false};
  bool sensor_initialized_{false};
  bool ldo_initialized_{false};

  uint16_t detected_sensor_id_{0};
  uint8_t detected_sensor_address_{0};

  TaskHandle_t streaming_task_handle_{nullptr};
  SemaphoreHandle_t frame_ready_semaphore_{nullptr};
  QueueHandle_t frame_queue_{nullptr};
  bool streaming_active_{false};
  bool streaming_should_stop_{false};

  static constexpr size_t FRAME_QUEUE_SIZE = 3;
  static constexpr size_t FRAME_BUFFER_COUNT = 1;
  static constexpr uint32_t STREAMING_TASK_STACK_SIZE = 4096;
  static constexpr UBaseType_t STREAMING_TASK_PRIORITY = 5;
#endif

 private:
  std::string name_{"Tab5 Camera SC2356"};
  uint8_t external_clock_pin_{0};
  uint32_t external_clock_frequency_{24000000};
  uint8_t sensor_address_{SC2356_SCCB_ADDR};
  GPIOPin *reset_pin_{nullptr};

  uint16_t frame_width_{640};
  uint16_t frame_height_{480};
  std::string pixel_format_{"RGB565"};
  uint8_t jpeg_quality_{10};
  uint8_t framerate_{15};

  bool error_state_{false};
  std::string last_error_{""};

  CallbackManager<void(uint8_t*, size_t)> on_frame_callbacks_;

  void set_error_(const std::string &error);
  void clear_error_();
  PixelFormat parse_pixel_format_(const std::string &format);
  size_t calculate_frame_size_() const;
};

}  // namespace tab5_camera
}  // namespace esphome

#endif  // USE_ESP32

























