#include "tab5_camera.h"
#include "esphome/core/log.h"
#include "esphome/core/helpers.h"
#include "esphome/core/hal.h"

#ifdef USE_ESP32
#ifdef HAS_ESP32_P4_CAMERA

#include "esp_timer.h"
#include "driver/ledc.h"
#include "esp_cache.h"

#ifdef CONFIG_CAMERA_SC2336
#include "sc2336.h"
#elif CONFIG_CAMERA_OV5645
#include "ov5645.h"
#endif

static const char *const TAG = "tab5_camera";

// Configuration constants
#define SCCB0_PORT_NUM I2C_NUM_0
#define TAB5_CAMERA_H_RES 640
#define TAB5_CAMERA_V_RES 480
#define TAB5_MIPI_CSI_LANE_BITRATE_MBPS 200
#define TAB5_ISP_CLOCK_HZ 50000000

namespace esphome {
namespace tab5_camera {

// -----------------------
// Destructor
// -----------------------
Tab5Camera::~Tab5Camera() {
  this->deinit_camera_();
}

// -----------------------
// Setup
// -----------------------
void Tab5Camera::setup() {
  ESP_LOGCONFIG(TAG, "Setting up Tab5 Camera...");

  this->frame_ready_semaphore_ = xSemaphoreCreateBinary();
  if (!this->frame_ready_semaphore_) {
    ESP_LOGE(TAG, "Failed to create frame ready semaphore");
    this->mark_failed();
    return;
  }

  this->frame_queue_ = xQueueCreate(FRAME_QUEUE_SIZE, sizeof(FrameData));
  if (!this->frame_queue_) {
    ESP_LOGE(TAG, "Failed to create frame queue");
    this->mark_failed();
    return;
  }

  if (!this->init_i2c_bus_()) {
    ESP_LOGE(TAG, "Failed to initialize I2C bus");
    this->mark_failed();
    return;
  }

  if (!this->init_sccb_()) {
    ESP_LOGE(TAG, "Failed to initialize SCCB");
    this->mark_failed();
    return;
  }

  if (!this->detect_camera_sensor_()) {
    ESP_LOGE(TAG, "Failed to detect camera sensor");
    this->mark_failed();
    return;
  }

  if (!this->init_camera_sensor_()) {
    ESP_LOGE(TAG, "Failed to initialize camera sensor");
    this->mark_failed();
    return;
  }

  ESP_LOGCONFIG(TAG, "Tab5 Camera setup completed successfully");
  this->camera_initialized_ = true;
}

// -----------------------
// Dump configuration
// -----------------------
void Tab5Camera::dump_config() {
  ESP_LOGCONFIG(TAG, "Tab5 Camera:");
  ESP_LOGCONFIG(TAG, "  Platform: ESP32-P4 MIPI-CSI");
  ESP_LOGCONFIG(TAG, "  Resolution: %dx%d", this->frame_width_, this->frame_height_);
  ESP_LOGCONFIG(TAG, "  Pixel Format: %s", this->pixel_format_.c_str());
  ESP_LOGCONFIG(TAG, "  Framerate: %d fps", this->framerate_);
  ESP_LOGCONFIG(TAG, "  SCCB Address: 0x%02X", this->sensor_address_);
  ESP_LOGCONFIG(TAG, "  SCCB SCL: GPIO%u", this->sccb_scl_pin_);
  ESP_LOGCONFIG(TAG, "  SCCB SDA: GPIO%u", this->sccb_sda_pin_);
  ESP_LOGCONFIG(TAG, "  SCCB Frequency: %u Hz", this->sccb_frequency_);
  if (this->external_clock_pin_ > 0) {
    ESP_LOGCONFIG(TAG, "  External Clock Pin: GPIO%u", this->external_clock_pin_);
    ESP_LOGCONFIG(TAG, "  External Clock Frequency: %u Hz", this->external_clock_frequency_);
  }
  if (this->reset_pin_) {
    LOG_PIN("  Reset Pin: ", this->reset_pin_);
  }
  if (this->is_failed()) {
    ESP_LOGCONFIG(TAG, "  Setup Failed");
  }
}

// -----------------------
// Setup priority
// -----------------------
float Tab5Camera::get_setup_priority() const {
  return setup_priority::HARDWARE - 1.0f;
}

// -----------------------
// Is ready
// -----------------------
bool Tab5Camera::is_ready() const {
  return this->camera_initialized_ && this->sensor_initialized_;
}

// -----------------------
// I2C initialization
// -----------------------
bool Tab5Camera::init_i2c_bus_() {
  ESP_LOGI(TAG, "Initializing I2C master bus");

  i2c_master_bus_config_t i2c_bus_config = {
    .i2c_port          = SCCB0_PORT_NUM,
    .clk_source        = I2C_CLK_SRC_DEFAULT,
    .scl_io_num        = this->sccb_scl_pin_,
    .sda_io_num        = this->sccb_sda_pin_,
    .glitch_ignore_cnt = 7,
    .intr_priority     = 0,
    .trans_queue_depth = 0,
    .flags = {
      .enable_internal_pullup = true,
    },
  };

  esp_err_t ret = i2c_new_master_bus(&i2c_bus_config, &this->i2c_bus_handle_);
  if (ret != ESP_OK) {
    ESP_LOGE(TAG, "Failed to create I2C master bus: %s", esp_err_to_name(ret));
    return false;
  }

  ESP_LOGI(TAG, "I2C master bus initialized successfully");
  return true;
}

// -----------------------
// SCCB initialization
// -----------------------
bool Tab5Camera::init_sccb_() {
  ESP_LOGI(TAG, "Initializing SCCB interface");

  sccb_i2c_config_t sccb_config = {
    .dev_addr_length = I2C_ADDR_BIT_LEN_7,
    .device_address  = this->sensor_address_,
    .scl_speed_hz    = this->sccb_frequency_,
  };

  esp_err_t ret = sccb_new_i2c_io(this->i2c_bus_handle_, &sccb_config, &this->sccb_handle_);
  if (ret != ESP_OK) {
    ESP_LOGE(TAG, "Failed to create SCCB interface: %s", esp_err_to_name(ret));
    return false;
  }

  ESP_LOGI(TAG, "SCCB interface initialized successfully");
  return true;
}

// -----------------------
// Camera detection
// -----------------------
bool Tab5Camera::detect_camera_sensor_() {
  ESP_LOGI(TAG, "Detecting camera sensor");

  esp_cam_sensor_config_t cam_config = {
    .sccb_handle = this->sccb_handle_,
    .reset_pin   = (this->reset_pin_) ? this->reset_pin_->pin : -1,
    .pwdn_pin    = -1,
    .xclk_pin    = -1,
  };

#ifdef CONFIG_CAMERA_SC2336
  this->cam_sensor_ = sc2336_detect(&cam_config);
#elif CONFIG_CAMERA_OV5645
  this->cam_sensor_ = ov5645_detect(&cam_config);
#else
  this->cam_sensor_ = sc2336_detect(&cam_config);
#endif

  if (!this->cam_sensor_) {
    ESP_LOGE(TAG, "Failed to detect camera sensor");
    return false;
  }

  this->sensor_initialized_ = true;
  return true;
}

// -----------------------
// Camera sensor initialization
// -----------------------
bool Tab5Camera::init_camera_sensor_() {
  if (!this->cam_sensor_) {
    ESP_LOGE(TAG, "Camera sensor not detected");
    return false;
  }

  if (!this->setup_external_clock_()) return false;
  if (!this->init_ldo_()) return false;
  if (!this->init_csi_controller_()) return false;
  if (!this->init_isp_processor_()) return false;
  if (!this->allocate_frame_buffers_()) return false;

  return true;
}

// -----------------------
// External clock
// -----------------------
bool Tab5Camera::setup_external_clock_() {
  if (this->external_clock_pin_ == 0) return true;

  ledc_timer_config_t timer_conf = {};
  timer_conf.duty_resolution = LEDC_TIMER_1_BIT;
  timer_conf.freq_hz = this->external_clock_frequency_;
  timer_conf.speed_mode = LEDC_LOW_SPEED_MODE;
  timer_conf.deconfigure = false;
  timer_conf.clk_cfg = LEDC_AUTO_CLK;
  timer_conf.timer_num = LEDC_TIMER_0;

  esp_err_t ret = ledc_timer_config(&timer_conf);
  if (ret != ESP_OK) return false;

  ledc_channel_config_t ch_conf = {};
  ch_conf.gpio_num = this->external_clock_pin_;
  ch_conf.speed_mode = LEDC_LOW_SPEED_MODE;
  ch_conf.channel = LEDC_CHANNEL_0;
  ch_conf.intr_type = LEDC_INTR_DISABLE;
  ch_conf.timer_sel = LEDC_TIMER_0;
  ch_conf.duty = 1;
  ch_conf.hpoint = 0;
  ch_conf.sleep_mode = LEDC_SLEEP_MODE_KEEP_ALIVE;

  ret = ledc_channel_config(&ch_conf);
  return ret == ESP_OK;
}

// -----------------------
// LDO initialization
// -----------------------
bool Tab5Camera::init_ldo_() {
  esp_ldo_channel_config_t ldo_cfg = {.chan_id = 3, .voltage_mv = 2500};
  return esp_ldo_acquire_channel(&ldo_cfg, &this->ldo_mipi_phy_) == ESP_OK;
}

// -----------------------
// CSI initialization
// -----------------------
bool Tab5Camera::init_csi_controller_() {
  esp_cam_ctlr_csi_config_t csi_config = {};
  csi_config.ctlr_id = 0;
  csi_config.h_res = this->frame_width_;
  csi_config.v_res = this->frame_height_;
  csi_config.lane_bit_rate_mbps = TAB5_MIPI_CSI_LANE_BITRATE_MBPS;
  csi_config.input_data_color_type = CAM_CTLR_COLOR_RAW8;
  csi_config.output_data_color_type = CAM_CTLR_COLOR_RGB565;
  csi_config.data_lane_num = 2;
  csi_config.byte_swap_en = false;
  csi_config.queue_items = 4;

  return esp_cam_new_csi_ctlr(&csi_config, &this->cam_handle_) == ESP_OK;
}

// -----------------------
// ISP initialization
// -----------------------
bool Tab5Camera::init_isp_processor_() {
  esp_isp_processor_cfg_t isp_config = {};
  isp_config.clk_hz = TAB5_ISP_CLOCK_HZ;
  isp_config.input_data_source = ISP_INPUT_DATA_SOURCE_CSI;
  isp_config.input_data_color_type = ISP_COLOR_RAW8;
  isp_config.output_data_color_type = ISP_COLOR_RGB565;
  isp_config.h_res = this->frame_width_;
  isp_config.v_res = this->frame_height_;

  return esp_isp_new_processor(&isp_config, &this->isp_proc_) == ESP_OK;
}

// -----------------------
// Frame buffer allocation
// -----------------------
bool Tab5Camera::allocate_frame_buffers_() {
  this->frame_buffer_size_ = calculate_frame_size_();
  this->frame_buffer_ = heap_caps_aligned_alloc(64, this->frame_buffer_size_,
                                                MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT);
  if (!this->frame_buffer_) {
    this->frame_buffer_ = heap_caps_aligned_alloc(64, this->frame_buffer_size_,
                                                  MALLOC_CAP_DMA | MALLOC_CAP_8BIT);
    if (!this->frame_buffer_) return false;
  }
  memset(this->frame_buffer_, 0, this->frame_buffer_size_);
  return true;
}

// -----------------------
// Parse pixel format
// -----------------------
PixelFormat Tab5Camera::parse_pixel_format_(const std::string &format) const {
  if (format == "RAW8") return PixelFormat::RAW8;
  if (format == "RAW10") return PixelFormat::RAW10;
  if (format == "YUV422") return PixelFormat::YUV422;
  if (format == "RGB565") return PixelFormat::RGB565;
  if (format == "JPEG") return PixelFormat::JPEG;
  return PixelFormat::RGB565;
}

// -----------------------
// Calculate frame size
// -----------------------
size_t Tab5Camera::calculate_frame_size_() const {
  uint16_t bpp = 2; // default RGB565
  switch (parse_pixel_format_(pixel_format_)) {
    case PixelFormat::RAW8: bpp = 1; break;
    case PixelFormat::RAW10: bpp = 2; break;
    case PixelFormat::YUV422: bpp = 2; break;
    case PixelFormat::RGB565: bpp = 2; break;
    case PixelFormat::JPEG: bpp = 1; break;
  }
  return frame_width_ * frame_height_ * bpp;
}

}  // namespace tab5_camera
}  // namespace esphome

#endif  // HAS_ESP32_P4_CAMERA
#endif  // USE_ESP32










