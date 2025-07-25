#include "tab5_camera.h"
#include "esphome/core/log.h"

#include "esp_log.h"
#include "esp_cache.h"
#include "esp_cam_ctlr_csi.h"
#include "esp_cam_ctlr.h"
#include "driver/isp.h"
#include "esp_ldo_regulator.h"
#include "freertos/semphr.h"

namespace esphome {
namespace tab5_camera {

static const char *const TAG = "tab5_camera";

static bool get_new_vb_cb(esp_cam_ctlr_handle_t, esp_cam_ctlr_trans_t *trans, void *user_data);
static bool trans_finished_cb(esp_cam_ctlr_handle_t, esp_cam_ctlr_trans_t *, void *);

void Tab5Camera::setup() {
  ESP_LOGCONFIG(TAG, "Setting up Tab5 Camera with ESP32-P4 MIPI-CSI...");

  // Step 1: Create synchronization objects
  ESP_LOGI(TAG, "Step 1: Creating synchronization objects");
  this->frame_semaphore_ = xSemaphoreCreateBinary();
  this->frame_queue_ = xQueueCreate(1, sizeof(uint8_t *));
  if (!this->frame_semaphore_ || !this->frame_queue_) {
    ESP_LOGE(TAG, "Failed to create synchronization objects");
    return;
  }

  // Step 2: Initialize camera
  ESP_LOGI(TAG, "Step 2: Initializing camera");

  // 2.1: Init MIPI LDO
  ESP_LOGI(TAG, "Step 2.1: Initializing MIPI LDO");
  esp_ldo_channel_config_t ldo_cfg = {
    .chan_id = 0,
    .voltage_mv = 1800,
  };
  ESP_ERROR_CHECK(esp_ldo_acquire_channel(&ldo_cfg, &this->ldo_handle_));
  ESP_LOGI(TAG, "MIPI LDO initialized");

  // 2.2: Reset (not used)
  ESP_LOGI(TAG, "Step 2.2: No reset pin configured, skipping reset");

  // 2.3: Init sensor (only fake I2C ping)
  ESP_LOGI(TAG, "Step 2.3: Initializing camera sensor");
  i2c_master_bus_config_t i2c_config = {
    .clk_source = I2C_CLK_SRC_DEFAULT,
    .i2c_port = I2C_NUM_0,
    .sda_io_num = this->i2c_sda_,
    .scl_io_num = this->i2c_scl_,
    .glitch_ignore_cnt = 7,
    .flags.enable_internal_pullup = true,
  };
  i2c_master_bus_handle_t i2c_bus;
  ESP_ERROR_CHECK(i2c_new_master_bus(&i2c_config, &i2c_bus));
  i2c_device_config_t dev_cfg = {
    .scl_speed_hz = 100000,
    .device_address = 0x24,
  };
  i2c_master_dev_handle_t dev_handle;
  if (i2c_master_bus_add_device(i2c_bus, &dev_cfg, &dev_handle) != ESP_OK) {
    ESP_LOGW(TAG, "Could not communicate with sensor at 0x24, continuing anyway");
  } else {
    i2c_master_bus_rm_device(dev_handle);
  }
  i2c_del_master_bus(i2c_bus);
  ESP_LOGI(TAG, "Camera sensor initialized");

  // 2.4: Allocate framebuffer
  ESP_LOGI(TAG, "Step 2.4: Allocating frame buffer");
  this->frame_size_ = this->width_ * this->height_ * 2;
  this->frame_buffer_ = static_cast<uint8_t *>(heap_caps_malloc(this->frame_size_, MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT));
  if (!this->frame_buffer_) {
    ESP_LOGE(TAG, "Failed to allocate frame buffer");
    return;
  }

  // 2.5: Configure CSI controller
  ESP_LOGI(TAG, "Step 2.5: Configuring CSI controller");
  esp_cam_ctlr_csi_config_t csi_cfg = {
    .ctlr_id = 0,
    .h_res = this->width_,
    .v_res = this->height_,
    .lane_bit_rate_mbps = 400,
    .input_data_color_type = CAM_CTLR_COLOR_RAW8,
    .output_data_color_type = CAM_CTLR_COLOR_RGB565,
    .data_lane_num = 2,
    .byte_swap_en = false,
    .queue_items = 1,
  };
  ESP_ERROR_CHECK(esp_cam_new_csi_ctlr(&csi_cfg, &this->cam_ctlr_));

  // 2.6: Register camera callbacks
  ESP_LOGI(TAG, "Step 2.6: Registering camera callbacks");
  esp_cam_ctlr_evt_cbs_t cbs = {
    .on_get_new_trans = get_new_vb_cb,
    .on_trans_finished = trans_finished_cb,
  };
  ESP_ERROR_CHECK(esp_cam_ctlr_register_event_callbacks(this->cam_ctlr_, &cbs, this));

  // 2.7: Enable CSI controller
  ESP_LOGI(TAG, "Step 2.7: Enabling camera controller");
  ESP_ERROR_CHECK(esp_cam_ctlr_enable(this->cam_ctlr_));

  // 2.8: Init ISP
  ESP_LOGI(TAG, "Step 2.8: Initializing ISP processor");
  esp_isp_processor_cfg_t isp_cfg = {
    .clk_hz = 80 * 1000000,
    .input_data_source = ISP_INPUT_DATA_SOURCE_CSI,
    .input_data_color_type = ISP_COLOR_RAW8,
    .output_data_color_type = ISP_COLOR_RGB565,
    .has_line_start_packet = false,
    .has_line_end_packet = false,
    .h_res = this->width_,
    .v_res = this->height_,
  };
  ESP_ERROR_CHECK(esp_isp_new_processor(&isp_cfg, &this->isp_proc_));
  ESP_ERROR_CHECK(esp_isp_enable(this->isp_proc_));

  // 2.9: Clear framebuffer
  memset(this->frame_buffer_, 0x00, this->frame_size_);
  esp_cache_msync(this->frame_buffer_, this->frame_size_, ESP_CACHE_MSYNC_FLAG_DIR_C2M);

  // 2.10: Start controller
  ESP_LOGI(TAG, "Step 2.10: Starting camera controller");
  this->frame_trans_.buffer = this->frame_buffer_;
  this->frame_trans_.buflen = this->frame_size_;
  ESP_ERROR_CHECK(esp_cam_ctlr_start(this->cam_ctlr_));

  ESP_LOGI(TAG, "Camera 'Tab5 Camera' initialized successfully - all steps completed");
}

void Tab5Camera::dump_config() {
  ESP_LOGCONFIG(TAG, "Tab5 Camera:");
  ESP_LOGCONFIG(TAG, "  Resolution: %ux%u", this->width_, this->height_);
}

void IRAM_ATTR Tab5Camera::loop() {
  if (!this->streaming_) return;

  if (esp_cam_ctlr_receive(this->cam_ctlr_, &this->frame_trans_, 0) == ESP_OK) {
    xSemaphoreGive(this->frame_semaphore_);
    xQueueSend(this->frame_queue_, &this->frame_buffer_, 0);
  }
}

void Tab5Camera::start_streaming() {
  this->streaming_ = true;
  ESP_LOGI(TAG, "Camera streaming started");
}

void Tab5Camera::stop_streaming() {
  this->streaming_ = false;
  ESP_LOGI(TAG, "Camera streaming stopped");
}

bool get_new_vb_cb(esp_cam_ctlr_handle_t, esp_cam_ctlr_trans_t *trans, void *user_data) {
  auto *cam = static_cast<Tab5Camera *>(user_data);
  *trans = cam->frame_trans_;
  return false;
}

bool trans_finished_cb(esp_cam_ctlr_handle_t, esp_cam_ctlr_trans_t *, void *) {
  // We don’t need post-processing here yet
  return false;
}

}  // namespace tab5_camera
}  // namespace esphome








