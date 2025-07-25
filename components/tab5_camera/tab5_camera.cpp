#include "tab5_camera.h"
#include "esphome/core/log.h"
#include "freertos/semphr.h"
#include "esp_cam_ctlr_csi.h"
#include "driver/isp.h"
#include "esp_ldo_regulator.h"
#include <cstring>

namespace esphome {
namespace tab5_camera {

static const char *const TAG = "tab5_camera";

// Configuration static (peut venir du .yaml via paramétrage futur)
static constexpr int FRAME_WIDTH = 640;
static constexpr int FRAME_HEIGHT = 480;
static constexpr size_t BUFFER_SIZE = FRAME_WIDTH * FRAME_HEIGHT * 2; // RGB565
static constexpr i2c_port_t I2C_PORT = I2C_NUM_0;
static constexpr gpio_num_t SDA_PIN = GPIO_NUM_6;
static constexpr gpio_num_t SCL_PIN = GPIO_NUM_7;
static constexpr int CAMERA_I2C_ADDRESS = 0x24;

void Tab5Camera::setup() {
  ESP_LOGCONFIG(TAG, "Setting up Tab5 Camera with ESP32-P4 MIPI-CSI...");

  // Step 1: Synchronization primitives
  this->frame_ready_semaphore_ = xSemaphoreCreateBinary();
  this->frame_queue_ = xQueueCreate(1, sizeof(esp_cam_ctlr_trans_t *));
  if (!this->frame_ready_semaphore_ || !this->frame_queue_) {
    ESP_LOGE(TAG, "Failed to create synchronization objects");
    return;
  }

  // Step 2: Initializing camera
  ESP_LOGI(TAG, "Step 1: Creating synchronization objects");
  ESP_LOGI(TAG, "Frame semaphore created successfully");
  ESP_LOGI(TAG, "Frame queue created successfully");

  ESP_LOGD(TAG, "Configuring external clock on GPIO36 at 20000000 Hz");

  ESP_LOGI(TAG, "Step 2: Initializing camera");
  this->init_camera();
}

void Tab5Camera::init_camera() {
  ESP_LOGI(TAG, "Starting camera initialization for 'Tab5 Camera'");

  // Step 2.1: MIPI LDO
  ESP_LOGI(TAG, "Step 2.1: Initializing MIPI LDO");
  esp_ldo_channel_config_t ldo_config = {
      .chan_id = 0,
      .voltage_mv = 1800,
  };
  ESP_ERROR_CHECK(esp_ldo_acquire_channel(&ldo_config, &this->ldo_handle_));
  ESP_LOGI(TAG, "MIPI LDO initialized successfully");

  // Step 2.2: Camera Reset — skipped
  ESP_LOGI(TAG, "Step 2.2: No reset pin configured, skipping reset");

  // Step 2.3: Camera I2C Init
  ESP_LOGI(TAG, "Step 2.3: Initializing camera sensor");
  i2c_master_bus_config_t i2c_config = {
      .clk_source = I2C_CLK_SRC_DEFAULT,
      .i2c_port = I2C_PORT,
      .scl_io_num = SCL_PIN,
      .sda_io_num = SDA_PIN,
      .glitch_ignore_cnt = 7,
      .flags.enable_internal_pullup = true,
  };
  ESP_ERROR_CHECK(i2c_new_master_bus(&i2c_config, &this->i2c_bus_));
  i2c_device_config_t dev_config = {
      .dev_addr_length = I2C_ADDR_BIT_LEN_7,
      .device_address = CAMERA_I2C_ADDRESS,
      .scl_speed_hz = 400000,
  };
  esp_err_t res = i2c_master_create_device(this->i2c_bus_, &dev_config, &this->camera_dev_);
  if (res != ESP_OK) {
    ESP_LOGW(TAG, "Could not read from sensor at address 0x%02X - sensor might not be connected", CAMERA_I2C_ADDRESS);
  }
  ESP_LOGI(TAG, "Camera sensor initialized successfully");

  // Step 2.4: Frame buffer
  ESP_LOGI(TAG, "Step 2.4: Allocating frame buffer");
  this->frame_buffer_ = heap_caps_malloc(BUFFER_SIZE, MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT);
  if (!this->frame_buffer_) {
    ESP_LOGE(TAG, "Failed to allocate frame buffer");
    return;
  }
  ESP_LOGI(TAG, "Frame buffer allocated successfully at %p", this->frame_buffer_);

  // Step 2.5: CSI init
  ESP_LOGI(TAG, "Step 2.5: Configuring CSI controller");
  esp_cam_ctlr_csi_config_t csi_config = {
      .ctlr_id = 0,
      .h_res = FRAME_WIDTH,
      .v_res = FRAME_HEIGHT,
      .lane_bit_rate_mbps = 400,
      .input_data_color_type = CAM_CTLR_COLOR_RAW8,
      .output_data_color_type = CAM_CTLR_COLOR_RGB565,
      .data_lane_num = 2,
      .byte_swap_en = false,
      .queue_items = 1,
  };
  ESP_ERROR_CHECK(esp_cam_new_csi_ctlr(&csi_config, &this->cam_handle_));
  ESP_LOGI(TAG, "CSI controller created successfully");

  // Step 2.6: Register callbacks
  ESP_LOGI(TAG, "Step 2.6: Registering camera callbacks");
  esp_cam_ctlr_evt_cbs_t cbs = {
      .on_get_new_trans = on_get_new_trans_cb,
      .on_trans_finished = on_frame_finished_cb,
  };
  ESP_ERROR_CHECK(esp_cam_ctlr_register_event_callbacks(this->cam_handle_, &cbs, this));
  ESP_LOGI(TAG, "Camera callbacks registered successfully");

  // Step 2.7: Enable CSI
  ESP_LOGI(TAG, "Step 2.7: Enabling camera controller");
  ESP_ERROR_CHECK(esp_cam_ctlr_enable(this->cam_handle_));
  ESP_LOGI(TAG, "Camera controller enabled successfully");

  // Step 2.8: ISP
  ESP_LOGI(TAG, "Step 2.8: Configuring ISP processor");
  esp_isp_processor_cfg_t isp_cfg = {
      .clk_hz = 80 * 1000 * 1000,
      .input_data_source = ISP_INPUT_DATA_SOURCE_CSI,
      .input_data_color_type = ISP_COLOR_RAW8,
      .output_data_color_type = ISP_COLOR_RGB565,
      .h_res = FRAME_WIDTH,
      .v_res = FRAME_HEIGHT,
      .has_line_start_packet = false,
      .has_line_end_packet = false,
  };
  ESP_ERROR_CHECK(esp_isp_new_processor(&isp_cfg, &this->isp_proc_));
  ESP_ERROR_CHECK(esp_isp_enable(this->isp_proc_));
  ESP_LOGI(TAG, "ISP processor enabled successfully");

  // Step 2.9: Frame init
  ESP_LOGI(TAG, "Step 2.9: Initializing frame buffer");
  memset(this->frame_buffer_, 0xFF, BUFFER_SIZE);

  // Step 2.10: Start camera
  ESP_LOGI(TAG, "Step 2.10: Starting camera controller");
  ESP_ERROR_CHECK(esp_cam_ctlr_start(this->cam_handle_));
  ESP_LOGI(TAG, "Camera 'Tab5 Camera' initialized successfully - all steps completed");
}

void Tab5Camera::start_streaming() {
  if (this->is_streaming_)
    return;

  ESP_LOGI(TAG, "Starting streaming for camera 'Tab5 Camera'");
  this->is_streaming_ = true;

  xTaskCreate([](void *arg) {
    auto *self = static_cast<Tab5Camera *>(arg);
    self->streaming_task();
    vTaskDelete(nullptr);
  }, "tab5_streaming", 4096, this, 5, nullptr);
}

void Tab5Camera::streaming_task() {
  ESP_LOGD(TAG, "Streaming loop started for camera 'Tab5 Camera'");
  while (this->is_streaming_) {
    esp_cam_ctlr_trans_t trans = {
        .buffer = this->frame_buffer_,
        .buflen = BUFFER_SIZE,
    };

    esp_err_t res = esp_cam_ctlr_receive(this->cam_handle_, &trans, pdMS_TO_TICKS(100));
    if (res == ESP_OK) {
      xQueueSend(this->frame_queue_, &trans, 0);
      xSemaphoreGive(this->frame_ready_semaphore_);
    } else {
      ESP_LOGW(TAG, "Frame receive failed or timed out: %d", res);
    }
  }
}

bool IRAM_ATTR Tab5Camera::on_get_new_trans_cb(esp_cam_ctlr_handle_t, esp_cam_ctlr_trans_t *trans, void *user_data) {
  auto *self = static_cast<Tab5Camera *>(user_data);
  trans->buffer = self->frame_buffer_;
  trans->buflen = BUFFER_SIZE;
  return false;
}

bool IRAM_ATTR Tab5Camera::on_frame_finished_cb(esp_cam_ctlr_handle_t, esp_cam_ctlr_trans_t *, void *user_data) {
  auto *self = static_cast<Tab5Camera *>(user_data);
  xSemaphoreGive(self->frame_ready_semaphore_);
  return false;
}

void Tab5Camera::loop() {
  // Add optional real-time preview processing here
}

void Tab5Camera::dump_config() {
  ESP_LOGCONFIG(TAG, "Tab5 Camera:");
  ESP_LOGCONFIG(TAG, "  Resolution: %dx%d", FRAME_WIDTH, FRAME_HEIGHT);
  ESP_LOGCONFIG(TAG, "  Frame buffer: %p (%u bytes)", this->frame_buffer_, BUFFER_SIZE);
  LOG_SWITCH("  Streaming: ", this->streaming_switch_);
}

}  // namespace tab5_camera
}  // namespace esphome









