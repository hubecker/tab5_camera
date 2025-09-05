#include "tab5_camera.h"
#include "esphome/core/log.h"
#include "esphome/core/helpers.h"
#include "esphome/core/hal.h"
#include "esp_timer.h"
#include "driver/ledc.h"
#include "esp_cam_sensor.h"
#include "driver/jpeg_encode.h"
#include "esp_video_buffer.h"
#include "esp_video_internal.h"
#include "esp_cam_ctlr_csi.h"

#ifdef USE_ESP32

static const char *const TAG = "tab5_camera";

#define TAB5_CAMERA_H_RES 640
#define TAB5_CAMERA_V_RES 480
#define TAB5_MIPI_CSI_LANE_BITRATE_MBPS 200
#define TAB5_ISP_CLOCK_HZ 50000000  
#define TAB5_STREAMING_STACK_SIZE 8192
#define TAB5_FRAME_QUEUE_LENGTH 8

namespace esphome {
namespace tab5_camera {

Tab5Camera::~Tab5Camera() {
#ifdef HAS_ESP32_P4_CAMERA
  this->deinit_camera_();
#endif
}

void Tab5Camera::setup() {
#ifdef HAS_ESP32_P4_CAMERA
  ESP_LOGCONFIG(TAG, "Setting up Tab5 Camera with ESP32-P4 MIPI-CSI...");

  this->frame_ready_semaphore_ = xSemaphoreCreateBinary();
  if (!this->frame_ready_semaphore_) {
    ESP_LOGE(TAG, "Failed to create frame ready semaphore");
    this->mark_failed();
    return;
  }

  this->frame_queue_ = xQueueCreate(TAB5_FRAME_QUEUE_LENGTH, sizeof(FrameData));
  if (!this->frame_queue_) {
    ESP_LOGE(TAG, "Failed to create frame queue");
    this->mark_failed();
    return;
  }

  if (!this->init_camera_()) {
    ESP_LOGE(TAG, "Failed to initialize camera - setup marked as failed");
    this->mark_failed();
    return;
  }

  ESP_LOGCONFIG(TAG, "Tab5 Camera '%s' setup completed successfully", this->name_.c_str());
#else
  ESP_LOGE(TAG, "ESP32-P4 MIPI-CSI API not available - Tab5 Camera component disabled");
  this->mark_failed();
#endif
}

void Tab5Camera::dump_config() {
  ESP_LOGCONFIG(TAG, "Tab5 Camera '%s':", this->name_.c_str());
#ifdef HAS_ESP32_P4_CAMERA
  ESP_LOGCONFIG(TAG, "  Platform: ESP32-P4 MIPI-CSI");
  ESP_LOGCONFIG(TAG, "  Resolution: %dx%d", TAB5_CAMERA_H_RES, TAB5_CAMERA_V_RES);
  ESP_LOGCONFIG(TAG, "  Pixel Format: %s", this->pixel_format_.c_str());
  ESP_LOGCONFIG(TAG, "  Framerate: %d fps", this->framerate_);
  if (this->external_clock_pin_ > 0) {
    ESP_LOGCONFIG(TAG, "  External Clock Pin: GPIO%u", this->external_clock_pin_);
    ESP_LOGCONFIG(TAG, "  External Clock Frequency: %u Hz", this->external_clock_frequency_);
  }
  ESP_LOGCONFIG(TAG, "  Frame Buffer Size: %zu bytes", this->frame_buffer_size_);
  ESP_LOGCONFIG(TAG, "  I2C Address: 0x%02X", this->sensor_address_);
  if (this->reset_pin_) {
    LOG_PIN("  Reset Pin: ", this->reset_pin_);
  }
#else
  ESP_LOGCONFIG(TAG, "  Status: ESP32-P4 MIPI-CSI API not available");
#endif

  if (this->is_failed()) {
    ESP_LOGCONFIG(TAG, "  Setup Failed");
  }
}

float Tab5Camera::get_setup_priority() const {
  return setup_priority::HARDWARE - 1.0f;
}

bool Tab5Camera::is_ready() const {
#ifdef HAS_ESP32_P4_CAMERA
  return this->camera_initialized_ && this->sensor_initialized_;
#else
  return false;
#endif
}

// --- Fonctions I2C/SCCB ---
bool Tab5Camera::init_sccb_() {
    i2c_master_bus_config_t i2c_bus_config = {
        .clk_source = I2C_CLK_SRC_DEFAULT,
        .i2c_port   = I2C_NUM_0,
        .scl_io_num = 21,
        .sda_io_num = 22,
        .glitch_ignore_cnt = 7,
    };
    i2c_master_bus_handle_t bus_handle;
    if (i2c_new_master_bus(&i2c_bus_config, &bus_handle) != ESP_OK) {
        set_error_("Failed to create I2C bus");
        return false;
    }

    sccb_i2c_config_t sccb_config = {
        .dev_addr_length = I2C_ADDR_BIT_LEN_7,
        .device_address  = this->sensor_address_,
        .scl_speed_hz    = 400000,
    };
    if (sccb_new_i2c_io(bus_handle, &sccb_config, &this->sccb_handle_) != ESP_OK) {
        i2c_del_master_bus(bus_handle);
        set_error_("Failed to create SCCB handle");
        return false;
    }

    this->i2c_bus_handle_ = bus_handle;
    return true;
}

bool Tab5Camera::detect_sensor_with_sccb_() {
    uint8_t val;
    if (read_byte(0x00, &val)) {
        ESP_LOGI(TAG, "Sensor detected via SCCB: reg0x00=0x%02X", val);
        return true;
    }
    return false;
}

// --- Reset et horloge ---
bool Tab5Camera::reset_sensor_() {
  if (this->reset_pin_) {
    this->reset_pin_->setup();
    this->reset_pin_->digital_write(false);
    vTaskDelay(50 / portTICK_PERIOD_MS);
    this->reset_pin_->digital_write(true);
    vTaskDelay(100 / portTICK_PERIOD_MS);
    uint8_t test_val;
    if (this->read_byte(0x00, &test_val)) {
        ESP_LOGI(TAG, "Sensor reset OK (reg0x00=0x%02X)", test_val);
        return true;
    }
    ESP_LOGW(TAG, "Sensor did not respond after hardware reset");
    return false;
  }
  // Software reset fallback
  if (!this->write_byte(0x12, 0x80)) return false;
  vTaskDelay(100 / portTICK_PERIOD_MS);
  return true;
}

bool Tab5Camera::setup_external_clock_() {
  if (this->external_clock_pin_ == 0) return true;
  ledc_timer_config_t timer_conf = {};
  timer_conf.duty_resolution = LEDC_TIMER_1_BIT;
  timer_conf.freq_hz = this->external_clock_frequency_;
  timer_conf.speed_mode = LEDC_LOW_SPEED_MODE;
  timer_conf.deconfigure = false;
  timer_conf.clk_cfg = LEDC_AUTO_CLK;
  timer_conf.timer_num = LEDC_TIMER_0;
  if (ledc_timer_config(&timer_conf) != ESP_OK) return false;

  ledc_channel_config_t ch_conf = {};
  ch_conf.gpio_num = this->external_clock_pin_;
  ch_conf.speed_mode = LEDC_LOW_SPEED_MODE;
  ch_conf.channel = LEDC_CHANNEL_0;
  ch_conf.intr_type = LEDC_INTR_DISABLE;
  ch_conf.timer_sel = LEDC_TIMER_0;
  ch_conf.duty = 1;
  ch_conf.hpoint = 0;
  ch_conf.sleep_mode = LEDC_SLEEP_MODE_KEEP_ALIVE;
  return ledc_channel_config(&ch_conf) == ESP_OK;
}

// --- Identification du capteur ---
bool Tab5Camera::identify_sensor_() {
    ESP_LOGI(TAG, "Identifying sensor at I2C 0x%02X", this->sensor_address_);
    if (!init_sccb_()) {
        ESP_LOGW(TAG, "Failed to init SCCB");
        return false;
    }

    uint8_t id_high=0, id_low=0;
    if (read_register_16(0x3107, &id_high) && read_register_16(0x3108, &id_low)) {
        uint16_t id = (id_high << 8) | id_low;
        ESP_LOGI(TAG, "Sensor ID=0x%04X", id);
        if (id == 0x2356) {
            sensor_initialized_ = true;
            return true;
        }
    }

    ESP_LOGW(TAG, "Sensor not identified - using generic config");
    return true;
}

// --- Initialisation LDO ---
bool Tab5Camera::init_ldo_() {
  if (this->ldo_initialized_) return true;
  esp_ldo_channel_config_t ldo_cfg = { .chan_id = 3, .voltage_mv = 2500 };
  esp_ldo_acquire_channel(&ldo_cfg, &this->ldo_mipi_phy_);
  this->ldo_initialized_ = true;
  return true;
}

// --- Initialisation du capteur ---
bool Tab5Camera::init_sensor_() {
  if (sensor_initialized_) return true;
  if (!identify_sensor_()) return false;
  if (!configure_minimal_sensor_()) return false;
  sensor_initialized_ = true;
  return true;
}

// --- Allocation du frame buffer ---
bool Tab5Camera::init_camera_() {
  if (this->camera_initialized_) {
    ESP_LOGI(TAG, "Camera already initialized, skipping");
    return true;
  }

  ESP_LOGI(TAG, "Starting camera initialization for '%s'", this->name_.c_str());

  // Étape 0: Configuration de l'horloge externe (CRITIQUE)
  ESP_LOGI(TAG, "Step 2.0: Setting up external clock");
  if (!this->setup_external_clock_()) {
    ESP_LOGE(TAG, "Failed to setup external clock - camera will not work");
    return false;
  }
  this->verify_external_clock_();
  ESP_LOGI(TAG, "External clock configured and verified successfully");

  // Étape 1: Initialisation du LDO MIPI
  ESP_LOGI(TAG, "Step 2.1: Initializing MIPI LDO");
  if (!this->init_ldo_()) {
    ESP_LOGE(TAG, "Failed to initialize MIPI LDO");
    return false;
  }
  ESP_LOGI(TAG, "MIPI LDO initialized successfully");

  // Étape 2: Reset de la caméra (APRÈS l'horloge)
  ESP_LOGI(TAG, "Step 2.2: Executing camera sensor reset");
  vTaskDelay(100 / portTICK_PERIOD_MS);  // Laisser l'horloge se stabiliser
  if (!this->reset_sensor_()) {
    ESP_LOGW(TAG, "Camera reset failed, but continuing initialization");
  }
  ESP_LOGI(TAG, "Camera reset sequence completed");

  // Étape 3: Initialisation du capteur I2C
  ESP_LOGI(TAG, "Step 2.3: Initializing camera sensor");
  if (!this->init_sensor_()) {
    ESP_LOGE(TAG, "Failed to initialize camera sensor");
    return false;
  }
  ESP_LOGI(TAG, "Camera sensor initialized successfully");

  // Étape 4: Allocation du frame buffer principal
  ESP_LOGI(TAG, "Step 2.4: Allocating main frame buffer");

  this->frame_buffer_size_ = TAB5_CAMERA_H_RES * TAB5_CAMERA_V_RES * 2; // RGB565 = 2 bytes par pixel
  this->frame_buffer_size_ = (this->frame_buffer_size_ + 63) & ~63; // Alignement 64 bytes

  ESP_LOGI(TAG, "Aligned frame buffer size: %zu bytes", this->frame_buffer_size_);

  this->frame_buffer_ = heap_caps_aligned_alloc(64, this->frame_buffer_size_, MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT);
  if (!this->frame_buffer_) {
    ESP_LOGE(TAG, "Failed to allocate aligned frame buffer in PSRAM - trying regular RAM");
    this->frame_buffer_ = heap_caps_aligned_alloc(64, this->frame_buffer_size_, MALLOC_CAP_DMA | MALLOC_CAP_8BIT);
    if (!this->frame_buffer_) {
      ESP_LOGE(TAG, "Failed to allocate aligned frame buffer in regular RAM also");
      return false;
    }
  }

  ESP_LOGI(TAG, "Main frame buffer allocated successfully at %p", this->frame_buffer_);

  // Étape 5: Configuration du contrôleur CSI
  ESP_LOGI(TAG, "Step 2.5: Configuring CSI controller");
  esp_cam_ctlr_csi_config_t csi_config = {};
  csi_config.ctlr_id = 0;
  csi_config.h_res = TAB5_CAMERA_H_RES;
  csi_config.v_res = TAB5_CAMERA_V_RES;
  csi_config.lane_bit_rate_mbps = TAB5_MIPI_CSI_LANE_BITRATE_MBPS;
  csi_config.input_data_color_type = CAM_CTLR_COLOR_RAW8;
  csi_config.output_data_color_type = CAM_CTLR_COLOR_RGB565;
  csi_config.data_lane_num = 2;
  csi_config.byte_swap_en = false;
  csi_config.queue_items = 4;

  esp_err_t ret = esp_cam_new_csi_ctlr(&csi_config, &this->cam_handle_);
  if (ret != ESP_OK) {
    ESP_LOGE(TAG, "CSI controller init failed: %s", esp_err_to_name(ret));
    return false;
  }
  ESP_LOGI(TAG, "CSI controller created successfully");
  
  // Étape 6: Configuration des callbacks
  ESP_LOGI(TAG, "Step 2.6: Registering camera callbacks");
  esp_cam_ctlr_evt_cbs_t cbs = {
    .on_get_new_trans = nullptr,
    .on_trans_finished = Tab5Camera::camera_get_finished_trans_callback,
  };
  
  ret = esp_cam_ctlr_register_event_callbacks(this->cam_handle_, &cbs, this);
  if (ret != ESP_OK) {
    ESP_LOGE(TAG, "Failed to register camera callbacks: %s", esp_err_to_name(ret));
    return false;
  }
  ESP_LOGI(TAG, "Camera callbacks registered successfully");
  
  // Étape 7: Activation du contrôleur de caméra
  ESP_LOGI(TAG, "Step 2.7: Enabling camera controller");
  ret = esp_cam_ctlr_enable(this->cam_handle_);
  if (ret != ESP_OK) {
    ESP_LOGE(TAG, "Failed to enable camera controller: %s", esp_err_to_name(ret));
    return false;
  }
  ESP_LOGI(TAG, "Camera controller enabled successfully");
  
  // Étape 8: Configuration de l'ISP
  ESP_LOGI(TAG, "Step 2.8: Configuring ISP processor");
  esp_isp_processor_cfg_t isp_config = {};
  isp_config.clk_hz = TAB5_ISP_CLOCK_HZ;
  isp_config.input_data_source = ISP_INPUT_DATA_SOURCE_CSI;
  isp_config.input_data_color_type = ISP_COLOR_RAW8;
  isp_config.output_data_color_type = ISP_COLOR_RGB565;
  isp_config.has_line_start_packet = false;
  isp_config.has_line_end_packet = false;
  isp_config.h_res = TAB5_CAMERA_H_RES;
  isp_config.v_res = TAB5_CAMERA_V_RES;
  
  ret = esp_isp_new_processor(&isp_config, &this->isp_proc_);
  if (ret != ESP_OK) {
    ESP_LOGE(TAG, "ISP processor init failed: %s", esp_err_to_name(ret));
    return false;
  }
  ESP_LOGI(TAG, "ISP processor created successfully");
  
  ret = esp_isp_enable(this->isp_proc_);
  if (ret != ESP_OK) {
    ESP_LOGE(TAG, "Failed to enable ISP processor: %s", esp_err_to_name(ret));
    return false;
  }
  ESP_LOGI(TAG, "ISP processor enabled successfully");
  
  // Étape 9: Initialisation du frame buffer
  ESP_LOGI(TAG, "Step 2.9: Initializing frame buffer");
  memset(this->frame_buffer_, 0x00, this->frame_buffer_size_);
  esp_cache_msync(this->frame_buffer_, this->frame_buffer_size_, ESP_CACHE_MSYNC_FLAG_DIR_C2M);
  ESP_LOGI(TAG, "Frame buffer initialized");
  
  // Étape 10: Démarrage de la caméra
  ESP_LOGI(TAG, "Step 2.10: Starting camera controller");
  ret = esp_cam_ctlr_start(this->cam_handle_);
  if (ret != ESP_OK) {
    ESP_LOGE(TAG, "Failed to start camera controller: %s", esp_err_to_name(ret));
    return false;
  }
  ESP_LOGI(TAG, "Camera controller started successfully");
  
  // Étape 11: Test de capture manuelle
  ESP_LOGI(TAG, "Step 2.11: Testing manual capture");
  if (this->test_manual_capture_()) {
    ESP_LOGI(TAG, "✓ Manual capture works - sensor is generating data");
  } else {
    ESP_LOGW(TAG, "⚠ Manual capture failed - sensor may not be generating data");
  }
  
  // Étape 12: Démarrage de la capture continue
  ESP_LOGI(TAG, "Step 2.12: Starting continuous capture");
  if (this->start_continuous_capture_()) {
    ESP_LOGI(TAG, "✓ Continuous capture started successfully");
  } else {
    ESP_LOGW(TAG, "⚠ Continuous capture failed - callbacks may not work");
  }
  
  this->camera_initialized_ = true;
  ESP_LOGI(TAG, "Camera '%s' initialized successfully - all steps completed", this->name_.c_str());
  return true;
}

void Tab5Camera::process_frame_(uint8_t* data, size_t len) {
  this->frame_count_++;
  this->last_frame_timestamp_ = millis();
  
  ESP_LOGI(TAG, "🖼 Processing frame #%u: %zu bytes", this->frame_count_, len);
  
  // Analyse basique de la frame
  if (len > 8) {
    ESP_LOGD(TAG, "Frame data: %02X %02X %02X %02X %02X %02X %02X %02X", 
             data[0], data[1], data[2], data[3], data[4], data[5], data[6], data[7]);
  }
  
  // Appeler les callbacks traditionnels
  this->on_frame_callbacks_.call(data, len);
  
  // Déclencher les triggers ESPHome
  this->trigger_on_frame_callbacks_(data, len);
}

void Tab5Camera::trigger_on_frame_callbacks_(uint8_t* data, size_t len) {
  for (auto *trigger : this->on_frame_triggers_) {
    trigger->trigger(data, len);
  }
}

bool Tab5Camera::camera_get_finished_trans_callback(esp_cam_ctlr_handle_t handle, esp_cam_ctlr_trans_t *trans, void *user_data) {
  Tab5Camera *camera = static_cast<Tab5Camera*>(user_data);
  if (!camera || !trans->buffer) {
    ESP_LOGE(TAG, "Invalid callback data");
    return false;
  }

  // Compteur de frames pour diagnostics
  static uint32_t frame_count = 0;
  frame_count++;
  
  ESP_LOGI(TAG, "🎬 Frame #%u CALLBACK: %zu bytes (expected: %zu)", 
           frame_count, trans->received_size, camera->frame_buffer_size_);

  if (trans->received_size == 0) {
    ESP_LOGW(TAG, "⚠ Frame #%u is empty - sensor might not be generating data", frame_count);
    
    // Relancer une nouvelle capture même en cas de frame vide
    esp_cam_ctlr_trans_t new_trans = {};
    new_trans.buffer = trans->buffer;  // Réutiliser le même buffer
    new_trans.buflen = camera->frame_buffer_size_;
    esp_cam_ctlr_receive(handle, &new_trans, 0);
    
    return false;
  }
  
  if (trans->received_size < 1000) {
    ESP_LOGW(TAG, "⚠ Frame #%u size is very small (%zu bytes)", frame_count, trans->received_size);
  }

  // Synchronisation du cache pour la frame reçue
  esp_cache_msync(trans->buffer, trans->received_size, ESP_CACHE_MSYNC_FLAG_DIR_M2C);
  
  // Vérification du contenu des premiers bytes
  uint8_t *data = static_cast<uint8_t*>(trans->buffer);
  ESP_LOGD(TAG, "Frame #%u first bytes: %02X %02X %02X %02X %02X %02X %02X %02X", 
           frame_count, data[0], data[1], data[2], data[3], data[4], data[5], data[6], data[7]);
  
  // Traitement de la frame
  camera->process_frame_(data, trans->received_size);
  
  // Création d'une structure FrameData pour la queue
  FrameData frame;
  frame.buffer = trans->buffer;
  frame.size = trans->received_size;
  frame.timestamp = esp_timer_get_time();
  frame.valid = true;
  
  // Envoi non-bloquant vers la queue applicative
  BaseType_t ret = xQueueSendFromISR(camera->frame_queue_, &frame, NULL);
  if (ret == pdTRUE) {
    xSemaphoreGiveFromISR(camera->frame_ready_semaphore_, NULL);
    ESP_LOGV(TAG, "Frame #%u queued successfully", frame_count);
  } else {
    ESP_LOGD(TAG, "Application frame queue full, dropping frame #%u", frame_count);
  }

  // CRUCIAL: Relancer une nouvelle capture pour maintenir le flux
  esp_cam_ctlr_trans_t new_trans = {};
  new_trans.buffer = trans->buffer;  // Réutiliser le même buffer
  new_trans.buflen = camera->frame_buffer_size_;
  esp_err_t capture_ret = esp_cam_ctlr_receive(handle, &new_trans, 0);
  if (capture_ret != ESP_OK) {
    ESP_LOGW(TAG, "Failed to restart capture: %s", esp_err_to_name(capture_ret));
  }

  return false; // Retourner false pour que le driver libère le buffer
}

bool Tab5Camera::take_snapshot() {
  if (!this->camera_initialized_) {
    ESP_LOGE(TAG, "Camera '%s' not initialized", this->name_.c_str());
    return false;
  }
  
  ESP_LOGI(TAG, "Taking snapshot with camera '%s'", this->name_.c_str());
  
  esp_cam_ctlr_trans_t trans = {
    .buffer = this->frame_buffer_,
    .buflen = this->frame_buffer_size_,
  };
  
  esp_err_t ret = esp_cam_ctlr_receive(this->cam_handle_, &trans, 5000 / portTICK_PERIOD_MS); // 5 secondes timeout
  if (ret != ESP_OK) {
    ESP_LOGW(TAG, "Camera '%s' capture failed: %s", this->name_.c_str(), esp_err_to_name(ret));
    return false;
  }
  
  ESP_LOGI(TAG, "Camera '%s' snapshot taken, size: %zu bytes", this->name_.c_str(), trans.received_size);
  
  // Synchronisation du cache
  esp_cache_msync(this->frame_buffer_, trans.received_size, ESP_CACHE_MSYNC_FLAG_DIR_M2C);
  
  // Traitement de la frame
  this->process_frame_(static_cast<uint8_t*>(this->frame_buffer_), trans.received_size);
  
  return true;
}

bool Tab5Camera::start_streaming() {
  if (!this->camera_initialized_) {
    ESP_LOGE(TAG, "Camera '%s' not initialized for streaming", this->name_.c_str());
    return false;
  }
  
  if (this->streaming_active_) {
    ESP_LOGW(TAG, "Camera '%s' streaming already active", this->name_.c_str());
    return true;
  }
  
  ESP_LOGI(TAG, "Starting streaming for camera '%s'", this->name_.c_str());
  
  this->streaming_should_stop_ = false;
  this->streaming_active_ = true;
  
  // Si la capture continue n'est pas active, la démarrer
  if (!this->continuous_capture_active_) {
    if (!this->start_continuous_capture_()) {
      ESP_LOGE(TAG, "Failed to start continuous capture");
      this->streaming_active_ = false;
      return false;
    }
  }
  
  // Création de la tâche de streaming
  BaseType_t result = xTaskCreate(
    Tab5Camera::streaming_task,
    "tab5_streaming",
    TAB5_STREAMING_STACK_SIZE,
    this,
    5,
    &this->streaming_task_handle_
  );
  
  if (result != pdPASS) {
    ESP_LOGE(TAG, "Failed to create streaming task");
    this->streaming_active_ = false;
    return false;
  }
  
  ESP_LOGI(TAG, "Camera '%s' streaming started", this->name_.c_str());
  return true;
}

bool Tab5Camera::stop_streaming() {
  if (!this->streaming_active_) {
    return true;
  }
  
  ESP_LOGI(TAG, "Stopping camera '%s' streaming...", this->name_.c_str());
  
  this->streaming_should_stop_ = true;
  
  if (this->streaming_task_handle_) {
    xSemaphoreGive(this->frame_ready_semaphore_);
    
    uint32_t timeout = 0;
    while (this->streaming_active_ && timeout < 50) {
      vTaskDelay(100 / portTICK_PERIOD_MS);
      timeout++;
    }
    
    if (this->streaming_active_) {
      ESP_LOGW(TAG, "Force stopping streaming task");
      vTaskDelete(this->streaming_task_handle_);
      this->streaming_active_ = false;
    }
    
    this->streaming_task_handle_ = nullptr;
  }
  
  ESP_LOGI(TAG, "Camera '%s' streaming stopped", this->name_.c_str());
  return true;
}

void Tab5Camera::streaming_task(void *parameter) {
  Tab5Camera *camera = static_cast<Tab5Camera*>(parameter);
  camera->streaming_loop_();
}

void Tab5Camera::streaming_loop_() {
  ESP_LOGI(TAG, "🎥 Streaming loop started for camera '%s' (callback-based)", this->name_.c_str());

  while (!this->streaming_should_stop_) {
    if (xSemaphoreTake(this->frame_ready_semaphore_, 100 / portTICK_PERIOD_MS) == pdTRUE) {
      FrameData frame;
      if (xQueueReceive(this->frame_queue_, &frame, 0) == pdTRUE) {
        ESP_LOGV(TAG, "📺 Frame received from callback, size: %zu bytes", frame.size);
        // Le traitement est déjà fait dans process_frame_
      }
    }
    vTaskDelay(1 / portTICK_PERIOD_MS);
  }

  this->streaming_active_ = false;
  ESP_LOGI(TAG, "Streaming loop ended for camera '%s'", this->name_.c_str());
  vTaskDelete(nullptr);
}

void Tab5Camera::deinit_camera_() {
  if (this->streaming_active_) {
    this->stop_streaming();
  }
  
  if (this->camera_initialized_) {
    ESP_LOGD(TAG, "Deinitializing camera '%s'...", this->name_.c_str());
    
    if (this->cam_handle_) {
      esp_cam_ctlr_stop(this->cam_handle_);
      esp_cam_ctlr_disable(this->cam_handle_);
      esp_cam_ctlr_del(this->cam_handle_);
      this->cam_handle_ = nullptr;
    }
    
    if (this->isp_proc_) {
      esp_isp_disable(this->isp_proc_);
      esp_isp_del_processor(this->isp_proc_);
      this->isp_proc_ = nullptr;
    }
    
    if (this->frame_buffer_) {
      heap_caps_free(this->frame_buffer_);
      this->frame_buffer_ = nullptr;
    }
    
    // Libération des buffers multiples
    for (size_t i = 0; i < NUM_FRAME_BUFFERS; i++) {
      if (this->frame_buffers_[i]) {
        heap_caps_free(this->frame_buffers_[i]);
        this->frame_buffers_[i] = nullptr;
      }
    }
    
    if (this->ldo_mipi_phy_) {
      esp_ldo_release_channel(this->ldo_mipi_phy_);
      this->ldo_mipi_phy_ = nullptr;
    }
    
    this->camera_initialized_ = false;
    this->sensor_initialized_ = false;
    this->ldo_initialized_ = false;
    this->continuous_capture_active_ = false;
    ESP_LOGD(TAG, "Camera '%s' deinitialized", this->name_.c_str());
  }
  
  if (this->frame_ready_semaphore_) {
    vSemaphoreDelete(this->frame_ready_semaphore_);
    this->frame_ready_semaphore_ = nullptr;
  }
  
  if (this->frame_queue_) {
    vQueueDelete(this->frame_queue_);
    this->frame_queue_ = nullptr;
  }
}

// Méthodes utilitaires
void Tab5Camera::set_error_(const std::string &error) {
  this->error_state_ = true;
  this->last_error_ = error;
  this->error_count_++;
  ESP_LOGE(TAG, "Camera error: %s", error.c_str());
}

void Tab5Camera::clear_error_() {
  this->error_state_ = false;
  this->last_error_ = "";
}

PixelFormat Tab5Camera::parse_pixel_format_(const std::string &format) const {
  if (format == "RAW8") return PixelFormat::RAW8;
  if (format == "RAW10") return PixelFormat::RAW10;
  if (format == "YUV422") return PixelFormat::YUV422;
  if (format == "RGB565") return PixelFormat::RGB565;
  if (format == "JPEG") return PixelFormat::JPEG;
  return PixelFormat::RGB565;
}

size_t Tab5Camera::calculate_frame_size_() const {
  uint16_t bytes_per_pixel = 2; // RGB565 par défaut
  
  switch (this->parse_pixel_format_(this->pixel_format_)) {
    case PixelFormat::RAW8:
      bytes_per_pixel = 1;
      break;
    case PixelFormat::RAW10:
      bytes_per_pixel = 2;
      break;
    case PixelFormat::YUV422:
      bytes_per_pixel = 2;
      break;
    case PixelFormat::RGB565:
      bytes_per_pixel = 2;
      break;
    case PixelFormat::JPEG:
      bytes_per_pixel = 1; // Variable, approximation
      break;
  }
  
  return this->frame_width_ * this->frame_height_ * bytes_per_pixel;
}

bool Tab5Camera::write_register_16(uint16_t reg, uint8_t val) {
  uint8_t buffer[3] = {
    static_cast<uint8_t>(reg >> 8),
    static_cast<uint8_t>(reg & 0xFF),
    val
  };
  
  if (!this->write(buffer, 3)) {
    ESP_LOGD(TAG, "Failed to write 16-bit register 0x%04X = 0x%02X", reg, val);
    return false;
  }
  
  return true;
}

bool Tab5Camera::read_register_16(uint16_t reg, uint8_t *val) {
  uint8_t buffer[2] = {
    static_cast<uint8_t>(reg >> 8),
    static_cast<uint8_t>(reg & 0xFF)
  };
  
  if (!this->write(buffer, 2) || !this->read(val, 1)) {
    ESP_LOGD(TAG, "Failed to read 16-bit register 0x%04X", reg);
    return false;
  }
  
  return true;
}

#endif

}  // namespace tab5_camera
}  // namespace esphome

#endif  // USE_ESP32









