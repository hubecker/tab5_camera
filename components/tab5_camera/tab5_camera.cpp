#include "tab5_camera.h"
#include "esphome/core/log.h"
#include "esphome/core/helpers.h"
#include "esphome/core/hal.h"
#include "esp_timer.h"

#ifdef USE_ESP32

static const char *const TAG = "tab5_camera";

// Constantes de configuration pour Tab5
#define TAB5_CAMERA_H_RES 640
#define TAB5_CAMERA_V_RES 480
#define TAB5_MIPI_CSI_LANE_BITRATE_MBPS 200
#define TAB5_ISP_CLOCK_HZ 80000000  // 80MHz
#define TAB5_STREAMING_STACK_SIZE 8192
#define TAB5_FRAME_QUEUE_LENGTH 16

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
  
  ESP_LOGI(TAG, "Step 1: Creating synchronization objects");
  
  // Création des objets de synchronisation
  this->frame_ready_semaphore_ = xSemaphoreCreateBinary();
  if (!this->frame_ready_semaphore_) {
    ESP_LOGE(TAG, "Failed to create frame ready semaphore");
    this->mark_failed();
    return;
  }
  ESP_LOGI(TAG, "Frame semaphore created successfully");
  
  this->frame_queue_ = xQueueCreate(TAB5_FRAME_QUEUE_LENGTH, sizeof(FrameData));
  if (!this->frame_queue_) {
    ESP_LOGE(TAG, "Failed to create frame queue");
    this->mark_failed();
    return;
  }
  ESP_LOGI(TAG, "Frame queue created successfully");
  
  // Configuration du pin de l'horloge externe
  if (this->external_clock_pin_ > 0) {
    ESP_LOGD(TAG, "Configuring external clock on GPIO%u at %u Hz", 
             this->external_clock_pin_, this->external_clock_frequency_);
  }
  
  ESP_LOGI(TAG, "Step 2: Initializing camera");
  
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
  
  // Debug des macros disponibles
  ESP_LOGCONFIG(TAG, "  Platform Debug:");
#ifdef USE_ESP32
  ESP_LOGCONFIG(TAG, "    USE_ESP32: YES");
#else
  ESP_LOGCONFIG(TAG, "    USE_ESP32: NO");
#endif

#ifdef CONFIG_IDF_TARGET_ESP32P4
  ESP_LOGCONFIG(TAG, "    CONFIG_IDF_TARGET_ESP32P4: YES");
#else
  ESP_LOGCONFIG(TAG, "    CONFIG_IDF_TARGET_ESP32P4: NO");
#endif

#ifdef HAS_ESP32_P4_CAMERA
  ESP_LOGCONFIG(TAG, "    HAS_ESP32_P4_CAMERA: YES");
  ESP_LOGCONFIG(TAG, "  Resolution: %dx%d", TAB5_CAMERA_H_RES, TAB5_CAMERA_V_RES);
  if (this->external_clock_pin_ > 0) {
    ESP_LOGCONFIG(TAG, "  External Clock Pin: GPIO%u", this->external_clock_pin_);
    ESP_LOGCONFIG(TAG, "  External Clock Frequency: %u Hz", this->external_clock_frequency_);
  }
  ESP_LOGCONFIG(TAG, "  Frame Buffer Size: %zu bytes", this->frame_buffer_size_);
  ESP_LOGCONFIG(TAG, "  Streaming Support: Available");
  ESP_LOGCONFIG(TAG, "  I2C Address: 0x%02X", this->get_i2c_address());
  
  if (this->reset_pin_) {
    LOG_PIN("  Reset Pin: ", this->reset_pin_);
  }
#else
  ESP_LOGCONFIG(TAG, "    HAS_ESP32_P4_CAMERA: NO");
  ESP_LOGCONFIG(TAG, "  Status: ESP32-P4 MIPI-CSI API not available");
#endif
  
  if (this->is_failed()) {
    ESP_LOGCONFIG(TAG, "  Setup Failed");
  }
}

float Tab5Camera::get_setup_priority() const {
  return setup_priority::HARDWARE - 1.0f;
}


bool Tab5Camera::init_ldo_() {
  if (this->ldo_initialized_) {
    ESP_LOGI(TAG, "LDO already initialized, skipping");
    return true;
  }
  
  ESP_LOGI(TAG, "Initializing MIPI LDO regulator");
  
  // Configuration plus flexible pour le LDO
  esp_ldo_channel_config_t ldo_mipi_phy_config = {
    .chan_id = 3,  // LDO channel ID pour MIPI
    .voltage_mv = 2500,  // 2.5V pour MIPI PHY
  };
  
  esp_err_t ret = esp_ldo_acquire_channel(&ldo_mipi_phy_config, &this->ldo_mipi_phy_);
  if (ret != ESP_OK) {
    ESP_LOGW(TAG, "Failed to acquire MIPI LDO channel: %s - trying to continue anyway", esp_err_to_name(ret));
    // Ne pas échouer complètement, certains boards peuvent ne pas avoir de LDO MIPI dédié
    this->ldo_mipi_phy_ = nullptr;
  } else {
    ESP_LOGI(TAG, "MIPI LDO regulator acquired successfully");
  }
  
  this->ldo_initialized_ = true;
  return true;  // Retourner true même si le LDO échoue
}

bool Tab5Camera::init_sensor_() {
  if (this->sensor_initialized_) {
    ESP_LOGI(TAG, "Sensor already initialized, skipping");
    return true;
  }
  
  ESP_LOGI(TAG, "🔧 Initializing SmartSens SC2356 sensor at I2C address 0x%02X", this->get_i2c_address());
  
  // === ÉTAPE 1: Test de communication I2C ===
  ESP_LOGI(TAG, "Step 1: Testing I2C communication with SC2356");
  
  // Test avec les registres typiques SC2356
  uint16_t test_regs[] = {0x3107, 0x3108, 0x3000, 0x3001, 0x3002};
  bool i2c_responding = false;
  
  for (int i = 0; i < sizeof(test_regs)/sizeof(test_regs[0]); i++) {
    uint8_t test_value;
    if (this->read_sensor_register_(test_regs[i], &test_value)) {
      ESP_LOGI(TAG, "✅ SC2356 responds: reg 0x%04X = 0x%02X", test_regs[i], test_value);
      i2c_responding = true;
      break;
    } else {
      ESP_LOGD(TAG, "❌ No response from reg 0x%04X", test_regs[i]);
    }
  }
  
  if (!i2c_responding) {
    ESP_LOGW(TAG, "⚠️  SC2356 sensor not responding to I2C - continuing anyway");
  }
  
  // === ÉTAPE 2: Configuration de base SC2356 ===
  ESP_LOGI(TAG, "Step 2: Configuring SC2356 basic registers");
  
  // Configuration typique pour les capteurs SmartSens SC23xx
  struct sensor_reg {
    uint16_t reg;
    uint8_t value;
    const char* description;
  };
  
  // Configuration de base pour SC2356
  sensor_reg sc2356_init_regs[] = {
    // Software reset
    {0x0103, 0x01, "Software reset"},
    
    // System control
    {0x0100, 0x00, "Standby mode"},
    
    // Clock configuration  
    {0x3001, 0x00, "System control 1"},
    {0x3002, 0x00, "System control 2"},
    {0x3003, 0x08, "System control 3"},
    
    // PLL configuration (ajuster selon horloge externe 24MHz)
    {0x3004, 0x03, "PLL control 1"},
    {0x3005, 0x20, "PLL control 2"},
    {0x3006, 0x91, "PLL control 3"},
    
    // Output format configuration
    {0x3010, 0x01, "MIPI control"},
    {0x3011, 0x06, "MIPI lanes (2 lanes)"},
    {0x3012, 0x80, "MIPI clock configuration"},
    
    // Frame size configuration pour 640x480
    {0x3208, 0x02, "Frame width high (640)"},
    {0x3209, 0x80, "Frame width low"},
    {0x320A, 0x01, "Frame height high (480)"},
    {0x320B, 0xE0, "Frame height low"},
    
    // Output window
    {0x3808, 0x02, "Output width high"},
    {0x3809, 0x80, "Output width low"},
    {0x380A, 0x01, "Output height high"},
    {0x380B, 0xE0, "Output height low"},
    
    // Format control
    {0x4300, 0x30, "Format control - RAW8"},
    
    // ISP control
    {0x5000, 0x06, "ISP control"},
    {0x5001, 0x00, "ISP control"},
  };
  
  // Attendre après reset
  vTaskDelay(50 / portTICK_PERIOD_MS);
  
  // Écriture des registres de configuration
  int successful_writes = 0;
  int total_regs = sizeof(sc2356_init_regs) / sizeof(sc2356_init_regs[0]);
  
  for (int i = 0; i < total_regs; i++) {
    if (this->write_sensor_register_(sc2356_init_regs[i].reg, sc2356_init_regs[i].value)) {
      ESP_LOGD(TAG, "✅ SC2356[0x%04X] = 0x%02X (%s)", 
               sc2356_init_regs[i].reg, sc2356_init_regs[i].value, sc2356_init_regs[i].description);
      successful_writes++;
    } else {
      ESP_LOGW(TAG, "❌ Failed to write SC2356[0x%04X] = 0x%02X (%s)", 
               sc2356_init_regs[i].reg, sc2356_init_regs[i].value, sc2356_init_regs[i].description);
    }
    
    // Petit délai entre les écritures
    vTaskDelay(1);
  }
  
  ESP_LOGI(TAG, "SC2356 configuration: %d/%d registers written successfully", successful_writes, total_regs);
  
  // === ÉTAPE 3: Démarrage du streaming ===
  ESP_LOGI(TAG, "Step 3: Starting SC2356 streaming mode");
  
  // Attendre la stabilisation
  vTaskDelay(100 / portTICK_PERIOD_MS);
  
  // Démarrer le streaming
  if (this->write_sensor_register_(0x0100, 0x01)) {
    ESP_LOGI(TAG, "✅ SC2356 streaming mode enabled");
  } else {
    ESP_LOGW(TAG, "❌ Failed to enable SC2356 streaming mode");
  }
  
  // Attendre que le capteur démarre
  vTaskDelay(100 / portTICK_PERIOD_MS);
  
  this->sensor_initialized_ = true;
  ESP_LOGI(TAG, "✅ SC2356 sensor initialization completed");
  return true;
}

void Tab5Camera::debug_camera_status() {
  ESP_LOGI(TAG, "=== Camera Debug Status ===");
  ESP_LOGI(TAG, "Camera initialized: %s", this->camera_initialized_ ? "YES" : "NO");
  ESP_LOGI(TAG, "Streaming active: %s", this->streaming_active_ ? "YES" : "NO");
  ESP_LOGI(TAG, "Frame buffer: %p", this->frame_buffer_);
  ESP_LOGI(TAG, "Frame buffer size: %zu bytes", this->frame_buffer_size_);
  ESP_LOGI(TAG, "Camera handle: %p", this->cam_handle_);
  ESP_LOGI(TAG, "ISP handle: %p", this->isp_proc_);
  ESP_LOGI(TAG, "Free heap: %lu bytes", esp_get_free_heap_size());
  ESP_LOGI(TAG, "Free PSRAM: %lu bytes", heap_caps_get_free_size(MALLOC_CAP_SPIRAM));
  ESP_LOGI(TAG, "I2C address: 0x%02X", this->get_i2c_address());
  ESP_LOGI(TAG, "===========================");
}

bool Tab5Camera::init_camera_() {
  if (this->camera_initialized_) {
    ESP_LOGI(TAG, "Camera already initialized, skipping");
    return true;
  }

  ESP_LOGI(TAG, "Starting camera initialization for '%s'", this->name_.c_str());

  // Étape 1: Initialisation du LDO MIPI
  ESP_LOGI(TAG, "Step 2.1: Initializing MIPI LDO");
  if (!this->init_ldo_()) {
    ESP_LOGE(TAG, "Failed to initialize MIPI LDO");
    return false;
  }
  ESP_LOGI(TAG, "MIPI LDO initialized successfully");

  // Étape 2: Reset de la caméra si pin disponible
  if (this->reset_pin_) {
    ESP_LOGI(TAG, "Step 2.2: Executing camera reset sequence");
    this->reset_pin_->setup();
    this->reset_pin_->digital_write(false);
    delayMicroseconds(10000);  // 10ms
    this->reset_pin_->digital_write(true);
    delayMicroseconds(10000);  // 10ms
    ESP_LOGI(TAG, "Camera reset completed");
  } else {
    ESP_LOGI(TAG, "Step 2.2: No reset pin configured, skipping reset");
  }

  // Étape 3: Initialisation du capteur I2C
  ESP_LOGI(TAG, "Step 2.3: Initializing camera sensor");
  if (!this->init_sensor_()) {
    ESP_LOGE(TAG, "Failed to initialize camera sensor");
    return false;
  }
  ESP_LOGI(TAG, "Camera sensor initialized successfully");

  // Étape 4: Allocation du frame buffer
  ESP_LOGI(TAG, "Step 2.4: Allocating frame buffer");

  // Calcul de la taille réelle requise
  this->frame_buffer_size_ = TAB5_CAMERA_H_RES * TAB5_CAMERA_V_RES * 2; // RGB565 = 2 bytes par pixel

  // Arrondi sur un multiple de 64 (cache line size)
  this->frame_buffer_size_ = (this->frame_buffer_size_ + 63) & ~63;

  ESP_LOGI(TAG, "Aligned frame buffer size: %zu bytes", this->frame_buffer_size_);

  // Allocation alignée en PSRAM
  this->frame_buffer_ = heap_caps_aligned_alloc(64, this->frame_buffer_size_, MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT);
  if (!this->frame_buffer_) {
    ESP_LOGE(TAG, "Failed to allocate aligned frame buffer in PSRAM (%zu bytes) - trying regular RAM", this->frame_buffer_size_);

    // Essai avec RAM normale
    this->frame_buffer_ = heap_caps_aligned_alloc(64, this->frame_buffer_size_, MALLOC_CAP_DMA | MALLOC_CAP_8BIT);
    if (!this->frame_buffer_) {
      ESP_LOGE(TAG, "Failed to allocate aligned frame buffer in regular RAM also");
      return false;
    }
  }

  ESP_LOGI(TAG, "Frame buffer allocated successfully at %p", this->frame_buffer_);
  
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
  csi_config.queue_items = 32;

  ESP_LOGI(TAG, "🔧 CSI config: queue_items=%d, h_res=%d, v_res=%d", 
         csi_config.queue_items, csi_config.h_res, csi_config.v_res);
  
  esp_err_t ret = esp_cam_new_csi_ctlr(&csi_config, &this->cam_handle_);
  if (ret != ESP_OK) {
    ESP_LOGE(TAG, "CSI controller init failed: %s", esp_err_to_name(ret));
    return false;
  }
  ESP_LOGI(TAG, "CSI controller created successfully");
  
  // Étape 6: Configuration des callbacks
  ESP_LOGI(TAG, "Step 2.6: Registering camera callbacks");
  esp_cam_ctlr_evt_cbs_t cbs = {
    .on_get_new_trans = Tab5Camera::camera_get_new_vb_callback,
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
  memset(this->frame_buffer_, 0xFF, this->frame_buffer_size_);
  esp_cache_msync(this->frame_buffer_, this->frame_buffer_size_, ESP_CACHE_MSYNC_FLAG_DIR_C2M);
  ESP_LOGI(TAG, "Frame buffer initialized");
  
  // Étape 10: Démarrage de la caméra
  ESP_LOGI(TAG, "Step 2.10: Starting camera controller");
  ret = esp_cam_ctlr_start(this->cam_handle_);
  if (ret != ESP_OK) {
    ESPN_LOGE(TAG, "Failed to start camera controller: %s", esp_err_to_name(ret));
    return false;
  }
  
  this->camera_initialized_ = true;
  ESP_LOGI(TAG, "Camera '%s' initialized successfully - all steps completed", this->name_.c_str());
  return true;
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
    
    if (this->ldo_mipi_phy_) {
      esp_ldo_release_channel(this->ldo_mipi_phy_);
      this->ldo_mipi_phy_ = nullptr;
    }
    
    this->camera_initialized_ = false;
    this->sensor_initialized_ = false;
    this->ldo_initialized_ = false;
    ESP_LOGD(TAG, "Camera '%s' deinitialized", this->name_.c_str());
  }
  
  // Nettoyage des objets de synchronisation
  if (this->frame_ready_semaphore_) {
    vSemaphoreDelete(this->frame_ready_semaphore_);
    this->frame_ready_semaphore_ = nullptr;
  }
  
  if (this->frame_queue_) {
    vQueueDelete(this->frame_queue_);
    this->frame_queue_ = nullptr;
  }
}

bool Tab5Camera::camera_get_new_vb_callback(esp_cam_ctlr_handle_t handle, esp_cam_ctlr_trans_t *trans, void *user_data) {
  Tab5Camera *camera = static_cast<Tab5Camera*>(user_data);
  if (!camera || !camera->streaming_active_) {
    return false;  // Arrêter si pas de caméra ou streaming arrêté
  }
  
  trans->buffer = camera->frame_buffer_;
  trans->buflen = camera->frame_buffer_size_;
  return true;  // Retourner true pour continuer
}

bool Tab5Camera::camera_get_finished_trans_callback(esp_cam_ctlr_handle_t handle, esp_cam_ctlr_trans_t *trans, void *user_data) {
  Tab5Camera *camera = static_cast<Tab5Camera*>(user_data);
  if (!camera) {
    ESP_LOGE(TAG, "camera_get_finished_trans_callback called with null user_data");
    return false;
  }

  // On crée une structure FrameData contenant les infos nécessaires
  FrameData frame;
  frame.buffer = camera->frame_buffer_;
  frame.size = camera->frame_buffer_size_;
  frame.timestamp = esp_timer_get_time();  // Optionnel, timestamp en microsecondes
  frame.valid = true;
  
  // On essaie de mettre la frame dans la queue (non bloquant)
  BaseType_t ret = xQueueSendFromISR(camera->frame_queue_, &frame, NULL);
  if (ret == pdTRUE) {
    // On signale qu'une frame est prête via le sémaphore (release)
    xSemaphoreGiveFromISR(camera->frame_ready_semaphore_, NULL);
  } else {
    // Queue pleine, on perd la frame (il faudrait envisager un overwrite si c'est critique)
    ESP_LOGW(TAG, "Frame queue full, dropping frame");
  }

  // Retourne false pour indiquer qu'on ne garde pas la possession exclusive de la buffer (selon doc IDF)
  return false;
}

bool Tab5Camera::take_snapshot() {
  if (!this->camera_initialized_) {
    ESP_LOGE(TAG, "Camera '%s' not initialized", this->name_.c_str());
    return false;
  }
  
  ESP_LOGD(TAG, "Taking snapshot with camera '%s'", this->name_.c_str());
  
  esp_cam_ctlr_trans_t trans = {
    .buffer = this->frame_buffer_,
    .buflen = this->frame_buffer_size_,
  };
  
  esp_err_t ret = esp_cam_ctlr_receive(this->cam_handle_, &trans, 1000 / portTICK_PERIOD_MS);
  if (ret != ESP_OK) {
    ESP_LOGW(TAG, "Camera '%s' capture failed: %s", this->name_.c_str(), esp_err_to_name(ret));
    return false;
  }
  
  ESP_LOGD(TAG, "Camera '%s' snapshot taken, size: %zu bytes", this->name_.c_str(), trans.buflen);
  
  // Synchronisation du cache
  esp_cache_msync(this->frame_buffer_, this->frame_buffer_size_, ESP_CACHE_MSYNC_FLAG_DIR_M2C);
  
  // Appel des callbacks
  this->on_frame_callbacks_.call(static_cast<uint8_t*>(this->frame_buffer_), this->frame_buffer_size_);
  
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
  
  ESP_LOGD(TAG, "Starting streaming for camera '%s'", this->name_.c_str());
  
  // AJOUTER CE DEBUG :
  this->debug_camera_status();
  
  // ===== TEST DE CAPTURE UNIQUE AVANT STREAMING =====
  ESP_LOGW(TAG, "🧪 Testing single capture before streaming...");
  
  esp_cam_ctlr_trans_t test_trans = {
    .buffer = this->frame_buffer_,
    .buflen = this->frame_buffer_size_,
  };
  
  esp_err_t test_ret = esp_cam_ctlr_receive(this->cam_handle_, &test_trans, 3000 / portTICK_PERIOD_MS);
  
  if (test_ret == ESP_OK) {
    ESP_LOGI(TAG, "✅ SINGLE CAPTURE SUCCESS! Size: %zu bytes", test_trans.buflen);
    
    uint8_t* buf = static_cast<uint8_t*>(test_trans.buffer);
    ESP_LOGW(TAG, "🔍 Sample data: %02x %02x %02x %02x", buf[0], buf[1], buf[2], buf[3]);
    
  } else {
    ESP_LOGE(TAG, "❌ SINGLE CAPTURE FAILED: %s", esp_err_to_name(test_ret));
    ESP_LOGE(TAG, "❌ This indicates camera/sensor is not working properly");
  }
  
  this->streaming_should_stop_ = false;
  this->streaming_active_ = true;
  
  // Création de la tâche de streaming avec priorité réduite
  BaseType_t result = xTaskCreate(
    Tab5Camera::streaming_task,
    "tab5_streaming",
    TAB5_STREAMING_STACK_SIZE,
    this,
    3,  // PRIORITÉ RÉDUITE
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
  
  ESP_LOGD(TAG, "Stopping camera '%s' streaming...", this->name_.c_str());
  
  this->streaming_should_stop_ = true;
  
  // Attendre l'arrêt de la tâche
  if (this->streaming_task_handle_) {
    // Signal pour débloquer la tâche si elle attend
    xSemaphoreGive(this->frame_ready_semaphore_);
    
    // Attendre la fin de la tâche
    uint32_t timeout = 0;
    while (this->streaming_active_ && timeout < 50) {  // 5 secondes max
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
  ESP_LOGD(TAG, "Streaming loop started for camera '%s'", this->name_.c_str());
  
  esp_cam_ctlr_trans_t trans = {
    .buffer = this->frame_buffer_,
    .buflen = this->frame_buffer_size_,
  };
  
  uint32_t frame_count = 0;
  uint32_t error_count = 0;
  
  while (!this->streaming_should_stop_) {
    // TIMEOUT RÉDUIT pour éviter l'accumulation
    esp_err_t ret = esp_cam_ctlr_receive(this->cam_handle_, &trans, 50 / portTICK_PERIOD_MS);
    
    if (ret == ESP_OK) {
      frame_count++;
      error_count = 0;
      
      ESP_LOGV(TAG, "📷 Frame %lu captured, size: %zu bytes", frame_count, trans.buflen);
      
      // Synchronisation du cache
      esp_cache_msync(this->frame_buffer_, this->frame_buffer_size_, ESP_CACHE_MSYNC_FLAG_DIR_M2C);
      
      // Appel des callbacks
      this->on_frame_callbacks_.call(static_cast<uint8_t*>(this->frame_buffer_), trans.buflen);
      
      // Log périodique
      if (frame_count % 10 == 0) {  // Plus fréquent pour debug
        ESP_LOGI(TAG, "✅ Streaming: %lu frames captured", frame_count);
      }
      
    } else if (ret == ESP_ERR_TIMEOUT) {
      // Timeout normal, continuer
      ESP_LOGV(TAG, "Frame timeout (normal)");
      
    } else {
      error_count++;
      ESP_LOGW(TAG, "❌ Frame error: %s (count: %lu)", esp_err_to_name(ret), error_count);
      
      if (error_count > 5) {  // Réduit le seuil
        ESP_LOGW(TAG, "Too many errors, pausing...");
        vTaskDelay(500 / portTICK_PERIOD_MS);  // Pause plus courte
        error_count = 0;
      }
    }
    
    // Pause entre captures
    vTaskDelay(10 / portTICK_PERIOD_MS);  // Pause plus longue (100ms)
  }
  
  this->streaming_active_ = false;
  ESP_LOGI(TAG, "Streaming ended - %lu frames total", frame_count);
  vTaskDelete(nullptr);
}




#endif

}  // namespace tab5_camera
}  // namespace esphome

#endif  // USE_ESP32








