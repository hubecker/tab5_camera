#include "tab5_camera.h"
#include "esphome/core/log.h"
#include "esphome/core/helpers.h"
#include "esphome/core/hal.h"
#include "esp_timer.h"
#include "esp_cam_ctlr_csi.h"

// Ajout des includes pour les capteurs ESP-IDF

#include "esp_cam_sensor.h"

#ifdef CONFIG_CAMERA_OV5645
#include "ov5645.h"
#endif

#ifdef CONFIG_CAMERA_SC2336
#include "sc2336.h"
#endif

#ifdef USE_ESP32

static const char *const TAG = "tab5_camera";

// Constantes de configuration pour Tab5
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
  ESP_LOGCONFIG(TAG, "  I2C Address: 0x%02X", this->address_);
  
  if (this->reset_pin_) {
    LOG_PIN("  Reset Pin: ", this->reset_pin_);
  }
  
  // Afficher le type de capteur détecté
  if (this->sensor_type_ != SENSOR_UNKNOWN) {
    ESP_LOGCONFIG(TAG, "  Detected Sensor: %s", this->get_sensor_name());
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

#ifdef HAS_ESP32_P4_CAMERA

const char* Tab5Camera::get_sensor_name() const {
  switch (this->sensor_type_) {
    case SENSOR_OV5645: return "OV5645";
    case SENSOR_SC2336: return "SC2336";
    case SENSOR_GENERIC: return "Generic/Unknown";
    default: return "Unknown";
  }
}

bool Tab5Camera::detect_and_init_sensor_() {
  ESP_LOGI(TAG, "🔍 Starting sensor detection and initialization...");
  
  // Étape 1: Configuration du bus I2C/SCCB
  i2c_master_bus_config_t i2c_bus_config = {
    .clk_source = I2C_CLK_SRC_DEFAULT,
    .i2c_port = I2C_NUM_0,
    .scl_io_num = this->scl_pin_,
    .sda_io_num = this->sda_pin_,
    .glitch_ignore_cnt = 7,
    .flags = {
      .enable_internal_pullup = true,
    }
  };
  
  esp_err_t ret = i2c_new_master_bus(&i2c_bus_config, &this->i2c_bus_handle_);
  if (ret != ESP_OK) {
    ESP_LOGE(TAG, "❌ Failed to create I2C master bus: %s", esp_err_to_name(ret));
    return false;
  }
  ESP_LOGI(TAG, "✅ I2C master bus created successfully");

  // Étape 2: Configuration SCCB avec l'adresse I2C
  sccb_i2c_config_t sccb_config = {
    .dev_addr_length = I2C_ADDR_BIT_LEN_7,
    .device_address = this->address_,
    .scl_speed_hz = 100000,  // 100kHz pour la compatibilité
  };
  
  ret = sccb_new_i2c_io(this->i2c_bus_handle_, &sccb_config, &this->sccb_handle_);
  if (ret != ESP_OK) {
    ESP_LOGE(TAG, "❌ Failed to create SCCB interface: %s", esp_err_to_name(ret));
    return false;
  }
  ESP_LOGI(TAG, "✅ SCCB interface created successfully");

  // Étape 3: Configuration du capteur pour la détection
  esp_cam_sensor_config_t sensor_config = {
    .sccb_handle = this->sccb_handle_,
    .reset_pin = this->reset_pin_ ? this->reset_pin_->get_pin() : -1,
    .pwdn_pin = -1,  // Power down pin non utilisé
    .xclk_pin = this->external_clock_pin_ > 0 ? this->external_clock_pin_ : -1,
  };

  // Étape 4: Tentative de détection des capteurs supportés
  this->sensor_type_ = SENSOR_UNKNOWN;
  this->cam_sensor_ = nullptr;

#ifdef CONFIG_CAMERA_OV5645
  ESP_LOGI(TAG, "🔍 Trying to detect OV5645 sensor...");
  this->cam_sensor_ = ov5645_detect(&sensor_config);
  if (this->cam_sensor_ != nullptr) {
    this->sensor_type_ = SENSOR_OV5645;
    ESP_LOGI(TAG, "📷 OV5645 sensor detected and initialized!");
    return true;
  }
  ESP_LOGD(TAG, "OV5645 not detected");
#endif

#ifdef CONFIG_CAMERA_SC2336
  ESP_LOGI(TAG, "🔍 Trying to detect SC2336 sensor...");
  this->cam_sensor_ = sc2336_detect(&sensor_config);
  if (this->cam_sensor_ != nullptr) {
    this->sensor_type_ = SENSOR_SC2336;
    ESP_LOGI(TAG, "📷 SC2336 sensor detected and initialized!");
    return true;
  }
  ESP_LOGD(TAG, "SC2336 not detected");
#endif

  // Étape 5: Si aucun capteur spécifique détecté, utiliser la méthode générique
  ESP_LOGW(TAG, "⚠️ No specific sensor detected, falling back to generic detection");
  return this->init_generic_sensor_();
}

bool Tab5Camera::init_generic_sensor_() {
  ESP_LOGI(TAG, "🔧 Initializing generic sensor configuration...");
  
  // Test de communication I2C de base
  uint8_t test_data;
  bool sensor_detected = false;
  
  // Test de plusieurs registres communs
  const uint8_t test_regs[] = {0x00, 0x01, 0x02, 0x0A, 0x0B, 0x0C, 0x0D};
  for (size_t i = 0; i < sizeof(test_regs); i++) {
    if (this->read_byte(test_regs[i], &test_data)) {
      ESP_LOGI(TAG, "Sensor responded: reg 0x%02X = 0x%02X", test_regs[i], test_data);
      sensor_detected = true;
    }
  }
  
  if (!sensor_detected) {
    ESP_LOGE(TAG, "❌ No sensor detected at I2C address 0x%02X - check wiring!", this->address_);
    return false;
  }
  
  // Tentative d'identification du capteur
  uint8_t id_reg_1, id_reg_2;
  if (this->read_byte(0x00, &id_reg_1) && this->read_byte(0x01, &id_reg_2)) {
    uint16_t sensor_id = (id_reg_1 << 8) | id_reg_2;
    ESP_LOGI(TAG, "🔍 Sensor ID: 0x%04X (reg 0x00=0x%02X, reg 0x01=0x%02X)", 
             sensor_id, id_reg_1, id_reg_2);
    
    // Identification basée sur l'ID
    switch (sensor_id) {
      case 0x00A2: ESP_LOGI(TAG, "📷 Detected: Possible OmniVision sensor (partial ID match)"); break;
      case 0x2640: ESP_LOGI(TAG, "📷 Detected: OV2640 sensor"); break;
      case 0x5640: ESP_LOGI(TAG, "📷 Detected: OV5640 sensor"); break;
      case 0x5645: ESP_LOGI(TAG, "📷 Detected: OV5645 sensor (but driver not enabled)"); break;
      default: ESP_LOGI(TAG, "📷 Unknown sensor - will use generic configuration"); break;
    }
  }
  
  // Configuration minimale pour démarrer la capture
  ESP_LOGI(TAG, "🔧 Configuring sensor for basic operation...");
  
  // Configuration basique générique
  const struct {
    uint8_t reg;
    uint8_t val;
    const char* desc;
  } basic_config[] = {
    {0x09, 0x00, "System control"},         // Mode normal
    {0x15, 0x00, "Output format"},          // Format par défaut
    {0x3A, 0x04, "TSLB register"},          // Output sequence
  };
  
  for (size_t i = 0; i < sizeof(basic_config) / sizeof(basic_config[0]); i++) {
    ESP_LOGD(TAG, "Setting %s: reg 0x%02X = 0x%02X", 
             basic_config[i].desc, basic_config[i].reg, basic_config[i].val);
    
    if (!this->write_byte(basic_config[i].reg, basic_config[i].val)) {
      ESP_LOGW(TAG, "Failed to write register 0x%02X - continuing anyway", basic_config[i].reg);
    }
    
    vTaskDelay(10 / portTICK_PERIOD_MS);
  }
  
  // Vérification que le capteur répond toujours
  uint8_t verify_reg;
  if (this->read_byte(0x00, &verify_reg)) {
    ESP_LOGI(TAG, "✅ Sensor still responsive after configuration (reg 0x00 = 0x%02X)", verify_reg);
  } else {
    ESP_LOGW(TAG, "⚠️ Sensor not responding after configuration");
  }
  
  this->sensor_type_ = SENSOR_GENERIC;
  ESP_LOGI(TAG, "ℹ️ Using minimal sensor configuration - full config needed for proper images");
  ESP_LOGI(TAG, "✅ Generic sensor initialized with basic configuration");
  
  return true;
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
    this->ldo_mipi_phy_ = nullptr;
  } else {
    ESP_LOGI(TAG, "MIPI LDO regulator acquired successfully");
  }
  
  this->ldo_initialized_ = true;
  return true;
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

  // Étape 3: Détection et initialisation du capteur avec ESP-IDF
  ESP_LOGI(TAG, "Step 2.3: Detecting and initializing camera sensor");
  if (!this->detect_and_init_sensor_()) {
    ESP_LOGE(TAG, "Failed to detect and initialize camera sensor");
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
  memset(this->frame_buffer_, 0xFF, this->frame_buffer_size_);
  esp_cache_msync(this->frame_buffer_, this->frame_buffer_size_, ESP_CACHE_MSYNC_FLAG_DIR_C2M);
  ESP_LOGI(TAG, "Frame buffer initialized");
  
  // Étape 10: Démarrage de la caméra
  ESP_LOGI(TAG, "Step 2.10: Starting camera controller");
  ret = esp_cam_ctlr_start(this->cam_handle_);
  if (ret != ESP_OK) {
    ESP_LOGE(TAG, "Failed to start camera controller: %s", esp_err_to_name(ret));
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
    
    // Nettoyage du capteur ESP-IDF
    if (this->cam_sensor_) {
      esp_cam_sensor_del_dev(this->cam_sensor_);
      this->cam_sensor_ = nullptr;
    }
    
    if (this->sccb_handle_) {
      esp_sccb_del_i2c_io(this->sccb_handle_);
      this->sccb_handle_ = nullptr;
    }
    
    if (this->i2c_bus_handle_) {
      i2c_del_master_bus(this->i2c_bus_handle_);
      this->i2c_bus_handle_ = nullptr;
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
    this->ldo_initialized_ = false;
    this->sensor_type_ = SENSOR_UNKNOWN;
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

// Callback de fin de transaction avec diagnostics
bool Tab5Camera::camera_get_finished_trans_callback(esp_cam_ctlr_handle_t handle, esp_cam_ctlr_trans_t *trans, void *user_data) {
  Tab5Camera *camera = static_cast<Tab5Camera*>(user_data);
  if (!camera || !trans->buffer) {
    ESP_LOGE(TAG, "camera_get_finished_trans_callback called with invalid data");
    return false;
  }

  // Compteur de frames pour diagnostics
  static uint32_t frame_count = 0;
  frame_count++;
  
  ESP_LOGI(TAG, "📸 Frame #%lu received: %zu bytes (expected: %zu)", 
           frame_count, trans->received_size, camera->frame_buffer_size_);

  // Vérification de la taille des données
  if (trans->received_size == 0) {
    ESP_LOGW(TAG, "⚠️ Frame #%lu is empty - sensor might not be generating data", frame_count);
    return false;
  }
  
  if (trans->received_size < 1000) {
    ESP_LOGW(TAG, "⚠️ Frame #%lu size is very small (%zu bytes)", frame_count, trans->received_size);
  }

  // Synchronisation du cache pour la frame reçue
  esp_cache_msync(trans->buffer, trans->received_size, ESP_CACHE_MSYNC_FLAG_DIR_M2C);
  
  // Vérification du contenu des premiers bytes (diagnostics)
  uint8_t *data = static_cast<uint8_t*>(trans->buffer);
  ESP_LOGD(TAG, "Frame #%lu first bytes: %02X %02X %02X %02X %02X %02X %02X %02X", 
           frame_count, data[0], data[1], data[2], data[3], data[4], data[5], data[6], data[7]);
  
  // Création d'une structure FrameData avec les vraies données reçues
  FrameData frame;
  frame.buffer = trans->buffer;
  frame.size = trans->received_size;
  frame.timestamp = esp_timer_get_time();
  
  // Envoi non-bloquant vers la queue applicative
  BaseType_t ret = xQueueSendFromISR(camera->frame_queue_, &frame, NULL);
  if (ret == pdTRUE) {
    // Signaler qu'une frame est disponible
    xSemaphoreGiveFromISR(camera->frame_ready_semaphore_, NULL);
    ESP_LOGV(TAG, "Frame #%lu queued successfully", frame_count);
  } else {
    ESP_LOGD(TAG, "Application frame queue full, dropping frame #%lu", frame_count);
  }

  return false
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
  
  ESP_LOGD(TAG, "Camera '%s' snapshot taken, size: %zu bytes", this->name_.c_str(), trans.received_size);
  
  // Synchronisation du cache
  esp_cache_msync(this->frame_buffer_, trans.received_size, ESP_CACHE_MSYNC_FLAG_DIR_M2C);
  
  // Appel des callbacks avec la taille réelle reçue
  this->on_frame_callbacks_.call(static_cast<uint8_t*>(this->frame_buffer_), trans.received_size);
  
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
  
  this->streaming_should_stop_ = false;
  this->streaming_active_ = true;
  
  // Pré-envoyer une transaction initiale pour démarrer le processus
  ESP_LOGI(TAG, "Sending initial transaction to start streaming");
  esp_cam_ctlr_trans_t initial_trans = {
    .buffer = this->frame_buffer_,
    .buflen = this->frame_buffer_size_,
  };
  
  esp_err_t ret = esp_cam_ctlr_receive(this->cam_handle_, &initial_trans, 1000 / portTICK_PERIOD_MS);
  if (ret != ESP_OK) {
    ESP_LOGW(TAG, "Failed to send initial transaction: %s", esp_err_to_name(ret));
    // Continue anyway, les callbacks peuvent compenser
  }
  
  // Création de la tâche de streaming
  BaseType_t result = xTaskCreate(
    Tab5Camera::streaming_task,
    "tab5_streaming",
    TAB5_STREAMING_STACK_SIZE,
    this,
    5,  // Priorité élevée pour le streaming
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

// NOUVELLE méthode de streaming - approche différente pour éviter le conflit de queue
void Tab5Camera::streaming_loop_() {
  ESP_LOGD(TAG, "Streaming loop started for camera '%s' (callback-based)", this->name_.c_str());

  // Au lieu d'appeler esp_cam_ctlr_receive() en boucle (qui sature la queue),
  // on utilise uniquement les callbacks pour récupérer les frames
  
  while (!this->streaming_should_stop_) {
    // Attendre qu'une frame soit disponible via le callback
    if (xSemaphoreTake(this->frame_ready_semaphore_, 100 / portTICK_PERIOD_MS) == pdTRUE) {
      
      // Récupérer les données de frame depuis notre queue applicative
      FrameData frame;
      if (xQueueReceive(this->frame_queue_, &frame, 0) == pdTRUE) {
        ESP_LOGV(TAG, "Frame received from callback, size: %zu bytes", frame.size);
        
        // Appel des callbacks avec les données reçues
        this->on_frame_callbacks_.call(static_cast<uint8_t*>(frame.buffer), frame.size);
      }
    }
    
    // Petite pause pour éviter de surcharger le système
    vTaskDelay(1 / portTICK_PERIOD_MS);
  }

  this->streaming_active_ = false;
  ESP_LOGD(TAG, "Streaming loop ended for camera '%s'", this->name_.c_str());
  vTaskDelete(nullptr);
}

#endif

}  // namespace tab5_camera
}  // namespace esphome

#endif  // USE_ESP32










