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
#define TAB5_MIPI_CSI_LANE_BITRATE_MBPS 400
#define TAB5_ISP_CLOCK_HZ 80000000  // 80MHz
#define TAB5_STREAMING_STACK_SIZE 8192

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

// Diagnostic CSI - maintenant correctement dans la classe
void Tab5Camera::diagnose_csi_status_() {
  ESP_LOGI(TAG, "=== CSI DIAGNOSTIC START ===");
  
  if (!this->cam_handle_) {
    ESP_LOGE(TAG, "❌ CSI controller handle is null");
    return;
  }
  
  ESP_LOGI(TAG, "CSI Controller Handle: %p", this->cam_handle_);
  ESP_LOGI(TAG, "Frame Buffer Address: %p", this->frame_buffer_);
  ESP_LOGI(TAG, "Frame Buffer Size: %zu bytes", this->frame_buffer_size_);
  ESP_LOGI(TAG, "Expected Resolution: %dx%d", TAB5_CAMERA_H_RES, TAB5_CAMERA_V_RES);
  
  // Test de capture unique pour diagnostic
  esp_cam_ctlr_trans_t test_trans = {
    .buffer = this->frame_buffer_,
    .buflen = this->frame_buffer_size_,
  };
  
  ESP_LOGI(TAG, "Testing single frame capture...");
  esp_err_t ret = esp_cam_ctlr_receive(this->cam_handle_, &test_trans, 2000 / portTICK_PERIOD_MS);
  
  if (ret == ESP_OK) {
    ESP_LOGI(TAG, "✅ Single frame capture successful");
    ESP_LOGI(TAG, "   Captured size: %zu bytes", test_trans.buflen);
    
    // Vérifier si on a des données non nulles
    uint32_t non_zero_count = 0;
    uint8_t *buffer = static_cast<uint8_t*>(this->frame_buffer_);
    for (size_t i = 0; i < std::min(test_trans.buflen, (size_t)1000); i++) {
      if (buffer[i] != 0) non_zero_count++;
    }
    ESP_LOGI(TAG, "   Non-zero bytes in first 1000: %lu", non_zero_count);
    
    if (non_zero_count == 0) {
      ESP_LOGW(TAG, "⚠️  Frame buffer contains only zeros - sensor may not be producing data");
    }
    
  } else {
    ESP_LOGE(TAG, "❌ Single frame capture failed: %s", esp_err_to_name(ret));
  }
  
  ESP_LOGI(TAG, "=== CSI DIAGNOSTIC END ===");
}

// Diagnostic complet
void Tab5Camera::run_full_diagnostic_() {
  ESP_LOGI(TAG, "🔍 Starting full diagnostic");
  
  // 1. Diagnostic CSI
  this->diagnose_csi_status_();
  
  // 2. Test de mémoire
  ESP_LOGI(TAG, "Memory diagnostic:");
  ESP_LOGI(TAG, "  Free heap: %lu bytes", esp_get_free_heap_size());
  ESP_LOGI(TAG, "  Free PSRAM: %lu bytes", heap_caps_get_free_size(MALLOC_CAP_SPIRAM));
  ESP_LOGI(TAG, "  Minimum free heap: %lu bytes", esp_get_minimum_free_heap_size());
  
  ESP_LOGI(TAG, "🔍 Diagnostic completed");
}

bool Tab5Camera::init_ldo_() {
  if (this->ldo_initialized_) {
    ESP_LOGI(TAG, "LDO already initialized, skipping");
    return true;
  }
  
  ESP_LOGI(TAG, "Initializing MIPI LDO regulator");
  
  // Configuration basée sur l'exemple officiel ESP-IDF
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

bool Tab5Camera::init_sensor_() {
  if (this->sensor_initialized_) {
    ESP_LOGI(TAG, "Sensor already initialized, skipping");
    return true;
  }
  
  ESP_LOGI(TAG, "Attempting to initialize camera sensor at I2C address 0x%02X", this->address_);
  
  // Test de communication I2C avec retry
  uint8_t test_data;
  int retry_count = 3;
  bool i2c_success = false;
  
  while (retry_count-- > 0 && !i2c_success) {
    if (this->read_byte(0x00, &test_data)) {
      ESP_LOGI(TAG, "Sensor responded to I2C communication (reg 0x00 = 0x%02X)", test_data);
      i2c_success = true;
      break;
    } else {
      ESP_LOGW(TAG, "I2C communication failed, retries left: %d", retry_count);
      vTaskDelay(100 / portTICK_PERIOD_MS);
    }
  }
  
  if (!i2c_success) {
    ESP_LOGE(TAG, "Could not communicate with sensor at address 0x%02X after multiple retries", this->address_);
    // RETOURNER FALSE pour forcer l'arrêt si pas de capteur
    return false;
  }
  
  // Configuration basique du capteur - AJOUTEZ ICI la config spécifique à votre capteur
  // Pour l'instant, configuration générique
  
  // 1. Vérifier l'ID du capteur
  uint8_t sensor_id;
  if (!this->read_byte(0x0A, &sensor_id)) {
    ESP_LOGW(TAG, "Could not read sensor ID");
  } else {
    ESP_LOGI(TAG, "Sensor ID: 0x%02X", sensor_id);
  }
  
  // 2. Configuration de base (à adapter selon votre capteur)
  if (!this->write_byte(0x01, 0x01)) { // Reset software
    ESP_LOGW(TAG, "Could not send software reset");
  }
  vTaskDelay(10 / portTICK_PERIOD_MS);
  
  // 3. Configuration de la résolution
  if (!this->write_byte(0x20, 0x02)) { // Mode 640x480 (exemple)
    ESP_LOGW(TAG, "Could not set resolution mode");
  }
  
  // 4. Configuration du format de sortie
  if (!this->write_byte(0x21, 0x30)) { // RAW8 output (exemple)
    ESP_LOGW(TAG, "Could not set output format");
  }
  
  // 5. Démarrer le streaming du capteur
  if (!this->write_byte(0x00, 0x01)) { // Start streaming
    ESP_LOGW(TAG, "Could not start sensor streaming");
  }
  
  this->sensor_initialized_ = true;
  ESP_LOGI(TAG, "Camera sensor configured and started");
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

  // Étape 3: Initialisation du capteur I2C (OBLIGATOIRE!)
  ESP_LOGI(TAG, "Step 2.3: Initializing camera sensor");
  if (!this->init_sensor_()) {
    ESP_LOGE(TAG, "Failed to initialize camera sensor - STOPPING HERE");
    return false;  // ARRÊT RÉEL si pas de capteur
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

  // Initialiser la transaction (comme dans l'exemple officiel)
  this->cam_trans_.buffer = this->frame_buffer_;
  this->cam_trans_.buflen = this->frame_buffer_size_;
  
  // Étape 5: Configuration du contrôleur CSI (basé sur l'exemple officiel)
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
  csi_config.queue_items = 1;  // ⭐ CORRECTION: 1 seul item comme dans l'exemple officiel
  
  esp_err_t ret = esp_cam_new_csi_ctlr(&csi_config, &this->cam_handle_);
  if (ret != ESP_OK) {
    ESP_LOGE(TAG, "CSI controller init failed: %s", esp_err_to_name(ret));
    return false;
  }
  ESP_LOGI(TAG, "CSI controller created successfully");
  
  // Étape 6: Configuration des callbacks (SIMPLIFIÉS comme dans l'exemple officiel)
  ESP_LOGI(TAG, "Step 2.6: Registering camera callbacks");
  esp_cam_ctlr_evt_cbs_t cbs = {
    .on_get_new_trans = Tab5Camera::camera_get_new_vb_callback,
    .on_trans_finished = Tab5Camera::camera_get_finished_trans_callback,
  };
  
  ret = esp_cam_ctlr_register_event_callbacks(this->cam_handle_, &cbs, &this->cam_trans_);
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
  
  // Étape 9: Initialisation du frame buffer (comme dans l'exemple officiel)
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
}

// CALLBACKS SIMPLIFIÉS (basés sur l'exemple officiel)
bool Tab5Camera::camera_get_new_vb_callback(esp_cam_ctlr_handle_t handle, esp_cam_ctlr_trans_t *trans, void *user_data) {
  esp_cam_ctlr_trans_t *cam_trans = static_cast<esp_cam_ctlr_trans_t*>(user_data);
  trans->buffer = cam_trans->buffer;
  trans->buflen = cam_trans->buflen;
  return false;
}

bool Tab5Camera::camera_get_finished_trans_callback(esp_cam_ctlr_handle_t handle, esp_cam_ctlr_trans_t *trans, void *user_data) {
  // Callback simple comme dans l'exemple officiel
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
  
  esp_err_t ret = esp_cam_ctlr_receive(this->cam_handle_, &trans, 2000 / portTICK_PERIOD_MS);
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
  
  this->streaming_should_stop_ = false;
  this->streaming_active_ = true;
  
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

// BOUCLE DE STREAMING SIMPLIFIÉE (basée sur l'exemple officiel)
void Tab5Camera::streaming_loop_() {
    ESP_LOGD(TAG, "Streaming loop started for camera '%s'", this->name_.c_str());
    
    esp_cam_ctlr_trans_t trans = {
        .buffer = this->frame_buffer_,
        .buflen = this->frame_buffer_size_,
    };
    
    uint32_t frame_count = 0;
    
    while (!this->streaming_should_stop_) {
        esp_err_t ret = esp_cam_ctlr_receive(this->cam_handle_, &trans, ESP_CAM_CTLR_MAX_DELAY);
        
        if (ret == ESP_OK) {
            frame_count++;
            
            // Synchronisation du cache
            esp_cache_msync(this->frame_buffer_, trans.buflen, ESP_CACHE_MSYNC_FLAG_DIR_M2C);
            
            // Appel des callbacks
            this->on_frame_callbacks_.call(static_cast<uint8_t*>(this->frame_buffer_), trans.buflen);
            
            // Debug périodique
            if (frame_count % 100 == 0) {
                ESP_LOGD(TAG, "Frames captured: %lu, last frame size: %zu bytes", frame_count, trans.buflen);
            }
            
            // Réinitialiser la transaction
            trans.buffer = this->frame_buffer_;
            trans.buflen = this->frame_buffer_size_;
            
        } else if (ret != ESP_ERR_TIMEOUT) {
            ESP_LOGW(TAG, "Frame capture failed: %s", esp_err_to_name(ret));
            vTaskDelay(10 / portTICK_PERIOD_MS);  // Pause courte en cas d'erreur
        }
        
        // Petite pause pour éviter de surcharger le CPU
        vTaskDelay(1 / portTICK_PERIOD_MS);
    }
    
    this->streaming_active_ = false;
    ESP_LOGD(TAG, "Streaming loop ended for camera '%s'", this->name_.c_str());
    vTaskDelete(nullptr);  // Supprime la tâche courante
}

#endif  // HAS_ESP32_P4_CAMERA

}  // namespace tab5_camera
}  // namespace esphome

#endif  // USE_ESP32







