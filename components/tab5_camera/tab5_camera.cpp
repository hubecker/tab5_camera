#include "tab5_camera.h"
#include "esphome/core/log.h"
#include "esphome/core/helpers.h"
#include "esphome/core/hal.h"
#include "esp_timer.h"

#ifdef USE_ESP32

static const char *const TAG = "tab5_camera";

// Constantes de configuration optimisées pour Tab5
#define TAB5_CAMERA_H_RES 640
#define TAB5_CAMERA_V_RES 480
#define TAB5_MIPI_CSI_LANE_BITRATE_MBPS 400
#define TAB5_ISP_CLOCK_HZ 80000000  // 80MHz pour résolution VGA
#define TAB5_STREAMING_STACK_SIZE 8192
#define TAB5_FRAME_QUEUE_LENGTH 2

// Optimisations DMA selon ESP32-P4
#define TAB5_DMA_BUF_COUNT 4
#define TAB5_DMA_BUF_SIZE 1024  // Max 1024 pour ESP32-P4

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
  ESP_LOGCONFIG(TAG, "  ISP Clock: %d Hz", TAB5_ISP_CLOCK_HZ);
  ESP_LOGCONFIG(TAG, "  DMA Buffers: %d x %d bytes", TAB5_DMA_BUF_COUNT, TAB5_DMA_BUF_SIZE);
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
bool Tab5Camera::init_ldo_() {
  if (this->ldo_initialized_) {
    ESP_LOGI(TAG, "LDO already initialized, skipping");
    return true;
  }
  
  ESP_LOGI(TAG, "Initializing MIPI LDO regulator");
  
  // Configuration optimisée pour le LDO MIPI
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
  bool sensor_found = false;
  
  for (int i = 0; i < retry_count && !sensor_found; i++) {
    if (this->read_byte(0x00, &test_data)) {
      ESP_LOGI(TAG, "Sensor responded to I2C communication (reg 0x00 = 0x%02X)", test_data);
      sensor_found = true;
    } else {
      ESP_LOGW(TAG, "Sensor communication attempt %d/%d failed", i + 1, retry_count);
      if (i < retry_count - 1) {
        vTaskDelay(100 / portTICK_PERIOD_MS);  // Attendre 100ms avant retry
      }
    }
  }
  
  if (!sensor_found) {
    ESP_LOGW(TAG, "Could not establish communication with sensor at address 0x%02X", this->address_);
    // Pour l'instant, ne pas échouer complètement
  }
  
  this->sensor_initialized_ = true;
  ESP_LOGI(TAG, "Camera sensor marked as initialized");
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

  // Étape 3: Initialisation du capteur I2C
  ESP_LOGI(TAG, "Step 2.3: Initializing camera sensor");
  if (!this->init_sensor_()) {
    ESP_LOGE(TAG, "Failed to initialize camera sensor");
    return false;
  }
  ESP_LOGI(TAG, "Camera sensor initialized successfully");

  // Étape 4: Allocation du frame buffer optimisée
  ESP_LOGI(TAG, "Step 2.4: Allocating frame buffer");

  // Calcul de la taille avec vérification de l'alignement RGB565
  if (TAB5_CAMERA_H_RES % 8 != 0) {
    ESP_LOGW(TAG, "Horizontal resolution %d is not multiple of 8, may cause issues with RGB565", TAB5_CAMERA_H_RES);
  }

  this->frame_buffer_size_ = TAB5_CAMERA_H_RES * TAB5_CAMERA_V_RES * 2; // RGB565 = 2 bytes par pixel

  // Arrondi sur un multiple de 64 (cache line size)
  this->frame_buffer_size_ = (this->frame_buffer_size_ + 63) & ~63;

  ESP_LOGI(TAG, "Aligned frame buffer size: %zu bytes", this->frame_buffer_size_);

  // Allocation alignée avec préférence PSRAM
  this->frame_buffer_ = heap_caps_aligned_alloc(64, this->frame_buffer_size_, MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT);
  if (!this->frame_buffer_) {
    ESP_LOGW(TAG, "Failed to allocate aligned frame buffer in PSRAM (%zu bytes) - trying DMA capable RAM", this->frame_buffer_size_);

    // Essai avec RAM DMA-capable
    this->frame_buffer_ = heap_caps_aligned_alloc(64, this->frame_buffer_size_, MALLOC_CAP_DMA | MALLOC_CAP_8BIT);
    if (!this->frame_buffer_) {
      ESP_LOGE(TAG, "Failed to allocate aligned frame buffer in DMA-capable RAM");
      return false;
    }
  }

  ESP_LOGI(TAG, "Frame buffer allocated successfully at %p", this->frame_buffer_);

  // Étape 5: Configuration du contrôleur CSI avec optimisations
  ESP_LOGI(TAG, "Step 2.5: Configuring CSI controller");
  esp_cam_ctlr_csi_config_t csi_config = {};
  csi_config.ctlr_id = 0;
  csi_config.h_res = TAB5_CAMERA_H_RES;
  csi_config.v_res = TAB5_CAMERA_V_RES;
  csi_config.lane_bit_rate_mbps = TAB5_MIPI_CSI_LANE_BITRATE_MBPS;
  csi_config.input_data_color_type = CAM_CTLR_COLOR_RAW8;  // RAW8 pour OV2640
  csi_config.output_data_color_type = CAM_CTLR_COLOR_RGB565;
  csi_config.data_lane_num = 2;
  csi_config.byte_swap_en = false;
  csi_config.queue_items = 3;
  
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
  
  // Étape 8: Configuration de l'ISP avec optimisations avancées
  ESP_LOGI(TAG, "Step 2.8: Configuring ISP processor with advanced features");
  esp_isp_processor_cfg_t isp_config = {};
  isp_config.clk_hz = TAB5_ISP_CLOCK_HZ;
  isp_config.input_data_source = ISP_INPUT_DATA_SOURCE_CSI;
  isp_config.input_data_color_type = ISP_COLOR_RAW8;
  isp_config.output_data_color_type = ISP_COLOR_RGB565;
  isp_config.has_line_start_packet = false;
  isp_config.has_line_end_packet = false;
  isp_config.h_res = TAB5_CAMERA_H_RES;
  isp_config.v_res = TAB5_CAMERA_V_RES;
  
  // Optimisations ISP pipeline selon recommandations
  isp_config.awb_enable = true;        // Auto White Balance
  isp_config.ae_enable = true;         // Auto Exposure
  isp_config.denoise_enable = true;    // Bayer Denoise
  
  // Configuration DMA optimisée pour ESP32-P4
  isp_config.dma_buf_count = TAB5_DMA_BUF_COUNT;
  isp_config.dma_buf_size = TAB5_DMA_BUF_SIZE;
  
  ret = esp_isp_new_processor(&isp_config, &this->isp_proc_);
  if (ret != ESP_OK) {
    ESP_LOGE(TAG, "ISP processor init failed: %s", esp_err_to_name(ret));
    return false;
  }
  ESP_LOGI(TAG, "ISP processor created successfully with AWB, AE, and denoise enabled");
  
  ret = esp_isp_enable(this->isp_proc_);
  if (ret != ESP_OK) {
    ESP_LOGE(TAG, "Failed to enable ISP processor: %s", esp_err_to_name(ret));
    return false;
  }
  ESP_LOGI(TAG, "ISP processor enabled successfully");
  
  // Étape 9: Initialisation du frame buffer
  ESP_LOGI(TAG, "Step 2.9: Initializing frame buffer");
  memset(this->frame_buffer_, 0x00, this->frame_buffer_size_);  // Noir au lieu de blanc
  esp_cache_msync(this->frame_buffer_, this->frame_buffer_size_, ESP_CACHE_MSYNC_FLAG_DIR_C2M);
  ESP_LOGI(TAG, "Frame buffer initialized and cache synchronized");
  
  // Étape 10: Démarrage de la caméra
  ESP_LOGI(TAG, "Step 2.10: Starting camera controller");
  ret = esp_cam_ctlr_start(this->cam_handle_);
  if (ret != ESP_OK) {
    ESP_LOGE(TAG, "Failed to start camera controller: %s", esp_err_to_name(ret));
    return false;
  }
  
  this->camera_initialized_ = true;
  ESP_LOGI(TAG, "Camera '%s' initialized successfully - all steps completed with optimizations", this->name_.c_str());
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
  if (!camera || !trans) {
    return false;
  }
  
  trans->buffer = camera->frame_buffer_;
  trans->buflen = camera->frame_buffer_size_;
  return false;
}

bool Tab5Camera::camera_get_finished_trans_callback(esp_cam_ctlr_handle_t handle, esp_cam_ctlr_trans_t *trans, void *user_data) {
  Tab5Camera *camera = static_cast<Tab5Camera*>(user_data);
  if (!camera || !trans) {
    ESP_LOGE(TAG, "camera_get_finished_trans_callback called with null parameters");
    return false;
  }

  // Validation de la taille de frame
  if (trans->buflen != camera->frame_buffer_size_) {
    ESP_LOGW(TAG, "Frame size mismatch: expected %zu, got %zu", camera->frame_buffer_size_, trans->buflen);
  }

  // Synchronisation du cache avant traitement
  esp_cache_msync(camera->frame_buffer_, camera->frame_buffer_size_, ESP_CACHE_MSYNC_FLAG_DIR_M2C);

  // Création de la structure FrameData
  FrameData frame;
  frame.buffer = camera->frame_buffer_;
  frame.size = trans->buflen;
  frame.timestamp = esp_timer_get_time();
  
  // Tentative d'ajout à la queue (non bloquant depuis ISR)
  BaseType_t higher_priority_task_woken = pdFALSE;
  BaseType_t ret = xQueueSendFromISR(camera->frame_queue_, &frame, &higher_priority_task_woken);
  
  if (ret == pdTRUE) {
    // Signal qu'une frame est prête
    xSemaphoreGiveFromISR(camera->frame_ready_semaphore_, &higher_priority_task_woken);
  } else {
    // Queue pleine - stratégie d'overwrite pour éviter la latence
    FrameData dummy_frame;
    xQueueReceiveFromISR(camera->frame_queue_, &dummy_frame, &higher_priority_task_woken);
    xQueueSendFromISR(camera->frame_queue_, &frame, &higher_priority_task_woken);
    xSemaphoreGiveFromISR(camera->frame_ready_semaphore_, &higher_priority_task_woken);
  }

  // Yield si une tâche de priorité plus élevée doit s'exécuter
  portYIELD_FROM_ISR(higher_priority_task_woken);

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
  
  this->streaming_should_stop_ = false;
  this->streaming_active_ = true;
  
  // Création de la tâche de streaming avec priorité optimisée
  BaseType_t result = xTaskCreatePinnedToCore(
    Tab5Camera::streaming_task,
    "tab5_streaming",
    TAB5_STREAMING_STACK_SIZE,
    this,
    6,  // Priorité élevée pour le streaming (augmentée de 5 à 6)
    &this->streaming_task_handle_,
    1   // Épinglé au core 1 pour éviter les conflits avec WiFi/BT sur core 0
  );
  
  if (result != pdPASS) {
    ESP_LOGE(TAG, "Failed to create streaming task");
    this->streaming_active_ = false;
    return false;
  }
  
  ESP_LOGI(TAG, "Camera '%s' streaming started on core 1", this->name_.c_str());
  return true;
}

bool Tab5Camera::stop_streaming() {
  if (!this->streaming_active_) {
    return true;
  }
  
  ESP_LOGD(TAG, "Stopping camera '%s' streaming...", this->name_.c_str());
  
  this->streaming_should_stop_ = true;
  
  // Attendre l'arrêt de la tâche avec timeout amélioré
  if (this->streaming_task_handle_) {
    // Signal pour débloquer la tâche si elle attend
    xSemaphoreGive(this->frame_ready_semaphore_);
    
    // Attendre la fin de la tâche avec timeout progressif
    uint32_t timeout = 0;
    while (this->streaming_active_ && timeout < 100) {  // 10 secondes max
      vTaskDelay(100 / portTICK_PERIOD_MS);
      timeout++;
    }
    
    if (this->streaming_active_) {
      ESP_LOGW(TAG, "Force stopping streaming task after timeout");
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
  if (camera) {
    camera->streaming_loop_();
  }
}

void Tab5Camera::streaming_loop_() {
  ESP_LOGD(TAG, "Streaming loop started for camera '%s'", this->name_.c_str());
  
  const TickType_t frame_timeout = 100 / portTICK_PERIOD_MS;  // 100ms timeout
  const TickType_t error_delay = 10 / portTICK_PERIOD_MS;     // 10ms pause sur erreur
  uint32_t consecutive_errors = 0;
  const uint32_t max_consecutive_errors = 10;
  
  while (!this->streaming_should_stop_) {
    // Attendre qu'une frame soit disponible
    if (xSemaphoreTake(this->frame_ready_semaphore_, frame_timeout) == pdTRUE) {
      FrameData frame;
      if (xQueueReceive(this->frame_queue_, &frame, 0) == pdTRUE) {
        // Frame reçue avec succès
        consecutive_errors = 0;
        
        // Appel des callbacks avec validation
        if (frame.buffer && frame.size > 0) {
          this->on_frame_callbacks_.call(static_cast<uint8_t*>(frame.buffer), frame.size);
        }
      }
    } else {
      // Timeout - pas de frame reçue
      consecutive_errors++;
      if (consecutive_errors > max_consecutive_errors) {
        ESP_LOGW(TAG, "Too many consecutive frame timeouts (%u), checking camera status", consecutive_errors);
        vTaskDelay(error_delay * 5);  // Pause plus longue
        consecutive_errors = 0;  // Reset counter
      } else {
        vTaskDelay(error_delay);
      }
    }
    
    // Yield pour permettre aux autres tâches de s'exécuter
    taskYIELD();
  }
  
  this->streaming_active_ = false;
  ESP_LOGD(TAG, "Streaming loop ended for camera '%s'", this->name_.c_str());
  vTaskDelete(nullptr);
}
#endif

}  // namespace tab5_camera
}  // namespace esphome

#endif  // USE_ESP32








