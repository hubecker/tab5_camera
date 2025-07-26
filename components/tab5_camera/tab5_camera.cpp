#include "tab5_camera.h"
#include "esphome/core/log.h"
#include "esphome/core/helpers.h"
#include "esphome/core/hal.h"
#include "esp_timer.h"

#ifdef USE_ESP32

static const char *const TAG = "tab5_camera";

// Constantes de configuration pour Tab5 (alignées sur l'exemple IDF)
#define TAB5_CAMERA_H_RES 800
#define TAB5_CAMERA_V_RES 640
#define TAB5_MIPI_CSI_LANE_BITRATE_MBPS 200
#define TAB5_ISP_CLOCK_HZ (80 * 1000 * 1000)  // 80MHz comme dans l'exemple IDF
#define TAB5_STREAMING_STACK_SIZE 8192
#define TAB5_FRAME_QUEUE_LENGTH 4

namespace esphome {
namespace tab5_camera {

Tab5Camera::~Tab5Camera() {
#ifdef HAS_ESP32_P4_CAMERA
  this->cleanup_resources();
#endif
}

void Tab5Camera::setup() {
#ifdef HAS_ESP32_P4_CAMERA
  ESP_LOGCONFIG(TAG, "Setting up Tab5 Camera with ESP32-P4 MIPI-CSI...");
  
  // Création des objets de synchronisation
  this->frame_ready_semaphore_ = xSemaphoreCreateBinary();
  if (!this->frame_ready_semaphore_) {
    ESP_LOGE(TAG, "Failed to create frame ready semaphore");
    this->mark_failed();
    return;
  }
  
  this->frame_queue_ = xQueueCreate(TAB5_FRAME_QUEUE_LENGTH, sizeof(FrameTransaction*));
  if (!this->frame_queue_) {
    ESP_LOGE(TAG, "Failed to create frame queue");
    this->mark_failed();
    return;
  }
  
  // Initialisation de la caméra
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
  ESP_LOGCONFIG(TAG, "  Resolution: %dx%d", TAB5_CAMERA_H_RES, TAB5_CAMERA_V_RES);
  ESP_LOGCONFIG(TAG, "  Frame Buffer Size: %zu bytes", this->frame_buffer_size_);
  ESP_LOGCONFIG(TAG, "  MIPI Lane Bitrate: %d Mbps", TAB5_MIPI_CSI_LANE_BITRATE_MBPS);
  ESP_LOGCONFIG(TAG, "  ISP Clock: %d Hz", TAB5_ISP_CLOCK_HZ);
  ESP_LOGCONFIG(TAG, "  I2C Address: 0x%02X", this->get_sensor_address());
  ESP_LOGCONFIG(TAG, "  SCCB SCL Pin: GPIO%u", this->sccb_scl_pin_);
  ESP_LOGCONFIG(TAG, "  SCCB SDA Pin: GPIO%u", this->sccb_sda_pin_);
  
  if (this->reset_pin_) {
    LOG_PIN("  Reset Pin: ", this->reset_pin_);
  }
  if (this->power_down_pin_) {
    LOG_PIN("  Power Down Pin: ", this->power_down_pin_);
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

#ifdef HAS_ESP32_P4_CAMERA

bool Tab5Camera::init_ldo_regulator() {
  if (this->ldo_initialized_) {
    ESP_LOGI(TAG, "LDO already initialized, skipping");
    return true;
  }
  
  ESP_LOGI(TAG, "Initializing MIPI LDO regulator");
  
  // Configuration LDO pour MIPI PHY (basée sur l'exemple IDF)
  esp_ldo_channel_config_t ldo_mipi_phy_config = {
    .chan_id = 3,  // Channel ID standard pour MIPI
    .voltage_mv = 2500,  // 2.5V pour MIPI PHY
  };
  
  esp_err_t ret = esp_ldo_acquire_channel(&ldo_mipi_phy_config, &this->ldo_mipi_phy_);
  if (ret != ESP_OK) {
    ESP_LOGE(TAG, "Failed to acquire MIPI LDO channel: %s", esp_err_to_name(ret));
    return false;
  }
  
  ESP_LOGI(TAG, "MIPI LDO regulator acquired successfully");
  this->ldo_initialized_ = true;
  return true;
}

bool Tab5Camera::init_sensor_() {
  if (this->sensor_initialized_) {
    ESP_LOGI(TAG, "Sensor already initialized, skipping");
    return true;
  }
  
  ESP_LOGI(TAG, "Initializing camera sensor at I2C address 0x%02X", this->get_sensor_address());
  
  // Reset du capteur si pin disponible
  if (this->reset_pin_) {
    ESP_LOGI(TAG, "Executing sensor reset sequence");
    this->reset_pin_->setup();
    this->reset_pin_->digital_write(false);
    delay(10);  // 10ms
    this->reset_pin_->digital_write(true);
    delay(10);  // 10ms pour stabilisation
  }
  
  // Power down control si disponible
  if (this->power_down_pin_) {
    ESP_LOGI(TAG, "Configuring power down pin");
    this->power_down_pin_->setup();
    this->power_down_pin_->digital_write(false);  // Active low généralement
    delay(5);
  }
  
  // Test de communication I2C basique
  uint8_t test_data;
  if (!this->read_byte(0x00, &test_data)) {
    ESP_LOGW(TAG, "Could not communicate with sensor at address 0x%02X", this->get_sensor_address());
    // Ne pas échouer immédiatement, certains capteurs ont des adresses de test différentes
  } else {
    ESP_LOGI(TAG, "Sensor responded to I2C communication (reg 0x00 = 0x%02X)", test_data);
  }
  
  // TODO: Ajouter ici l'initialisation spécifique du capteur selon le modèle
  // Pour l'instant, marquer comme initialisé
  this->sensor_initialized_ = true;
  ESP_LOGI(TAG, "Camera sensor marked as initialized");
  return true;
}

bool Tab5Camera::allocate_frame_buffers_() {
  ESP_LOGI(TAG, "Allocating frame buffers");
  
  // Calcul de la taille de frame (RGB565 = 2 bytes par pixel)
  this->frame_buffer_size_ = TAB5_CAMERA_H_RES * TAB5_CAMERA_V_RES * 2;
  
  // Alignement sur cache line (64 bytes)
  this->frame_buffer_size_ = (this->frame_buffer_size_ + 63) & ~63;
  
  ESP_LOGI(TAG, "Frame buffer size: %zu bytes (aligned)", this->frame_buffer_size_);
  
  // Allocation des transactions
  this->frame_transactions_ = (FrameTransaction*)heap_caps_calloc(
    this->frame_buffer_count_, 
    sizeof(FrameTransaction), 
    MALLOC_CAP_8BIT
  );
  
  if (!this->frame_transactions_) {
    ESP_LOGE(TAG, "Failed to allocate frame transactions");
    return false;
  }
  
  // Allocation des buffers individuels
  for (uint8_t i = 0; i < this->frame_buffer_count_; i++) {
    // Allocation en PSRAM d'abord, puis RAM si échec
    this->frame_transactions_[i].buffer = heap_caps_aligned_alloc(
      64, this->frame_buffer_size_, 
      MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT
    );
    
    if (!this->frame_transactions_[i].buffer) {
      ESP_LOGW(TAG, "PSRAM allocation failed for buffer %d, trying regular RAM", i);
      this->frame_transactions_[i].buffer = heap_caps_aligned_alloc(
        64, this->frame_buffer_size_, 
        MALLOC_CAP_DMA | MALLOC_CAP_8BIT
      );
    }
    
    if (!this->frame_transactions_[i].buffer) {
      ESP_LOGE(TAG, "Failed to allocate frame buffer %d", i);
      this->deallocate_frame_buffers_();
      return false;
    }
    
    this->frame_transactions_[i].size = this->frame_buffer_size_;
    this->frame_transactions_[i].in_use = false;
    
    // Initialisation du buffer (blanc)
    memset(this->frame_transactions_[i].buffer, 0xFF, this->frame_buffer_size_);
    esp_cache_msync(this->frame_transactions_[i].buffer, this->frame_buffer_size_, ESP_CACHE_MSYNC_FLAG_DIR_C2M);
    
    ESP_LOGD(TAG, "Frame buffer %d allocated at %p", i, this->frame_transactions_[i].buffer);
  }
  
  // Le buffer courant pointe vers le premier
  this->current_frame_buffer_ = this->frame_transactions_[0].buffer;
  
  ESP_LOGI(TAG, "Successfully allocated %d frame buffers", this->frame_buffer_count_);
  return true;
}

void Tab5Camera::deallocate_frame_buffers_() {
  if (this->frame_transactions_) {
    for (uint8_t i = 0; i < this->frame_buffer_count_; i++) {
      if (this->frame_transactions_[i].buffer) {
        heap_caps_free(this->frame_transactions_[i].buffer);
        this->frame_transactions_[i].buffer = nullptr;
      }
    }
    heap_caps_free(this->frame_transactions_);
    this->frame_transactions_ = nullptr;
  }
  this->current_frame_buffer_ = nullptr;
}

bool Tab5Camera::init_camera_controller() {
  ESP_LOGI(TAG, "Initializing CSI camera controller");
  
  // Configuration CSI (ordre des champs corrigé)
  
  esp_cam_ctlr_csi_config_t csi_config = {
    .ctlr_id = 0,
    .h_res = TAB5_CAMERA_H_RES,
    .v_res = TAB5_CAMERA_V_RES,
    .data_lane_num = this->mipi_data_lanes_,
    .lane_bit_rate_mbps = TAB5_MIPI_CSI_LANE_BITRATE_MBPS,
    .input_data_color_type = CAM_CTLR_COLOR_RAW8,
    .output_data_color_type = CAM_CTLR_COLOR_RGB565,
    .byte_swap_en = false,
    .queue_items = 1,
  }
  
  esp_err_t ret = esp_cam_new_csi_ctlr(&csi_config, &this->cam_handle_);
  if (ret != ESP_OK) {
    ESP_LOGE(TAG, "Failed to create CSI controller: %s", esp_err_to_name(ret));
    return false;
  }
  
  // Configuration des callbacks (comme dans l'exemple IDF)
  esp_cam_ctlr_evt_cbs_t cbs = {
    .on_get_new_trans = Tab5Camera::on_trans_finished_callback_,
    .on_trans_finished = Tab5Camera::on_trans_finished_callback_,
  };
  
  ret = esp_cam_ctlr_register_event_callbacks(this->cam_handle_, &cbs, this);
  if (ret != ESP_OK) {
    ESP_LOGE(TAG, "Failed to register camera callbacks: %s", esp_err_to_name(ret));
    return false;
  }
  
  // Activation du contrôleur
  ret = esp_cam_ctlr_enable(this->cam_handle_);
  if (ret != ESP_OK) {
    ESP_LOGE(TAG, "Failed to enable camera controller: %s", esp_err_to_name(ret));
    return false;
  }
  
  ESP_LOGI(TAG, "CSI camera controller initialized successfully");
  return true;
}

bool Tab5Camera::init_isp_processor() {
  ESP_LOGI(TAG, "Initializing ISP processor");
  
  // Configuration ISP (exactement comme dans l'exemple IDF)
  esp_isp_processor_cfg_t isp_config = {
    .clk_hz = TAB5_ISP_CLOCK_HZ,
    .input_data_source = ISP_INPUT_DATA_SOURCE_CSI,
    .input_data_color_type = ISP_COLOR_RAW8,
    .output_data_color_type = ISP_COLOR_RGB565,
    .has_line_start_packet = false,
    .has_line_end_packet = false,
    .h_res = TAB5_CAMERA_H_RES,
    .v_res = TAB5_CAMERA_V_RES,
  };
  
  esp_err_t ret = esp_isp_new_processor(&isp_config, &this->isp_proc_);
  if (ret != ESP_OK) {
    ESP_LOGE(TAG, "Failed to create ISP processor: %s", esp_err_to_name(ret));
    return false;
  }
  
  ret = esp_isp_enable(this->isp_proc_);
  if (ret != ESP_OK) {
    ESP_LOGE(TAG, "Failed to enable ISP processor: %s", esp_err_to_name(ret));
    return false;
  }
  
  this->isp_initialized_ = true;
  ESP_LOGI(TAG, "ISP processor initialized successfully");
  return true;
}

bool Tab5Camera::init_camera_() {
  if (this->camera_initialized_) {
    ESP_LOGI(TAG, "Camera already initialized, skipping");
    return true;
  }

  ESP_LOGI(TAG, "Starting complete camera initialization");

  // 1. Initialisation du régulateur LDO
  if (!this->init_ldo_regulator()) {
    ESP_LOGE(TAG, "Failed to initialize LDO regulator");
    return false;
  }

  // 2. Initialisation du capteur
  if (!this->init_sensor_()) {
    ESP_LOGE(TAG, "Failed to initialize sensor");
    return false;
  }

  // 3. Allocation des buffers de frame
  if (!this->allocate_frame_buffers_()) {
    ESP_LOGE(TAG, "Failed to allocate frame buffers");
    return false;
  }

  // 4. Initialisation du contrôleur CSI
  if (!this->init_camera_controller()) {
    ESP_LOGE(TAG, "Failed to initialize camera controller");
    return false;
  }

  // 5. Initialisation du processeur ISP
  if (!this->init_isp_processor()) {
    ESP_LOGE(TAG, "Failed to initialize ISP processor");
    return false;
  }

  // 6. Démarrage du contrôleur de caméra
  ESP_LOGI(TAG, "Starting camera controller");
  esp_err_t ret = esp_cam_ctlr_start(this->cam_handle_);
  if (ret != ESP_OK) {
    ESP_LOGE(TAG, "Failed to start camera controller: %s", esp_err_to_name(ret));
    return false;
  }

  this->camera_initialized_ = true;
  ESP_LOGI(TAG, "Camera '%s' fully initialized and started", this->name_.c_str());
  return true;
}

void Tab5Camera::cleanup_resources() {
  ESP_LOGD(TAG, "Cleaning up camera resources");
  
  if (this->streaming_active_) {
    this->stop_streaming();
  }
  
  if (this->camera_initialized_) {
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
    
    this->deallocate_frame_buffers_();
    
    if (this->ldo_mipi_phy_) {
      esp_ldo_release_channel(this->ldo_mipi_phy_);
      this->ldo_mipi_phy_ = nullptr;
    }
    
    this->camera_initialized_ = false;
    this->sensor_initialized_ = false;
    this->ldo_initialized_ = false;
    this->isp_initialized_ = false;
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

// Callbacks statiques (comme dans l'exemple IDF)
bool IRAM_ATTR Tab5Camera::on_trans_finished_callback_(esp_cam_ctlr_handle_t handle, 
                                                      esp_cam_ctlr_trans_t *trans, 
                                                      void *user_data) {
  Tab5Camera *camera = static_cast<Tab5Camera*>(user_data);
  if (!camera) {
    return false;
  }
  
  // Logique similaire à l'exemple IDF - très simple
  if (camera->streaming_active_) {
    // Signaler qu'une nouvelle frame est disponible
    BaseType_t higher_priority_task_woken = pdFALSE;
    xSemaphoreGiveFromISR(camera->frame_ready_semaphore_, &higher_priority_task_woken);
    return higher_priority_task_woken == pdTRUE;
  }
  
  return false;
}

bool Tab5Camera::take_snapshot() {
  if (!this->camera_initialized_) {
    ESP_LOGE(TAG, "Camera not initialized");
    return false;
  }
  
  ESP_LOGD(TAG, "Taking snapshot");
  
  // Utilisation du pattern de l'exemple IDF
  esp_cam_ctlr_trans_t trans = {
    .buffer = this->current_frame_buffer_,
    .buflen = this->frame_buffer_size_,
  };
  
  esp_err_t ret = esp_cam_ctlr_receive(this->cam_handle_, &trans, pdMS_TO_TICKS(FRAME_TIMEOUT_MS));
  if (ret != ESP_OK) {
    ESP_LOGW(TAG, "Snapshot capture failed: %s", esp_err_to_name(ret));
    return false;
  }
  
  // Synchronisation du cache
  esp_cache_msync(this->current_frame_buffer_, this->frame_buffer_size_, ESP_CACHE_MSYNC_FLAG_DIR_M2C);
  
  // Appel des callbacks
  this->on_frame_callbacks_.call(static_cast<uint8_t*>(this->current_frame_buffer_), this->frame_buffer_size_);
  
  ESP_LOGD(TAG, "Snapshot taken successfully, size: %zu bytes", trans.buflen);
  return true;
}

bool Tab5Camera::start_streaming() {
  if (!this->camera_initialized_) {
    ESP_LOGE(TAG, "Camera not initialized for streaming");
    return false;
  }
  
  if (this->streaming_active_) {
    ESP_LOGW(TAG, "Streaming already active");
    return true;
  }
  
  ESP_LOGI(TAG, "Starting camera streaming");
  
  this->streaming_should_stop_ = false;
  this->streaming_active_ = true;
  
  // Création de la tâche de streaming
  BaseType_t result = xTaskCreate(
    Tab5Camera::streaming_task_,
    "tab5_streaming",
    TAB5_STREAMING_STACK_SIZE,
    this,
    STREAMING_TASK_PRIORITY,
    &this->streaming_task_handle_
  );
  
  if (result != pdPASS) {
    ESP_LOGE(TAG, "Failed to create streaming task");
    this->streaming_active_ = false;
    return false;
  }
  
  ESP_LOGI(TAG, "Camera streaming started successfully");
  return true;
}

bool Tab5Camera::stop_streaming() {
  if (!this->streaming_active_) {
    return true;
  }
  
  ESP_LOGI(TAG, "Stopping camera streaming");
  
  this->streaming_should_stop_ = true;
  
  // Signal pour débloquer la tâche
  if (this->frame_ready_semaphore_) {
    xSemaphoreGive(this->frame_ready_semaphore_);
  }
  
  // Attendre l'arrêt de la tâche
  if (this->streaming_task_handle_) {
    uint32_t timeout = 0;
    while (this->streaming_active_ && timeout < 50) {  // 5 secondes max
      vTaskDelay(pdMS_TO_TICKS(100));
      timeout++;
    }
    
    if (this->streaming_active_) {
      ESP_LOGW(TAG, "Force stopping streaming task");
      vTaskDelete(this->streaming_task_handle_);
      this->streaming_active_ = false;
    }
    
    this->streaming_task_handle_ = nullptr;
  }
  
  ESP_LOGI(TAG, "Camera streaming stopped");
  return true;
}

void Tab5Camera::streaming_task_(void *parameter) {
  Tab5Camera *camera = static_cast<Tab5Camera*>(parameter);
  camera->streaming_loop_();
}

void Tab5Camera::streaming_loop_() {
  ESP_LOGD(TAG, "Streaming loop started");

  esp_cam_ctlr_trans_t trans = {
    .buffer = this->current_frame_buffer_,
    .buflen = this->frame_buffer_size_,
  };

  while (!this->streaming_should_stop_) {
    // Pattern similaire à l'exemple IDF: boucle continue avec esp_cam_ctlr_receive
    esp_err_t ret = esp_cam_ctlr_receive(this->cam_handle_, &trans, pdMS_TO_TICKS(100));

    if (ret == ESP_OK) {
      // Synchronisation du cache
      esp_cache_msync(this->current_frame_buffer_, this->frame_buffer_size_, ESP_CACHE_MSYNC_FLAG_DIR_M2C);
      
      // Appel des callbacks
      this->on_frame_callbacks_.call(static_cast<uint8_t*>(this->current_frame_buffer_), this->frame_buffer_size_);
    } else if (ret != ESP_ERR_TIMEOUT) {
      ESP_LOGW(TAG, "Frame capture failed: %s", esp_err_to_name(ret));
    }

    // Petit délai pour éviter la surcharge CPU
    vTaskDelay(pdMS_TO_TICKS(1));
  }

  this->streaming_active_ = false;
  ESP_LOGD(TAG, "Streaming loop ended");
  vTaskDelete(nullptr);
}

// Getters avec support des résolutions dynamiques
uint16_t Tab5Camera::get_frame_width() const {
  if (this->resolution_ == CameraResolution::CUSTOM) {
    return this->custom_width_;
  }
  return TAB5_CAMERA_H_RES;  // Pour l'instant, valeur fixe
}

uint16_t Tab5Camera::get_frame_height() const {
  if (this->resolution_ == CameraResolution::CUSTOM) {
    return this->custom_height_;
  }
  return TAB5_CAMERA_V_RES;  // Pour l'instant, valeur fixe
}

// Fonction utilitaire pour parser le format de pixel depuis string
PixelFormat Tab5Camera::parse_pixel_format_(const std::string &format) {
  if (format == "RAW8") return PixelFormat::RAW8;
  if (format == "RAW10") return PixelFormat::RAW10;
  if (format == "YUV422") return PixelFormat::YUV422;
  if (format == "RGB565") return PixelFormat::RGB565;
  if (format == "JPEG") return PixelFormat::JPEG;
  
  ESP_LOGW(TAG, "Unknown pixel format '%s', defaulting to RAW8", format.c_str());
  return PixelFormat::RAW8;
}

#endif // HAS_ESP32_P4_CAMERA

}  // namespace tab5_camera
}  // namespace esphome

#endif  // USE_ESP32







