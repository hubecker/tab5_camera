#include "tab5_camera.h"
#include "esphome/core/log.h"
#include "esphome/core/helpers.h"
#include "esphome/core/hal.h"
#include "esp_timer.h"
#include "esp_cam_ctlr_csi.h"

#if CONFIG_CAMERA_OV5645
#include "ov5645.h"
#define SCCB0_CAM_DEVICE_ADDR OV5645_SCCB_ADDR
#elif CONFIG_CAMERA_SC2336
#include "sc2336.h"
#define SCCB0_CAM_DEVICE_ADDR SC2336_SCCB_ADDR
#else
#define SCCB0_CAM_DEVICE_ADDR 0x01
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

  ESP_LOGI(TAG, "Initializing OV5645 camera sensor at I2C address 0x%02X", this->address_);

  // Fonctions helper pour les registres 16 bits
  auto write_reg16 = [this](uint16_t reg, uint8_t val) -> bool {
    uint8_t reg_high = (reg >> 8) & 0xFF;
    uint8_t reg_low = reg & 0xFF;
    return this->write_byte(reg_high, reg_low) && this->write_byte(reg_low, val);
  };

  auto read_reg16 = [this](uint16_t reg, uint8_t* val) -> bool {
    uint8_t reg_high = (reg >> 8) & 0xFF;
    uint8_t reg_low = reg & 0xFF;
    return this->write_byte(reg_high, reg_low) && this->read_byte(reg_low, val);
  };

  // Identification du capteur
  uint8_t id_high, id_low;
  if (read_reg16(0x300A, &id_high) && read_reg16(0x300B, &id_low)) {
    uint16_t sensor_id = (id_high << 8) | id_low;
    ESP_LOGI(TAG, "Sensor ID: 0x%04X", sensor_id);
    if (sensor_id != 0x5645) {
      ESP_LOGW(TAG, "Unexpected sensor ID (0x5645 expected)");
    }
  } else {
    ESP_LOGE(TAG, "Failed to read sensor ID");
    return false;
  }

  // Configuration complète OV5645 (registres 16 bits)
  const struct {
    uint16_t reg;
    uint8_t val;
  } ov5645_init_regs[] = {
    // Reset et configuration de base
    {0x3103, 0x11}, {0x3008, 0x82},
    {0x3008, 0x42}, {0x3103, 0x03},
    
    // Horloge et timing
    {0x3017, 0xff}, {0x3018, 0xff},
    {0x3034, 0x1a}, {0x3035, 0x21},
    {0x3036, 0x46}, {0x3037, 0x13},
    
    // Contrôles numériques
    {0x3108, 0x01}, {0x3630, 0x36},
    {0x3631, 0x0e}, {0x3632, 0xe2},
    {0x3633, 0x12}, {0x3621, 0xe0},
    
    // Contrôles analogiques
    {0x3704, 0xa0}, {0x3703, 0x5a},
    {0x3715, 0x78}, {0x3717, 0x01},
    {0x370b, 0x60}, {0x3705, 0x1a},
    
    // AWB et autres réglages
    {0x3905, 0x02}, {0x3906, 0x10},
    {0x3901, 0x0a}, {0x3731, 0x12},
    {0x3600, 0x08}, {0x3601, 0x33},
    
    // Configuration supplémentaire
    {0x302d, 0x60}, {0x3620, 0x52},
    {0x371b, 0x20}, {0x471c, 0x50},
    {0x3a13, 0x43}, {0x3a18, 0x00},
    
    // Configuration ISP
    {0x3a19, 0xf8}, {0x3635, 0x13},
    {0x3636, 0x03}, {0x3634, 0x40},
    {0x3622, 0x01}, {0x3c01, 0x34},
    
    // Format de sortie
    {0x3c04, 0x28}, {0x3c05, 0x98},
    {0x3c06, 0x00}, {0x3c07, 0x08},
    {0x3c08, 0x00}, {0x3c09, 0x1c},
    
    // Derniers réglages
    {0x3c0a, 0x9c}, {0x3c0b, 0x40},
    {0x3810, 0x00}, {0x3811, 0x10},
    {0x3812, 0x00}, {0x3708, 0x64},
    
    // Activation finale
    {0x4001, 0x02}, {0x4005, 0x1a},
    {0x3000, 0x00}, {0x3004, 0xff},
    {0x300e, 0x58}, {0x302e, 0x00},
    {0x4300, 0x61}, {0x501f, 0x00},
    {0x440e, 0x00}, {0x5000, 0xa7},
    {0x3008, 0x02}  // Démarrage final
  };

  // Écriture de toute la configuration
  ESP_LOGI(TAG, "Writing OV5645 configuration registers...");
  for (size_t i = 0; i < sizeof(ov5645_init_regs)/sizeof(ov5645_init_regs[0]); i++) {
    if (!write_reg16(ov5645_init_regs[i].reg, ov5645_init_regs[i].val)) {
      ESP_LOGW(TAG, "Failed to write register 0x%04X", ov5645_init_regs[i].reg);
    }
    vTaskDelay(2 / portTICK_PERIOD_MS); // Délai entre les écritures
  }

  // Vérification finale
  uint8_t status;
  if (read_reg16(0x3029, &status)) {
    ESP_LOGI(TAG, "Sensor status: 0x%02X", status);
    if (!(status & 0x08)) {
      ESP_LOGW(TAG, "Sensor not streaming properly");
    }
  } else {
    ESP_LOGW(TAG, "Failed to read sensor status");
  }

  this->sensor_initialized_ = true;
  ESP_LOGI(TAG, "OV5645 initialization complete");
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

// SUPPRIMÉ - Cette fonction causait le problème de queue pleine !
// Elle empêchait la queue interne du contrôleur CSI de fonctionner correctement
/*
bool Tab5Camera::camera_get_new_vb_callback(esp_cam_ctlr_handle_t handle, esp_cam_ctlr_trans_t *trans, void *user_data) {
  // Cette fonction est supprimée - elle causait le conflit avec la queue interne
}
*/

// CORRIGÉ - Callback de fin de transaction avec diagnostics
bool Tab5Camera::camera_get_finished_trans_callback(esp_cam_ctlr_handle_t handle, esp_cam_ctlr_trans_t *trans, void *user_data) {
  Tab5Camera *camera = static_cast<Tab5Camera*>(user_data);
  if (!camera || !trans->buffer) {
    ESP_LOGE(TAG, "camera_get_finished_trans_callback called with invalid data");
    return false;
  }

  // Compteur de frames pour diagnostics
  static uint32_t frame_count = 0;
  frame_count++;
  
  ESP_LOGI(TAG, " Frame #%lu received: %zu bytes (expected: %zu)", 
           frame_count, trans->received_size, camera->frame_buffer_size_);

  // Vérification de la taille des données
  if (trans->received_size == 0) {
    ESP_LOGW(TAG, " Frame #%lu is empty - sensor might not be generating data", frame_count);
    return false;
  }
  
  if (trans->received_size < 1000) {  // Taille suspicieusement petite
    ESP_LOGW(TAG, " Frame #%lu size is very small (%zu bytes)", frame_count, trans->received_size);
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
    // Queue pleine - on peut choisir d'overwrite ou de dropper
    ESP_LOGD(TAG, "Application frame queue full, dropping frame #%lu", frame_count);
  }

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










