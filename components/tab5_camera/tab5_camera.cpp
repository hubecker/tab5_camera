#include "tab5_camera.h"
#include "esphome/core/log.h"

#ifdef USE_ESP32

namespace esphome {
namespace tab5_camera {

static const char *TAG = "tab5_camera";

#ifdef HAS_ESP32_P4_CAMERA

// Méthode privée d'initialisation caméra
bool Tab5Camera::init_camera_() {
  ESP_LOGI(TAG, "Initialisation du contrôleur caméra...");
  if (!this->init_ldo_()) {
    this->set_error_("Impossible d'initialiser LDO");
    return false;
  }
  // Création du contrôleur caméra (esp_cam_ctlr_create_csi)
  esp_cam_ctlr_config_t cfg{};
  cfg.xclk_gpio_num = this->external_clock_pin_;
  cfg.xclk_freq_hz = this->external_clock_frequency_;
  // ... autres configurations, pin, format etc.

  esp_err_t err = esp_cam_ctlr_create_csi(&cfg, &this->cam_handle_);
  if (err != ESP_OK) {
    this->set_error_(std::string("esp_cam_ctlr_create_csi failed: ") + esp_err_to_name(err));
    return false;
  }

  if (!this->init_isp_processor()) {
    this->set_error_("Impossible d'initialiser ISP");
    return false;
  }

  this->camera_initialized_ = true;
  return true;
}

bool Tab5Camera::init_ldo_() {
  esp_ldo_regulator_config_t ldo_cfg{};
  ldo_cfg.regulator_name = "mipi_phy";
  esp_err_t err = esp_ldo_regulator_create(&ldo_cfg, &this->ldo_mipi_phy_);
  if (err != ESP_OK) {
    ESP_LOGE(TAG, "Erreur creation LDO: %s", esp_err_to_name(err));
    return false;
  }
  err = esp_ldo_regulator_enable(this->ldo_mipi_phy_);
  if (err != ESP_OK) {
    ESP_LOGE(TAG, "Erreur activation LDO: %s", esp_err_to_name(err));
    return false;
  }
  this->ldo_initialized_ = true;
  return true;
}

bool Tab5Camera::init_isp_processor() {
  isp_proc_config_t isp_cfg{};
  // config ISP selon besoin, taille frame, format pixel, etc.
  esp_err_t err = isp_proc_create(&isp_cfg, &this->isp_proc_);
  if (err != ESP_OK) {
    ESP_LOGE(TAG, "Erreur creation ISP proc: %s", esp_err_to_name(err));
    return false;
  }
  return true;
}

void Tab5Camera::deinit_camera_() {
  if (this->isp_proc_) {
    isp_proc_delete(this->isp_proc_);
    this->isp_proc_ = nullptr;
  }
  if (this->cam_handle_) {
    esp_cam_ctlr_delete(this->cam_handle_);
    this->cam_handle_ = nullptr;
  }
  if (this->ldo_mipi_phy_) {
    esp_ldo_regulator_disable(this->ldo_mipi_phy_);
    esp_ldo_regulator_delete(this->ldo_mipi_phy_);
    this->ldo_mipi_phy_ = nullptr;
  }
  this->camera_initialized_ = false;
  this->ldo_initialized_ = false;
}

// Fonction publique setup()
void Tab5Camera::setup() {
  ESP_LOGCONFIG(TAG, "Setup Tab5 Camera...");
  if (!this->init_camera_()) {
    this->set_error_("Échec initialisation caméra");
  }
}

// Fonction dump_config()
void Tab5Camera::dump_config() {
  ESP_LOGCONFIG(TAG, "Tab5 Camera Configuration:");
  ESP_LOGCONFIG(TAG, "  Name: %s", this->name_.c_str());
  ESP_LOGCONFIG(TAG, "  Resolution: %ux%u", this->frame_width_, this->frame_height_);
  ESP_LOGCONFIG(TAG, "  Pixel Format: %s", this->pixel_format_.c_str());
  ESP_LOGCONFIG(TAG, "  JPEG Quality: %u", this->jpeg_quality_);
  ESP_LOGCONFIG(TAG, "  Framerate: %u fps", this->framerate_);
  ESP_LOGCONFIG(TAG, "  Streaming active: %s", this->streaming_active_ ? "YES" : "NO");
}

// Retourne la priorité setup (normale)
float Tab5Camera::get_setup_priority() const {
  return esphome::setup_priority::PROCESSOR;
}

// Démarrer le streaming (bool, conforme au header)
bool Tab5Camera::start_streaming() {
  if (!this->camera_initialized_) {
    this->set_error_("Camera non initialisée");
    return false;
  }
  if (this->streaming_active_) {
    ESP_LOGW(TAG, "Streaming déjà actif");
    return true;
  }

  this->streaming_should_stop_ = false;
  this->frame_ready_semaphore_ = xSemaphoreCreateBinary();
  this->frame_queue_ = xQueueCreate(FRAME_QUEUE_SIZE, sizeof(FrameData));
  if (!this->frame_ready_semaphore_ || !this->frame_queue_) {
    this->set_error_("Erreur allocation sémaphore ou queue");
    return false;
  }

  BaseType_t res = xTaskCreatePinnedToCore(
      streaming_task,
      "tab5_cam_stream",
      STREAMING_TASK_STACK_SIZE,
      this,
      STREAMING_TASK_PRIORITY,
      &this->streaming_task_handle_,
      tskNO_AFFINITY);
  if (res != pdPASS) {
    this->set_error_("Impossible de créer la tâche streaming");
    return false;
  }
  this->streaming_active_ = true;
  ESP_LOGI(TAG, "Streaming démarré");
  return true;
}

// Arrêter le streaming
bool Tab5Camera::stop_streaming() {
  if (!this->streaming_active_)
    return true;

  this->streaming_should_stop_ = true;
  if (this->streaming_task_handle_) {
    vTaskDelete(this->streaming_task_handle_);
    this->streaming_task_handle_ = nullptr;
  }

  if (this->frame_ready_semaphore_) {
    vSemaphoreDelete(this->frame_ready_semaphore_);
    this->frame_ready_semaphore_ = nullptr;
  }

  if (this->frame_queue_) {
    vQueueDelete(this->frame_queue_);
    this->frame_queue_ = nullptr;
  }

  this->streaming_active_ = false;
  ESP_LOGI(TAG, "Streaming arrêté");
  return true;
}

// Prendre un snapshot (simple fonction)
bool Tab5Camera::take_snapshot() {
  // Implémentation dépend du contrôleur, exemple simple
  if (!this->camera_initialized_) {
    this->set_error_("Camera non initialisée");
    return false;
  }
  // Appel fonction capture frame
  ESP_LOGI(TAG, "Snapshot pris");
  return true;
}

// Tâche de streaming statique (conforme header)
void Tab5Camera::streaming_task(void *parameter) {
  Tab5Camera *self = static_cast<Tab5Camera *>(parameter);
  if (self == nullptr) {
    vTaskDelete(nullptr);
    return;
  }
  self->streaming_loop_();
  vTaskDelete(nullptr);
}

// Boucle streaming (non statique)
void Tab5Camera::streaming_loop_() {
  ESP_LOGI(TAG, "Streaming loop démarrée");
  while (!this->streaming_should_stop_) {
    // Exemple lecture frame
    // esp_cam_ctlr_capture_frame(...)

    // Simuler traitement frame et appeler callback
    uint8_t *frame = this->get_frame_buffer();
    size_t size = this->get_frame_buffer_size();

    // Appeler callbacks enregistrés
    this->on_frame_callbacks_.call(frame, size);

    vTaskDelay(pdMS_TO_TICKS(1000 / this->framerate_));
  }
  ESP_LOGI(TAG, "Streaming loop terminée");
}

// Callbacks statiques (renommées comme dans header)
bool IRAM_ATTR Tab5Camera::camera_get_new_vb_callback(esp_cam_ctlr_handle_t handle,
                                                      esp_cam_ctlr_trans_t *trans,
                                                      void *user_data) {
  (void)handle;
  (void)trans;
  (void)user_data;
  // Traitement spécifique
  return true;
}

bool IRAM_ATTR Tab5Camera::camera_get_finished_trans_callback(esp_cam_ctlr_handle_t handle,
                                                              esp_cam_ctlr_trans_t *trans,
                                                              void *user_data) {
  (void)handle;
  (void)trans;
  (void)user_data;
  // Traitement spécifique
  return true;
}

// Gestion des erreurs
void Tab5Camera::set_error_(const std::string &error) {
  this->error_state_ = true;
  this->last_error_ = error;
  ESP_LOGE(TAG, "Erreur: %s", error.c_str());
}

void Tab5Camera::clear_error_() {
  this->error_state_ = false;
  this->last_error_.clear();
}

// TODO : parse_pixel_format_(), calculate_frame_size_() ...

#else  // HAS_ESP32_P4_CAMERA non défini

void Tab5Camera::setup() {
  ESP_LOGW(TAG, "Caméra non supportée sur cette plateforme");
}

void Tab5Camera::dump_config() {
  ESP_LOGW(TAG, "Caméra non supportée");
}

float Tab5Camera::get_setup_priority() const { return setup_priority::PROCESSOR; }

bool Tab5Camera::start_streaming() { return false; }
bool Tab5Camera::stop_streaming() { return false; }
bool Tab5Camera::take_snapshot() { return false; }

#endif  // HAS_ESP32_P4_CAMERA

}  // namespace tab5_camera
}  // namespace esphome

#endif  // USE_ESP32










