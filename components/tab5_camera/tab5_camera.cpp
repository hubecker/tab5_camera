#include "tab5_camera.h"
#include "esphome/core/log.h"
#include "esphome/core/helpers.h"
#include "esphome/core/hal.h"
#include "esp_timer.h"



#ifdef USE_ESP32
#ifdef HAS_ESP32_P4_CAMERA

static const char *const TAG = "tab5_camera";

// Constantes de configuration
#define TAB5_MIPI_CSI_LANE_BITRATE_MBPS 400
#define TAB5_ISP_CLOCK_HZ 80000000  // 80MHz

namespace esphome {
namespace tab5_camera {

// Déclaration des nouvelles variables membres pour les tampons et transactions
uint8_t **dma_buffers_ = nullptr;
esp_cam_ctlr_trans_t *transactions_ = nullptr;

Tab5Camera::~Tab5Camera() {
this->deinit_camera_();
}

bool Tab5Camera::write_sensor_register_(uint16_t reg, uint8_t value) {
  uint8_t data[3];
  data[0] = (reg >> 8) & 0xFF;  // MSB
  data[1] = reg & 0xFF;         // LSB
  data[2] = value;

  if (!this->write(data, 3)) {
    ESP_LOGE(TAG, "Failed to write register 0x%04X = 0x%02X", reg, value);
    return false;
  }
  return true;
}

bool Tab5Camera::read_sensor_register_(uint16_t reg, uint8_t *value) {
  uint8_t reg_data[2] = { uint8_t(reg >> 8), uint8_t(reg & 0xFF) };
  if (!this->write(reg_data, 2, false)) {  // false = pas de stop
    ESP_LOGE(TAG, "Failed to set register address for read: 0x%04X", reg);
    return false;
  }

  if (!this->read(value, 1)) {
    ESP_LOGE(TAG, "Failed to read register 0x%04X", reg);
    return false;
  }

  return true;
}


// Fonction pour forcer le mode RAW8 du capteur
void Tab5Camera::force_sensor_raw8_mode() {
  ESP_LOGI(TAG, "Configuring sensor for RAW8 mode...");
  
  // Séquence I2C minimale pour forcer le RAW8 (souvent capteurs SmartSens)
  // Tu peux adapter selon le datasheet ou ton init_reglist
  this->write_sensor_register_(0x3013, 0x01);  // Soft reset (si besoin)
  delay(10);
  this->write_sensor_register_(0x302A, 0x00);  // RAW8 mode
  this->write_sensor_register_(0x0100, 0x01);  // Start streaming
  
  ESP_LOGI(TAG, "Sensor configured for RAW8 mode");
}

void Tab5Camera::setup() {
ESP_LOGCONFIG(TAG, "Setting up Tab5 Camera with ESP32-P4 MIPI-CSI...");

// La file d'attente stockera des pointeurs vers les transactions terminées
this->frame_queue_ = xQueueCreate(FRAME_QUEUE_SIZE, sizeof(esp_cam_ctlr_trans_t *));
if (!this->frame_queue_) {
  ESP_LOGE(TAG, "Failed to create frame queue");
  this->mark_failed();
  return;
}
ESP_LOGI(TAG, "Frame queue created successfully");

if (!this->init_camera_()) {
  ESP_LOGE(TAG, "Failed to initialize camera - setup marked as failed");
  this->mark_failed();
  return;
}

ESP_LOGCONFIG(TAG, "Tab5 Camera '%s' setup completed successfully", this->name_.c_str());
}

void Tab5Camera::dump_config() {
ESP_LOGCONFIG(TAG, "Tab5 Camera '%s':", this->name_.c_str());
LOG_I2C_DEVICE(this);
if (this->is_failed()) {
  ESP_LOGE(TAG, "  Component has failed to setup.");
  return;
}
ESP_LOGCONFIG(TAG, "  Resolution: %dx%d", this->frame_width_, this->frame_height_);
ESP_LOGCONFIG(TAG, "  Framerate: %d fps", this->framerate_);
ESP_LOGCONFIG(TAG, "  JPEG Quality: %d", this->jpeg_quality_);
if (this->reset_pin_) {
  LOG_PIN("  Reset Pin: ", this->reset_pin_);
}
if (this->external_clock_pin_ > 0) {
  ESP_LOGCONFIG(TAG, "  External Clock Pin: GPIO%u", this->external_clock_pin_);
  ESP_LOGCONFIG(TAG, "  External Clock Frequency: %u Hz", this->external_clock_frequency_);
}
ESP_LOGCONFIG(TAG, "  Streaming Buffers: %d", FRAME_BUFFER_COUNT);
}

float Tab5Camera::get_setup_priority() const {
return setup_priority::HARDWARE - 1.0f;
}

bool Tab5Camera::init_ldo_() {
if (this->ldo_initialized_) return true;
ESP_LOGI(TAG, "Initializing MIPI LDO regulator");
esp_ldo_channel_config_t ldo_mipi_phy_config = {
    .chan_id = 3,
    .voltage_mv = 2500,
};
esp_err_t ret = esp_ldo_acquire_channel(&ldo_mipi_phy_config, &this->ldo_mipi_phy_);
if (ret != ESP_OK) {
  ESP_LOGW(TAG, "Failed to acquire MIPI LDO channel: %s. Some boards may not need this.", esp_err_to_name(ret));
}
this->ldo_initialized_ = true;
return true;
}

bool Tab5Camera::init_sensor_() {
if (this->sensor_initialized_) return true;
ESP_LOGI(TAG, "Initializing camera sensor at I2C address 0x%02X", this->address_);

if (this->reset_pin_) {
  ESP_LOGI(TAG, "Resetting camera sensor...");
  this->reset_pin_->setup();
  this->reset_pin_->digital_write(false);
  delay(10);
  this->reset_pin_->digital_write(true);
  delay(10);
}

// Configuration du capteur en mode RAW8
this->force_sensor_raw8_mode();

this->sensor_initialized_ = true;
ESP_LOGI(TAG, "Camera sensor initialized");
return true;
}

bool Tab5Camera::init_camera_() {
if (this->camera_initialized_) return true;

if (!this->init_ldo_()) return false;
if (!this->init_sensor_()) return false;

// Calcul de la taille de buffer
size_t buffer_size = this->frame_width_ * this->frame_height_ * 2; // YUV422/RGB565 = 2 bytes/pixel
buffer_size = (buffer_size + 63) & ~63; // Align on 64-byte boundary for cache
ESP_LOGI(TAG, "Allocating %d buffers of size %zu bytes each", FRAME_BUFFER_COUNT, buffer_size);

// Allocation des pointeurs pour les tampons et les transactions
dma_buffers_ = (uint8_t **) heap_caps_calloc(FRAME_BUFFER_COUNT, sizeof(uint8_t *), MALLOC_CAP_SPIRAM);
transactions_ = (esp_cam_ctlr_trans_t *) heap_caps_calloc(FRAME_BUFFER_COUNT, sizeof(esp_cam_ctlr_trans_t), MALLOC_CAP_INTERNAL);
if (!dma_buffers_ || !transactions_) {
  ESP_LOGE(TAG, "Failed to allocate memory for buffer/transaction pointers");
  return false;
}

// Allocation de chaque tampon DMA
for (int i = 0; i < FRAME_BUFFER_COUNT; i++) {
  dma_buffers_[i] = (uint8_t *) heap_caps_aligned_alloc(64, buffer_size, MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT);
  if (!dma_buffers_[i]) {
    ESP_LOGE(TAG, "Failed to allocate DMA buffer %d", i);
    return false;
  }
  transactions_[i].buffer = dma_buffers_[i];
  transactions_[i].buflen = buffer_size;
}
ESP_LOGI(TAG, "DMA buffers allocated successfully.");

// Configuration du contrôleur CSI - Using alternative initialization method to avoid field order issues
esp_cam_ctlr_csi_config_t csi_config;
memset(&csi_config, 0, sizeof(csi_config));
csi_config.ctlr_id = 0;
csi_config.h_res = this->frame_width_;
csi_config.v_res = this->frame_height_;
csi_config.data_lane_num = 2;
csi_config.lane_bit_rate_mbps = TAB5_MIPI_CSI_LANE_BITRATE_MBPS;
csi_config.input_data_color_type = CAM_CTLR_COLOR_RAW8; // Le capteur sort du RAW
csi_config.output_data_color_type = CAM_CTLR_COLOR_YUV422; // L'ISP convertit en YUV422
csi_config.byte_swap_en = false;
csi_config.queue_items = FRAME_BUFFER_COUNT + 2; // Augmenter légèrement la taille de la file pour éviter les timeouts

esp_err_t ret = esp_cam_new_csi_ctlr(&csi_config, &this->cam_handle_);
if (ret != ESP_OK) {
  ESP_LOGE(TAG, "CSI controller init failed: %s", esp_err_to_name(ret));
  return false;
}

// Configuration des callbacks
esp_cam_ctlr_evt_cbs_t cbs = {
    .on_trans_finished = Tab5Camera::camera_get_finished_trans_callback,
};
ret = esp_cam_ctlr_register_event_callbacks(this->cam_handle_, &cbs, this);
if (ret != ESP_OK) {
  ESP_LOGE(TAG, "Failed to register camera callbacks: %s", esp_err_to_name(ret));
  return false;
}

// Configuration de l'ISP
esp_isp_processor_cfg_t isp_config = {
    .clk_hz = TAB5_ISP_CLOCK_HZ,
    .input_data_source = ISP_INPUT_DATA_SOURCE_CSI,
    .input_data_color_type = ISP_COLOR_RAW8,
    .output_data_color_type = ISP_COLOR_YUV422,
    .h_res = this->frame_width_,
    .v_res = this->frame_height_,
};
ret = esp_isp_new_processor(&isp_config, &this->isp_proc_);
if (ret != ESP_OK) {
  ESP_LOGE(TAG, "ISP processor init failed: %s", esp_err_to_name(ret));
  return false;
}

ret = esp_isp_enable(this->isp_proc_);
if (ret != ESP_OK) {
  ESP_LOGE(TAG, "Failed to enable ISP: %s", esp_err_to_name(ret));
  return false;
}

ret = esp_cam_ctlr_enable(this->cam_handle_);
if (ret != ESP_OK) {
  ESP_LOGE(TAG, "Failed to enable camera controller: %s", esp_err_to_name(ret));
  return false;
}

this->camera_initialized_ = true;
ESP_LOGI(TAG, "Camera hardware initialized successfully.");
return true;
}

void Tab5Camera::deinit_camera_() {
if (this->streaming_active_) {
  this->stop_streaming();
}

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

if (dma_buffers_) {
  for (int i = 0; i < FRAME_BUFFER_COUNT; i++) {
    heap_caps_free(dma_buffers_[i]);
  }
  heap_caps_free(dma_buffers_);
  dma_buffers_ = nullptr;
}

if (transactions_) {
  heap_caps_free(transactions_);
  transactions_ = nullptr;
}

if (this->ldo_mipi_phy_) {
  esp_ldo_release_channel(this->ldo_mipi_phy_);
  this->ldo_mipi_phy_ = nullptr;
}

if (this->frame_queue_) {
  vQueueDelete(this->frame_queue_);
  this->frame_queue_ = nullptr;
}

this->camera_initialized_ = false;
ESP_LOGI(TAG, "Camera deinitialized.");
}

bool Tab5Camera::camera_get_finished_trans_callback(esp_cam_ctlr_handle_t handle, esp_cam_ctlr_trans_t *trans, void *user_data) {
Tab5Camera *camera = static_cast<Tab5Camera *>(user_data);
if (camera->streaming_active_) {
  xQueueSendFromISR(camera->frame_queue_, &trans, NULL);
}
return false;
}

bool Tab5Camera::start_streaming() {
if (!this->camera_initialized_) {
  ESP_LOGE(TAG, "Camera not initialized.");
  return false;
}
if (this->streaming_active_) {
  ESP_LOGW(TAG, "Streaming already active.");
  return true;
}

ESP_LOGI(TAG, "Starting streaming...");
this->streaming_should_stop_ = false;

// IMPORTANT: Démarrer le contrôleur AVANT d'envoyer les tampons
esp_err_t ret = esp_cam_ctlr_start(this->cam_handle_);
if (ret != ESP_OK) {
  ESP_LOGE(TAG, "Failed to start camera controller: %s", esp_err_to_name(ret));
  return false;
}

// Maintenant fournir les tampons au pilote avec un délai entre chaque
for (int i = 0; i < FRAME_BUFFER_COUNT; i++) {
  ret = esp_cam_ctlr_receive(this->cam_handle_, &transactions_[i], pdMS_TO_TICKS(100));
  if (ret != ESP_OK) {
    ESP_LOGE(TAG, "Failed to queue buffer %d: %s", i, esp_err_to_name(ret));
    // Arrêter le contrôleur en cas d'échec
    esp_cam_ctlr_stop(this->cam_handle_);
    return false;
  }
  ESP_LOGD(TAG, "Queued buffer %d successfully", i);
  // Petit délai pour laisser le temps au système de traiter
  vTaskDelay(pdMS_TO_TICKS(10));
}

// Créer la tâche de traitement des images
BaseType_t result = xTaskCreate(
    Tab5Camera::streaming_task, "tab5_streaming", STREAMING_TASK_STACK_SIZE, this, STREAMING_TASK_PRIORITY, &this->streaming_task_handle_);

if (result != pdPASS) {
  ESP_LOGE(TAG, "Failed to create streaming task");
  esp_cam_ctlr_stop(this->cam_handle_);
  return false;
}

this->streaming_active_ = true;
ESP_LOGI(TAG, "Streaming started successfully.");
return true;
}

bool Tab5Camera::stop_streaming() {
if (!this->streaming_active_) {
  return true;
}

ESP_LOGI(TAG, "Stopping streaming...");
this->streaming_should_stop_ = true;

if (this->streaming_task_handle_) {
  // Envoyer un message nul pour débloquer la tâche si elle attend sur la file
  esp_cam_ctlr_trans_t *null_trans = nullptr;
  xQueueSend(this->frame_queue_, &null_trans, 0);
  
  // Attendre que la tâche se termine
  vTaskDelay(pdMS_TO_TICKS(200)); 
}

esp_cam_ctlr_stop(this->cam_handle_);

// Vider la file d'attente
esp_cam_ctlr_trans_t *trans;
while(xQueueReceive(this->frame_queue_, &trans, 0) == pdTRUE);

this->streaming_active_ = false;
this->streaming_task_handle_ = nullptr;
ESP_LOGI(TAG, "Streaming stopped.");
return true;
}

void Tab5Camera::streaming_task(void *parameter) {
static_cast<Tab5Camera *>(parameter)->streaming_loop_();
}

void Tab5Camera::streaming_loop_() {
ESP_LOGD(TAG, "Streaming loop started.");
esp_cam_ctlr_trans_t *trans = nullptr;

while (!this->streaming_should_stop_) {
  // Attendre qu'une image soit prête depuis la file d'attente
  if (xQueueReceive(this->frame_queue_, &trans, pdMS_TO_TICKS(1000)) != pdTRUE) {
    continue; // Timeout, on continue la boucle
  }
  
  if (!trans) { // Message nul pour arrêter la tâche
    break;
  }

  // Une image est prête dans trans->buffer avec la taille trans->buflen
  // TODO: Convertir l'image YUV422 en JPEG ici si nécessaire
  
  // Pour l'instant, on appelle les callbacks avec l'image brute
  this->on_frame_callbacks_.call((uint8_t *) trans->buffer, trans->buflen);

  // Renvoyer le tampon au pilote pour qu'il puisse être réutilisé
  esp_err_t ret = esp_cam_ctlr_receive(this->cam_handle_, trans, 0);
  if (ret != ESP_OK) {
    ESP_LOGE(TAG, "Failed to re-queue buffer: %s", esp_err_to_name(ret));
  }
}

ESP_LOGD(TAG, "Streaming loop finished.");
vTaskDelete(nullptr);
}

}  // namespace tab5_camera
}  // namespace esphome

#endif  // HAS_ESP32_P4_CAMERA
#endif  // USE_ESP32
















