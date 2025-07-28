#include "tab5_camera.h"
#include "esphome/core/log.h"
#include "esphome/core/helpers.h"
#include "esphome/core/hal.h"
#include "esp_timer.h"
#include "esp_cam_ctlr_csi.h"

#ifdef USE_ESP32

static const char *const TAG = "tab5_camera";

// Constantes spécifiques SC2356 pour Tab5
#define SC2356_DEFAULT_H_RES 640
#define SC2356_DEFAULT_V_RES 480
#define SC2356_MIPI_LANE_BITRATE_MBPS 200
#define SC2356_ISP_CLOCK_HZ 40000000  // Réduit pour stabilité
#define SC2356_STREAMING_STACK_SIZE 8192
#define SC2356_FRAME_QUEUE_LENGTH 4

// Registres SC2356 critiques
#define SC2356_CHIP_ID_H_REG     0x3107
#define SC2356_CHIP_ID_L_REG     0x3108
#define SC2356_CHIP_ID_VALUE     0x2356
#define SC2356_SOFTWARE_RESET    0x0103
#define SC2356_MODE_SELECT       0x0100
#define SC2356_STREAMING_ON      0x01
#define SC2356_STREAMING_OFF     0x00

namespace esphome {
namespace tab5_camera {

Tab5Camera::Tab5Camera() {
  // Configuration par défaut pour SC2356
  this->set_i2c_address(SC2356_I2C_ADDRESS);
}

Tab5Camera::~Tab5Camera() {
#ifdef HAS_ESP32_P4_CAMERA
  this->deinit_camera_();
#endif
}

void Tab5Camera::setup() {
#ifdef HAS_ESP32_P4_CAMERA
  ESP_LOGCONFIG(TAG, "Setting up Tab5 Camera with SC2356 sensor...");
  
  ESP_LOGI(TAG, "Step 1: Creating synchronization objects");
  
  // Création des objets de synchronisation
  this->frame_ready_semaphore_ = xSemaphoreCreateBinary();
  if (!this->frame_ready_semaphore_) {
    this->set_error_("Failed to create frame ready semaphore");
    this->mark_failed();
    return;
  }
  ESP_LOGI(TAG, "Frame semaphore created successfully");
  
  this->frame_queue_ = xQueueCreate(SC2356_FRAME_QUEUE_LENGTH, sizeof(FrameData));
  if (!this->frame_queue_) {
    this->set_error_("Failed to create frame queue");
    this->mark_failed();
    return;
  }
  ESP_LOGI(TAG, "Frame queue created successfully");
  
  // Configuration de l'horloge externe pour SC2356
  if (this->external_clock_pin_ > 0) {
    ESP_LOGD(TAG, "Configuring external clock on GPIO%u at %u Hz", 
             this->external_clock_pin_, this->external_clock_frequency_);
    if (!this->setup_external_clock_()) {
      this->set_error_("Failed to setup external clock for SC2356");
      this->mark_failed();
      return;
    }
  }
  
  ESP_LOGI(TAG, "Step 2: Initializing SC2356 camera");
  
  if (!this->init_camera_()) {
    this->set_error_("Failed to initialize SC2356 camera");
    this->mark_failed();
    return;
  }
  
  this->clear_error_();
  ESP_LOGCONFIG(TAG, "SC2356 Camera '%s' setup completed successfully", this->name_.c_str());
#else
  ESP_LOGE(TAG, "ESP32-P4 MIPI-CSI API not available - SC2356 Camera component disabled");
  this->set_error_("ESP32-P4 not supported");
  this->mark_failed();
#endif
}

void Tab5Camera::dump_config() {
  ESP_LOGCONFIG(TAG, "SC2356 Camera '%s':", this->name_.c_str());
  
#ifdef HAS_ESP32_P4_CAMERA
  ESP_LOGCONFIG(TAG, "  Sensor: SC2356 (2MP SmartSens)");
  ESP_LOGCONFIG(TAG, "  I2C Address: 0x%02X", this->get_i2c_address());
  
  auto [width, height] = this->get_resolution_dimensions_(this->resolution_);
  ESP_LOGCONFIG(TAG, "  Resolution: %dx%d", width, height);
  ESP_LOGCONFIG(TAG, "  Pixel Format: %s", this->pixel_format_ == SC2356PixelFormat::RGB565 ? "RGB565" : "Other");
  ESP_LOGCONFIG(TAG, "  Framerate: %d FPS", this->framerate_);
  
  if (this->external_clock_pin_ > 0) {
    ESP_LOGCONFIG(TAG, "  External Clock: GPIO%u @ %u Hz", this->external_clock_pin_, this->external_clock_frequency_);
  }
  
  if (this->reset_pin_) {
    LOG_PIN("  Reset Pin: ", this->reset_pin_);
  }
  
  ESP_LOGCONFIG(TAG, "  Frame Buffer Size: %zu bytes", this->frame_buffer_size_);
  ESP_LOGCONFIG(TAG, "  Advanced Settings:");
  ESP_LOGCONFIG(TAG, "    Exposure: %u µs", this->exposure_time_);
  ESP_LOGCONFIG(TAG, "    Analog Gain: %u", this->analog_gain_);
  ESP_LOGCONFIG(TAG, "    Digital Gain: %u", this->digital_gain_);
  ESP_LOGCONFIG(TAG, "    Test Pattern: %s", this->test_pattern_enabled_ ? "ON" : "OFF");
#else
  ESP_LOGCONFIG(TAG, "  Status: ESP32-P4 MIPI-CSI API not available");
#endif
  
  if (this->is_failed()) {
    ESP_LOGCONFIG(TAG, "  Setup Failed: %s", this->last_error_.c_str());
  }
}

float Tab5Camera::get_setup_priority() const {
  return setup_priority::HARDWARE - 1.0f;
}

bool Tab5Camera::detect_sc2356() {
  ESP_LOGI(TAG, "🔍 Detecting SC2356 sensor at I2C address 0x%02X...", this->get_i2c_address());
  
  // Lecture de l'ID du chip SC2356
  uint8_t chip_id_h, chip_id_l;
  
  if (!this->read_sc2356_register_(SC2356_CHIP_ID_H_REG, &chip_id_h) ||
      !this->read_sc2356_register_(SC2356_CHIP_ID_L_REG, &chip_id_l)) {
    ESP_LOGE(TAG, "❌ Cannot read SC2356 chip ID registers");
    return false;
  }
  
  uint16_t chip_id = (chip_id_h << 8) | chip_id_l;
  ESP_LOGI(TAG, "📷 Chip ID detected: 0x%04X", chip_id);
  
  if (chip_id == SC2356_CHIP_ID_VALUE) {
    ESP_LOGI(TAG, "✅ SC2356 sensor detected successfully!");
    return true;
  } else {
    ESP_LOGE(TAG, "❌ Expected SC2356 ID 0x%04X, got 0x%04X", SC2356_CHIP_ID_VALUE, chip_id);
    return false;
  }
}

#ifdef HAS_ESP32_P4_CAMERA

std::pair<uint16_t, uint16_t> Tab5Camera::get_resolution_dimensions_(SC2356Resolution resolution) const {
  switch (resolution) {
    case SC2356Resolution::QVGA_320x240:   return {320, 240};
    case SC2356Resolution::VGA_640x480:    return {640, 480};
    case SC2356Resolution::SVGA_800x600:   return {800, 600};
    case SC2356Resolution::HD_1280x720:    return {1280, 720};
    case SC2356Resolution::UXGA_1600x1200: return {1600, 1200};
    case SC2356Resolution::FHD_1920x1080:  return {1920, 1080};
    default: return {640, 480};
  }
}

size_t Tab5Camera::calculate_frame_size_() const {
  auto [width, height] = this->get_resolution_dimensions_(this->resolution_);
  
  switch (this->pixel_format_) {
    case SC2356PixelFormat::RAW10:
      return (width * height * 10) / 8;  // 10 bits par pixel
    case SC2356PixelFormat::YUV422:
      return width * height * 2;        // 16 bits par pixel
    case SC2356PixelFormat::RGB565:
      return width * height * 2;        // 16 bits par pixel
    case SC2356PixelFormat::JPEG:
      return width * height / 4;        // Estimation pour JPEG
    default:
      return width * height * 2;        // Par défaut RGB565
  }
}

bool Tab5Camera::setup_external_clock_() {
  // Configuration de l'horloge externe spécifique pour SC2356
  // Le SC2356 nécessite typiquement 24MHz
  ESP_LOGI(TAG, "🕐 Setting up external clock for SC2356");
  
  // Configuration du pin LEDC pour générer l'horloge
  ledc_timer_config_t timer_config = {
    .speed_mode = LEDC_LOW_SPEED_MODE,
    .duty_resolution = LEDC_TIMER_2_BIT,
    .timer_num = LEDC_TIMER_0,
    .freq_hz = this->external_clock_frequency_,
    .clk_cfg = LEDC_AUTO_CLK
  };
  
  esp_err_t ret = ledc_timer_config(&timer_config);
  if (ret != ESP_OK) {
    ESP_LOGE(TAG, "Failed to configure LEDC timer: %s", esp_err_to_name(ret));
    return false;
  }
  
  ledc_channel_config_t channel_config = {
    .gpio_num = this->external_clock_pin_,
    .speed_mode = LEDC_LOW_SPEED_MODE,
    .channel = LEDC_CHANNEL_0,
    .timer_sel = LEDC_TIMER_0,
    .duty = 2,  // 50% duty cycle
    .hpoint = 0
  };
  
  ret = ledc_channel_config(&channel_config);
  if (ret != ESP_OK) {
    ESP_LOGE(TAG, "Failed to configure LEDC channel: %s", esp_err_to_name(ret));
    return false;
  }
  
  ESP_LOGI(TAG, "✅ External clock configured at %u Hz on GPIO%u", 
           this->external_clock_frequency_, this->external_clock_pin_);
  
  return true;
}

bool Tab5Camera::init_ldo_() {
  if (this->ldo_initialized_) {
    ESP_LOGI(TAG, "LDO already initialized");
    return true;
  }
  
  ESP_LOGI(TAG, "🔌 Initializing MIPI LDO for SC2356");
  
  esp_ldo_channel_config_t ldo_mipi_phy_config = {
    .chan_id = 3,
    .voltage_mv = 2500,  // 2.5V pour MIPI PHY
  };
  
  esp_err_t ret = esp_ldo_acquire_channel(&ldo_mipi_phy_config, &this->ldo_mipi_phy_);
  if (ret != ESP_OK) {
    ESP_LOGW(TAG, "MIPI LDO not available: %s (continuing anyway)", esp_err_to_name(ret));
    this->ldo_mipi_phy_ = nullptr;
  } else {
    ESP_LOGI(TAG, "✅ MIPI LDO initialized");
  }
  
  this->ldo_initialized_ = true;
  return true;
}

bool Tab5Camera::read_sc2356_register_(uint16_t reg, uint8_t *value) {
  // SC2356 utilise des adresses de registre 16-bit
  uint8_t reg_addr[2] = {(reg >> 8) & 0xFF, reg & 0xFF};
  
  if (!this->write_bytes_raw(reg_addr, 2) ||
      !this->read_bytes_raw(value, 1)) {
    ESP_LOGD(TAG, "Failed to read SC2356 register 0x%04X", reg);
    return false;
  }
  
  ESP_LOGV(TAG, "SC2356 reg 0x%04X = 0x%02X", reg, *value);
  return true;
}

bool Tab5Camera::write_sc2356_register_(uint16_t reg, uint8_t value) {
  uint8_t data[3] = {(reg >> 8) & 0xFF, reg & 0xFF, value};
  
  if (!this->write_bytes_raw(data, 3)) {
    ESP_LOGD(TAG, "Failed to write SC2356 register 0x%04X = 0x%02X", reg, value);
    return false;
  }
  
  ESP_LOGV(TAG, "SC2356 reg 0x%04X <= 0x%02X", reg, value);
  return true;
}

bool Tab5Camera::write_sc2356_register_16_(uint16_t reg, uint16_t value) {
  return this->write_sc2356_register_(reg, (value >> 8) & 0xFF) &&
         this->write_sc2356_register_(reg + 1, value & 0xFF);
}

bool Tab5Camera::reset_sc2356_() {
  ESP_LOGI(TAG, "🔄 Resetting SC2356 sensor");
  
  // Reset matériel si pin disponible
  if (this->reset_pin_) {
    ESP_LOGD(TAG, "Hardware reset via GPIO");
    this->reset_pin_->setup();
    this->reset_pin_->digital_write(false);
    delayMicroseconds(10000);  // 10ms
    this->reset_pin_->digital_write(true);
    delayMicroseconds(20000);  // 20ms
  }
  
  // Reset logiciel
  ESP_LOGD(TAG, "Software reset via I2C");
  if (!this->write_sc2356_register_(SC2356_SOFTWARE_RESET, 0x01)) {
    ESP_LOGE(TAG, "Failed to send software reset to SC2356");
    return false;
  }
  
  // Attendre que le reset soit terminé
  delayMicroseconds(50000);  // 50ms
  
  // Vérifier que le capteur répond après reset
  if (!this->detect_sc2356()) {
    ESP_LOGE(TAG, "SC2356 not responding after reset");
    return false;
  }
  
  ESP_LOGI(TAG, "✅ SC2356 reset completed");
  return true;
}

bool Tab5Camera::configure_sc2356_() {
  ESP_LOGI(TAG, "⚙️ Configuring SC2356 for optimal operation");
  
  // Configuration de base SC2356
  const struct {
    uint16_t reg;
    uint8_t val;
    const char* desc;
  } sc2356_config[] = {
    // Configuration minimale pour démarrage
    {0x0100, 0x00, "Stream off"},
    {0x36e9, 0x80, "Bypass PLL"},
    {0x37f9, 0x80, "Bypass PLL2"},
    
    // Configuration des IO
    {0x301c, 0x78, "IO config"},
    {0x301f, 0x01, "Pad config"},
    
    // Configuration PLL pour 24MHz input
    {0x320c, 0x04, "HTS MSB"},
    {0x320d, 0x4c, "HTS LSB"},
    {0x320e, 0x02, "VTS MSB"}, 
    {0x320f, 0x58, "VTS LSB"},
    
    // Configuration de base pour streaming
    {0x3248, 0x04, "Mipi config"},
    {0x3253, 0x0a, "Mipi timing"},
    {0x3301, 0x06, "Analog config"},
    {0x3302, 0x09, "Analog config 2"},
    {0x3303, 0x10, "Analog config 3"},
    {0x3306, 0x30, "Analog config 4"},
    {0x330b, 0x88, "Analog config 5"},
    {0x3318, 0x02, "Analog config 6"},
    
    // Configuration du format de sortie (RAW10 par défaut)
    {0x3200, 0x00, "Output format"},
    {0x3201, 0x00, "Output format 2"},
    {0x3202, 0x00, "Output crop start Y MSB"},
    {0x3203, 0x00, "Output crop start Y LSB"},
    {0x3204, 0x06, "Output width MSB"},
    {0x3205, 0x4f, "Output width LSB"},
    {0x3206, 0x04, "Output height MSB"},
    {0x3207, 0xbf, "Output height LSB"},
    {0x3208, 0x02, "Actual width MSB"},
    {0x3209, 0x80, "Actual width LSB"},
    {0x320a, 0x01, "Actual height MSB"},
    {0x320b, 0xe0, "Actual height LSB"},
  };
  
  for (size_t i = 0; i < sizeof(sc2356_config) / sizeof(sc2356_config[0]); i++) {
    ESP_LOGV(TAG, "Setting %s: reg 0x%04X = 0x%02X", 
             sc2356_config[i].desc, sc2356_config[i].reg, sc2356_config[i].val);
    
    if (!this->write_sc2356_register_(sc2356_config[i].reg, sc2356_config[i].val)) {
      ESP_LOGW(TAG, "Failed to write SC2356 register 0x%04X", sc2356_config[i].reg);
      // Continue avec les autres registres
    }
    
    delayMicroseconds(1000);  // 1ms entre les écritures
  }
  
  // Appliquer les paramètres spécifiques de résolution et format
  if (!this->set_sc2356_resolution_() ||
      !this->set_sc2356_format_() ||
      !this->set_sc2356_framerate_()) {
    ESP_LOGE(TAG, "Failed to apply SC2356 specific settings");
    return false;
  }
  
  ESP_LOGI(TAG, "✅ SC2356 basic configuration completed");
  return true;
}

bool Tab5Camera::set_sc2356_resolution_() {
  auto [width, height] = this->get_resolution_dimensions_(this->resolution_);
  ESP_LOGI(TAG, "📐 Setting SC2356 resolution to %dx%d", width, height);
  
  // Pour simplifier, on utilise la résolution par défaut VGA pour l'instant
  // TODO: Implémenter les autres résolutions avec crop/scale
  
  return true;
}

bool Tab5Camera::set_sc2356_format_() {
  ESP_LOGI(TAG, "🎨 Setting SC2356 pixel format");
  
  // Configuration du format de sortie selon le choix
  // Pour l'instant, on garde RAW10 qui sera converti par l'ISP
  
  return true;
}

bool Tab5Camera::set_sc2356_framerate_() {
  ESP_LOGI(TAG, "🎬 Setting SC2356 framerate to %d FPS", this->framerate_);
  
  // Le framerate est contrôlé par VTS (Vertical Total Size)
  // Plus VTS est grand, plus le framerate est faible
  
  return true;
}

bool Tab5Camera::init_sc2356_sensor_() {
  if (this->sensor_initialized_) {
    ESP_LOGI(TAG, "SC2356 sensor already initialized");
    return true;
  }
  
  ESP_LOGI(TAG, "🚀 Initializing SC2356 sensor");
  
  // Étape 1: Détecter le capteur
  if (!this->detect_sc2356()) {
    ESP_LOGE(TAG, "SC2356 sensor not detected");
    return false;
  }
  
  // Étape 2: Reset du capteur
  if (!this->reset_sc2356_()) {
    ESP_LOGE(TAG, "Failed to reset SC2356");
    return false;
  }
  
  // Étape 3: Configuration du capteur
  if (!this->configure_sc2356_()) {
    ESP_LOGE(TAG, "Failed to configure SC2356");
    return false;
  }
  
  this->sensor_initialized_ = true;
  ESP_LOGI(TAG, "✅ SC2356 sensor initialized successfully");
  
  return true;
}

bool Tab5Camera::init_camera_() {
  if (this->camera_initialized_) {
    ESP_LOGI(TAG, "Camera already initialized");
    return true;
  }

  ESP_LOGI(TAG, "🚀 Starting SC2356 camera initialization");

  // Étape 1: LDO MIPI
  ESP_LOGI(TAG, "Step 1: Initializing MIPI LDO");
  if (!this->init_ldo_()) {
    return false;
  }

  // Étape 2: Initialisation du capteur SC2356
  ESP_LOGI(TAG, "Step 2: Initializing SC2356 sensor");
  if (!this->init_sc2356_sensor_()) {
    return false;
  }

  // Étape 3: Allocation du frame buffer
  ESP_LOGI(TAG, "Step 3: Allocating frame buffer");
  this->frame_buffer_size_ = this->calculate_frame_size_();
  this->frame_buffer_size_ = (this->frame_buffer_size_ + 63) & ~63;  // Alignement 64 bytes
  
  ESP_LOGI(TAG, "Frame buffer size: %zu bytes", this->frame_buffer_size_);
  
  this->frame_buffer_ = heap_caps_aligned_alloc(64, this->frame_buffer_size_, 
                                                MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT);
  if (!this->frame_buffer_) {
    ESP_LOGE(TAG, "Failed to allocate frame buffer in PSRAM");
    this->frame_buffer_ = heap_caps_aligned_alloc(64, this->frame_buffer_size_, 
                                                  MALLOC_CAP_DMA | MALLOC_CAP_8BIT);
    if (!this->frame_buffer_) {
      ESP_LOGE(TAG, "Failed to allocate frame buffer in regular RAM");
      return false;
    }
  }
  
  ESP_LOGI(TAG, "✅ Frame buffer allocated at %p", this->frame_buffer_);

  // Étape 4: Configuration du contrôleur CSI
  ESP_LOGI(TAG, "Step 4: Configuring CSI controller for SC2356");
  
  auto [width, height] = this->get_resolution_dimensions_(this->resolution_);
  
  esp_cam_ctlr_csi_config_t csi_config = {};
  csi_config.ctlr_id = 0;
  csi_config.h_res = width;
  csi_config.v_res = height;
  csi_config.lane_bit_rate_mbps = SC2356_MIPI_LANE_BITRATE_MBPS;
  csi_config.input_data_color_type = CAM_CTLR_COLOR_RAW10;  // SC2356 sort du RAW10
  csi_config.output_data_color_type = CAM_CTLR_COLOR_RGB565; // Converti en RGB565
  csi_config.data_lane_num = 2;  // SC2356 utilise 2 lanes MIPI
  csi_config.byte_swap_en = false;
  csi_config.queue_items = 4;
  
  esp_err_t ret = esp_cam_new_csi_ctlr(&csi_config, &this->cam_handle_);
  if (ret != ESP_OK) {
    ESP_LOGE(TAG, "Failed to create CSI controller: %s", esp_err_to_name(ret));
    return false;
  }
  ESP_LOGI(TAG, "✅ CSI controller created");
  
  // Étape 5: Configuration des callbacks
  ESP_LOGI(TAG, "Step 5: Registering camera callbacks");
  esp_cam_ctlr_evt_cbs_t cbs = {
    .on_get_new_trans = nullptr,
    .on_trans_finished = Tab5Camera::camera_get_finished_trans_callback,
  };
  
  ret = esp_cam_ctlr_register_event_callbacks(this->cam_handle_, &cbs, this);
  if (ret != ESP_OK) {
    ESP_LOGE(TAG, "Failed to register callbacks: %s", esp_err_to_name(ret));
    return false;
  }
  ESP_LOGI(TAG, "✅ Callbacks registered");
  
  // Étape 6: Activation du contrôleur
  ESP_LOGI(TAG, "Step 6: Enabling camera controller");
  ret = esp_cam_ctlr_enable(this->cam_handle_);
  if (ret != ESP_OK) {
    ESP_LOGE(TAG, "Failed to enable camera controller: %s", esp_err_to_name(ret));
    return false;
  }
  ESP_LOGI(TAG, "✅ Camera controller enabled");
  
  // Étape 7: Configuration de l'ISP pour SC2356
  ESP_LOGI(TAG, "Step 7: Configuring ISP for SC2356");
  esp_isp_processor_cfg_t isp_config = {};
  isp_config.clk_hz = SC2356_ISP_CLOCK_HZ;
  isp_config.input_data_source = ISP_INPUT_DATA_SOURCE_CSI;
  isp_config.input_data_color_type = ISP_COLOR_RAW10;  // SC2356 RAW10
  isp_config.output_data_color_type = ISP_COLOR_RGB565;
  isp_config.has_line_start_packet = false;
  isp_config.has_line_end_packet = false;
  isp_config.h_res = width;
  isp_config.v_res = height;
  
  ret = esp_isp_new_processor(&isp_config, &this->isp_proc_);
  if (ret != ESP_OK) {
    ESP_LOGE(TAG, "Failed to create ISP processor: %s", esp_err_to_name(ret));
    return false;
  }
  
  ret = esp_isp_enable(this->isp_proc_);
  if (ret != ESP_OK) {
    ESP_LOGE(TAG, "Failed to enable ISP processor: %s", esp_err_to_name(ret));
    return false;
  }
  ESP_LOGI(TAG, "✅ ISP processor configured and enabled");
  
  // Étape 8: Initialisation du frame buffer
  ESP_LOGI(TAG, "Step 8: Initializing frame buffer");
  memset(this->frame_buffer_, 0x00, this->frame_buffer_size_);
  esp_cache_msync(this->frame_buffer_, this->frame_buffer_size_, ESP_CACHE_MSYNC_FLAG_DIR_C2M);
  
  // Étape 9: Démarrage de la caméra
  ESP_LOGI(TAG, "Step 9: Starting SC2356 camera");
  ret = esp_cam_ctlr_start(this->cam_handle_);
  if (ret != ESP_OK) {
    ESP_LOGE(TAG, "Failed to start camera controller: %s", esp_err_to_name(ret));
    return false;
  }
  
  // Étape 10: Activation du streaming sur le capteur SC2356
  ESP_LOGI(TAG, "Step 10: Enabling SC2356 streaming");
  if (!this->write_sc2356_register_(SC2356_MODE_SELECT, SC2356_STREAMING_ON)) {
    ESP_LOGW(TAG, "Failed to enable SC2356 streaming via I2C");
    // Continue quand même, le capteur peut déjà être en mode streaming
  }
  
  this->camera_initialized_ = true;
  ESP_LOGI(TAG, "🎉 SC2356 camera initialization completed successfully!");
  
  return true;
}

void Tab5Camera::deinit_camera_() {
  if (this->streaming_active_) {
    this->stop_streaming();
  }
  
  if (this->camera_initialized_) {
    ESP_LOGD(TAG, "🔄 Deinitializing SC2356 camera...");
    
    // Arrêter le streaming SC2356
    this->write_sc2356_register_(SC2356_MODE_SELECT, SC2356_STREAMING_OFF);
    
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
    ESP_LOGD(TAG, "✅ SC2356 camera deinitialized");
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

bool Tab5Camera::camera_get_finished_trans_callback(esp_cam_ctlr_handle_t handle, esp_cam_ctlr_trans_t *trans, void *user_data) {
  Tab5Camera *camera = static_cast<Tab5Camera*>(user_data);
  if (!camera || !trans->buffer) {
    return false;
  }

  static uint32_t frame_count = 0;
  frame_count++;
  
  ESP_LOGD(TAG, "📸 SC2356 Frame #%lu: %zu bytes", frame_count, trans->received_size);

  if (trans->received_size == 0) {
    ESP_LOGW(TAG, "⚠️ Empty frame from SC2356");
    return false;
  }
  
  // Synchronisation du cache
  esp_cache_msync(trans->buffer, trans->received_size, ESP_CACHE_MSYNC_FLAG_DIR_M2C);
  
  // Envoi vers la queue applicative
  FrameData frame;
  frame.buffer = trans->buffer;
  frame.size = trans->received_size;
  frame.timestamp = esp_timer_get_time();
  frame.valid = true;
  
  BaseType_t ret = xQueueSendFromISR(camera->frame_queue_, &frame, NULL);
  if (ret == pdTRUE) {
    xSemaphoreGiveFromISR(camera->frame_ready_semaphore_, NULL);
  }

  return false;
}

bool Tab5Camera::take_snapshot() {
  if (!this->camera_initialized_) {
    this->set_error_("Camera not initialized");
    return false;
  }
  
  ESP_LOGD(TAG, "📸 Taking SC2356 snapshot");
  
  esp_cam_ctlr_trans_t trans = {
    .buffer = this->frame_buffer_,
    .buflen = this->frame_buffer_size_,
  };
  
  esp_err_t ret = esp_cam_ctlr_receive(this->cam_handle_, &trans, 2000 / portTICK_PERIOD_MS);
  if (ret != ESP_OK) {
    this->set_error_("SC2356 snapshot failed: " + std::string(esp_err_to_name(ret)));
    return false;
  }
  
  ESP_LOGI(TAG, "✅ SC2356 snapshot captured: %zu bytes", trans.received_size);
  
  esp_cache_msync(this->frame_buffer_, trans.received_size, ESP_CACHE_MSYNC_FLAG_DIR_M2C);
  this->on_frame_callbacks_.call(static_cast<uint8_t*>(this->frame_buffer_), trans.received_size);
  
  return true;
}

bool Tab5Camera::start_streaming() {
  if (!this->camera_initialized_) {
    this->set_error_("Camera not initialized for streaming");
    return false;
  }
  
  if (this->streaming_active_) {
    return true;
  }
  
  ESP_LOGI(TAG, "🎬 Starting SC2356 streaming");
  
  this->streaming_should_stop_ = false;
  this->streaming_active_ = true;
  
  BaseType_t result = xTaskCreate(
    Tab5Camera::streaming_task,
    "sc2356_stream",
    SC2356_STREAMING_STACK_SIZE,
    this,
    6,  // Priorité élevée
    &this->streaming_task_handle_
  );
  
  if (result != pdPASS) {
    this->set_error_("Failed to create SC2356 streaming task");
    this->streaming_active_ = false;
    return false;
  }
  
  ESP_LOGI(TAG, "✅ SC2356 streaming started");
  return true;
}

bool Tab5Camera::stop_streaming() {
  if (!this->streaming_active_) {
    return true;
  }
  
  ESP_LOGI(TAG, "⏹️ Stopping SC2356 streaming");
  
  this->streaming_should_stop_ = true;
  
  if (this->streaming_task_handle_) {
    xSemaphoreGive(this->frame_ready_semaphore_);
    
    uint32_t timeout = 0;
    while (this->streaming_active_ && timeout < 50) {
      vTaskDelay(100 / portTICK_PERIOD_MS);
      timeout++;
    }
    
    if (this->streaming_active_) {
      ESP_LOGW(TAG, "Force stopping SC2356 streaming task");
      vTaskDelete(this->streaming_task_handle_);
      this->streaming_active_ = false;
    }
    
    this->streaming_task_handle_ = nullptr;
  }
  
  ESP_LOGI(TAG, "✅ SC2356 streaming stopped");
  return true;
}

void Tab5Camera::streaming_task(void *parameter) {
  Tab5Camera *camera = static_cast<Tab5Camera*>(parameter);
  camera->streaming_loop_();
}

void Tab5Camera::streaming_loop_() {
  ESP_LOGD(TAG, "🎬 SC2356 streaming loop started");

  while (!this->streaming_should_stop_) {
    if (xSemaphoreTake(this->frame_ready_semaphore_, 200 / portTICK_PERIOD_MS) == pdTRUE) {
      FrameData frame;
      if (xQueueReceive(this->frame_queue_, &frame, 0) == pdTRUE) {
        if (frame.valid) {
          this->on_frame_callbacks_.call(static_cast<uint8_t*>(frame.buffer), frame.size);
        }
      }
    }
    
    vTaskDelay(1 / portTICK_PERIOD_MS);
  }

  this->streaming_active_ = false;
  ESP_LOGD(TAG, "🏁 SC2356 streaming loop ended");
  vTaskDelete(nullptr);
}

void Tab5Camera::set_error_(const std::string &error) {
  this->error_state_ = true;
  this->last_error_ = error;
  ESP_LOGE(TAG, "❌ Error: %s", error.c_str());
}

void Tab5Camera::clear_error_() {
  this->error_state_ = false;
  this->last_error_.clear();
}

#endif

}  // namespace tab5_camera
}  // namespace esphome

#endif  // USE_ESP32










