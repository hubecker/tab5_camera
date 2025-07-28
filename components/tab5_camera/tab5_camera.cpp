#include "tab5_camera.h"
#include "esphome/core/log.h"
#include "esphome/core/helpers.h"
#include "esphome/core/hal.h"
#include "esp_timer.h"
#include "esp_cam_ctlr_csi.h"

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
  
  // Affichage du capteur détecté si disponible
  if (this->detected_sensor_id_ != 0) {
    ESP_LOGCONFIG(TAG, "  Detected Sensor: ID 0x%04X at address 0x%02X", 
                  this->detected_sensor_id_, this->detected_sensor_address_);
  }
  
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

// NOUVELLE MÉTHODE: Détection automatique du capteur
uint16_t Tab5Camera::detect_sensor_id_() {
  ESP_LOGI(TAG, "Detecting Tab5 camera sensor...");
  
  // Test des adresses I2C connues pour Tab5
  const struct {
    uint8_t address;
    const char* name;
    uint16_t id_reg_high;
    uint16_t id_reg_low;
    uint16_t expected_id;
  } sensor_configs[] = {
    {OV5645_SCCB_ADDR, "OV5645", 0x300A, 0x300B, OV5645_CHIP_ID},
    {SC2336_SCCB_ADDR, "SC2336", 0x3107, 0x3108, SC2336_CHIP_ID},
    {SC2356_SCCB_ADDR, "SC2356", 0x3107, 0x3108, SC2356_CHIP_ID},
    // Adresses alternatives
    {0x78, "OV5645_ALT", 0x300A, 0x300B, OV5645_CHIP_ID},
    {0x60, "SC2336_ALT", 0x3107, 0x3108, SC2336_CHIP_ID},
  };
  
  for (size_t i = 0; i < sizeof(sensor_configs) / sizeof(sensor_configs[0]); i++) {
    ESP_LOGI(TAG, "Testing %s at I2C address 0x%02X...", 
             sensor_configs[i].name, sensor_configs[i].address);
    
    // Temporairement changer l'adresse pour le test
    uint8_t original_addr = this->address_;
    this->address_ = sensor_configs[i].address;
    
    // Test de communication basique
    if (!this->test_sensor_communication_(sensor_configs[i].address)) {
      this->address_ = original_addr;
      continue;
    }
    
    // Lecture des registres d'ID
    uint8_t id_high, id_low;
    if (this->read_byte_16(sensor_configs[i].id_reg_high, &id_high) && 
        this->read_byte_16(sensor_configs[i].id_reg_low, &id_low)) {
      
      uint16_t sensor_id = (id_high << 8) | id_low;
      ESP_LOGI(TAG, "%s ID registers: 0x%04X=0x%02X, 0x%04X=0x%02X, Combined ID=0x%04X", 
               sensor_configs[i].name,
               sensor_configs[i].id_reg_high, id_high, 
               sensor_configs[i].id_reg_low, id_low, 
               sensor_id);
      
      if (sensor_id == sensor_configs[i].expected_id) {
        ESP_LOGI(TAG, "✅ Detected %s sensor (ID: 0x%04X) at address 0x%02X", 
                 sensor_configs[i].name, sensor_id, sensor_configs[i].address);
        
        // Sauvegarder les informations du capteur détecté
        this->detected_sensor_id_ = sensor_id;
        this->detected_sensor_address_ = sensor_configs[i].address;
        
        // Garder cette adresse pour la suite
        return sensor_id;
      }
    }
    
    // Restaurer l'adresse originale si pas de match
    this->address_ = original_addr;
    vTaskDelay(50 / portTICK_PERIOD_MS);
  }
  
  ESP_LOGE(TAG, "❌ No compatible Tab5 sensor detected!");
  return 0;
}

// NOUVELLE MÉTHODE: Test de communication avec un capteur
bool Tab5Camera::test_sensor_communication_(uint8_t address) {
  // Test de lecture de quelques registres pour vérifier la communication
  uint8_t test_data;
  int successful_reads = 0;
  
  // Pour les capteurs OmniVision, tester les registres de base
  if (address == OV5645_SCCB_ADDR || address == 0x78) {
    if (this->read_byte_16(0x300A, &test_data)) successful_reads++;
    if (this->read_byte_16(0x300B, &test_data)) successful_reads++;
    if (this->read_byte_16(0x3008, &test_data)) successful_reads++;
  }
  // Pour les capteurs SmartSens, tester leurs registres
  else if (address == SC2336_SCCB_ADDR || address == SC2356_SCCB_ADDR || address == 0x60) {
    if (this->read_byte_16(0x3107, &test_data)) successful_reads++;
    if (this->read_byte_16(0x3108, &test_data)) successful_reads++;
    if (this->read_byte_16(0x0100, &test_data)) successful_reads++;
  }
  
  return successful_reads >= 2;  // Au moins 2 lectures réussies
}

// MÉTHODE MISE À JOUR: Initialisation du capteur avec détection automatique
bool Tab5Camera::init_sensor_() {
  if (this->sensor_initialized_) {
    ESP_LOGI(TAG, "Sensor already initialized, skipping");
    return true;
  }
  
  ESP_LOGI(TAG, "Starting Tab5 sensor detection and initialization...");
  
  // Détection automatique du capteur
  uint16_t sensor_id = this->detect_sensor_id_();
  if (sensor_id == 0) {
    ESP_LOGE(TAG, "No compatible sensor detected!");
    return false;
  }
  
  // Configuration spécifique selon le capteur détecté
  bool config_success = false;
  switch (sensor_id) {
    case OV5645_CHIP_ID:
      ESP_LOGI(TAG, "Configuring OV5645 sensor...");
      config_success = this->configure_ov5645_();
      break;
      
    case SC2336_CHIP_ID:
      ESP_LOGI(TAG, "Configuring SC2336 sensor...");
      config_success = this->configure_sc2336_();
      break;
      
    case SC2356_CHIP_ID:
      ESP_LOGI(TAG, "Configuring SC2356 sensor...");
      config_success = this->configure_sc2356_();
      break;
      
    default:
      ESP_LOGE(TAG, "Unknown sensor ID: 0x%04X", sensor_id);
      return false;
  }
  
  if (!config_success) {
    ESP_LOGE(TAG, "Failed to configure sensor ID 0x%04X", sensor_id);
    return false;
  }
  
  this->sensor_initialized_ = true;
  ESP_LOGI(TAG, "✅ Tab5 sensor initialized successfully");
  return true;
}

// NOUVELLE MÉTHODE: Configuration spécifique OV5645
bool Tab5Camera::configure_ov5645_() {
  ESP_LOGI(TAG, "Configuring OV5645 sensor for 640x480 @ 30fps...");
  
  // Configuration OV5645 optimisée pour Tab5
  const struct {
    uint16_t reg;
    uint8_t val;
    const char* desc;
    uint16_t delay_ms;
  } ov5645_config[] = {
    // Software reset et initialization
    {0x3103, 0x11, "System control", 0},
    {0x3008, 0x82, "Software reset", 100},  // Délai important après reset
    
    // Clock settings
    {0x3017, 0x40, "IO direction control", 0},
    {0x3018, 0x00, "IO direction control", 0},
    {0x3034, 0x18, "PLL control", 0},
    {0x3035, 0x14, "PLL control", 0},
    {0x3036, 0x38, "PLL control", 0},
    {0x3037, 0x13, "PLL control", 10},
    
    // Format control pour RAW8 vers RGB565
    {0x4300, 0x6f, "Format control", 0},
    {0x501f, 0x01, "Format MUX control", 0},
    
    // Window size pour 640x480
    {0x3800, 0x00, "X start high", 0},
    {0x3801, 0x00, "X start low", 0},
    {0x3802, 0x00, "Y start high", 0},
    {0x3803, 0x04, "Y start low", 0},
    {0x3804, 0x0a, "X end high", 0},
    {0x3805, 0x3f, "X end low", 0},
    {0x3806, 0x07, "Y end high", 0},
    {0x3807, 0x9b, "Y end low", 0},
    
    // Output size 640x480
    {0x3808, 0x02, "X output size high", 0},
    {0x3809, 0x80, "X output size low", 0},  // 640
    {0x380a, 0x01, "Y output size high", 0},
    {0x380b, 0xe0, "Y output size low", 0},  // 480
    
    // Timing
    {0x380c, 0x07, "HTS high", 0},
    {0x380d, 0x68, "HTS low", 0},
    {0x380e, 0x03, "VTS high", 0},
    {0x380f, 0xd8, "VTS low", 0},
    
    // Subsample et binning
    {0x3810, 0x00, "X offset high", 0},
    {0x3811, 0x10, "X offset low", 0},
    {0x3812, 0x00, "Y offset high", 0},
    {0x3813, 0x06, "Y offset low", 0},
    {0x3814, 0x31, "X increment", 0},
    {0x3815, 0x31, "Y increment", 0},
    
    // Contrôles d'exposition
    {0x3503, 0x07, "AEC/AGC control", 0},
    
    // MIPI settings
    {0x300e, 0x45, "MIPI control", 0},
    {0x302e, 0x08, "MIPI control", 0},
    
    // Start streaming
    {0x3008, 0x02, "Normal operation", 20},
  };
  
  for (size_t i = 0; i < sizeof(ov5645_config) / sizeof(ov5645_config[0]); i++) {
    ESP_LOGD(TAG, "Writing %s: reg 0x%04X = 0x%02X", 
             ov5645_config[i].desc, ov5645_config[i].reg, ov5645_config[i].val);
    
    if (!this->write_byte_16(ov5645_config[i].reg, ov5645_config[i].val)) {
      ESP_LOGW(TAG, "Failed to write OV5645 register 0x%04X - continuing", ov5645_config[i].reg);
    }
    
    if (ov5645_config[i].delay_ms > 0) {
      vTaskDelay(ov5645_config[i].delay_ms / portTICK_PERIOD_MS);
    } else {
      vTaskDelay(5 / portTICK_PERIOD_MS);  // Délai minimal entre écritures
    }
  }
  
  ESP_LOGI(TAG, "✅ OV5645 sensor configured successfully");
  return true;
}

// NOUVELLE MÉTHODE: Configuration spécifique SC2336
bool Tab5Camera::configure_sc2336_() {
  ESP_LOGI(TAG, "Configuring SC2336 sensor for 640x480...");
  
  // Configuration SC2336 optimisée pour Tab5
  const struct {
    uint16_t reg;
    uint8_t val;
    const char* desc;
    uint16_t delay_ms;
  } sc2336_config[] = {
    // Software reset
    {0x0103, 0x01, "Software reset", 100},
    
    // System control
    {0x0100, 0x00, "Standby mode", 10},
    
    // Clock settings
    {0x300c, 0x64, "PLL control", 0},
    {0x300d, 0x00, "PLL control", 0},
    {0x300e, 0x02, "PLL control", 0},
    {0x300f, 0x00, "PLL control", 10},
    
    // Format settings pour RAW8
    {0x3018, 0x32, "MIPI control", 0},
    {0x3019, 0x0c, "MIPI control", 0},
    
    // Window settings pour 640x480
    {0x3200, 0x00, "X start high", 0},
    {0x3201, 0x04, "X start low", 0},  
    {0x3202, 0x00, "Y start high", 0},
    {0x3203, 0x04, "Y start low", 0},
    {0x3204, 0x02, "X end high", 0},
    {0x3205, 0x8b, "X end low", 0},
    {0x3206, 0x01, "Y end high", 0},
    {0x3207, 0xeb, "Y end low", 0},
    
    // Output size
    {0x3208, 0x02, "X output size high", 0},
    {0x3209, 0x80, "X output size low", 0},  // 640
    {0x320a, 0x01, "Y output size high", 0}, 
    {0x320b, 0xe0, "Y output size low", 0},  // 480
    
    // Timing
    {0x320c, 0x05, "HTS high", 0},
    {0x320d, 0x46, "HTS low", 0},
    {0x320e, 0x02, "VTS high", 0},
    {0x320f, 0x58, "VTS low", 0},
    
    // MIPI settings
    {0x3301, 0x05, "MIPI settings", 0},
    {0x3304, 0x28, "MIPI settings", 0},
    {0x3306, 0x30, "MIPI settings", 0},
    
    // Start streaming
    {0x0100, 0x01, "Start streaming", 20},
  };
  
  for (size_t i = 0; i < sizeof(sc2336_config) / sizeof(sc2336_config[0]); i++) {
    ESP_LOGD(TAG, "Writing %s: reg 0x%04X = 0x%02X", 
             sc2336_config[i].desc, sc2336_config[i].reg, sc2336_config[i].val);
    
    if (!this->write_byte_16(sc2336_config[i].reg, sc2336_config[i].val)) {
      ESP_LOGW(TAG, "Failed to write SC2336 register 0x%04X - continuing", sc2336_config[i].reg);
    }
    
    if (sc2336_config[i].delay_ms > 0) {
      vTaskDelay(sc2336_config[i].delay_ms / portTICK_PERIOD_MS);
    } else {
      vTaskDelay(5 / portTICK_PERIOD_MS);
    }
  }
  
  ESP_LOGI(TAG, "✅ SC2336 sensor configured successfully");
  return true;
}

// NOUVELLE MÉTHODE: Configuration spécifique SC2356
bool Tab5Camera::configure_sc2356_() {
  ESP_LOGI(TAG, "Configuring SC2356 sensor for 640x480...");
  
  // Configuration SC2356 (similaire à SC2336 mais avec quelques différences)
  const struct {
    uint16_t reg;
    uint8_t val;
    const char* desc;
    uint16_t delay_ms;
  } sc2356_config[] = {
    // Software reset
    {0x0103, 0x01, "Software reset", 100},
    
    // System control
    {0x0100, 0x00, "Standby mode", 10},
    
    // Clock settings (ajustés pour SC2356)
    {0x300c, 0x50, "PLL control", 0},
    {0x300d, 0x00, "PLL control", 0},
    {0x300e, 0x02, "PLL control", 0},
    {0x300f, 0x00, "PLL control", 10},
    
    // Format settings pour RAW8
    {0x3018, 0x32, "MIPI control", 0},
    {0x3019, 0x0c, "MIPI control", 0},
    
    // Window settings pour 640x480
    {0x3200, 0x00, "X start high", 0},
    {0x3201, 0x08, "X start low", 0},  
    {0x3202, 0x00, "Y start high", 0},
    {0x3203, 0x08, "Y start low", 0},
    {0x3204, 0x02, "X end high", 0},
    {0x3205, 0x87, "X end low", 0},
    {0x3206, 0x01, "Y end high", 0},
    {0x3207, 0xe7, "Y end low", 0},
    
    // Output size
    {0x3208, 0x02, "X output size high", 0},
    {0x3209, 0x80, "X output size low", 0},  // 640
    {0x320a, 0x01, "Y output size high", 0}, 
    {0x320b, 0xe0, "Y output size low", 0},  // 480
    
    // Timing (ajusté pour SC2356)
    {0x320c, 0x05, "HTS high", 0},
    {0x320d, 0x46, "HTS low", 0},
    {0x320e, 0x02, "VTS high", 0},
    {0x320f, 0x58, "VTS low", 0},
    
    // MIPI settings
    {0x3301, 0x05, "MIPI settings", 0},
    {0x3304, 0x28, "MIPI settings", 0},
    {0x3306, 0x30, "MIPI settings", 0},
    
    // Start streaming
    {0x0100, 0x01, "Start streaming", 20},
  };
  
  for (size_t i = 0; i < sizeof(sc2356_config) / sizeof(sc2356_config[0]); i++) {
    ESP_LOGD(TAG, "Writing %s: reg 0x%04X = 0x%02X", 
             sc2356_config[i].desc, sc2356_config[i].reg, sc2356_config[i].val);
    
    if (!this->write_byte_16(sc2356_config[i].reg, sc2356_config[i].val)) {
      ESP_LOGW(TAG, "Failed to write SC2356 register 0x%04X - continuing", sc2356_config[i].reg);
    }
    
    if (sc2356_config[i].delay_ms > 0) {
      vTaskDelay(sc2356_config[i].delay_ms / portTICK_PERIOD_MS);
    } else {
      vTaskDelay(5 / portTICK_PERIOD_MS);
    }
  }
  
  ESP_LOGI(TAG, "✅ SC2356 sensor configured successfully");
  return true;
}

// NOUVELLES MÉTHODES: Communication I2C 16-bit
bool Tab5Camera::read_byte_16(uint16_t reg, uint8_t *data) {
  uint8_t reg_buf[2] = {(uint8_t)(reg >> 8), (uint8_t)(reg & 0xFF)};
  
  if (!this->write(reg_buf, 2, false)) {
    return false;
  }
  
  return this->read(data, 1);
}

bool Tab5Camera::write_byte_16(uint16_t reg, uint8_t data) {
  uint8_t buf[3] = {(uint8_t)(reg >> 8), (uint8_t)(reg & 0xFF), data};
  return this->write(buf, 3, true);
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

  // Étape 3: Initialisation du capteur I2C (MODIFIÉE)
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
  csi_config.queue_items = 4;  // ← AUGMENTÉ: plus de buffers dans la queue

  esp_err_t ret = esp_cam_new_csi_ctlr(&csi_config, &this->cam_handle_);
  if (ret != ESP_OK) {
    ESP_LOGE(TAG, "CSI controller init failed: %s", esp_err_to_name(ret));
    return false;
  }
  ESP_LOGI(TAG, "CSI controller created successfully");
  
  // Étape 6: Configuration des callbacks - CORRECTION CRITIQUE
  ESP_LOGI(TAG, "Step 2.6: Registering camera callbacks");
  esp_cam_ctlr_evt_cbs_t cbs = {
    .on_get_new_trans = nullptr,  // ← IMPORTANT: laisser à nullptr pour utiliser la queue interne !
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










