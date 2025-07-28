#include "tab5_camera.h"
#include "esphome/core/log.h"
#include "esphome/core/helpers.h"
#include "esphome/core/hal.h"
#include "esp_timer.h"
#include "driver/ledc.h"

#ifdef USE_ESP32

#ifdef HAS_ESP32_P4_CAMERA
#include "esp_cam_ctlr_csi.h"
#endif

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
  
  // Configuration de l'horloge externe AVANT l'initialisation I2C
  if (this->external_clock_pin_ > 0) {
    ESP_LOGI(TAG, "Step 1.5: Setting up external clock");
    if (!this->setup_external_clock_()) {
      ESP_LOGE(TAG, "Failed to setup external clock - continuing anyway");
    }
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

// NOUVELLE MÉTHODE: Configuration de l'horloge externe avec LEDC
// MÉTHODE CORRIGÉE: Configuration de l'horloge externe avec LEDC
bool Tab5Camera::setup_external_clock_() {
  if (this->external_clock_pin_ == 0) {
    ESP_LOGW(TAG, "No external clock pin configured");
    return true;  // Pas critique
  }
  
  ESP_LOGI(TAG, "Setting up external clock on GPIO%u at %u Hz", 
           this->external_clock_pin_, this->external_clock_frequency_);
  
  // Configuration du pin en mode horloge via LEDC
  gpio_num_t clock_pin = static_cast<gpio_num_t>(this->external_clock_pin_);
  
  // Configuration du timer LEDC pour générer l'horloge
  ledc_timer_config_t timer_config = {};
  timer_config.duty_resolution = LEDC_TIMER_1_BIT;  // 1 bit = signal carré 50%
  timer_config.freq_hz = this->external_clock_frequency_;
  timer_config.speed_mode = LEDC_LOW_SPEED_MODE;  // ← CORRECTION: Utiliser LOW_SPEED_MODE
  timer_config.timer_num = LEDC_TIMER_0;
  timer_config.clk_cfg = LEDC_AUTO_CLK;
  
  esp_err_t ret = ledc_timer_config(&timer_config);
  if (ret != ESP_OK) {
    ESP_LOGE(TAG, "Failed to configure LEDC timer: %s", esp_err_to_name(ret));
    return false;
  }
  
  // Configuration du canal LEDC
  ledc_channel_config_t channel_config = {};
  channel_config.channel = LEDC_CHANNEL_0;
  channel_config.duty = 1;  // 50% duty cycle (1/2 pour 1-bit resolution)
  channel_config.gpio_num = clock_pin;
  channel_config.speed_mode = LEDC_LOW_SPEED_MODE;  // ← CORRECTION: Cohérent avec timer
  channel_config.hpoint = 0;
  channel_config.timer_sel = LEDC_TIMER_0;
  
  ret = ledc_channel_config(&channel_config);
  if (ret != ESP_OK) {
    ESP_LOGE(TAG, "Failed to configure LEDC channel: %s", esp_err_to_name(ret));
    return false;
  }
  
  ESP_LOGI(TAG, "✅ External clock configured successfully on GPIO%u", this->external_clock_pin_);
  
  // Petit délai pour que l'horloge se stabilise
  vTaskDelay(100 / portTICK_PERIOD_MS);
  
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
    // Ne pas échouer complètement, certains boards peuvent ne pas avoir de LDO MIPI dédié
    this->ldo_mipi_phy_ = nullptr;
  } else {
    ESP_LOGI(TAG, "MIPI LDO regulator acquired successfully");
  }
  
  this->ldo_initialized_ = true;
  return true;  // Retourner true même si le LDO échoue
}

// MÉTHODE MISE À JOUR: Détection automatique du capteur avec scan I2C complet
uint16_t Tab5Camera::detect_sensor_id_() {
  ESP_LOGI(TAG, "🔍 Scanning I2C bus for connected devices...");
  
  // ÉTAPE 1: Scan complet du bus I2C pour voir tous les périphériques
  bool devices_found = false;
  for (uint8_t addr = 0x08; addr <= 0x77; addr++) {
    this->set_i2c_address(addr);
    
    // Test simple de présence avec un scan I2C basic
    if (this->write(nullptr, 0, true) == i2c::ERROR_OK) {
      ESP_LOGI(TAG, "✅ Device found at address 0x%02X", addr);
      devices_found = true;
      
      // Test de lecture d'un registre générique
      uint8_t test_data = 0;
      if (this->read(&test_data, 1) == i2c::ERROR_OK) {
        ESP_LOGI(TAG, "   - Can read from device, first byte: 0x%02X", test_data);
      }
    }
  }
  
  if (!devices_found) {
    ESP_LOGE(TAG, "❌ No I2C devices found! Check your I2C wiring (SDA/SCL) and pull-ups");
    return 0;
  }
  
  ESP_LOGI(TAG, "🔍 Now testing Tab5 camera sensors specifically...");
  
  // ÉTAPE 2: Test spécifique des capteurs connus avec plus d'adresses
  const struct {
    uint8_t address;
    const char *name;
    uint16_t id_reg_high;
    uint16_t id_reg_low;
    uint16_t expected_id;
  } sensor_configs[] = {
    {0x3C, "OV5645", 0x300A, 0x300B, OV5645_CHIP_ID},
    {0x30, "SC2336", 0x3107, 0x3108, SC2336_CHIP_ID}, 
    {0x30, "SC2356", 0x3107, 0x3108, SC2356_CHIP_ID},
    // Capteurs à des adresses alternatives courantes
    {0x78, "OV5645_ALT", 0x300A, 0x300B, OV5645_CHIP_ID},
    {0x60, "SC2336_ALT", 0x3107, 0x3108, SC2336_CHIP_ID},
    {0x20, "Sensor_Test1", 0x0000, 0x0001, 0x0000}, // Test générique
    {0x1A, "Sensor_Test2", 0x0000, 0x0001, 0x0000}, // Autres tests
  };

  uint8_t original_addr = this->address_;

  for (size_t i = 0; i < sizeof(sensor_configs) / sizeof(*sensor_configs); i++) {
    const auto &cfg = sensor_configs[i];
    this->set_i2c_address(cfg.address);

    ESP_LOGI(TAG, "Testing sensor %s at 0x%02X...", cfg.name, cfg.address);
    
    // Test de présence avec scan simple
    if (this->write(nullptr, 0, true) != i2c::ERROR_OK) {
      ESP_LOGW(TAG, "❌ No response from 0x%02X", cfg.address);
      continue;
    }
    
    ESP_LOGI(TAG, "✅ Device responds at 0x%02X, attempting to read sensor ID...", cfg.address);
    
    // Délai plus long pour stabiliser
    vTaskDelay(50 / portTICK_PERIOD_MS);
    
    uint8_t id_high = 0, id_low = 0;
    bool high_ok = this->read_byte_16(cfg.id_reg_high, &id_high);
    bool low_ok = this->read_byte_16(cfg.id_reg_low, &id_low);

    ESP_LOGI(TAG, "ID read results for %s:", cfg.name);
    ESP_LOGI(TAG, "  - High reg (0x%04X): %s -> 0x%02X", 
             cfg.id_reg_high, high_ok ? "SUCCESS" : "FAILED", id_high);
    ESP_LOGI(TAG, "  - Low reg (0x%04X):  %s -> 0x%02X", 
             cfg.id_reg_low, low_ok ? "SUCCESS" : "FAILED", id_low);

    if (high_ok && low_ok) {
      uint16_t sensor_id = (id_high << 8) | id_low;
      ESP_LOGI(TAG, "📋 %s combined ID: 0x%04X (expected: 0x%04X)", cfg.name, sensor_id, cfg.expected_id);

      if (sensor_id == cfg.expected_id) {
        ESP_LOGI(TAG, "🎉 SUCCESS! Detected %s sensor at address 0x%02X", cfg.name, cfg.address);
        this->detected_sensor_id_ = sensor_id;
        this->detected_sensor_address_ = cfg.address;
        return sensor_id;
      } else if (sensor_id != 0x0000 && sensor_id != 0xFFFF && sensor_id != 0xA2A2) {
        ESP_LOGW(TAG, "⚠️  Unknown but valid sensor ID 0x%04X at address 0x%02X", sensor_id, cfg.address);
        // Peut-être un capteur compatible mais avec un ID différent
      }
    } else {
      ESP_LOGW(TAG, "❌ Failed to read ID registers from %s at 0x%02X", cfg.name, cfg.address);
    }

    vTaskDelay(30 / portTICK_PERIOD_MS);
  }

  this->set_i2c_address(original_addr);
  ESP_LOGE(TAG, "❌ No compatible Tab5 camera sensor detected!");
  ESP_LOGE(TAG, "💡 Possible issues:");
  ESP_LOGE(TAG, "   - Camera module not connected properly");
  ESP_LOGE(TAG, "   - Missing external 24MHz clock");
  ESP_LOGE(TAG, "   - Camera power supply issue");
  ESP_LOGE(TAG, "   - Need camera reset sequence");
  return 0;
}

// MÉTHODE MISE À JOUR: Test de communication avec retry et meilleurs diagnostics
bool Tab5Camera::test_sensor_communication_(uint8_t address) {
  ESP_LOGD(TAG, "Testing communication with sensor at 0x%02X", address);
  
  // Test simple de présence I2C
  if (this->write(nullptr, 0, true) != i2c::ERROR_OK) {
    ESP_LOGD(TAG, "No I2C response from 0x%02X", address);
    return false;
  }
  
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
  } else {
    // Test générique pour adresses inconnues
    if (this->read_byte_16(0x0000, &test_data)) successful_reads++;
    if (this->read_byte_16(0x0001, &test_data)) successful_reads++;
  }
  
  ESP_LOGD(TAG, "Successful register reads from 0x%02X: %d/3", address, successful_reads);
  return successful_reads >= 1;  // Au moins 1 lecture réussie
}

// MÉTHODE MISE À JOUR: Communication I2C plus robuste avec retry amélioré
bool Tab5Camera::read_byte_16(uint16_t reg, uint8_t *data) {
  if (!data) return false;
  
  uint8_t reg_buf[2] = {(uint8_t)(reg >> 8), (uint8_t)(reg & 0xFF)};
  
  // Tentative de lecture avec retry et délais adaptatifs
  for (int retry = 0; retry < 5; retry++) {
    if (retry > 0) {
      vTaskDelay((retry * 10) / portTICK_PERIOD_MS);  // Délai croissant
    }
    
    // Phase 1: Écriture de l'adresse du registre
    i2c::ErrorCode write_result = this->write(reg_buf, 2, false);
    if (write_result != i2c::ERROR_OK) {
      ESP_LOGV(TAG, "Write register address 0x%04X failed (attempt %d): %d", reg, retry + 1, write_result);
      continue;
    }
    
    // Petit délai entre write et read
    delayMicroseconds(100);
    
    // Phase 2: Lecture de la donnée
    i2c::ErrorCode read_result = this->read(data, 1);
    if (read_result == i2c::ERROR_OK) {
      ESP_LOGVV(TAG, "Successfully read 0x%02X from register 0x%04X", *data, reg);
      return true;
    }
    
    ESP_LOGV(TAG, "Read data from 0x%04X failed (attempt %d): %d", reg, retry + 1, read_result);
  }
  
  ESP_LOGD(TAG, "Failed to read register 0x%04X after 5 attempts", reg);
  *data = 0xA2;  // Valeur par défaut pour diagnostics
  return false;
}

bool Tab5Camera::write_byte_16(uint16_t reg, uint8_t data) {
  uint8_t buf[3] = {(uint8_t)(reg >> 8), (uint8_t)(reg & 0xFF), data};
  
  // Tentative d'écriture avec retry et délais adaptatifs
  for (int retry = 0; retry < 5; retry++) {
    if (retry > 0) {
      vTaskDelay((retry * 10) / portTICK_PERIOD_MS);
    }
    
    i2c::ErrorCode result = this->write(buf, 3, true);
    if (result == i2c::ERROR_OK) {
      ESP_LOGVV(TAG, "Successfully wrote 0x%02X to register 0x%04X", data, reg);
      return true;
    }
    
    ESP_LOGV(TAG, "Write register 0x%04X=0x%02X failed (attempt %d): %d", reg, data, retry + 1, result);
  }
  
  ESP_LOGD(TAG, "Failed to write register 0x%04X after 5 attempts", reg);
  return false;
}

// MÉTHODE MISE À JOUR: Initialisation du capteur avec détection automatique et reset
bool Tab5Camera::init_sensor_() {
  if (this->sensor_initialized_) {
    ESP_LOGI(TAG, "Sensor already initialized, skipping");
    return true;
  }
  
  ESP_LOGI(TAG, "Starting Tab5 sensor detection and initialization...");
  
  // Si un reset pin est configuré, faire le reset du capteur
  if (this->reset_pin_) {
    ESP_LOGI(TAG, "Executing sensor reset sequence...");
    this->reset_pin_->setup();
    this->reset_pin_->digital_write(false);  // Reset actif
    vTaskDelay(50 / portTICK_PERIOD_MS);     // 50ms reset
    this->reset_pin_->digital_write(true);   // Sortie de reset
    vTaskDelay(100 / portTICK_PERIOD_MS);    // 100ms pour stabilisation
    ESP_LOGI(TAG, "Sensor reset completed");
  } else {
    ESP_LOGW(TAG, "No reset pin configured - sensor might need reset!");
  }
  
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
      ESP_LOGE(TAG, "Unknown or unsupported sensor ID: 0x%04X", sensor_id);
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

// MÉTHODE MISE À JOUR: Configuration spécifique OV5645 avec reset
bool Tab5Camera::configure_ov5645_() {
  ESP_LOGI(TAG, "Configuring OV5645 sensor for 640x480 @ 30fps...");
  
  // Configuration OV5645 optimisée pour Tab5 MIPI-CSI
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
    
    // MIPI CSI specific settings
    {0x300e, 0x45, "MIPI control", 0},
    {0x302e, 0x08, "MIPI control", 0},
    {0x4300, 0x30, "Format control for RAW8", 0},  // RAW8 output
    {0x501f, 0x00, "ISP control", 0},
    
    // Window size pour output 640x480
    {0x3800, 0x00, "X start high", 0},    // X start = 0
    {0x3801, 0x00, "X start low", 0},
    {0x3802, 0x00, "Y start high", 0},    // Y start = 4
    {0x3803, 0x04, "Y start low", 0},
    {0x3804, 0x0a, "X end high", 0},      // X end = 2623
    {0x3805, 0x3f, "X end low", 0},
    {0x3806, 0x07, "Y end high", 0},      // Y end = 1955 
    {0x3807, 0x9b, "Y end low", 0},
    
    // Output size 640x480
    {0x3808, 0x02, "X output size high", 0},  // 640
    {0x3809, 0x80, "X output size low", 0},
    {0x380a, 0x01, "Y output size high", 0},  // 480
    {0x380b, 0xe0, "Y output size low", 0},
    
    // Timing pour 30 FPS
    {0x380c, 0x07, "HTS high", 0},        // Line length
    {0x380d, 0x68, "HTS low", 0},
    {0x380e, 0x03, "VTS high", 0},        // Frame length  
    {0x380f, 0xd8, "VTS low", 0},
    
    // Offset
    {0x3810, 0x00, "X offset high", 0},
    {0x3811, 0x10, "X offset low", 0},
    {0x3812, 0x00, "Y offset high", 0},
    {0x3813, 0x06, "Y offset low", 0},
    
    // Subsample et binning pour 640x480
    {0x3814, 0x31, "X increment", 0},
    {0x3815, 0x31, "Y increment", 0},
    
    // Disable AEC/AGC for fixed exposure  
    {0x3503, 0x07, "AEC/AGC disable", 0},
    
    // MIPI lane settings pour 2 lanes
    {0x3018, 0x00, "MIPI lanes", 0},  // 2 lanes
    
    // Start streaming
    {0x3008, 0x02, "Normal operation", 50},
  };
  
  for (size_t i = 0; i < sizeof(ov5645_config) / sizeof(ov5645_config[0]); i++) {
    ESP_LOGD(TAG, "OV5645: %s (0x%04X = 0x%02X)", 
             ov5645_config[i].desc, ov5645_config[i].reg, ov5645_config[i].val);
    
    if (!this->write_byte_16(ov5645_config[i].reg, ov5645_config[i].val)) {
      ESP_LOGW(TAG, "Failed to write OV5645 register 0x%04X - continuing", ov5645_config[i].reg);
    }
    
    if (ov5645_config[i].delay_ms > 0) {
      vTaskDelay(ov5645_config[i].delay_ms / portTICK_PERIOD_MS);
    } else {
      vTaskDelay(2 / portTICK_PERIOD_MS);  // Délai minimal
    }
  }
  
  ESP_LOGI(TAG, "✅ OV5645 sensor configured successfully for MIPI-CSI");
  return true;
}

// MÉTHODE MISE À JOUR: Configuration spécifique SC2336
bool Tab5Camera::configure_sc2336_() {
  ESP_LOGI(TAG, "Configuring SC2336 sensor for 640x480...");
  
  // Configuration SC2336 optimisée pour Tab5 MIPI-CSI
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
    
    // Clock settings - ajustés pour MIPI
    {0x300c, 0x64, "PLL control", 0},
    {0x300d, 0x00, "PLL control", 0}, 
    {0x300e, 0x02, "PLL control", 0},
    {0x300f, 0x00, "PLL control", 10},
    
    // MIPI CSI settings
    {0x3018, 0x32, "MIPI 2-lane", 0},
    {0x3019, 0x0c, "MIPI control", 0},
    {0x301f, 0x01, "MIPI enable", 0},
    
    // Window settings optimisés pour 640x480 
    {0x3200, 0x00, "X start high", 0},
    {0x3201, 0x04, "X start low", 0},  
    {0x3202, 0x00, "Y start high", 0},
    {0x3203, 0x04, "Y start low", 0},
    {0x3204, 0x02, "X end high", 0},
    {0x3205, 0x8b, "X end low", 0},
    {0x3206, 0x01, "Y end high", 0},
    {0x3207, 0xeb, "Y end low", 0},
    
    // Output size exact 640x480
    {0x3208, 0x02, "X output size high", 0},
    {0x3209, 0x80, "X output size low", 0},  // 640
    {0x320a, 0x01, "Y output size high", 0}, 
    {0x320b, 0xe0, "Y output size low", 0},  // 480
    
    // Timing pour frame rate stable
    {0x320c, 0x05, "HTS high", 0},
    {0x320d, 0x46, "HTS low", 0},
    {0x320e, 0x02, "VTS high", 0},
    {0x320f, 0x58, "VTS low", 0},
    
    // MIPI configuration
    {0x3301, 0x05, "MIPI settings", 0},
    {0x3304, 0x28, "MIPI settings", 0},
    {0x3306, 0x30, "MIPI settings", 0},
    
    // ISP settings pour RAW8 
    {0x3e00, 0x00, "Exposure high", 0},
    {0x3e01, 0x46, "Exposure mid", 0},
    {0x3e02, 0x10, "Exposure low", 0},
    
    // Start streaming
    {0x0100, 0x01, "Start streaming", 50},
  };
  
  for (size_t i = 0; i < sizeof(sc2336_config) / sizeof(sc2336_config[0]); i++) {
    ESP_LOGD(TAG, "SC2336: %s (0x%04X = 0x%02X)", 
             sc2336_config[i].desc, sc2336_config[i].reg, sc2336_config[i].val);
    
    if (!this->write_byte_16(sc2336_config[i].reg, sc2336_config[i].val)) {
      ESP_LOGW(TAG, "Failed to write SC2336 register 0x%04X - continuing", sc2336_config[i].reg);
    }
    
    if (sc2336_config[i].delay_ms > 0) {
      vTaskDelay(sc2336_config[i].delay_ms / portTICK_PERIOD_MS);
    } else {
      vTaskDelay(2 / portTICK_PERIOD_MS);
    }
  }
  
  ESP_LOGI(TAG, "✅ SC2336 sensor configured successfully for MIPI-CSI");
  return true;
}

// MÉTHODE MISE À JOUR: Configuration spécifique SC2356
bool Tab5Camera::configure_sc2356_() {
  ESP_LOGI(TAG, "Configuring SC2356 sensor for 640x480...");
  
  // Configuration SC2356 optimisée (similaire à SC2336 mais ajustée)
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
    
    // Clock settings ajustés pour SC2356
    {0x300c, 0x50, "PLL control", 0},
    {0x300d, 0x00, "PLL control", 0}, 
    {0x300e, 0x02, "PLL control", 0},
    {0x300f, 0x00, "PLL control", 10},
    
    // MIPI CSI settings 
    {0x3018, 0x32, "MIPI 2-lane", 0},
    {0x3019, 0x0c, "MIPI control", 0},
    {0x301f, 0x01, "MIPI enable", 0},
    
    // Window settings pour 640x480
    {0x3200, 0x00, "X start high", 0},
    {0x3201, 0x08, "X start low", 0},  // Léger décalage pour SC2356
    {0x3202, 0x00, "Y start high", 0},
    {0x3203, 0x08, "Y start low", 0},
    {0x3204, 0x02, "X end high", 0},
    {0x3205, 0x87, "X end low", 0},
    {0x3206, 0x01, "Y end high", 0},
    {0x3207, 0xe7, "Y end low", 0},
    
    // Output size 640x480  
    {0x3208, 0x02, "X output size high", 0},
    {0x3209, 0x80, "X output size low", 0},  // 640
    {0x320a, 0x01, "Y output size high", 0}, 
    {0x320b, 0xe0, "Y output size low", 0},  // 480
    
    // Timing
    {0x320c, 0x05, "HTS high", 0},
    {0x320d, 0x46, "HTS low", 0},
    {0x320e, 0x02, "VTS high", 0},
    {0x320f, 0x58, "VTS low", 0},
    
    // MIPI configuration pour SC2356
    {0x3301, 0x05, "MIPI settings", 0},
    {0x3304, 0x28, "MIPI settings", 0},
    {0x3306, 0x30, "MIPI settings", 0},
    
    // Exposure settings
    {0x3e00, 0x00, "Exposure high", 0},
    {0x3e01, 0x46, "Exposure mid", 0},
    {0x3e02, 0x10, "Exposure low", 0},
    
    // Start streaming
    {0x0100, 0x01, "Start streaming", 50},
  };
  
  for (size_t i = 0; i < sizeof(sc2356_config) / sizeof(sc2356_config[0]); i++) {
    ESP_LOGD(TAG, "SC2356: %s (0x%04X = 0x%02X)", 
             sc2356_config[i].desc, sc2356_config[i].reg, sc2356_config[i].val);
    
    if (!this->write_byte_16(sc2356_config[i].reg, sc2356_config[i].val)) {
      ESP_LOGW(TAG, "Failed to write SC2356 register 0x%04X - continuing", sc2356_config[i].reg);
    }
    
    if (sc2356_config[i].delay_ms > 0) {
      vTaskDelay(sc2356_config[i].delay_ms / portTICK_PERIOD_MS);
    } else {
      vTaskDelay(2 / portTICK_PERIOD_MS);
    }
  }
  
  ESP_LOGI(TAG, "✅ SC2356 sensor configured successfully for MIPI-CSI");
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

bool Tab5Camera::camera_get_finished_trans_callback(esp_cam_ctlr_handle_t handle, esp_cam_ctlr_trans_t *trans, void *user_data) {
  Tab5Camera *camera = static_cast<Tab5Camera*>(user_data);
  if (!camera || !trans->buffer) {
    ESP_LOGE(TAG, "camera_get_finished_trans_callback called with invalid data");
    return false;
  }

  // Compteur de frames pour diagnostics
  static uint32_t frame_count = 0;
  frame_count++;
  
  ESP_LOGI(TAG, "📸 Frame #%lu received: %zu bytes", frame_count, trans->received_size);

  // Vérification de la taille des données
  if (trans->received_size == 0) {
    ESP_LOGW(TAG, "Frame #%lu is empty - sensor might not be generating data", frame_count);
    return false;
  }
  
  if (trans->received_size < 1000) {  
    ESP_LOGW(TAG, "Frame #%lu size is very small (%zu bytes)", frame_count, trans->received_size);
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
  }
  
  // Création de la tâche de streaming
  BaseType_t result = xTaskCreate(
    Tab5Camera::streaming_task,
    "tab5_streaming",
    TAB5_STREAMING_STACK_SIZE,
    this,
    5,  
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
    while (this->streaming_active_ && timeout < 50) { 
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
  ESP_LOGD(TAG, "Streaming loop started for camera '%s' (callback-based)", this->name_.c_str());

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

// Méthodes utilitaires NOUVELLES (étaient manquantes)
void Tab5Camera::set_error_(const std::string &error) {
  this->error_state_ = true;
  this->last_error_ = error;
  ESP_LOGE(TAG, "Tab5Camera error: %s", error.c_str());
}

void Tab5Camera::clear_error_() {
  this->error_state_ = false;
  this->last_error_.clear();
}

PixelFormat Tab5Camera::parse_pixel_format_(const std::string &format) {
  if (format == "RAW8") return PixelFormat::RAW8;
  if (format == "RAW10") return PixelFormat::RAW10;
  if (format == "YUV422") return PixelFormat::YUV422;
  if (format == "RGB565") return PixelFormat::RGB565;
  if (format == "JPEG") return PixelFormat::JPEG;
  return PixelFormat::RGB565; // Par défaut
}

size_t Tab5Camera::calculate_frame_size_() const {
  size_t pixel_size = 2; // RGB565 par défaut
  if (this->pixel_format_ == "RAW8") pixel_size = 1;
  else if (this->pixel_format_ == "RAW10") pixel_size = 2;
  else if (this->pixel_format_ == "YUV422") pixel_size = 2;
  else if (this->pixel_format_ == "RGB565") pixel_size = 2;
  
  return this->frame_width_ * this->frame_height_ * pixel_size;
}

#endif // HAS_ESP32_P4_CAMERA

}  // namespace tab5_camera
}  // namespace esphome

#endif  // USE_ESP32











