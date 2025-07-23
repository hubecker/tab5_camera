#pragma once

#include "esphome/core/component.h"
#include "esphome/core/hal.h"
#include "esphome/core/automation.h"
#include <functional>

#ifdef USE_ESP32

// Nouvelle API ESP32-P4
#if __has_include("esp_cam_ctlr_csi.h")
#include "esp_cam_ctlr_csi.h"
#include "esp_cam_ctlr.h"
#include "driver/isp.h"
#include "esp_cache.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
#define HAS_ESP32_P4_CAMERA
#endif

namespace esphome {
namespace tab5_camera {

class Tab5Camera;

// ===== TRIGGER POUR NOUVELLES FRAMES =====
class Tab5CameraOnFrameTrigger : public Trigger<> {
 public:
  explicit Tab5CameraOnFrameTrigger(Tab5Camera *parent) { parent->add_on_frame_callback([this]() { this->trigger(); }); }
};

// ===== CLASSE PRINCIPALE =====
class Tab5Camera : public Component {
 public:
  void setup() override;
  void loop() override;
  void dump_config() override;
  float get_setup_priority() const override;
  
  // Configuration de base
  void set_name(const std::string &name) { this->name_ = name; }
  void set_external_clock_pin(uint8_t pin) { this->external_clock_pin_ = pin; }
  void set_external_clock_frequency(uint32_t frequency) { this->external_clock_frequency_ = frequency; }
  void set_reset_pin(GPIOPin *pin) { this->reset_pin_ = pin; }
  
  // Méthodes snapshot
  bool take_snapshot();
  
  // ===== MÉTHODES STREAMING LIVE =====
  void start_streaming();
  void stop_streaming();
  bool is_streaming() const { return this->streaming_active_; }
  void set_streaming_fps(uint8_t fps) { this->streaming_fps_ = fps; }
  
  // Accès aux données des frames
  bool has_new_frame() const { return this->new_frame_available_; }
  void* get_frame_buffer() { 
    this->new_frame_available_ = false;
    return this->frame_buffer_; 
  }
  size_t get_frame_size() const { return this->frame_buffer_size_; }
  uint16_t get_frame_width() const { return this->frame_width_; }
  uint16_t get_frame_height() const { return this->frame_height_; }
  
  // Callbacks pour les événements
  void add_on_frame_callback(std::function<void()> callback) { 
    this->on_frame_callbacks_.push_back(callback); 
  }
  void add_on_streaming_start_callback(std::function<void()> callback) { 
    this->on_streaming_start_callbacks_.push_back(callback); 
  }
  void add_on_streaming_stop_callback(std::function<void()> callback) { 
    this->on_streaming_stop_callbacks_.push_back(callback); 
  }
  
  // Statistiques
  uint32_t get_frame_count() const { return this->frame_count_; }
  float get_current_fps() const { return this->current_fps_; }
  uint32_t get_dropped_frames() const { return this->dropped_frames_; }
  
  // Triggers pour ESPHome
  Tab5CameraOnFrameTrigger *get_on_frame_trigger() { return new Tab5CameraOnFrameTrigger(this); }

 protected:
#ifdef HAS_ESP32_P4_CAMERA
  // Méthodes internes d'initialisation
  bool init_camera_();
  void deinit_camera_();
  
  // Méthodes de streaming
  void streaming_task_();
  static void streaming_task_wrapper_(void *parameter);
  void process_new_frame_();
  void calculate_fps_();
  
  // Callbacks ESP32-P4
  static bool camera_get_new_vb_callback_(esp_cam_ctlr_handle_t handle, esp_cam_ctlr_trans_t *trans, void *user_data);
  static bool camera_get_finished_trans_callback_(esp_cam_ctlr_handle_t handle, esp_cam_ctlr_trans_t *trans, void *user_data);
  
  // Handles ESP32-P4
  esp_cam_ctlr_handle_t cam_handle_{nullptr};
  isp_proc_handle_t isp_proc_{nullptr};
  
  // Gestion des buffers
  void *frame_buffer_{nullptr};
  void *double_buffer_{nullptr};  // Double buffering pour streaming fluide
  size_t frame_buffer_size_{0};
  uint16_t frame_width_{640};
  uint16_t frame_height_{480};
  
  // État de la caméra
  bool camera_initialized_{false};
  
  // ===== VARIABLES DE STREAMING =====
  bool streaming_active_{false};
  uint8_t streaming_fps_{30};  // FPS par défaut
  TaskHandle_t streaming_task_handle_{nullptr};
  SemaphoreHandle_t frame_mutex_{nullptr};
  
  // État des frames
  volatile bool new_frame_available_{false};
  uint32_t frame_count_{0};
  uint32_t dropped_frames_{0};
  
  // Calcul FPS
  uint32_t last_fps_calc_time_{0};
  uint32_t last_frame_count_{0};
  float current_fps_{0.0f};
  
  // Buffer management
  bool using_double_buffer_{true};
  volatile bool buffer_swap_needed_{false};
#endif
  
  // Configuration de base
  std::string name_;
  uint8_t external_clock_pin_{36};
  uint32_t external_clock_frequency_{20000000}; // 20MHz
  GPIOPin *reset_pin_{nullptr};
  
  // Callbacks
  std::vector<std::function<void()>> on_frame_callbacks_;
  std::vector<std::function<void()>> on_streaming_start_callbacks_;
  std::vector<std::function<void()>> on_streaming_stop_callbacks_;
};

// ===== ACTIONS ESPHOME =====
template<typename... Ts> class StartStreamingAction : public Action<Ts...> {
 public:
  StartStreamingAction(Tab5Camera *camera) : camera_(camera) {}
  void play(Ts... x) override { this->camera_->start_streaming(); }

 protected:
  Tab5Camera *camera_;
};

template<typename... Ts> class StopStreamingAction : public Action<Ts...> {
 public:
  StopStreamingAction(Tab5Camera *camera) : camera_(camera) {}
  void play(Ts... x) override { this->camera_->stop_streaming(); }

 protected:
  Tab5Camera *camera_;
};

template<typename... Ts> class TakeSnapshotAction : public Action<Ts...> {
 public:
  TakeSnapshotAction(Tab5Camera *camera) : camera_(camera) {}
  void play(Ts... x) override { this->camera_->take_snapshot(); }

 protected:
  Tab5Camera *camera_;
};

}  // namespace tab5_camera
}  // namespace esphome

#endif  // USE_ESP32




