#include "tab5_camera.h"

#ifdef HAS_ESP32_P4_CAMERA

namespace esphome {
namespace tab5_camera {

static const char *const TAG = "tab5_camera";

Tab5Camera::~Tab5Camera() {
  cleanup_resources();
}

float Tab5Camera::get_setup_priority() const {
  return setup_priority::HARDWARE;
}

void Tab5Camera::setup() {
  ESP_LOGI(TAG, "Initialisation de la caméra %s", this->name_.c_str());
  clear_error_();

  if (!init_camera_()) {
    set_error_("Échec init_camera_");
    return;
  }

  if (!init_sensor_()) {
    set_error_("Échec init_sensor_");
    return;
  }

  if (!init_ldo_()) {
    set_error_("Échec init_ldo_");
    return;
  }

  if (!init_camera_controller()) {
    set_error_("Échec init_camera_controller");
    return;
  }

  if (!init_isp_processor()) {
    set_error_("Échec init_isp_processor");
    return;
  }

  if (!allocate_dma_buffer_()) {
    set_error_("Échec allocation buffer DMA");
    return;
  }

  camera_initialized_ = true;
  ESP_LOGI(TAG, "Caméra %s initialisée avec succès", this->name_.c_str());
}

void Tab5Camera::dump_config() {
  ESP_LOGCONFIG(TAG, "Caméra Tab5 :");
  ESP_LOGCONFIG(TAG, "  Nom : %s", this->name_.c_str());
  ESP_LOGCONFIG(TAG, "  Résolution : %ux%u", this->frame_width_, this->frame_height_);
  ESP_LOGCONFIG(TAG, "  Format pixel : %s", this->pixel_format_.c_str());
  ESP_LOGCONFIG(TAG, "  Framerate : %u fps", this->framerate_);
  ESP_LOGCONFIG(TAG, "  Qualité JPEG : %u", this->jpeg_quality_);
}

bool Tab5Camera::start_streaming() {
  if (!camera_initialized_ || streaming_active_) return false;

  streaming_should_stop_ = false;
  frame_ready_semaphore_ = xSemaphoreCreateBinary();
  frame_queue_ = xQueueCreate(FRAME_QUEUE_SIZE, sizeof(FrameData));

  xTaskCreatePinnedToCore(
    &Tab5Camera::streaming_task, "tab5_stream", STREAMING_TASK_STACK_SIZE, this,
    STREAMING_TASK_PRIORITY, &streaming_task_handle_, tskNO_AFFINITY);

  streaming_active_ = true;
  return true;
}

bool Tab5Camera::stop_streaming() {
  if (!streaming_active_) return false;

  streaming_should_stop_ = true;
  if (streaming_task_handle_ != nullptr) {
    vTaskDelete(streaming_task_handle_);
    streaming_task_handle_ = nullptr;
  }

  if (frame_ready_semaphore_ != nullptr) {
    vSemaphoreDelete(frame_ready_semaphore_);
    frame_ready_semaphore_ = nullptr;
  }

  if (frame_queue_ != nullptr) {
    vQueueDelete(frame_queue_);
    frame_queue_ = nullptr;
  }

  streaming_active_ = false;
  return true;
}

bool Tab5Camera::take_snapshot() {
  if (!camera_initialized_ || streaming_active_) return false;
  // Simulation de capture unique
  FrameData frame;
  frame.buffer = frame_buffer_;
  frame.size = calculate_frame_size_();
  frame.valid = true;
  on_frame_callbacks_.call(reinterpret_cast<uint8_t *>(frame.buffer), frame.size);
  return true;
}

void Tab5Camera::streaming_task(void *parameter) {
  static_cast<Tab5Camera *>(parameter)->streaming_loop_();
  vTaskDelete(nullptr);
}

void Tab5Camera::streaming_loop_() {
  while (!streaming_should_stop_) {
    FrameData frame;
    if (xQueueReceive(frame_queue_, &frame, pdMS_TO_TICKS(100))) {
      if (frame.valid && frame.buffer != nullptr) {
        // Option MJPEG : encoder ici si configuré
        on_frame_callbacks_.call(reinterpret_cast<uint8_t *>(frame.buffer), frame.size);
      }
    }
  }
}

bool Tab5Camera::allocate_dma_buffer_() {
  size_t size = calculate_frame_size_();
  frame_buffer_ = heap_caps_malloc(size, MALLOC_CAP_DMA);
  if (!frame_buffer_) {
    ESP_LOGE(TAG, "❌ Échec d'allocation du buffer DMA de %u octets", (unsigned int) size);
    return false;
  }
  frame_buffer_size_ = size;
  ESP_LOGI(TAG, "🌀 Buffer DMA alloué (%u octets)", (unsigned int) size);
  return true;
}

void Tab5Camera::set_error_(const std::string &error) {
  this->error_state_ = true;
  this->last_error_ = error;
  ESP_LOGE(TAG, "%s", error.c_str());
}

void Tab5Camera::clear_error_() {
  this->error_state_ = false;
  this->last_error_.clear();
}

size_t Tab5Camera::calculate_frame_size_() const {
  return static_cast<size_t>(this->frame_width_) * this->frame_height_ * 2; // pour YUV422
}

PixelFormat Tab5Camera::parse_pixel_format_(const std::string &format) {
  if (format == "YUV422") return PixelFormat::YUV422;
  if (format == "RGB565") return PixelFormat::RGB565;
  if (format == "RAW10") return PixelFormat::RAW10;
  if (format == "RAW8") return PixelFormat::RAW8;
  return PixelFormat::YUV422;
}

}  // namespace tab5_camera
}  // namespace esphome

#endif








