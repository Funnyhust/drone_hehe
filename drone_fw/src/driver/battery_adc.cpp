#include "driver/battery_adc.h"
#include "board_pinmap.h"
#include "config.h"

// Đặt thành 1 để GIẢ LẬP pin sụt dần (Demo), đặt thành 0 để DÙNG CẢM BIẾN THẬT
#define FAKE_BATTERY 1

#if !FAKE_BATTERY
// Số lượng mẫu lọc trung bình động
#define FILTER_SAMPLES 20
static float battery_samples[FILTER_SAMPLES];
static uint8_t sample_index = 0;
// Hệ số nhân cầu phân áp: R1 = 27k, R2 = 10k -> scale = (27 + 10)/10 = 3.7
// Điện áp thực = Vadc * 3.7
static const float adc_to_voltage_factor = (3.3f * 3.7f) / 4095.0f;
#endif

static float current_voltage = 0.0f;
static BatteryState current_state = BATTERY_NORMAL;

void batteryInit() {
#if FAKE_BATTERY
  // Khởi động pin giả lập ban đầu là 12.6V (Đầy 100%)
  current_voltage = 12.6f;
  current_state = BATTERY_NORMAL;
#else
  pinMode(PIN_BATTERY_ADC, INPUT);

#if defined(ARDUINO_ARCH_STM32)
  // Bật ADC lên 12-bit để khớp với công thức chia cho 4095
  analogReadResolution(12);
#endif

  // Đọc nháp vài lần để ổn định cảm biến ADC
  analogRead(PIN_BATTERY_ADC);
  delayMicroseconds(1000);

  // Đọc tức thời và điền đầy bộ lọc ban đầu để tránh sụt áp ảo từ 0V lúc khởi
  // động
  uint16_t raw = analogRead(PIN_BATTERY_ADC);
  float initial_v = (float)raw * adc_to_voltage_factor;

  for (uint8_t i = 0; i < FILTER_SAMPLES; i++) {
    battery_samples[i] = initial_v;
  }

  current_voltage = initial_v;
  sample_index = 0;

  // Cập nhật trạng thái ban đầu
#if ENABLE_BATTERY_CHECK
  if (current_voltage < BATTERY_THRESHOLD_CRITICAL) {
    current_state = BATTERY_CRITICAL;
  } else if (current_voltage < BATTERY_THRESHOLD_LOW) {
    current_state = BATTERY_LOW;
  } else {
    current_state = BATTERY_NORMAL;
  }
#else
  current_state = BATTERY_NORMAL;
#endif
#endif
}

void batteryUpdate() {
#if FAKE_BATTERY
  // Giả lập pin giảm dần từ 12.6V theo thời gian chạy (uptime) của drone
  // Mỗi giây giảm 0.005V -> Từ 12.6V về 10.5V (0%) mất đúng 420 giây (~7 phút)
  uint32_t uptime_sec = millis() / 1000;
  float fake_v = 12.6f - ((float)uptime_sec * 0.005f);

  if (fake_v < 9.8f) {
    fake_v = 9.8f; // Giới hạn dưới tối thiểu để tránh số âm
  }
  current_voltage = fake_v;
#else
  // 1. Đọc giá trị ADC thô từ chân PA1
  uint16_t raw = analogRead(PIN_BATTERY_ADC);
  float instant_v = (float)raw * adc_to_voltage_factor;

  // 2. Cập nhật vào bộ đệm xoay vòng của bộ lọc trung bình động
  battery_samples[sample_index] = instant_v;
  sample_index = (sample_index + 1) % FILTER_SAMPLES;

  // 3. Tính trung bình cộng các mẫu
  float sum = 0.0f;
  for (uint8_t i = 0; i < FILTER_SAMPLES; i++) {
    sum += battery_samples[i];
  }
  current_voltage = sum / FILTER_SAMPLES;
#endif

  // Phân loại trạng thái pin dựa trên điện áp (có Hysteresis 0.2V chống dội)
  BatteryState previous_state = current_state;
#if ENABLE_BATTERY_CHECK
  if (current_state == BATTERY_NORMAL) {
    if (current_voltage < BATTERY_THRESHOLD_CRITICAL) {
      current_state = BATTERY_CRITICAL;
    } else if (current_voltage < BATTERY_THRESHOLD_LOW) {
      current_state = BATTERY_LOW;
    }
  } else if (current_state == BATTERY_LOW) {
    if (current_voltage < BATTERY_THRESHOLD_CRITICAL) {
      current_state = BATTERY_CRITICAL;
    } else if (current_voltage > (BATTERY_THRESHOLD_LOW + 0.2f)) {
      current_state = BATTERY_NORMAL;
    }
  } else if (current_state == BATTERY_CRITICAL) {
    if (current_voltage > (BATTERY_THRESHOLD_CRITICAL + 0.2f)) {
      current_state = BATTERY_LOW;
    }
  }
#else
  current_state = BATTERY_NORMAL;
#endif

  if (current_state != previous_state) {
#if ENABLE_DEBUG
    char v_str[10];
    dtostrf(current_voltage, 4, 2, v_str);
    if (current_state == BATTERY_LOW) {
      Serial.printf("[BATTERY STATE CHANGE] LOW BATTERY! Voltage: %sV\r\n",
                    v_str);
    } else if (current_state == BATTERY_CRITICAL) {
      Serial.printf("[BATTERY STATE CHANGE] CRITICAL BATTERY! Voltage: %sV\r\n",
                    v_str);
    } else {
      Serial.printf("[BATTERY STATE CHANGE] NORMAL BATTERY. Voltage: %sV\r\n",
                    v_str);
    }
#endif
  }
}

float batteryGetVoltage() { return current_voltage; }

BatteryState batteryGetState() { return current_state; }
