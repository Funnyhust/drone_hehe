#include "middleware/blackbox.h"

// Khởi tạo bộ đệm tĩnh trong RAM
static BlackboxEntry log_buffer[BLACKBOX_LIMIT];
static uint16_t write_head = 0;
static uint16_t entry_count = 0;

void blackboxInit() {
  blackboxReset();
}

void blackboxLog(uint32_t loop_time_us, float out_roll, float out_pitch, float out_yaw,
                 float vbat, uint8_t lq, int8_t rssi) {
  
  // Ghi trực tiếp vào vị trí con trỏ ghi hiện tại
  log_buffer[write_head].loop_time_us = loop_time_us;
  log_buffer[write_head].out_roll    = (int16_t)out_roll;
  log_buffer[write_head].out_pitch   = (int16_t)out_pitch;
  log_buffer[write_head].out_yaw     = (int16_t)out_yaw;
  log_buffer[write_head].vbat_mv     = (uint16_t)(vbat * 1000.0f); // Quy đổi sang mV
  log_buffer[write_head].lq          = lq;
  log_buffer[write_head].rssi        = rssi;

  // Dịch con trỏ ghi dạng vòng tròn
  write_head = (write_head + 1) % BLACKBOX_LIMIT;

  // Tăng số lượng mẫu ghi nhận được
  if (entry_count < BLACKBOX_LIMIT) {
    entry_count++;
  }
}

void blackboxDumpSerial() {
  if (entry_count == 0) {
    Serial.println("Blackbox: No log data available.");
    return;
  }

  Serial.println("--- START BLACKBOX CSV ---");
  Serial.println("LoopTime_us,Roll_Out,Pitch_Out,Yaw_Out,Vbat_mV,LQ,RSSI");

  // Tìm vị trí của phần tử cũ nhất trong bộ đệm vòng
  uint16_t read_index;
  if (entry_count < BLACKBOX_LIMIT) {
    read_index = 0;
  } else {
    read_index = write_head; // Nếu bộ đệm đầy, phần tử cũ nhất chính là vị trí ghi tiếp theo
  }

  for (uint16_t i = 0; i < entry_count; i++) {
    const BlackboxEntry *entry = &log_buffer[read_index];

    Serial.print(entry->loop_time_us);   Serial.print(",");
    Serial.print(entry->out_roll);       Serial.print(",");
    Serial.print(entry->out_pitch);      Serial.print(",");
    Serial.print(entry->out_yaw);        Serial.print(",");
    Serial.print(entry->vbat_mv);        Serial.print(",");
    Serial.print(entry->lq);             Serial.print(",");
    Serial.println(entry->rssi);

    read_index = (read_index + 1) % BLACKBOX_LIMIT;
  }

  Serial.println("--- END BLACKBOX CSV ---");
}

void blackboxReset() {
  write_head = 0;
  entry_count = 0;
}
