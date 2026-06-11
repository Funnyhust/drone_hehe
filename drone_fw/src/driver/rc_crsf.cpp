#include "driver/rc_crsf.h"
#include "board_pinmap.h"

// Bộ đệm lưu trữ các kênh điều khiển (us)
static volatile uint16_t crsf_channels[CRSF_NUM_CHANNELS];

// Các biến lưu trữ trạng thái liên kết sóng
static volatile uint32_t last_crsf_packet_time = 0;
static volatile uint8_t crsf_lq = 0;
static volatile int8_t crsf_rssi = -120;
static volatile uint32_t crsf_rx_cnt = 0;

// Các định nghĩa về cấu trúc và gói tin CRSF
#define CRSF_SYNC_BYTE 0xC8
#define CRSF_FRAMETYPE_LINK_STATISTICS 0x14
#define CRSF_FRAMETYPE_RC_CHANNELS_PACKED 0x16

// Các trạng thái của máy trạng thái phân tích gói UART
enum ParserState { STATE_WAIT_SYNC, STATE_WAIT_LEN, STATE_WAIT_PAYLOAD };

static ParserState parser_state = STATE_WAIT_SYNC;
static uint8_t frame_len = 0;
static uint8_t write_index = 0;
static uint8_t rx_buffer[64]; // Bộ đệm chứa payload + CRC (byte đầu là Type)

// =============================================================================
// Thuật toán kiểm tra lỗi CRC-8 của CRSF (đa thức 0xD5)
// =============================================================================
static uint8_t crsf_crc8(const uint8_t *data, uint8_t len) {
  uint8_t crc = 0;
  for (uint8_t i = 0; i < len; i++) {
    crc ^= data[i];
    for (uint8_t j = 0; j < 8; j++) {
      if (crc & 0x80) {
        crc = (crc << 1) ^ 0xD5;
      } else {
        crc <<= 1;
      }
    }
  }
  return crc;
}

// =============================================================================
// Hàm giải nén 16 kênh điều khiển 11-bit từ gói tin CRSF 22 bytes
// =============================================================================
static void crsf_decode_channels(const uint8_t *p_payload) {
  uint16_t raw_ch[CRSF_NUM_CHANNELS];

  // Giải nén các bitfield 11-bit
  raw_ch[0] = (p_payload[0] | p_payload[1] << 8) & 0x07FF;
  raw_ch[1] = (p_payload[1] >> 3 | p_payload[2] << 5) & 0x07FF;
  raw_ch[2] =
      (p_payload[2] >> 6 | p_payload[3] << 2 | p_payload[4] << 10) & 0x07FF;
  raw_ch[3] = (p_payload[4] >> 1 | p_payload[5] << 7) & 0x07FF;
  raw_ch[4] = (p_payload[5] >> 4 | p_payload[6] << 4) & 0x07FF;
  raw_ch[5] =
      (p_payload[6] >> 7 | p_payload[7] << 1 | p_payload[8] << 9) & 0x07FF;
  raw_ch[6] = (p_payload[8] >> 2 | p_payload[9] << 6) & 0x07FF;
  raw_ch[7] = (p_payload[9] >> 5 | p_payload[10] << 3) & 0x07FF;
  raw_ch[8] = (p_payload[11] | p_payload[12] << 8) & 0x07FF;
  raw_ch[9] = (p_payload[12] >> 3 | p_payload[13] << 5) & 0x07FF;
  raw_ch[10] =
      (p_payload[13] >> 6 | p_payload[14] << 2 | p_payload[15] << 10) & 0x07FF;
  raw_ch[11] = (p_payload[15] >> 1 | p_payload[16] << 7) & 0x07FF;
  raw_ch[12] = (p_payload[16] >> 4 | p_payload[17] << 4) & 0x07FF;
  raw_ch[13] =
      (p_payload[17] >> 7 | p_payload[18] << 1 | p_payload[19] << 9) & 0x07FF;
  raw_ch[14] = (p_payload[19] >> 2 | p_payload[20] << 6) & 0x07FF;
  raw_ch[15] = (p_payload[20] >> 5 | p_payload[21] << 3) & 0x07FF;

  // Quy đổi giá trị thô (172 - 1811) sang xung microseconds (988us - 2012us)
  for (uint8_t i = 0; i < CRSF_NUM_CHANNELS; i++) {
    // Giới hạn giá trị thô để tránh tràn phép tính
    uint16_t raw = raw_ch[i];
    if (raw < 172)
      raw = 172;
    if (raw > 1811)
      raw = 1811;

    // Công thức scale tuyến tính: 988 + (raw - 172) * (2012 - 988) / (1811 -
    // 172) Rút gọn thành: 988 + (raw - 172) * 1024 / 1639
    uint32_t pulse_us = 988 + ((uint32_t)(raw - 172) * 1024) / 1639;
    crsf_channels[i] = (uint16_t)pulse_us;
  }
}

// =============================================================================
// API thực thi
// =============================================================================

void crsfInit() {
  // Cài đặt mặc định an toàn cho các kênh
  for (uint8_t i = 0; i < CRSF_NUM_CHANNELS; i++) {
    crsf_channels[i] =
        (i == 2) ? 1000 : CRSF_CHANNEL_MID_US; // Kênh 2 là Throttle = 1000us
  }

  // Khởi tạo Serial2 chuẩn phần cứng kết nối bộ thu ELRS (PA2 TX / PA3 RX) ở
  // 420000 baud Thiết lập chân RX và TX tường minh để đảm bảo không bị lỗi ánh
  // xạ chân trên các phiên bản Core khác nhau
  Serial2.setRx(PIN_ELRS_RX); // PA3
  Serial2.setTx(PIN_ELRS_TX); // PA2
  Serial2.begin(420000);

  last_crsf_packet_time = 0;
  crsf_lq = 0;
  crsf_rssi = -120;
  parser_state = STATE_WAIT_SYNC;
}

void crsfUpdate() {
  while (Serial2.available() > 0) {
    uint8_t rx_byte = Serial2.read();
    crsf_rx_cnt++;

    switch (parser_state) {
    case STATE_WAIT_SYNC:
      if (rx_byte == CRSF_SYNC_BYTE) {
        parser_state = STATE_WAIT_LEN;
      }
      break;

    case STATE_WAIT_LEN:
      // Độ dài gói tin (Length) hợp lệ nằm trong khoảng 4 đến 64 bytes
      if (rx_byte >= 4 && rx_byte <= 64) {
        frame_len = rx_byte;
        write_index = 0;
        parser_state = STATE_WAIT_PAYLOAD;
      } else {
        parser_state = STATE_WAIT_SYNC; // Sai định dạng gói, tìm lại sync
      }
      break;

    case STATE_WAIT_PAYLOAD:
      rx_buffer[write_index++] = rx_byte;

      // Đã nhận đủ phần còn lại (payload + 1 byte CRC)
      if (write_index == frame_len) {
        // Tính toán CRC-8 trên toàn bộ payload (trừ byte CRC cuối cùng)
        uint8_t calculated_crc = crsf_crc8(rx_buffer, frame_len - 1);
        uint8_t received_crc = rx_buffer[frame_len - 1];

        if (calculated_crc == received_crc) {
          // Giải mã gói tin dựa vào loại (Type byte ở rx_buffer[0])
          uint8_t packet_type = rx_buffer[0];

          if (packet_type == CRSF_FRAMETYPE_RC_CHANNELS_PACKED &&
              frame_len == 24) {
            crsf_decode_channels(
                &rx_buffer[1]); // Giải nén payload kênh bắt đầu từ byte thứ 2
            last_crsf_packet_time = millis(); // Cập nhật mốc thời gian nhận tin
          } else if (packet_type == CRSF_FRAMETYPE_LINK_STATISTICS &&
                     frame_len == 12) {
            crsf_lq =
                rx_buffer[3]; // Vị trí Link Quality trong payload Link Stats
            // Chỉ số RSSI 1 thu được dạng dương, đổi sang dBm âm
            crsf_rssi = -((int8_t)rx_buffer[1]);
            last_crsf_packet_time = millis(); // Cập nhật mốc thời gian
          }
        }
        // Reset máy trạng thái để chờ gói tin tiếp theo
        parser_state = STATE_WAIT_SYNC;
      }
      break;

    default:
      parser_state = STATE_WAIT_SYNC;
      break;
    }
  }
}

uint16_t crsfGetChannel(uint8_t ch) {
  if (ch >= CRSF_NUM_CHANNELS)
    return CRSF_CHANNEL_MID_US;

  // Giới hạn giá trị trả về trong khoảng an toàn tuyệt đối
  uint16_t val = crsf_channels[ch];
  if (val < CRSF_CHANNEL_MIN_US)
    val = CRSF_CHANNEL_MIN_US;
  if (val > CRSF_CHANNEL_MAX_US)
    val = CRSF_CHANNEL_MAX_US;
  return val;
}

bool crsfIsLinkActive() {
  if (last_crsf_packet_time == 0)
    return false;
  // Failsafe Hard Kill: Nếu mất tin quá 200ms
  return (millis() - last_crsf_packet_time) <= 200;
}

bool crsfIsLinkWarning() {
  if (last_crsf_packet_time == 0)
    return false;
  uint32_t dt = millis() - last_crsf_packet_time;
  // Failsafe Warning: Mất tin từ 100ms đến 200ms
  return (dt > 100 && dt <= 200);
}

uint8_t crsfGetLq() {
  if (!crsfIsLinkActive())
    return 0;
  return crsf_lq;
}

int8_t crsfGetRssi() {
  if (!crsfIsLinkActive())
    return -120;
  return crsf_rssi;
}

uint32_t crsfGetRxCnt() { return crsf_rx_cnt; }

void crsfSendTelemetryBattery(uint16_t voltage_centi_v, uint16_t current_centi_a, uint32_t capacity_mah, uint8_t remaining_percent) {
  uint8_t frame[12];
  frame[0] = 0xC8; // CRSF Telemetry Sync Byte (0xC8)
  frame[1] = 10;   // Length: Type (1) + Payload (8) + CRC (1) = 10
  frame[2] = 0x08; // Type: CRSF_FRAMETYPE_BATTERY_SENSOR
  
  // Đổi từ centivolts (0.01V) / centiamperes (0.01A) sang decivolts (0.1V) / deciamperes (0.1A) theo chuẩn CRSF
  uint16_t voltage_deciv = voltage_centi_v / 10;
  uint16_t current_decia = current_centi_a / 10;

  // Payload (Big-Endian)
  frame[3] = (voltage_deciv >> 8) & 0xFF;
  frame[4] = voltage_deciv & 0xFF;
  frame[5] = (current_decia >> 8) & 0xFF;
  frame[6] = current_decia & 0xFF;
  
  // Capacity 24-bit (mAh)
  frame[7] = (capacity_mah >> 16) & 0xFF;
  frame[8] = (capacity_mah >> 8) & 0xFF;
  frame[9] = capacity_mah & 0xFF;
  
  frame[10] = remaining_percent;
  
  // CRC8 tính từ byte Type (frame[2]) đến byte Remaining (frame[10]) - tổng 9 bytes
  frame[11] = crsf_crc8(&frame[2], 9);
  
  // Gửi gói tin telemetry ra Serial2 kết nối ELRS RX
  Serial2.write(frame, 12);
}
