#include "driver/mpu6050.h"
#include "driver/soft_i2c.h"

// Biến toàn cục lưu trữ offset cảm biến (mặc định bằng 0)
static int16_t offset_ax = 0;
static int16_t offset_ay = 0;
static int16_t offset_az = 0;
static int16_t offset_gx = 0;
static int16_t offset_gy = 0;
static int16_t offset_gz = 0;

uint8_t mpu6050Init() {
  uint8_t who_am_i = 0;

  // 1. Đánh thức MPU6050 và chọn clock nguồn từ Gyro X (độ ổn định cao hơn clock nội)
  if (softI2cWriteReg(MPU6050_ADDR, 0x6B, 0x01) != I2C_OK) {
    return 1; // Lỗi giao tiếp I2C
  }

  // 2. Kiểm tra thanh ghi WHO_AM_I (thanh ghi 0x75), MPU6050 phải trả về 0x68
  if (softI2cReadReg(MPU6050_ADDR, 0x75, &who_am_i) != I2C_OK || who_am_i != 0x68) {
    return 1; // Thiết bị không phản hồi hoặc không đúng loại MPU6050
  }

  // 3. Cấu hình tốc độ trích mẫu: Sample Rate = 1kHz (SMPLRT_DIV = 0)
  softI2cWriteReg(MPU6050_ADDR, 0x19, 0x00);

  // 4. Cấu hình bộ lọc số thông thấp (DLPF CONFIG): set 42Hz Bandwidth (CONFIG = 0x03)
  softI2cWriteReg(MPU6050_ADDR, 0x1A, 0x03);

  // 5. Cấu hình dải đo Gyroscope: FS_SEL = 3 tương ứng +-2000 deg/s (GYRO_CONFIG = 0x18)
  softI2cWriteReg(MPU6050_ADDR, 0x1B, 0x18);

  // 6. Cấu hình dải đo Accelerometer: AFS_SEL = 2 tương ứng +-8g (ACCEL_CONFIG = 0x10)
  softI2cWriteReg(MPU6050_ADDR, 0x1C, 0x10);

  return 0; // Khởi tạo thành công
}

uint8_t mpu6050Read(MpuData *p_data) {
  uint8_t buf[14];

  // Burst Read 14 bytes liên tiếp bắt đầu từ thanh ghi 0x3B (ACCEL_XOUT_H)
  if (softI2cReadBytes(MPU6050_ADDR, 0x3B, buf, 14) != I2C_OK) {
    return 1; // Lỗi đọc I2C
  }

  // Ghép các byte thô thu được (mỗi giá trị là 16-bit signed)
  int16_t ax_raw_orig = (int16_t)((buf[0] << 8) | buf[1]);
  int16_t ay_raw_orig = (int16_t)((buf[2] << 8) | buf[3]);
  int16_t az_raw_orig = (int16_t)((buf[4] << 8) | buf[5]);
  p_data->temp_raw    = (int16_t)((buf[6] << 8) | buf[7]);
  int16_t gx_raw_orig = (int16_t)((buf[8] << 8) | buf[9]);
  int16_t gy_raw_orig = (int16_t)((buf[10] << 8) | buf[11]);
  int16_t gz_raw_orig = (int16_t)((buf[12] << 8) | buf[13]);

  // Áp dụng các giá trị hiệu chỉnh Offset
  p_data->ax_raw = ax_raw_orig - offset_ax;
  p_data->ay_raw = ay_raw_orig - offset_ay;
  p_data->az_raw = az_raw_orig - offset_az;
  p_data->gx_raw = gx_raw_orig - offset_gx;
  p_data->gy_raw = gy_raw_orig - offset_gy;
  p_data->gz_raw = gz_raw_orig - offset_gz;

  // Quy đổi sang đơn vị vật lý thực tế
  p_data->ax = (float)p_data->ax_raw / MPU6050_ACCEL_SO_8G;
  p_data->ay = (float)p_data->ay_raw / MPU6050_ACCEL_SO_8G;
  p_data->az = (float)p_data->az_raw / MPU6050_ACCEL_SO_8G;
  p_data->gx = (float)p_data->gx_raw / MPU6050_GYRO_SO_2000;
  p_data->gy = (float)p_data->gy_raw / MPU6050_GYRO_SO_2000;
  p_data->gz = (float)p_data->gz_raw / MPU6050_GYRO_SO_2000;

  // Công thức quy đổi nhiệt độ của MPU6050 (oC)
  p_data->temp = ((float)p_data->temp_raw / 340.0f) + 36.53f;

  return 0;
}

void mpu6050Calibrate() {
  int32_t sum_ax = 0, sum_ay = 0, sum_az = 0;
  int32_t sum_gx = 0, sum_gy = 0, sum_gz = 0;
  MpuData temp_data;
  uint16_t valid_samples = 0;

  // Reset tạm thời offset về 0 để đọc các giá trị thô thực tế của cảm biến
  mpu6050SetOffsets(0, 0, 0, 0, 0, 0);

  // Thu thập 2000 mẫu liên tục (đáp ứng chu kỳ khoảng 2 giây)
  for (uint16_t i = 0; i < 2000; i++) {
    // Đọc cảm biến
    if (mpu6050Read(&temp_data) == 0) {
      sum_ax += temp_data.ax_raw;
      sum_ay += temp_data.ay_raw;
      sum_az += temp_data.az_raw;
      sum_gx += temp_data.gx_raw;
      sum_gy += temp_data.gy_raw;
      sum_gz += temp_data.gz_raw;
      valid_samples++;
    }
    // Trễ 1ms giữa các lần đọc (tổng cộng 2000ms = 2s)
    delayMicroseconds(1000);
  }

  // Nếu thu thập mẫu thành công
  if (valid_samples > 0) {
    offset_ax = sum_ax / valid_samples;
    offset_ay = sum_ay / valid_samples;
    
    // Trục Z hướng thẳng đứng chịu gia tốc trọng trường 1g.
    // Với dải +-8g (4096 LSB/g), giá trị lý thuyết Z là +4096 LSB.
    // Lấy trung bình đọc được trừ đi giá trị lý thuyết 1g để ra offset thực tế.
    offset_az = (sum_az / valid_samples) - 4096;
    
    offset_gx = sum_gx / valid_samples;
    offset_gy = sum_gy / valid_samples;
    offset_gz = sum_gz / valid_samples;
  }
}

void mpu6050SetOffsets(int16_t ax_off, int16_t ay_off, int16_t az_off,
                       int16_t gx_off, int16_t gy_off, int16_t gz_off) {
  offset_ax = ax_off;
  offset_ay = ay_off;
  offset_az = az_off;
  offset_gx = gx_off;
  offset_gy = gy_off;
  offset_gz = gz_off;
}

void mpu6050GetOffsets(int16_t *ax_off, int16_t *ay_off, int16_t *az_off,
                       int16_t *gx_off, int16_t *gy_off, int16_t *gz_off) {
  *ax_off = offset_ax;
  *ay_off = offset_ay;
  *az_off = offset_az;
  *gx_off = offset_gx;
  *gy_off = offset_gy;
  *gz_off = offset_gz;
}
