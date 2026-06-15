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

uint8_t mpu6050CalibrateGyro(float *p_stddev) {
  int32_t sum_gx = 0, sum_gy = 0, sum_gz = 0;
  int64_t sum_sq_gx = 0, sum_sq_gy = 0, sum_sq_gz = 0;
  MpuData temp_data;
  uint16_t valid_samples = 0;

  // Reset tạm thời offset Gyro về 0 để đo các giá trị thô thực tế
  int16_t prev_gx_off = offset_gx;
  int16_t prev_gy_off = offset_gy;
  int16_t prev_gz_off = offset_gz;
  mpu6050SetOffsets(offset_ax, offset_ay, offset_az, 0, 0, 0);

  // Thu thập 1000 mẫu liên tục ở tần số 1kHz (~1 giây)
  for (uint16_t i = 0; i < 1000; i++) {
    if (mpu6050Read(&temp_data) == 0) {
      sum_gx += temp_data.gx_raw;
      sum_sq_gx += (int32_t)temp_data.gx_raw * temp_data.gx_raw;

      sum_gy += temp_data.gy_raw;
      sum_sq_gy += (int32_t)temp_data.gy_raw * temp_data.gy_raw;

      sum_gz += temp_data.gz_raw;
      sum_sq_gz += (int32_t)temp_data.gz_raw * temp_data.gz_raw;

      valid_samples++;
    }
    delayMicroseconds(1000); // 1ms delay
  }

  // Khôi phục lại offset cũ trước khi đánh giá
  mpu6050SetOffsets(offset_ax, offset_ay, offset_az, prev_gx_off, prev_gy_off, prev_gz_off);

  if (valid_samples < 800) {
    return 1; // Lỗi giao tiếp I2C quá nhiều
  }

  // Tính trung bình cộng (Mean)
  float mean_gx = (float)sum_gx / valid_samples;
  float mean_gy = (float)sum_gy / valid_samples;
  float mean_gz = (float)sum_gz / valid_samples;

  // Tính phương sai (Variance): E[X^2] - (E[X])^2
  float var_gx = ((float)sum_sq_gx / valid_samples) - (mean_gx * mean_gx);
  float var_gy = ((float)sum_sq_gy / valid_samples) - (mean_gy * mean_gy);
  float var_gz = ((float)sum_sq_gz / valid_samples) - (mean_gz * mean_gz);

  if (var_gx < 0) var_gx = 0;
  if (var_gy < 0) var_gy = 0;
  if (var_gz < 0) var_gz = 0;

  // Tính độ lệch chuẩn (Standard Deviation)
  float stddev_gx = sqrt(var_gx);
  float stddev_gy = sqrt(var_gy);
  float stddev_gz = sqrt(var_gz);

  // Lấy độ lệch chuẩn lớn nhất trong 3 trục để đánh giá độ rung động
  float max_stddev = stddev_gx;
  if (stddev_gy > max_stddev) max_stddev = stddev_gy;
  if (stddev_gz > max_stddev) max_stddev = stddev_gz;

  if (p_stddev != nullptr) {
    *p_stddev = max_stddev;
  }

  // Nếu độ lệch chuẩn vượt quá ngưỡng cho phép, tức là drone đang bị di chuyển/rung lắc
  if (max_stddev > GYRO_CALIB_STDDEV_THRESHOLD) {
    return 2; // Lỗi di chuyển (CALIBRATION_FAILED_IMU_MOVING)
  }

  // Nếu đạt yêu cầu, cập nhật offset Gyro mới
  offset_gx = (int16_t)mean_gx;
  offset_gy = (int16_t)mean_gy;
  offset_gz = (int16_t)mean_gz;

  return 0; // Thành công
}

uint8_t mpu6050CalibrateAccel() {
  int32_t sum_ax = 0, sum_ay = 0, sum_az = 0;
  MpuData temp_data;
  uint16_t valid_samples = 0;

  // Reset tạm thời offset Accel để đo giá trị thực tế của cảm biến
  int16_t prev_ax_off = offset_ax;
  int16_t prev_ay_off = offset_ay;
  int16_t prev_az_off = offset_az;
  mpu6050SetOffsets(0, 0, 0, offset_gx, offset_gy, offset_gz);

  // Thu thập 4000 mẫu cân bằng tĩnh (khoảng 4 giây)
  for (uint16_t i = 0; i < 4000; i++) {
    if (mpu6050Read(&temp_data) == 0) {
      sum_ax += temp_data.ax_raw;
      sum_ay += temp_data.ay_raw;
      sum_az += temp_data.az_raw;
      valid_samples++;
    }
    delayMicroseconds(1000);
  }

  // Khôi phục lại offset cũ trước khi đánh giá
  mpu6050SetOffsets(prev_ax_off, prev_ay_off, prev_az_off, offset_gx, offset_gy, offset_gz);

  if (valid_samples < 3200) {
    return 1; // Lỗi đọc I2C quá nhiều
  }

  // Tính trung bình và cập nhật offset mới
  offset_ax = sum_ax / valid_samples;
  offset_ay = sum_ay / valid_samples;
  
  // Trục Z hướng thẳng đứng chịu gia tốc trọng trường 1g.
  // Với dải +-8g (4096 LSB/g), giá trị lý thuyết Z là +4096 LSB.
  offset_az = (sum_az / valid_samples) - 4096;

  return 0; // Thành công
}

uint8_t mpu6050ValidateAccelStatic(float *p_g_total) {
  MpuData temp_data;
  float sum_ax = 0, sum_ay = 0, sum_az = 0;
  uint8_t valid_count = 0;

  // Đọc 10 mẫu liên tục để tính trung bình chống nhiễu tức thời
  for (uint8_t i = 0; i < 10; i++) {
    if (mpu6050Read(&temp_data) == 0) {
      sum_ax += temp_data.ax;
      sum_ay += temp_data.ay;
      sum_az += temp_data.az;
      valid_count++;
    }
    delayMicroseconds(2000); // Trễ 2ms giữa các lần đọc
  }

  if (valid_count == 0) {
    return 1; // Lỗi giao tiếp cảm biến hoàn toàn
  }

  float ax_mean = sum_ax / valid_count;
  float ay_mean = sum_ay / valid_count;
  float az_mean = sum_az / valid_count;

  // Độ lớn tổng vector g
  float g_total = sqrt(ax_mean * ax_mean + ay_mean * ay_mean + az_mean * az_mean);
  if (p_g_total != nullptr) {
    *p_g_total = g_total;
  }

  // Kiểm tra dải cho phép từ 0.95g đến 1.05g
  if (g_total < 0.95f || g_total > 1.05f) {
    return 1; // Vượt ngoài dải an toàn (IMU_ERROR)
  }

  return 0; // Hợp lệ
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
