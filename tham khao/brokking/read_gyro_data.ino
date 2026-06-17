///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//This part reads the raw gyro and accelerometer data from the MPU-6050
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void gyro_signalen(void) {
  HWire.beginTransmission(gyro_address);                       //// (1) Gọi cảm biến 0x68
  HWire.write(0x3B);                                           //(2) "Mở thanh ghi 0x3B"
  HWire.endTransmission();                                     //(3) Gửi đi
  HWire.requestFrom(gyro_address, 14);                         //(4) Yêu cầu 14 byte
  //I2C truyền 8 bit/lần, nhưng dữ liệu cảm biến là 16 bit, dịch bit
  acc_y = HWire.read() << 8 | HWire.read();                    //Add the low and high byte to the acc_x variable.
  acc_x = HWire.read() << 8 | HWire.read();                    //Add the low and high byte to the acc_y variable.
  acc_z = HWire.read() << 8 | HWire.read();                    //Add the low and high byte to the acc_z variable.
//"X" của cảm biến (đọc đầu) → gán làm acc_y của drone (= trục pitch của drone).
//"Y" của cảm biến (đọc thứ 2) → gán làm acc_x của drone (= trục roll).
//"Z" của cảm biến (đọc thứ 3) → acc_z (lực nâng/trọng lực) — trùng khớp.

  temperature = HWire.read() << 8 | HWire.read();              //Add the low and high byte to the temperature variable.
  gyro_roll = HWire.read() << 8 | HWire.read();                //Read high and low part of the angular data.
  gyro_pitch = HWire.read() << 8 | HWire.read();               //Read high and low part of the angular data.
  gyro_yaw = HWire.read() << 8 | HWire.read();                 //Read high and low part of the angular data.
  gyro_pitch *= -1;                                            //Invert the direction of the axis.
  gyro_yaw *= -1;                                              //Invert the direction of the axis.
  //Cảm biến quy ước "xoay theo chiều kim đồng hồ = dương" có thể khác với quy ước drone ("nghiêng trước = pitch dương", "xoay phải = yaw dương").

  //Trừ offset hiệu chỉnh
  acc_y -= manual_acc_pitch_cal_value;                         //Subtact the manual accelerometer pitch calibration value. // Trừ offset accel Y (= pitch của drone)
  acc_x -= manual_acc_roll_cal_value;                          //Subtact the manual accelerometer roll calibration value. Trừ offset accel X
  gyro_roll -= manual_gyro_roll_cal_value;                     //Subtact the manual gyro roll calibration value.
  gyro_pitch -= manual_gyro_pitch_cal_value;                   //Subtact the manual gyro pitch calibration value.
  gyro_yaw -= manual_gyro_yaw_cal_value;                       //Subtact the manual gyro yaw calibration value.
}


//Gửi 1 lệnh I2C yêu cầu 14 byte từ địa chỉ 0x3B (mẹo auto-increment).
//Đọc 14 byte rồi ghép thành 7 giá trị 16-bit bằng << 8 | read().
//Gán biến lệch trục (acc_y/x/z thay vì x/y/z) — khớp quy ước drone.
//Đảo dấu gyro_pitch và gyro_yaw cho đúng chiều bay.
//Trừ offset đã hiệu chỉnh cho 5 đại lượng.
