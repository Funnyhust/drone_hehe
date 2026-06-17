///////////////////////////////////////////////////////////////////////////////////////
//Terms of use
///////////////////////////////////////////////////////////////////////////////////////
//THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
//IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
//FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
//AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
//LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
//OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
//THE SOFTWARE.
///////////////////////////////////////////////////////////////////////////////////////
//Safety note
///////////////////////////////////////////////////////////////////////////////////////
//Always remove the propellers and stay away from the motors unless you
//are 100% certain of what you are doing.
///////////////////////////////////////////////////////////////////////////////////////
#include <Wire.h> // khai báo thư viện để giao tiếp i2c với cảm biến

//Manual accelerometer calibration values for IMU angles:
int16_t manual_acc_pitch_cal_value = 0; 
int16_t manual_acc_roll_cal_value = 0; // Hai biến này lưu độ lệch gốc (offset) của gia tốc kế để sau trừ đi.

//Manual gyro calibration values.
//Set the use_manual_calibration variable to true to use the manual calibration variables.
uint8_t use_manual_calibration = false; //Công tắc bật/tắt chế độ hiệu chỉnh thủ công, để false thì đèn đỏ sẽ nháy vài giây rồi bay, calib thủ công
int16_t manual_gyro_pitch_cal_value = 0;
int16_t manual_gyro_roll_cal_value = 0;
int16_t manual_gyro_yaw_cal_value = 0; //3 biến lưu độ trôi gốc của con quay hồi chuyển trên 3 trục pitch/roll/yaw


TwoWire HWire(2, I2C_FAST_MODE); // dùng bộ I2C số 2 của STM32 (nằm ở chân PB10 = SCL, PB11 = SDA);chạy tốc độ nhanh 400kHz

//Let's declare some variables so we can use them in the complete program.
//int16_t = signed 16 bit integer
//uint16_t = unsigned 16 bit integer
uint8_t disable_throttle, flip32; //công tắc khóa ga (cho an toàn). flip32 = cờ nhận biết loại board (Bluepill hay Flip32) để bật LED đúng chiều.
uint32_t loop_timer; //Lưu lại mốc thời gian của hàm micros() để vi điều khiển so sánh và chốt chính xác vòng lặp chạy đúng 4000 micro-giây (250Hz).
float angle_roll_acc, angle_pitch_acc, angle_pitch, angle_roll; //2 biến angle_pitch / angle_roll = góc cuối sau bộ lọc bù (mượt + không trôi) 2 biến ..._acc = góc trung gian tính từ accel
float battery_voltage; //Khai báo biến lưu góc đo và điện áp
int16_t loop_counter; //Chỉ để đếm xem vòng lặp chính đã chạy được bao nhiêu lần
uint8_t data, start, warning; //data dùng để hứng ký tự gõ từ bàn phím (ví dụ phím 'a', 'h'). start quản lý trình tự khởi động (0 = khóa, 1 = chờ, 2 = đã Arming motor). warning là cờ báo lỗi (nếu phát hiện chưa gạt ga về mức 0 lúc bật máy, biến này sẽ nhảy lên 1).
int16_t acc_axis[4], gyro_axis[4], temperature; //Khớp đúng chuẩn đầu ra 16-bit của cảm biến. acc_axis lưu gia tốc X, Y, Z. gyro_axis lưu vận tốc góc X, Y, Z. Bỏ trống phần tử số 0
int32_t gyro_axis_cal[4], acc_axis_cal[4]; //giá trị hiệu chỉnh của các trục gyro và accel
int32_t cal_int; //Vì khi calib, máy sẽ cộng dồn 4000 lần giá trị của cảm biến lại với nhau trước khi đem chia trung bình, dùng 32bit để tránh tràn bộ nhớ
int32_t channel_1_start, channel_1;
int32_t channel_2_start, channel_2;
int32_t channel_3_start, channel_3;
int32_t channel_4_start, channel_4; // Mỗi kênh tay điều khiển có 2 biến: ..._start lưu thời điểm xung bắt đầu lên cao; channel_x lưu độ rộng xung
int32_t channel_5_start, channel_5; 
int32_t channel_6_start, channel_6; //Timer sẽ lưu thời điểm bắt đầu thấy xung điện mức CAO của từng kênh. Timer lấy thời điểm kết thúc xung trừ đi thời điểm bắt đầu để ra độ rộng xung thực tế từ 1000muys-2000muys

//The I2C address of the MPU-6050 is 0x68 in hexadecimal form.
  uint8_t gyro_address = 0x68; // Chuẩn giao tiếp I2C của cảm biến là chuỗi nhị phân 110100, chân 7 nối đất là 0 -> 0x68 là địa chỉ i2c của cảm biến, lưu địa chỉ đó vào stm32 để đọc và xử lí

void setup() {
  pinMode(4, INPUT_ANALOG); //Thiết lập chân PA4 thành ngõ vào chế độ Analog để đo điện áp pin.
  //Port PB3 and PB4 are used as JTDO and JNTRST by default.
  //The following function connects PB3 and PB4 to the alternate output function.
  afio_cfg_debug_ports(AFIO_DEBUG_SW_ONLY);                     //Connects PB3 and PB4 to output function.Nó tắt chức năng JTAG của các chân PB3/PB4 để điều khiển led, ko thì led ko sáng

  //Tự nhận diện loại board Có 2 loại board phổ biến: Bluepill thường và Flip32
  pinMode(PB3, INPUT);                                         //tạm đặt 2 chân thành đầu vào để đọc thử mức điện trên đó
  pinMode(PB4, INPUT);                                         //Set PB4 as input.
  if (digitalRead(PB3) || digitalRead(PB3))flip32 = 1;         //đọc mức 0 hay 1 của chân PB3 nếu đọc được mức cao → "à, đây là Flip32" → gán cờ flip32 = 1.
  else flip32 = 0; //Với Bluepill, 2 chân này đọc về 0 

//Sau khi đoán xong loại board, đặt 2 chân thành OUTPUT (đầu ra — để xuất tín hiệu điều khiển LED chứ không còn đọc nữa)
  pinMode(PB3, OUTPUT);                                         //Set PB3 as output.
  pinMode(PB4, OUTPUT);                                         //Set PB4 as output. Chuyển chân PB3/PB4 sang chế độ xuất tín hiệu để điều khiển LED.

  green_led(LOW);                                               //Set output PB3 low.
  red_led(LOW);                                                 //Set output PB4 low.Tắt sạch LED trước khi bắt đầu

  Serial.begin(57600);                                          //Set the serial output to 57600 kbps. Mở cổng Serial để giao tiếp với máy tính ở tốc độ 57600 bps UART, FTDI
  delay(100);                                                    //Give the serial port some time to start to prevent data loss.
  timer_setup();                                                //chuyên cấu hình Timer của STM32 để đọc tay điều khiển + tạo xung ESC
  delay(50);                                                    //Give the timers some time to start. Đợi Timer hoạt động ổn định.

  HWire.begin();                                                //Start the I2C as master Bắt đầu giao thức I2C
  HWire.beginTransmission(gyro_address);                        //bắt đầu kết nối với icm tại địa chỉ 0x68
  HWire.write(0x6B);                                            //We want to write to the PWR_MGMT_1 register (6B hex). 107 trong hệ thập phân, gọi cảm biến để hiển thị dữ liệu
  HWire.write(0x00);                                            //Set the register bits as 00000000 to activate the gyro. Địa chỉ tớ 0x68 - Địa chỉ thanh ghi 0x6B - Giá trị 0x00
  HWire.endTransmission();                                      //End the transmission with the gyro.

  HWire.beginTransmission(gyro_address);                        //Start communication with the ICM.
  HWire.write(0x1B);                                            //We want to write to the GYRO_CONFIG register (1B hex). cấu hình gyro
  HWire.write(0x08);                                            //Set the register bits as 00001000 (500dps full scale). // ghi giá trị 0x08 vào ô 0x1B / /0x08 = 00001000 → bit 4-3 = 01 → ±500 dps
  //Đây chính là dòng tạo ra hằng số 65.5 trong phần tính góc! Ở thang ±500°/s, datasheet nói "65.5 đơn vị thô = 1°/giây"  //→ nên code chia cho 65.5 để đổi số thô ra °/s.
  HWire.endTransmission();                                      //End the transmission with the gyro.

  HWire.beginTransmission(gyro_address);                        //Start communication with the ICM
  HWire.write(0x1C);                                            //We want to write to the ACCEL_CONFIG register (1A hex). cấu hình accel
  HWire.write(0x10);                                            //Set the register bits as 0001 0000 (+/- 8g full scale range). //Ghi 0x10 vào ô 0x1C (ACCEL_CONFIG) → đặt thang đo accelerometer ở ±8g.
  HWire.endTransmission();                                      //End the transmission with the gyro.

  HWire.beginTransmission(gyro_address);                        //Start communication with the ICM
  HWire.write(0x1A);                                            //We want to write to the CONFIG register (1A hex). cấu hình lọc
  HWire.write(0x03);                                            //Set the register bits as 00000011 (Set Digital Low Pass Filter to ~43Hz). //
  HWire.endTransmission();                                      //End the transmission with the gyro.
  //Ghi 0x03 vào ô 0x1A (CONFIG) → bật bộ lọc thông thấp số (DLPF) ~43Hz ngay bên trong cảm biến. Nhờ vậy nhiễu rung tần số cao bị lọc bớt trước khi dữ liệu vào vi điều khiển
  print_intro();                                                //Gọi hàm in danh sách lệnh (a, b, c...) ra Serial Monitor để bạn biết gõ gì.
}

void loop() {
  delay(10);

  if (Serial.available() > 0) {                                 //Serial.available() trả về số byte đang chờ trong bộ đệm (tức bạn đã gõ gì chưa). Nếu > 0 (có ký tự) thì vào trong xử lý
    data = Serial.read();                                       //Read the incomming byte. đọc một ký tự bạn vừa gõ, lưu vào biến data. Ví dụ bạn gõ b thì data = 'b'.
    delay(100);                                                 //Wait for any other bytes to come in. 
    while (Serial.available() > 0)loop_counter = Serial.read(); //Empty the Serial buffer. đọc bỏ hết các ký tự thừa còn lại (đổ vào loop_counter rồi bỏ qua). Mục đích: tránh ký tự rác ảnh hưởng vòng sau
    disable_throttle = 1;                                       //Set the throttle to 1000us to disable the motors. khóa ga ngay khi vừa nhận lệnh, cho an toàn
  }

  if (!disable_throttle) {                                      //If the throttle is not disabled. kênh 3 là kênh ga throttle !disable_throttle = "nếu ga không bị khóa". Khi đó: gán channel_3 (giá trị ga từ tay điều khiển) thẳng ra cả 4 ESC. 
                                                                //→ Bạn đẩy cần ga lên/xuống, cả 4 motor nhận đúng mức đó
    TIMER4_BASE->CCR1 = channel_3;                              //Set the throttle receiver input pulse to the ESC 1 output pulse. Giá trị bạn nạp vào CCR1 sẽ quyết định độ rộng xung
    TIMER4_BASE->CCR2 = channel_3;                              //Set the throttle receiver input pulse to the ESC 2 output pulse. timer4_base Đây là địa chỉ cơ sở (Base Address) trong bộ nhớ của khối ngoại vi Timer 4.
    TIMER4_BASE->CCR3 = channel_3;                              //Set the throttle receiver input pulse to the ESC 3 output pulse.
    TIMER4_BASE->CCR4 = channel_3;                              //Set the throttle receiver input pulse to the ESC 4 output pulse.
  }
  else {                                                        //If the throttle is disabled nếu ga bị khóa (disable_throttle = 1): xuất 1000 (= 1000µs = mức ga 0) cho cả 4 ESC → motor đứng yên, an toàn.
    TIMER4_BASE->CCR1 = 1000;                                   //Set the ESC 1 output to 1000us to disable the motor.
    TIMER4_BASE->CCR2 = 1000;                                   //Set the ESC 2 output to 1000us to disable the motor.
    TIMER4_BASE->CCR3 = 1000;                                   //Set the ESC 3 output to 1000us to disable the motor.
    TIMER4_BASE->CCR4 = 1000;                                   //Set the ESC 4 output to 1000us to disable the motor.
  }

  if (data == 'a') {
    Serial.println(F("In 6 kênh tay điều khiển + mũi tên hướng gạt")); //In liên tục giá trị 6 kênh tay điều khiển ra Serial Monitor, kèm mũi tên chỉ hướng bạn đang gạt cần
    Serial.println(F("You can exit by sending a q (quit)."));
    delay(2500);
    reading_receiver_signals();
  }

  if (data == 'b') {
    Serial.println(F("Dò bus I2C, in địa chỉ thiết bị (phải thấy 0x68)"));
    i2c_scanner();
  }

  if (data == 'c') {
    Serial.println(F("In giá trị thô gyro (có thể tự calib)"));
    Serial.println(F("You can exit by sending a q (quit)."));
    read_gyro_values();
  }

  if (data == 'd') {
    Serial.println(F("In giá trị thô accelerometer"));
    Serial.println(F("You can exit by sending a q (quit)."));
    delay(2500);
    read_gyro_values();
  }

  if (data == 'e') {
    Serial.println(F("Tính & in góc pitch/roll/yaw (dùng bộ lọc bù)"));
    Serial.println(F("You can exit by sending a q (quit)."));
    check_imu_angles();
  }

  if (data == 'f') {
    Serial.println(F("Nháy đèn đỏ rồi xanh"));
    test_leds();
  }

  if (data == 'g') {
    Serial.println(F("In điện áp pin (có lọc nhiễu)"));
    Serial.println(F("You can exit by sending a q (quit)."));
    check_battery_voltage();
  }

  if (data == 'h') {
    Serial.println(F("In 5 số offset để chép vào code bay"));
    manual_imu_calibration();
  }

  if (data == '1') {
    Serial.println(F("Check motor 1 (front right, counter clockwise direction)."));
    Serial.println(F("You can exit by sending a q (quit)."));
    delay(2500);
    check_motor_vibrations();
  }

  if (data == '2') {
    Serial.println(F("Check motor 2 (rear right, clockwise direction)."));
    Serial.println(F("You can exit by sending a q (quit)."));
    delay(2500);
    check_motor_vibrations();
  }

  if (data == '3') {
    Serial.println(F("Check motor 3 (rear left, counter clockwise direction)."));
    Serial.println(F("You can exit by sending a q (quit)."));
    delay(2500);
    check_motor_vibrations();
  }

  if (data == '4') {
    Serial.println(F("Check motor 4 (front lefft, clockwise direction)."));
    Serial.println(F("You can exit by sending a q (quit)."));
    delay(2500);
    check_motor_vibrations();
  }

  if (data == '5') {
    Serial.println(F("Check motor all motors."));
    Serial.println(F("You can exit by sending a q (quit)."));
    delay(2500);
    check_motor_vibrations();
  }
}

void gyro_signalen(void) {
  //Read the ICM data.
  HWire.beginTransmission(gyro_address);                       //Start communication with the gyro.
  HWire.write(0x3B);                                           //Start reading @ register 43h and auto increment with every read. dữ liệu đo
  HWire.endTransmission();                                     //End the transmission.
  HWire.requestFrom(gyro_address, 14);                         //Request 14 bytes from the ICM.

  acc_axis[1] = HWire.read() << 8 | HWire.read();              //Add the low and high byte to the acc_x variable.
  acc_axis[2] = HWire.read() << 8 | HWire.read();              //Add the low and high byte to the acc_y variable.
  acc_axis[3] = HWire.read() << 8 | HWire.read();              //Add the low and high byte to the acc_z variable.
  temperature = HWire.read() << 8 | HWire.read();              //Add the low and high byte to the temperature variable.
  gyro_axis[1] = HWire.read() << 8 | HWire.read();             //Read high and low part of the angular data.
  gyro_axis[2] = HWire.read() << 8 | HWire.read();             //Read high and low part of the angular data.
  gyro_axis[3] = HWire.read() << 8 | HWire.read();             //Read high and low part of the angular data.
  gyro_axis[2] *= -1;                                          //Invert gyro so that nose up gives positive value.
  gyro_axis[3] *= -1;                                          //Invert gyro so that nose right gives positive value.

  acc_axis[1] -= manual_acc_pitch_cal_value;                   //Subtact the manual accelerometer pitch calibration value.
  acc_axis[2] -= manual_acc_roll_cal_value;                    //Subtact the manual accelerometer roll calibration value.
  gyro_axis[1] -= manual_gyro_roll_cal_value;                  //Subtact the manual gyro roll calibration value.
  gyro_axis[2] -= manual_gyro_pitch_cal_value;                 //Subtact the manual gyro pitch calibration value.
  gyro_axis[3] -= manual_gyro_yaw_cal_value;                   //Subtact the manual gyro yaw calibration value.
}

void red_led(int8_t level) {
  if (flip32)digitalWrite(PB4, !level);
  else digitalWrite(PB4, level);
}
void green_led(int8_t level) {
  if (flip32)digitalWrite(PB3, !level);
  else digitalWrite(PB3, level);
}
