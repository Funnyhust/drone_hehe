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

#include <Wire.h>                          //Include the Wire.h library so we can communicate with the gyro.
TwoWire HWire(2, I2C_FAST_MODE);          //tạo đối tượng I2C số 2 (PB10/PB11) chạy ở 400 kHz để đọc ICM-20602

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//PID gain and limit settings
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
float pid_p_gain_roll = 1.0;               //Gain setting for the pitch and roll P-controller (default = 1.3).
float pid_i_gain_roll = 0.012;              //Gain setting for the pitch and roll I-controller (default = 0.04).
float pid_d_gain_roll = 4.0;              //Gain setting for the pitch and roll D-controller (default = 18.0).
int pid_max_roll = 400;                    //Maximum output of the PID-controller (+/-).
//Vì xung ESC nằm trong 1000–2000µs (dải 1000), nên ±400 cho PID là khoảng 40% dải — đủ để điều chỉnh mạnh mà không che lấp hoàn toàn lệnh ga của người dùng

float pid_p_gain_pitch = pid_p_gain_roll;  //Gain setting for the pitch P-controller.
float pid_i_gain_pitch = pid_i_gain_roll;  //Gain setting for the pitch I-controller.
float pid_d_gain_pitch = pid_d_gain_roll;  //Gain setting for the pitch D-controller.
int pid_max_pitch = pid_max_roll;          //Maximum output of the PID-controller (+/-). 

float pid_p_gain_yaw = 3.0;                //Gain setting for the pitch P-controller (default = 4.0).
float pid_i_gain_yaw = 0.01;               //Gain setting for the pitch I-controller (default = 0.02).
float pid_d_gain_yaw = 0.0;                //Gain setting for the pitch D-controller (default = 0.0).
int pid_max_yaw = 400;                     //Maximum output of the PID-controller (+/-). //Vì yaw thường ổn định, ít vọt qua mức, không cần khâu D triệt dao động. Thêm Kd cho yaw có khi còn gây rung do nhiễu.

boolean auto_level = true;                 //Auto level on (true) or off (false). bật chế độ auto-level (drone tự về thăng bằng khi thả cần)

// Sai số gốc của con chip cảm biến, offset cua 5 gia tri, tru di nhung gia tri nay de ok
//Manual accelerometer calibration values for IMU angles:
int16_t manual_acc_pitch_cal_value = 66; //Accel trục Y (= pitch drone) lệch a khi phẳng
int16_t manual_acc_roll_cal_value = -37; //Accel trục X (= roll drone) lệch b khi phẳng

//Manual gyro calibration values.
//Set the use_manual_calibration variable to true to use the manual calibration variables.

uint8_t use_manual_calibration = true;    // Set to false or true;
int16_t manual_gyro_pitch_cal_value = -8; //Gyro pitch lệch c khi đứng yên
int16_t manual_gyro_roll_cal_value = 20; //Gyro roll lệch d khi đứng yên
int16_t manual_gyro_yaw_cal_value = 72; //Gyro yaw lệch e khi đứng yên

uint8_t gyro_address = 0x68;               //The I2C address of the MPU-6050 is 0x68 in hexadecimal form.

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//Declaring global variables
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//int16_t = signed 16 bit integer
//uint16_t = unsigned 16 bit integer

uint8_t last_channel_1, last_channel_2, last_channel_3, last_channel_4; //lưu độ rộng xung của từng kênh (1000–2000µs) — giá trị thật từ tay điều khiển. lưu trạng thái xung trước đó (HIGH/LOW) để biết xung vừa bắt đầu hay vừa kết thúc.
uint8_t highByte, lowByte, flip32, start;
uint8_t error, error_counter, error_led; //phục vụ phát hiện và báo lỗi qua LED đỏ

int16_t esc_1, esc_2, esc_3, esc_4; // Độ rộng xung ra 4 ESC
int16_t throttle, cal_int; // Mức ga sau xử lý
int16_t temperature, count_var; // Đếm vòng calibrate
int16_t acc_x, acc_y, acc_z;  //// Gia tốc thô 3 trục
int16_t gyro_pitch, gyro_roll, gyro_yaw; // Gyro thô 3 trục


int32_t channel_1_start, channel_1;
int32_t channel_2_start, channel_2;
int32_t channel_3_start, channel_3;
int32_t channel_4_start, channel_4;
int32_t channel_5_start, channel_5;
int32_t channel_6_start, channel_6; //lưu thời điểm xung bắt đầu
int32_t acc_total_vector; // Vector gia tốc tổng = √(X²+Y²+Z²)
int32_t gyro_roll_cal, gyro_pitch_cal, gyro_yaw_cal; // Offset gyro

uint32_t loop_timer, error_timer; //cặp đôi với micros() để ép vòng lặp đúng 4000µs = 250Hz

//Đây là bộ biến trạng thái PID cho mỗi trục
float roll_level_adjust, pitch_level_adjust;  //Lượng bù góc khi bật auto-level (= angle_roll × 15)

float pid_error_temp; //chênh lệch giữa tốc độ xoay thực tế và mong muốn (đơn vị °/giây)

float pid_i_mem_roll, pid_roll_setpoint, gyro_roll_input, pid_output_roll, pid_last_roll_d_error;//Bộ nhớ tích lũy I-term — tồn tại qua nhiều vòng lặp, Tốc độ xoay mong muốn (°/s) — tính từ cần + auto-level, Tốc độ xoay thực (°/s), đầu vào PID — đo từ gyro
//Đầu ra PID cuối cùng, đưa vào trộn ESC, Sai số vòng trước — dùng tính đạo hàm D

float pid_i_mem_pitch, pid_pitch_setpoint, gyro_pitch_input, pid_output_pitch, pid_last_pitch_d_error;//Bộ nhớ tích lũy I-term cho trục pitch,cộng dồn Ki × sai_số, Tốc độ xoay pitch mong muốn (°/giây). Tính từ cần pitch + bù auto-level, Đo từ gyro
// Tốc độ xoay pitch thực tế (°/giây), Đầu ra cuối cùng của PID pitch (P + I + D), giới hạn ±400. Đem cộng/trừ vào throttle khi trộn ESC, Sai số pitch của vòng lặp trước đó. Dùng để tính đạo hàm D ở vòng hiện tại:

float pid_i_mem_yaw, pid_yaw_setpoint, gyro_yaw_input, pid_output_yaw, pid_last_yaw_d_error; //Bộ nhớ tích lũy I-term cho yaw. Cộng dồn theo thời gian. Tốc độ xoay yaw mong muốn (°/giây). CHỈ tính từ cần yaw, KHÔNG có bù auto-level. 
//Tốc độ xoay yaw thực tế (°/giây). Đo từ gyro, đã lọc 70/30., Đầu ra cuối cùng của PID yaw. Đem trộn ESC. Sai số yaw vòng trước. Cho khâu D. D=0 o yaw nen ko anh huong

float angle_roll_acc, angle_pitch_acc, angle_pitch, angle_roll;//Góc roll tính CHỈ TỪ ACCEL (đơn vị độ), qua công thức asin(acc_x / acc_total) × 57.296, Góc pitch tính CHỈ TỪ ACCEL, tương tự.
//Góc pitch CUỐI CÙNG sau bộ lọc bù (99,96% gyro + 0,04% accel). Đây là góc drone thực sự dùng để tính auto-level. Góc roll CUỐI CÙNG sau bộ lọc bù.

float battery_voltage;//Điện áp pin hiện tại

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//Setup routine 
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//Cấu hình chân và LED
void setup() {
  pinMode(4, INPUT_ANALOG);                                    //Thiết lập chân PA4 thành ngõ vào chế độ Analog để đo điện áp pin.
  //Port PB3 and PB4 are used as JTDO and JNTRST by default.
  //The following function connects PB3 and PB4 to the
  //alternate output function.
  afio_cfg_debug_ports(AFIO_DEBUG_SW_ONLY);                    //Connects PB3 and PB4 to output function.Nó tắt chức năng JTAG của các chân PB3/PB4 để điều khiển led, ko thì led ko sáng

  //On the Flip32 the LEDs are connected differently. A check is needed for controlling the LEDs.
  //Tự nhận diện loại board Có 2 loại board phổ biến: Bluepill thường và Flip32
  pinMode(PB3, INPUT);                                         //tạm đặt 2 chân thành đầu vào để đọc thử mức điện trên đó
  pinMode(PB4, INPUT);                                         //Set PB4 as input.
  if (digitalRead(PB3) || digitalRead(PB3))flip32 = 1;         //đọc mức 0 hay 1 của chân PB3 nếu đọc được mức cao → "à, đây là Flip32" → gán cờ flip32 = 1.
  else flip32 = 0;

//Sau khi đoán xong loại board, đặt 2 chân thành OUTPUT (đầu ra — để xuất tín hiệu điều khiển LED chứ không còn đọc nữa)
  pinMode(PB3, OUTPUT);                                         //Set PB3 as output.
  pinMode(PB4, OUTPUT);                                         //Set PB4 as output.

  green_led(LOW);                                               // // Đèn xanh TẮT
  red_led(HIGH);                                                //Đèn đỏ BẬT
  //Đây là trạng thái an toàn ban đầu

  //Serial.begin(57600);                                        //Set the serial output to 57600 kbps. (for debugging only)
  //delay(250);                                                 //Give the serial port some time to start to prevent data loss.

  timer_setup();                                                // cấu hình
  //Timer2 (PA0–PA3): chế độ Input Capture, đo xung kênh 1–4 từ tay điều khiển.
  //Timer3 (PA6, PA7): tương tự, đo xung kênh 5, 6.
  //Timer4 (PB6–PB9): chế độ PWM out, tạo xung ra 4 ESC ở tần số 250Hz.
  delay(50);                                                    //Give the timers some time to start.
  
 //Kiểm tra cảm biến
  HWire.begin();                                                //Start the I2C as master
  HWire.beginTransmission(gyro_address);                        //Start communication with the ICM.
  error = HWire.endTransmission();                              //End the transmission and register the exit status.
  while (error != 0) {                                          //// Cảm biến không trả lời?
    error = 2;                                                  //→ Mã lỗi 2
    error_signal();                                             //Nháy LED đỏ báo lỗi 2, nháy 2 lần
    delay(4);
  }

  gyro_setup();                                                 //gửi 4 lệnh I2C đã giảng ở setup: đánh thức cảm biến (0x6B = 0x00), thang gyro ±500°/s (0x1B = 0x08), thang accel ±8g (0x1C = 0x10), bộ lọc 43Hz (0x1A = 0x03).
  
//Đoạn này chỉ chạy khi use_manual_calibration = false (tức không dùng giá trị offset cố định
//Nếu use_manual_calibration = true (như code của bạn), thì bỏ qua bước trễ vì offset đã đo sẵn từ lệnh h trước đó
  if (!use_manual_calibration) { 
    //Create a 5 second delay before calibration. Trễ 5 giây để cảm biến ổn định nhiệt
    for (count_var = 0; count_var < 1250; count_var++) {        //1250 loops of 4 microseconds = 5 seconds
      if (count_var % 125 == 0) {                               //Every 125 loops (500ms).
        digitalWrite(PB4, !digitalRead(PB4));                   //// Đảo LED đỏ
      }
      delay(4);                                                 //Delay 4 microseconds
    }
    count_var = 0;                                              //Set start back to 0.
  }

  calibrate_gyro();                                             //Calibrate the gyro offset.
  //Nếu use_manual_calibration = true (code của bạn): nó nạp 5 giá trị offset bạn đặt sẵn vào các biến gyro_*_cal.
  //Nếu false: đọc 2000 mẫu, tính trung bình → offset mới.
 
  //Wait until the receiver is active.  Chờ tay điều khiển chốt an toàn 2 Nếu kênh nào < 990 nghĩa là:Tay điều khiển chưa bật Bộ thu chưa bắt được sóng Hoặc kết nối lỏng
  while (channel_1 < 990 || channel_2 < 990 || channel_3 < 990 || channel_4 < 990)  {
    error = 3;                                                  //Set the error status to 3.
    error_signal();                                             //Show the error via the red LED. // Nháy lỗi 3
    delay(4);
  }
  error = 0;                                                    //Reset the error status to 0.
  //đợi cả 4 kênh chính (roll, pitch, throttle, yaw) đều xuất tín hiệu hợp lệ
  
  //Wait until the throtle is set to the lower position. Chờ ga thấp tránh motor quay vọt khi cấp nguồn
  while (channel_3 < 990 || channel_3 > 1050)  {
    error = 4;                                                  //Set the error status to 4.
    error_signal();                                             //Show the error via the red LED.
    delay(4);
  }
  error = 0;                                                    //Reset the error status to 0.

  //When everything is done, turn off the led. Hoàn tất khởi động 
  red_led(LOW);                                                 //Set output PB4 low. Tắt đèn đỏ

  //Load the battery voltage to the battery_voltage variable.
  //The STM32 uses a 12 bit analog to digital converter.
  //analogRead => 0 = 0V ..... 4095 = 3.3V
  //The voltage divider (1k & 10k) is 1:11.
  //analogRead => 0 = 0V ..... 4095 = 36.3V
  //36.3 / 4095 = 112.81.
  battery_voltage = (float)analogRead(4) / 112.81;  //đọc pin lần đầu

  loop_timer = micros();                                        //Set the timer for the first loop. // Đánh dấu mốc thời gian vòng lặp đầu

  green_led(HIGH);                                              //Turn on the green led.
}
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//Main program loop
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void loop() {

  error_signal();                                                                  //Show the errors via the red LED. // Nháy LED nếu có lỗi đang xảy ra error_signal() chạy đầu mỗi vòng — nếu biến error khác 0 thì nháy LED đỏ theo mã lỗi
  gyro_signalen();                                                                 //Read the gyro and accelerometer data. // Đọc 14 byte từ cảm biến qua I2C Ghép thành 6 giá trị 16-bit: acc_x, acc_y, acc_z, temperature, gyro_roll, gyro_pitch, gyro_yaw
 
  //65.5 = 1 deg/sec Đây là bộ lọc bù thứ HAI Mục đích: làm mượt tín hiệu gyro trước khi đưa vào PID. giá trị mới = 70% (giá trị cũ) + 30% (mẫu mới) gyro_roll / 65.5 quy đổi giá trị thô sang °/giây
  //Làm mượt input cho PID, chống nhiễu khâu D Lọc bù tính góc (dòng 213-214)99,96/0,04 Kết hợp gyro+accel ra góc cuối
  gyro_roll_input = (gyro_roll_input * 0.7) + (((float)gyro_roll / 65.5) * 0.3);   //Gyro pid input is deg/sec. 
  gyro_pitch_input = (gyro_pitch_input * 0.7) + (((float)gyro_pitch / 65.5) * 0.3);//Gyro pid input is deg/sec.
  gyro_yaw_input = (gyro_yaw_input * 0.7) + (((float)gyro_yaw / 65.5) * 0.3);      //Gyro pid input is deg/sec.


  ////////////////////////////////////////////////////////////////////////////////////////////////////
  //This is the added IMU code from the videos:
  //https://youtu.be/4BoIE8YQwM8
  //https://youtu.be/j-kE0AMEWy4
  ////////////////////////////////////////////////////////////////////////////////////////////////////

  //Gyro angle calculations
  //0.0000611 = 1 / (250Hz / 65.5)
  angle_pitch += (float)gyro_pitch * 0.0000611;                                    //Calculate the traveled pitch angle and add this to the angle_pitch variable. cộng dồn vào angle_pitch qua các vòng → ra góc tích lũy.
  angle_roll += (float)gyro_roll * 0.0000611;                                      //Calculate the traveled roll angle and add this to the angle_roll variable.

  //0.000001066 = 0.0000611 * (3.142(PI) / 180degr) The Arduino sin function is in radians and not degrees. 
  angle_pitch -= angle_roll * sin((float)gyro_yaw * 0.000001066);                  //If the IMU has yawed transfer the roll angle to the pitch angel.
  angle_roll += angle_pitch * sin((float)gyro_yaw * 0.000001066);                  //If the IMU has yawed transfer the pitch angle to the roll angel.

  //Accelerometer angle calculations
  acc_total_vector = sqrt((acc_x * acc_x) + (acc_y * acc_y) + (acc_z * acc_z));    //Calculate the total accelerometer vector. Lúc drone nằm yên, giá trị này ≈ 4096 (= 1g ở thang ±8g).
  //vector gia tốc tổng theo Pythagoras 3D

  if (abs(acc_y) < acc_total_vector) {                                             //Prevent the asin function to produce a NaN. đổi radian sang độ (57.296 = 180/π). chuẩn hóa về [-1, 1] trước rồi tính arcsin → ra góc (radian).
    angle_pitch_acc = asin((float)acc_y / acc_total_vector) * 57.296;              //Calculate the pitch angle.
  }
  if (abs(acc_x) < acc_total_vector) {                                             //Prevent the asin function to produce a NaN.
    angle_roll_acc = asin((float)acc_x / acc_total_vector) * 57.296;               //Calculate the roll angle.
  }

  //Bộ lọc bù
  angle_pitch = angle_pitch * 0.9996 + angle_pitch_acc * 0.0004;                   //Correct the drift of the gyro pitch angle with the accelerometer pitch angle.
  angle_roll = angle_roll * 0.9996 + angle_roll_acc * 0.0004;                      //Correct the drift of the gyro roll angle with the accelerometer roll angle.
  //Sau 2 dòng này, angle_pitch và angle_roll là góc cuối cùng — chính xác và mượt.


// ==========================================================
// PID VÒNG NGOÀI (OUTER LOOP)
// ==========================================================
// Tính toán lượng bù góc dựa trên sai số (Góc đặt mặc định là 0 độ)
// Lượng bù = Góc thực tế * Kp (Với Kp = 15)
  pitch_level_adjust = angle_pitch * 15;                                           //Calculate the pitch angle correction. angle_roll × 15 quy đổi góc nghiêng (độ) thành tốc độ xoay tương ứng (°/giây) để bù về thăng bằng:
  //Drone đang nghiêng 1° → muốn nó về 0° nhanh → setpoint sẽ thêm 15°/giây để xoay ngược.
  roll_level_adjust = angle_roll * 15;                                             //Calculate the roll angle correction.

  if (!auto_level) {                                                               //If the quadcopter is not in auto-level mode nếu chế độ acro (không tự cân bằng) thì set bù bằng 0. Lúc đó PID setpoint chỉ phụ thuộc cần điều khiển,
    //drone giữ nguyên góc nghiêng đến khi bạn lái về
    pitch_level_adjust = 0;                                                        //Set the pitch angle correction to zero.
    roll_level_adjust = 0;                                                         //Set the roll angle correcion to zero.
  }


  //For starting the motors: throttle low and yaw left (step 1). Máy trạng thái arming + reset PID
  ////Step 1: ga thấp + yaw trái
  if (channel_3 < 1050 && channel_4 < 1050)start = 1;

  //Step 2: ga thấp + yaw giữa → MỞ KHÓA
  //When yaw stick is back in the center position start the motors (step 2).
  if (start == 1 && channel_3 < 1050 && channel_4 > 1450) {
    start = 2;

    green_led(LOW);                                                                //Turn off the green led.
    angle_pitch = angle_pitch_acc;                                                 //Set the gyro pitch angle equal to the accelerometer pitch angle when the quadcopter is started. // Đồng bộ góc với accel
    angle_roll = angle_roll_acc;                                                   //Set the gyro roll angle equal to the accelerometer roll angle when the quadcopter is started.

    //Reset the PID controllers for a bumpless start.   //Reset PID cho khởi động êm
    pid_i_mem_roll = 0;
    pid_last_roll_d_error = 0;
    pid_i_mem_pitch = 0;
    pid_last_pitch_d_error = 0;
    pid_i_mem_yaw = 0;
    pid_last_yaw_d_error = 0;
  }
  //Đặt tất cả tích lũy I và đạo hàm D về 0 vì  Nếu không reset, ngay khi arm motor sẽ vọt theo lượng tích lũy i và d này
  
  //Stopping the motors: throttle low and yaw right.//Step 3: ga thấp + yaw phải → KHÓA LẠI
  if (start == 2 && channel_3 < 1050 && channel_4 > 1950) {
    start = 0;
    green_led(HIGH);                                                               //Turn on the green led.
  }

  //The PID set point in degrees per second is determined by the roll receiver input.
  //In the case of deviding by 3 the max roll rate is aprox 164 degrees per second ( (500-8)/3 = 164d/s ).
  pid_roll_setpoint = 0; //khởi đầu giả định cần ở giữa
  //We need a little dead band of 16us for better results.
  if (channel_1 > 1508)pid_roll_setpoint = channel_1 - 1508;  //cần lệch phải, setpoint = channel_1 - 1508 (số dương).
  else if (channel_1 < 1492)pid_roll_setpoint = channel_1 - 1492; //cần lệch trái, setpoint = channel_1 - 1492 (số âm).

  pid_roll_setpoint -= roll_level_adjust;                                          //Subtract the angle correction from the standardized receiver roll input value. Đây là chỗ góc nghiêng tham gia vào setpoint
  //vd KHÔNG gạt cần (channel_1 = 1500) → setpoint thô = 0. Nhưng drone đang nghiêng angle_roll = 5° → roll_level_adjust = 5 × 15 = 75. Setpoint = 0 - 75 = -75. → PID sẽ ra lệnh xoay -75 đơn vị
  //nếu angle_roll dương (nghiêng phải) thì cần xoay ngược (sang trái) → setpoint âm.
 
  pid_roll_setpoint /= 3.0;                                                        //Divide the setpoint for the PID roll controller by 3 to get angles in degrees. Đến đây setpoint đang có đơn vị "µs PWM" hỗn hợp với "góc × 15". Chia 3 quy về °/giây


  //The PID set point in degrees per second is determined by the pitch receiver input.
  //In the case of deviding by 3 the max pitch rate is aprox 164 degrees per second ( (500-8)/3 = 164d/s ). 
  pid_pitch_setpoint = 0;
  //We need a little dead band of 16us for better results.
  if (channel_2 > 1508)pid_pitch_setpoint = channel_2 - 1508;
  else if (channel_2 < 1492)pid_pitch_setpoint = channel_2 - 1492;

  pid_pitch_setpoint -= pitch_level_adjust;                                        //Subtract the angle correction from the standardized receiver pitch input value.
  pid_pitch_setpoint /= 3.0;                                                       //Divide the setpoint for the PID pitch controller by 3 to get angles in degrees.

  //The PID set point in degrees per second is determined by the yaw receiver input.
  //In the case of deviding by 3 the max yaw rate is aprox 164 degrees per second ( (500-8)/3 = 164d/s ).Tính setpoint cho yaw — KHÁC roll/pitch KHÔNG có bù góc auto-level
  pid_yaw_setpoint = 0;
  //We need a little dead band of 16us for better results. chỉ tính yaw setpoint khi cần ga > 1050 (tức ga không ở đáy). Mục đích: chống yaw khi tắt motor
  if (channel_3 > 1050) { //Do not yaw when turning off the motors. Điểm khác: kiểm tra ga
    if (channel_4 > 1508)pid_yaw_setpoint = (channel_4 - 1508) / 3.0;
    else if (channel_4 < 1492)pid_yaw_setpoint = (channel_4 - 1492) / 3.0;
  }

  calculate_pid();                                                                 //PID inputs are known. So we can calculate the pid output.

  //The battery voltage is needed for compensation.
  //A complementary filter is used to reduce noise.
  //1410.1 = 112.81 / 0.08.
  battery_voltage = battery_voltage * 0.92 + ((float)analogRead(4) / 1410.1); //gộp 2 phép tính (chia để ra volt + nhân 0.08 cho lọc bù) thành một phép chia

  //Turn on the led if battery voltage is to low. In this case under 10.0V
  if (battery_voltage < 10.0 && error == 0)error = 1; //Pin LiPo 3S đầy ~12.6V, an toàn xuống ~10.5V, dưới 10.0V là báo động đỏ (có thể hỏng pin nếu xả tiếp). Code bật mã lỗi 1 

  throttle = channel_3;                                                            //We need the throttle signal as a base signal.

  if (start == 2) {                                                                //The motors are started.
    if (throttle > 1800) throttle = 1800;                                          //We need some room to keep full control at full throttle. 
    //Vì công thức: esc_n = throttle ± pid_output. Nếu ga = 2000 và PID ra max +400 cho roll, esc_n = 2400 → vượt 2000 → bị cắt về 2000. Lúc đó mất khả năng điều khiển (motor đã max rồi, không thể tăng thêm để bù lệch).
    esc_1 = throttle - pid_output_pitch + pid_output_roll - pid_output_yaw;        //Calculate the pulse for esc 1 (front-right - CCW).
    esc_2 = throttle + pid_output_pitch + pid_output_roll + pid_output_yaw;        //Calculate the pulse for esc 2 (rear-right - CW).
    esc_3 = throttle + pid_output_pitch - pid_output_roll - pid_output_yaw;        //Calculate the pulse for esc 3 (rear-left - CCW).
    esc_4 = throttle - pid_output_pitch - pid_output_roll + pid_output_yaw;        //Calculate the pulse for esc 4 (front-left - CW).
    //drone "ngả trước" = motor TRƯỚC giảm, motor SAU tăng → drone chúi mũi xuống → bay tiến.
    //ESC 1, 4 (trước): -pid_pitch (giảm khi pid_pitch dương)
    //ESC 2, 3 (sau): +pid_pitch (tăng khi pid_pitch dương)
    //drone "nghiêng trái" = motor PHẢI tăng, motor TRÁI giảm → drone nghiêng trái → bay ngang. ESC 1, 2 (phải): +pid_roll ESC 3, 4 (trái): -pid_roll
    //quadcopter có 2 cặp motor quay ngược chiều (CW & CCW) để tự triệt mô-men. Khi tăng cặp CCW + giảm cặp CW → tổng mô-men dương → drone xoay theo CW (Newton 3). Đảo lại → xoay CCW.ESC 1, 3 (CCW): -pid_yaw ESC 2, 4 (CW): +pid_yaw

    if (esc_1 < 1100) esc_1 = 1100;                                                //Keep the motors running.
    if (esc_2 < 1100) esc_2 = 1100;                                                //Keep the motors running.
    if (esc_3 < 1100) esc_3 = 1100;                                                //Keep the motors running.
    if (esc_4 < 1100) esc_4 = 1100;                                                //Keep the motors running.

    if (esc_1 > 2000)esc_1 = 2000;                                                 //Limit the esc-1 pulse to 2000us.
    if (esc_2 > 2000)esc_2 = 2000;                                                 //Limit the esc-2 pulse to 2000us.
    if (esc_3 > 2000)esc_3 = 2000;                                                 //Limit the esc-3 pulse to 2000us.
    if (esc_4 > 2000)esc_4 = 2000;                                                 //Limit the esc-4 pulse to 2000us.
  }

  else {
    esc_1 = 1000;                                                                  //If start is not 2 keep a 1000us pulse for ess-1.
    esc_2 = 1000;                                                                  //If start is not 2 keep a 1000us pulse for ess-2.
    esc_3 = 1000;                                                                  //If start is not 2 keep a 1000us pulse for ess-3.
    esc_4 = 1000;                                                                  //If start is not 2 keep a 1000us pulse for ess-4.
  }

  //Xuất xung ra ESC qua Timer4 
  TIMER4_BASE->CCR1 = esc_1;                                                       //Set the throttle receiver input pulse to the ESC 1 output pulse.
  TIMER4_BASE->CCR2 = esc_2;                                                       //Set the throttle receiver input pulse to the ESC 2 output pulse.
  TIMER4_BASE->CCR3 = esc_3;                                                       //Set the throttle receiver input pulse to the ESC 3 output pulse.
  TIMER4_BASE->CCR4 = esc_4;                                                       //Set the throttle receiver input pulse to the ESC 4 output pulse.
  TIMER4_BASE->CNT = 5000;                                                         //This will reset timer 4 and the ESC pulses are directly created.

  ////////////////////////////////////////////////////////////////////////////////////////////////////
  //Creating the pulses for the ESC's is explained in this video:
  //https://youtu.be/Nju9rvZOjVQ
  ////////////////////////////////////////////////////////////////////////////////////////////////////

  //! ! ! ! ! ! ! ! ! ! ! ! ! ! ! ! ! ! ! ! ! ! ! ! ! ! ! ! ! ! ! ! ! ! ! ! ! ! ! ! ! ! ! ! ! ! ! ! ! !
  //Because of the angle calculation the loop time is getting very important. If the loop time is
  //longer or shorter than 4000us the angle calculation is off. If you modify the code make sure
  //that the loop time is still 4000us and no longer! More information can be found on
  //the Q&A page:
  //! ! ! ! ! ! ! ! ! ! ! ! ! ! ! ! ! ! ! ! ! ! ! ! ! ! ! ! ! ! ! ! ! ! ! ! ! ! ! ! ! ! ! ! ! ! ! ! ! !

  if (micros() - loop_timer > 4050)error = 5;                                      //Turn on the LED if the loop time exceeds 4050us.
  //Nếu vòng lặp này đã chạy hơn 4050µs (chậm hơn dự kiến) → đặt mã lỗi 5. Bình thường mỗi vòng chỉ tốn ~3000µs, có dư cho 1000µs. Quá 4050 nghĩa là có gì đó kéo chậm bất thường
  while (micros() - loop_timer < 4000);                                            //We wait until 4000us are passed.
  loop_timer = micros();                                                           //Set the timer for the next loop.
}
