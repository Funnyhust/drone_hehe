void check_imu_angles(void) {
  uint8_t first_angle = 0; //cờ đánh dấu "đây có phải lần tính góc đầu tiên không"
  loop_counter = 0; // đặt lại bộ đếm dùng cho việc in
  first_angle = false;
  if (use_manual_calibration)cal_int = 2000;                                            //quyết định có calibrate gyro hay không.
  else {
    cal_int = 0;                                                                        //If manual calibration is not used.
    manual_gyro_pitch_cal_value = 0;                                                    //Set the manual pitch calibration variable to 0.
    manual_gyro_roll_cal_value = 0;                                                     //Set the manual roll calibration variable to 0.
    manual_gyro_yaw_cal_value = 0;                                                      //Set the manual yaw calibration variable to 0.
  }
  while (data != 'q') {                                                                 //Stay in this loop until the data variable data holds a q.
    loop_timer = micros() + 4000;                                                       //Set the loop_timer variable to the current micros() value + 4000. vòng lặp này phải kết thúc vào thời điểm = bây giờ + 4000µs
    if (Serial.available() > 0) {                                                       //If serial data is available.
      data = Serial.read();                                                             //Read the incomming byte.
      delay(100);                                                                       //Wait for any other bytes to come in.
      while (Serial.available() > 0)loop_counter = Serial.read();                       //Empty the Serial buffer.
    }

    if (cal_int == 0) {                                                                 //If manual calibration is not used.
      gyro_axis_cal[1] = 0;                                                             //Reset calibration variables for next calibration.
      gyro_axis_cal[2] = 0;                                                             //Reset calibration variables for next calibration.
      gyro_axis_cal[3] = 0;                                                             //Reset calibration variables for next calibration.
      Serial.print("Calibrating the gyro");
      //Let's take multiple gyro data samples so we can determine the average gyro offset (calibration).
      for (cal_int = 0; cal_int < 2000 ; cal_int ++) {                                  //Take 2000 readings for calibration.
        if (cal_int % 125 == 0) {
          digitalWrite(PB3, !digitalRead(PB3));                                         //Change the led status to indicate calibration.
          Serial.print(".");
        }
        gyro_signalen();                                                                //Read the gyro output.
        gyro_axis_cal[1] += gyro_axis[1];                                               //Ad roll value to gyro_roll_cal.
        gyro_axis_cal[2] += gyro_axis[2];                                               //Ad pitch value to gyro_pitch_cal.
        gyro_axis_cal[3] += gyro_axis[3];                                               //Ad yaw value to gyro_yaw_cal.
        delay(4);                                                                       //Small delay to simulate a 250Hz loop during calibration.
      }
      Serial.println(".");
      green_led(LOW);                                               //Set output PB3 low.
      //Now that we have 2000 measures, we need to devide by 2000 to get the average gyro offset.
      gyro_axis_cal[1] /= 2000;                                                         //Divide the roll total by 2000.
      gyro_axis_cal[2] /= 2000;                                                         //Divide the pitch total by 2000.
      gyro_axis_cal[3] /= 2000;                                                         //Divide the yaw total by 2000.

      manual_gyro_pitch_cal_value = gyro_axis_cal[2];                                   //Set the manual pitch calibration variable to the detected value.
      manual_gyro_roll_cal_value = gyro_axis_cal[1];                                    //Set the manual roll calibration variable to the detected value.
      manual_gyro_yaw_cal_value = gyro_axis_cal[3];                                     //Set the manual yaw calibration variable to the detected value. Calibrate gyro
    }
    
    gyro_signalen();                                                                    //gyro_signalen();
    //Gyro angle calculations
    //0.0000611 = 1 / (250Hz / 65.5)  Trong đó 250 = tần số vòng lặp (Hz), 65.5 = độ nhạy gyro thang ±500°/s. đổi "số thô trong 1 vòng lặp" thành "góc đã quay (độ) trong vòng đó".
    angle_pitch += gyro_axis[2] * 0.0000611;                                            //Calculate the traveled pitch angle and add this to the angle_pitch variable. giá trị thô gyro trục pitch.
    angle_roll += gyro_axis[1] * 0.0000611;                                             //Calculate the traveled roll angle and add this to the angle_roll variable.

    //0.000001066 = 0.0000611 * (3.142(PI) / 180 độ) The Arduino sin function is in radians Hai dòng này chuyển một phần góc giữa pitch và roll mỗi khi có yaw, để góc luôn đúng với hướng thực. vì hàm sin của Arduino dùng radian
    angle_pitch -= angle_roll * sin(gyro_axis[3] * 0.000001066);                        //If the IMU has yawed transfer the roll angle to the pitch angel.
    angle_roll += angle_pitch * sin(gyro_axis[3] * 0.000001066);                        //If the IMU has yawed transfer the pitch angle to the roll angel. gyro_axis[3] là gyro trục yaw

    //Accelerometer angle calculations
    if (acc_axis[1] > 4096)acc_axis[1] = 4096;                                          //Limit the maximum accelerometer value. ép giá trị accel không vượt quá ±4096. Vì 4096 = giá trị ứng với 1g (thang ±8g)
    if (acc_axis[1] < -4096)acc_axis[1] = -4096;                                        //Limit the maximum accelerometer value.
    if (acc_axis[2] > 4096)acc_axis[2] = 4096;                                          //Limit the maximum accelerometer value.
    if (acc_axis[2] < -4096)acc_axis[2] = -4096;                                        //Limit the maximum accelerometer value.
    
    //Hàm arcsin ở dưới chỉ nhận giá trị trong khoảng −1 đến +1, nên phải đảm bảo acc/4096 không vượt ±1
    //57.296 = 1 / (3.142 / 180) The Arduino asin function is in radians
    angle_pitch_acc = asin((float)acc_axis[1] / 4096) * 57.296;                         //Calculate the pitch angle.
    angle_roll_acc = asin((float)acc_axis[2] / 4096) * 57.296;                          //Calculate the roll angle. góc tính theo accel


    if (!first_angle) {                                                                 //When this is the first time.
      angle_pitch = angle_pitch_acc;                                                    //Set the pitch angle to the accelerometer angle.
      angle_roll = angle_roll_acc;                                                      //Set the roll angle to the accelerometer angle.
      first_angle = true;
    }
    else {                                                                              //When this is not the first time. Mỗi vòng kéo nhẹ góc gyro về phía accel một xíu → vừa mượt vừa không trôi, lọc bù
      angle_pitch = angle_pitch * 0.9996 + angle_pitch_acc * 0.0004;                    //Correct the drift of the gyro pitch angle with the accelerometer pitch angle.
      angle_roll = angle_roll * 0.9996 + angle_roll_acc * 0.0004;                       //Correct the drift of the gyro roll angle with the accelerometer roll angle.
      //Con số 0.04% nhỏ vậy là vì dòng này chạy 250 lần mỗi giây. Mỗi giây accel "kéo" tổng cộng 250 × 0.04% = 10%
    }

    //We can't print all the data at once. This takes to long and the angular readings will be off.
    if (loop_counter == 0)Serial.print("Pitch: ");
    if (loop_counter == 1)Serial.print(angle_pitch , 1);
    if (loop_counter == 2)Serial.print(" Roll: ");
    if (loop_counter == 3)Serial.print(angle_roll , 1);
    if (loop_counter == 4)Serial.print(" Yaw: ");
    if (loop_counter == 5)Serial.print(gyro_axis[3] / 65.5 , 0);
    if (loop_counter == 6)Serial.print(" Temp: ");
    if (loop_counter == 7)Serial.println(temperature / 326.8 + 25.0 , 1);
    loop_counter ++;
    if (loop_counter == 60)loop_counter = 0;

    while (loop_timer > micros());
  }
  loop_counter = 0;                                                                     //Reset the loop counter variable to 0.
  print_intro();                                                                        //Print the intro to the serial monitor.
}
