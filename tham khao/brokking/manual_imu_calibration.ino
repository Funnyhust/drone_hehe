void manual_imu_calibration(void) {
  data = 0; //Reset các biến tích lũy
  acc_axis_cal[1] = 0;
  acc_axis_cal[2] = 0;
  gyro_axis_cal[1] = 0;
  gyro_axis_cal[2] = 0;
  gyro_axis_cal[3] = 0;
  Serial.print("Calibrating the gyro");
  //Let's take multiple gyro data samples to stabilize the gyro. Đo offset của cả accel lẫn gyro một cách chính xác, rồi in ra 5 con số để bạn copy vào code bay
  for (cal_int = 0; cal_int < 2000 ; cal_int ++) {                                  //Take 2000 readings for calibration. Mục đích: để cảm biến "ổn định" (warm-up / stabilize). Khi vừa bật, cảm biến cần một lúc để giá trị ổn định
    if (cal_int % 125 == 0) {                                                       //Vòng này "chạy không" 2000 lần (8 giây) để cảm biến vào trạng thái ổn định, rồi mới đo thật ở vòng sau
      digitalWrite(PB3, !digitalRead(PB3));                                         //Change the led status to indicate calibration.
      Serial.print(".");
    }
    gyro_signalen();                                                                //Read the gyro output.
    delay(4);                                                                       //Small delay to simulate a 250Hz loop during calibration.
  }

  //Let's take multiple gyro data samples so we can determine the average gyro offset (calibration).
  for (cal_int = 0; cal_int < 4000 ; cal_int ++) {                                  //Take 2000 readings for calibration. Vòng này đo thật, lấy 4000 mẫu (nhiều gấp đôi lệnh c → chính xác hơn). Cộng dồn vào các ô tích lũy. Phần nháy đèn + dấu chấm quen rồi.
    if (cal_int % 125 == 0) {
      digitalWrite(PB3, !digitalRead(PB3));                                         //Change the led status to indicate calibration.
      Serial.print(".");
    }
    //hàm gyro_signalen() lúc đọc đã tự trừ offset cũ rồi (sẽ thấy khi giảng hàm đó). Nên ở đây phải cộng lại offset cũ để ra giá trị thô gốc trước khi trừ. Mục đích: tính ra offset tuyệt đối (so với gốc), không bị ảnh hưởng bởi offset đã đặt từ trước
    gyro_signalen();                                                                //Read the gyro output.
    acc_axis_cal[1] += acc_axis[1] + manual_acc_pitch_cal_value;                    //Ad the Y accelerometer value to the calibration value.
    acc_axis_cal[2] += acc_axis[2] + manual_acc_roll_cal_value;                     //Ad the X accelerometer value to the calibration value.
    gyro_axis_cal[1] += gyro_axis[1] + manual_gyro_roll_cal_value;                  //Ad roll value to gyro_roll_cal.
    gyro_axis_cal[2] += gyro_axis[2] + manual_gyro_pitch_cal_value;                 //Ad pitch value to gyro_pitch_cal.
    gyro_axis_cal[3] += gyro_axis[3] + manual_gyro_yaw_cal_value;                   //Ad yaw value to gyro_yaw_cal. 
    delay(4);                                                                       //Small delay to simulate a 250Hz loop during calibration.
  }
  Serial.println(".");
  green_led(LOW);                                               //Set output PB3 low.
  //Now that we have 2000 measures, we need to devide by 2000 to get the average gyro offset.
  acc_axis_cal[1] /= 4000;                                                          //Divide the accelerometer Y value by 4000.
  acc_axis_cal[2] /= 4000;                                                          //Divide the accelerometer X value by 4000.
  gyro_axis_cal[1] /= 4000;                                                         //Divide the roll total by 4000.
  gyro_axis_cal[2] /= 4000;                                                         //Divide the pitch total by 4000.
  gyro_axis_cal[3] /= 4000;                                                         //Divide the yaw total by 4000.
  //Chia tổng cho 4000 (số mẫu) → ra offset trung bình của từng trục. Tắt đèn xanh báo xong.

  //Print the calibration values on the serial monitor.
  Serial.print("manual_acc_pitch_cal_value = ");
  Serial.println(acc_axis_cal[1]);
  Serial.print("manual_acc_roll_cal_value = ");
  Serial.println(acc_axis_cal[2]);
  Serial.print("manual_gyro_pitch_cal_value = ");
  Serial.println(gyro_axis_cal[2]);
  Serial.print("manual_gyro_roll_cal_value = ");
  Serial.println(gyro_axis_cal[1]);
  Serial.print("manual_gyro_yaw_cal_value = ");
  Serial.println(gyro_axis_cal[3]);

  print_intro();                                                                        //Print the intro to the serial monitor.
}
