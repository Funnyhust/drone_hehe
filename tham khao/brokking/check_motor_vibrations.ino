void check_motor_vibrations(void) {
  //Let's declare some variables so we can use them in this subroutine.
  //int16_t = signed 16 bit integer
  //uint16_t = unsigned 16 bit integer
  int32_t vibration_array[20], avarage_vibration_level, vibration_total_result; //mảng 20 ô lưu các mẫu độ rung gần nhất. mức rung trung bình. tổng kết quả rung.
  uint8_t array_counter, throttle_init_ok, vibration_counter;
  uint32_t wait_timer; // mốc thời gian chờ.
  throttle_init_ok = 0;
  while (data != 'q') {                                                                //Stay in this loop until the data variable data holds a q.
    loop_timer = micros() + 4000;                                                      //Set the loop_timer variable to the current micros() value + 4000.
    if (Serial.available() > 0) {                                                      //If serial data is available
      data = Serial.read();                                                            //Read the incomming byte
      delay(100);                                                                      //Wait for any other bytes to come in
      while (Serial.available() > 0)loop_counter = Serial.read();                      //Empty the Serial buffer
    }

    if (throttle_init_ok) {                                                            //If the throttle is detected in the lowest position.
      gyro_signalen();                                                                 //Get the raw gyro and accelerometer data.
      //Calculate the total accelerometer vector.
      
      vibration_array[0] = sqrt((acc_axis[1] * acc_axis[1]) + (acc_axis[2] * acc_axis[2]) + (acc_axis[3] * acc_axis[3])); //Đọc cảm biến, rồi tính độ lớn tổng của gia tốc trên cả 3 trục, bằng công thức vector tổng = √(X² + Y² + Z²)
      //Khi drone đứng yên, vector tổng luôn ≈ 1g (chỉ có trọng lực), ổn định. Khi motor rung, lực rung cộng vào làm vector tổng dao động lên xuống quanh giá trị 1g. Mức dao động đó chính là độ rung.
      
      for (array_counter = 16; array_counter > 0; array_counter--) {                   //Do this loop 16 times to create an array of accelrometer vectors.
        vibration_array[array_counter] = vibration_array[array_counter - 1];           //Shift every variable one position up in the array. // dịch mảng
        avarage_vibration_level += vibration_array[array_counter];                     //Add the array value to the acc_av_vector variable. // cộng dồn
      } 
      avarage_vibration_level /= 17;                                                   //Divide the acc_av_vector by 17 to get the avarage total accelerometer vector.//Lấy trung bình để có một "đường nền ổn định" so sánh với giá trị tức thời.

      if (vibration_counter < 20) {                                                    //If the vibration_counter is less than 20 do this.
        vibration_counter ++;                                                          //Increment the vibration_counter variable.
        vibration_total_result += abs(vibration_array[0] - avarage_vibration_level);   //Add the absolute difference between the avarage vector and current vector to the vibration_total_result variable. 
        //độ lệch giữa giá trị tức thời (ô [0]) và mức trung bình. abs = trị tuyệt đối (bỏ dấu âm). → Lệch càng nhiều = rung càng mạnh.

      }
      else { 
        vibration_counter = 0;                                                         //If the vibration_counter is equal or larger than 20 do this.
        Serial.println(vibration_total_result / 50);                                   //Print the total accelerometer vector divided by 50 on the serial monitor.
        vibration_total_result = 0;                                                    //Reset the vibration_total_result variable.
      }
      //Con số in ra chính là MỨC RUNG. Số nhỏ (vài chục) = rung ít, tốt. Số lớn (hàng trăm) = rung nhiều, cần cân cánh quạt. Đây là con số bạn dùng để cân cánh.

      if (data == '1') {                                                               //If the user requested 1.
        TIMER4_BASE->CCR1 = channel_3;                                                 //Set the ESC 1 output pulse equal to the throttle input.
        TIMER4_BASE->CCR2 = 1000;                                                      //Keep the ESC 2 pulse at 1000us.
        TIMER4_BASE->CCR3 = 1000;                                                      //Keep the ESC 3 pulse at 1000us.
        TIMER4_BASE->CCR4 = 1000;                                                      //Keep the ESC 4 pulse at 1000us.
      }
      if (data == '2') {                                                               //If the user requested 2.
        TIMER4_BASE->CCR1 = 1000;                                                      //Keep the ESC 1 pulse at 1000us.
        TIMER4_BASE->CCR2 = channel_3;                                                 //Set the ESC 2 output pulse equal to the throttle input.
        TIMER4_BASE->CCR3 = 1000;                                                      //Keep the ESC 3 pulse at 1000us.
        TIMER4_BASE->CCR4 = 1000;                                                      //Keep the ESC 4 pulse at 1000us.
      }
      if (data == '3') {                                                               //If the user requested 3.
        TIMER4_BASE->CCR1 = 1000;                                                      //Keep the ESC 1 pulse at 1000us.
        TIMER4_BASE->CCR2 = 1000;                                                      //Keep the ESC 2 pulse at 1000us.
        TIMER4_BASE->CCR3 = channel_3;                                                 //Set the ESC 3 output pulse equal to the throttle input.
        TIMER4_BASE->CCR4 = 1000;                                                      //Keep the ESC 4 pulse at 1000us.
      }
      if (data == '4') {                                                               //If the user requested 4.
        TIMER4_BASE->CCR1 = 1000;                                                      //Keep the ESC 1 pulse at 1000us.
        TIMER4_BASE->CCR2 = 1000;                                                      //Keep the ESC 2 pulse at 1000us.
        TIMER4_BASE->CCR3 = 1000;                                                      //Keep the ESC 3 pulse at 1000us.
        TIMER4_BASE->CCR4 = channel_3;                                                 //Set the ESC 4 output pulse equal to the throttle input.
      }
      if (data == '5') {                                                               //If the user requested 5.
        TIMER4_BASE->CCR1 = channel_3;                                                 //Set the ESC 1 output pulse equal to the throttle input.
        TIMER4_BASE->CCR2 = channel_3;                                                 //Set the ESC 2 output pulse equal to the throttle input.
        TIMER4_BASE->CCR3 = channel_3;                                                 //Set the ESC 3 output pulse equal to the throttle input.
        TIMER4_BASE->CCR4 = channel_3;                                                 //Set the ESC 4 output pulse equal to the throttle input.
      }

    }
    else {                                                                             //// throttle_init_ok còn = 0 (chưa xác nhận ga thấp)
      wait_timer = millis() + 10000;                                                   //Set the wait_timer variable to the current millis() value incremented by 10 seconds.
      if (channel_3 > 1050) {                                                          //If the trottle channel is not in the lowest position.
        Serial.println(F("Throttle is not in the lowest position."));                  //Print a message on the serial monitor.
        Serial.print(F("Throttle value is: "));
        Serial.println(channel_3);
        Serial.print(F(""));
        Serial.print(F("Waiting for 10 seconds:"));
      }
      while (wait_timer > millis() && !throttle_init_ok) {                             //Stay in this wait loop for 10 seconds.
        if (channel_3 < 1050)throttle_init_ok = 1;                                     //If the throttle is in the lowest position set the throttle_init_ok variable.
        delay(500);                                                                    //Wait for 500 milliseconds.
        Serial.print(F("."));                                                          //Print a dot to show something is still working.
      }
    }
    if (!throttle_init_ok) {                                                           //If the throttle is not detected low after the 10 seconds quit this loop and return to the main menu.
      data = 'q';
    }
    while (loop_timer > micros());                                                     //Create a 250Hz loop. Ép vòng lặp đúng 4000µs
  }
  loop_counter = 0;                                                                    //Reset the loop counter variable to 0.
  print_intro();                                                                       //Print the intro to the serial monitor.
}
