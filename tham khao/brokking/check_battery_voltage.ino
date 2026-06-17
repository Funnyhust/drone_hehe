void check_battery_voltage(void) {
  loop_counter = 0;                                                                       //reset bộ đếm vòng
  battery_voltage = analogRead(4);                                                        //đọc lần đầu điện áp. analogRead(4) đọc chân PA4 (chân nối mạch chia áp đo pin), trả về số nguyên 0–4095 tương ứng mức điện áp. Gán làm giá trị khởi đầu cho bộ lọc bên dưới.
  while (data != 'q') {                                                                   //Stay in this loop until the data variable data holds a q.
    delayMicroseconds(4000);                                                              //Wait for 4000us to simulate a 250Hz loop.
    if (Serial.available() > 0) {                                                         //If serial data is available.
      data = Serial.read();                                                               //Read the incomming byte.
      delay(100);                                                                         //Wait for any other bytes to come in.
      while (Serial.available() > 0)loop_counter = Serial.read();                         //Empty the Serial buffer.
    }
    loop_counter++;//mỗi vòng tăng 1. Vòng chạy 250 lần/giây (mỗi vòng 4000µs). Nên if (loop_counter == 250) đúng mỗi 1 giây một lần → in điện áp mỗi giây, không in liên tục (đỡ rối màn hình).
    if (loop_counter == 250) {                                                            //Print the battery voltage every second. 
      Serial.print("Voltage = ");                                                         //Print some preliminary information.
      Serial.print(battery_voltage / 112.81, 1);                                          //Print the avarage battery voltage to the serial monitor. Mạch chia áp tỉ lệ 1/11 (R3 10k + R4 1k) → điện áp pin tối đa đo được = 3.3V × 11 = 36.3V.
                                                                                          //→ 4095 / 36.3 = 112.81 đơn vị ADC cho mỗi 1 volt pin.
      Serial.println("V");                                                                //Print some trailing information.
      loop_counter = 0;                                                                   //Reset the loop counter. để đếm lại từ đầu.
    }
    //A complimentary filter is used to filter out the voltage spikes caused by the ESC's. Bộ lọc bù nhiễu điện áp điện áp mới = 99% (điện áp cũ) + 1% (giá trị vừa đọc)
    battery_voltage = (battery_voltage * 0.99) + ((float)analogRead(4) * 0.01);
  }
  loop_counter = 0;                                                                       //Reset the loop counter.
  print_intro();                                                                          //Print the intro to the serial monitor.
}
