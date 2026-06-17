void reading_receiver_signals(void) {
  while (data != 'q') {                                                                   //Stay in this loop until the data variable data holds a q. q để thoát vòng lặp
    delay(250);                                                                           //Print the receiver values on the screen every 250ms
    if (Serial.available() > 0) {                                                         //If serial data is available //kiểm tra có ký tự không (available), đọc vào data (read), xả sạch bộ đệm
      data = Serial.read();                                                               //Read the incomming byte
      delay(100);                                                                         //Wait for any other bytes to come in
      while (Serial.available() > 0)loop_counter = Serial.read();                         //Empty the Serial buffer
    }
    //ARM tay cầm
    //For starting the motors: throttle low and yaw left (step 1). Đây là máy trạng thái mở/khóa động cơ — một cơ chế an toàn quan trọng. Ý tưởng: không cho motor quay bừa
    if (channel_3 < 1100 && channel_4 < 1100)start = 1;
    //When yaw stick is back in the center position start the motors (step 2). 0 = khóa, 1 = chuẩn bị, 2 = đã mở
    if (start == 1 && channel_3 < 1100 && channel_4 > 1450)start = 2;
    //Stopping the motors: throttle low and yaw right.
    if (start == 2 && channel_3 < 1100 && channel_4 > 1900)start = 0;

    Serial.print("Start:");
    Serial.print(start);

    Serial.print("  Roll:");
    if (channel_1 - 1480 < 0)Serial.print("<<<"); //nếu channel_1 < 1480 (gạt sang trái) → in <<<
    else if (channel_1 - 1520 > 0)Serial.print(">>>");//gạt phải
    else Serial.print("-+-"); //nằm trong 1480–1520 (ở giữa) → in -+- (ký hiệu "cân bằng")
    Serial.print(channel_1);

    Serial.print("  Pitch:");
    if (channel_2 - 1480 < 0)Serial.print("^^^"); 
    else if (channel_2 - 1520 > 0)Serial.print("vvv"); //Pitch: ^^^ (đẩy trước) / vvv (kéo sau)
    else Serial.print("-+-");
    Serial.print(channel_2);

    Serial.print("  Throttle:");
    if (channel_3 - 1480 < 0)Serial.print("vvv");
    else if (channel_3 - 1520 > 0)Serial.print("^^^");
    else Serial.print("-+-"); //ga lên xuống
    Serial.print(channel_3);

    Serial.print("  Yaw:");
    if (channel_4 - 1480 < 0)Serial.print("<<<");
    else if (channel_4 - 1520 > 0)Serial.print(">>>");
    else Serial.print("-+-"); //<<< (xoay trái) / >>> (xoay phải)
    Serial.print(channel_4);

    Serial.print("  CH5:");
    Serial.print(channel_5);

    Serial.print("  CH6:");
    Serial.println(channel_6);

  }
  print_intro(); // in lại menu
}
