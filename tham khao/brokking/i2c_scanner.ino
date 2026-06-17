void i2c_scanner(void) {
  //Let's declare some variables so we can use them in this subroutine.
  data = 0; //xóa biến lệnh về 0 
  uint8_t error, address, done; //address = địa chỉ đang dò thử (1 đến 126) .error = mã kết quả khi gọi thử địa chỉ (0 = có thiết bị, khác 0 = không). done = cờ báo xong (thực ra không dùng mấy).
  uint16_t nDevices; //đếm số thiết bị tìm thấy.
  Serial.println("Scanning address 1 till 127...");
  Serial.println("");
  nDevices = 0; //In dòng "Đang quét địa chỉ 1 đến 127...", in một dòng trống, và đặt bộ đếm nDevices = 0 (chưa tìm thấy cái nào)
  for (address = 1; address < 127; address++) { 
    HWire.beginTransmission(address); //quét i2c, gọi thử địa chỉ
    error = HWire.endTransmission(); 

    if (error == 0) {
      Serial.print("I2C device found at address 0x"); //nếu địa chỉ này có thiết bị trả lời thì: In "I2C device found at address 0x" (tìm thấy thiết bị ở địa chỉ 0x...).
      if (address < 16)
        Serial.print("0"); ///nếu số nhỏ hơn 16 (hex chỉ 1 chữ số như 0x8), in thêm số 0 đằng trước cho thành 0x08
      Serial.println(address, HEX);

      nDevices++;
    }
    else if (error == 4) {
      Serial.print("Unknown error at address 0x"); //error == 4 là một mã lỗi đặc biệt (lỗi không xác định, hiếm gặp).
      if (address < 16)
        Serial.print("0");
      Serial.println(address, HEX);
    }
  }
  done = 1;
  if (nDevices == 0)
    Serial.println("No I2C devices found");
  else
    Serial.println("done");
  delay(2000);
  print_intro();
}
