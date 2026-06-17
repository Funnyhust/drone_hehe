//
//In this file the timers for reading the receiver pulses and for creating the output ESC pulses are set.
//More information can be found in these two videos:
//STM32 for Arduino - Connecting an RC receiver via input capture mode: https://youtu.be/JFSFbSg0l2M
//STM32 for Arduino - Electronic Speed Controller (ESC) - STM32F103C8T6: https://youtu.be/Nju9rvZOjVQ
//
void timer_setup(void) {
  Timer2.attachCompare1Interrupt(handler_channel_1); //// Khi chân tín hiệu kênh 1 đổi trạng thái, gọi hàm ngắt 'handler_channel_1' -> Phản hồi theo thời gian thực Không làm treo/đứng chương trình Đa nhiệm
  Timer2.attachCompare2Interrupt(handler_channel_2);
  Timer2.attachCompare3Interrupt(handler_channel_3);
  Timer2.attachCompare4Interrupt(handler_channel_4); //// ... Gắn các hàm ngắt cho 4 kênh của Timer 2
  TIMER2_BASE->CR1 = TIMER_CR1_CEN; // Bật (Enable) bộ đếm Timer 2. CEN = Counter Enable.
  TIMER2_BASE->CR2 = 0; // Reset thanh ghi điều khiển 2 (không dùng các tính năng nâng cao).
  TIMER2_BASE->SMCR = 0; // Reset thanh ghi Slave Mode (Timer chạy độc lập, không phụ thuộc Timer khác).
  //Đặt về 0 để tắt các tính năng đồng bộ phức tạp hoặc chế độ Slave
  TIMER2_BASE->DIER = TIMER_DIER_CC1IE | TIMER_DIER_CC2IE | TIMER_DIER_CC3IE | TIMER_DIER_CC4IE; // Bật tính năng ngắt Capture/Compare cho cả 4 kênh.
  TIMER2_BASE->EGR = 0; // Xóa thanh ghi tạo sự kiện (Event Generation Register).
  TIMER2_BASE->CCMR1 = 0b100000001;                       // Cài đặt chân cắm kênh 1 2  thành "Input" để lắng nghe và đo đạc độ rộng xung.
  TIMER2_BASE->CCMR2 = 0b100000001;                       // Cấu hình kênh 3 & 4 làm Input Capture tương tự như trên.
  TIMER2_BASE->CCER = TIMER_CCER_CC1E | TIMER_CCER_CC2E | TIMER_CCER_CC3E | TIMER_CCER_CC4E;                  //// Kích hoạt bộ đo Capture/Compare để bắt đầu ghi nhận tín hiệu.
  TIMER2_BASE->PSC = 71;                                        // Bộ chia tần số (Prescaler). Xung nhịp chip 72MHz chia cho (71+1) = 1MHz. Mỗi bước đếm = 1 microgiây (µs).
  TIMER2_BASE->ARR = 0xFFFF;                                    // Giá trị nạp lại tự động (Auto-Reload). Bằng 65535, tức là Timer đếm đến 65535 µs mới quay về 0.
  TIMER2_BASE->DCR = 0;                                         // Vô hiệu hóa DMA (Truy cập bộ nhớ trực tiếp) vì không dùng đến.

  Timer3.attachCompare1Interrupt(handler_channel_5);
  Timer3.attachCompare2Interrupt(handler_channel_6); // Gắn ngắt cho kênh 5,6
  TIMER3_BASE->CR1 = TIMER_CR1_CEN; //// Bật bộ đếm Timer 3
  TIMER3_BASE->CR2 = 0; 
  TIMER3_BASE->SMCR = 0; //reset
  TIMER3_BASE->DIER = TIMER_DIER_CC1IE | TIMER_DIER_CC2IE; // Bật ngắt cho kênh 1 và 2 của Timer 3 (tương ứng với kênh 5,6 của tay điều khiển)
  TIMER3_BASE->EGR = 0;
  TIMER3_BASE->CCMR1 = 0b100000001; //Register is set like this due to a bug in the define table (30-09-2017) // Cấu hình Input Capture cho kênh 1 & 2 của Timer 3
  TIMER3_BASE->CCMR2 = 0; // Kênh 3 & 4 của Timer 3 không dùng nên cho bằng 0
  TIMER3_BASE->CCER = TIMER_CCER_CC1E | TIMER_CCER_CC2E; // Kích hoạt Capture/Compare
  TIMER3_BASE->PSC = 71;  //// Đếm ở tốc độ 1 microgiây/nhịp
  TIMER3_BASE->ARR = 0xFFFF;// Đếm tối đa đến 65535
  TIMER3_BASE->DCR = 0;// Không dùng DMA

//A test is needed to check if the throttle input is active and valid. Otherwise the ESC's might start without any warning. Vòng lặp kiểm tra an toàn
  loop_counter = 0;
  // Vòng lặp sẽ giữ chương trình đứng đây nếu: 
  
  while ((channel_3 > 2100 || channel_3 < 900) && warning == 0) {                           // (Cần ga > 2100µs HOẶC Cần ga < 900µs) VÀ (chưa bị khóa an toàn - warning == 0)
    delay(100); 
    loop_counter++; // Tăng biến đếm thêm 1
    if (loop_counter == 40) { // Nếu đã chờ 40 lần (tương đương 40 * 100ms = 4 giây)
      // In ra màn hình Serial thông báo lỗi và yêu cầu người dùng kiểm tra tay điều khiển
      Serial.println(F("Waiting for a valid receiver channel-3 input signal"));
      Serial.println(F(""));
      Serial.println(F("The input pulse should be between 1000 till 2000us"));
      Serial.print(F("Current channel-3 receiver input value = "));
      Serial.println(channel_3);
      Serial.println(F(""));
      Serial.println(F("Is the receiver connected and the transmitter on?"));
      Serial.println(F("For more support and questions: www.brokking.net"));
      Serial.println(F(""));
      Serial.print(F("Waiting for another 5 seconds."));
    }
    if (loop_counter > 40 && loop_counter % 10 == 0)Serial.print(F(".")); // Đã quá 4 giây (40 vòng) và cứ mỗi 1 giây (10 vòng) thì in thêm một dấu chấm "."

      if (loop_counter == 90) { // Nếu đã chờ 90 lần (tương đương 9 giây) mà vẫn lỗi
      Serial.println(F(""));
      Serial.println(F(""));
      Serial.println(F("The ESC outputs are disabled for safety!!!"));
      warning = 1;
    }
  }
  if (warning == 0) {
    TIMER4_BASE->CR1 = TIMER_CR1_CEN | TIMER_CR1_ARPE; //// Bật Timer 4 và bật Preload. Preload giúp việc thay đổi độ rộng xung diễn ra mượt mà ở chu kỳ tiếp theo chứ không giật cục ngay lập tức.
    TIMER4_BASE->CR2 = 0; // Reset thanh ghi điều khiển 2 (không dùng các tính năng nâng cao).
    TIMER4_BASE->SMCR = 0; // Reset thanh ghi Slave Mode (Timer chạy độc lập, không phụ thuộc Timer khác).
    TIMER4_BASE->DIER = 0; // Không dùng ngắt cho Timer 4 (vì nó chỉ dùng để xuất tín hiệu ra bằng phần cứng)
    TIMER4_BASE->EGR = 0;  // Xóa thanh ghi tạo sự kiện (Event Generation Register).
    
    // Cấu hình 4 kênh của Timer 4 sang chế độ xuất PWM mode 1 (mã nhị phân 110)
    TIMER4_BASE->CCMR1 = (0b110 << 4) | TIMER_CCMR1_OC1PE |(0b110 << 12) | TIMER_CCMR1_OC2PE;  //Trong sách hướng dẫn (Datasheet) của STM32, quy định nhập mã 110 vào thanh ghi sẽ kích hoạt PWM Mode 1 TIMER_CCMR1_OC2PE: Bật tính năng Preload mượt mà cho Kênh 2.
    // TIMER_CCMR1_OC1PE: Chữ PE viết tắt của Preload Enable. Đây là một tính năng an toàn cực kỳ quan trọng.
    // dịch bit Nó đẩy chuỗi 110 sang trái 4 ô. Tại sao lại là 4? Vì theo thiết kế phần cứng của thanh ghi CCMR1, vị trí để cài đặt chế độ cho Kênh 1 nằm ở bit số 4, 5, 6.
    //Cấu hình cho Kênh 2 0b110 << 12: Vẫn là kích hoạt PWM Mode 1 (110), nhưng lần này dịch sang trái 12 ô. Vì vị trí để cài đặt cho Kênh 2 nằm ở bit số 12, 13, 14 của thanh ghi.    
    
    TIMER4_BASE->CCMR2 = (0b110 << 4) | TIMER_CCMR2_OC3PE |(0b110 << 12) | TIMER_CCMR2_OC4PE; // kênh 3 và 4
    TIMER4_BASE->CCER = TIMER_CCER_CC1E | TIMER_CCER_CC2E | TIMER_CCER_CC3E | TIMER_CCER_CC4E; //// Mở cổng xuất tín hiệu cho cả 4 kênh
    TIMER4_BASE->PSC = 71; 
    TIMER4_BASE->ARR = 4000; //Chu kỳ ESC 4000µs = 250Hz
    TIMER4_BASE->DCR = 0; // Không dùng DMA
    TIMER4_BASE->CCR1 = 1000; // Đặt sẵn giá trị độ rộng xung kênh 1 là 1000µs (dòng này chỉ là dự phòng/khởi tạo)

    TIMER4_BASE->CCR1 = channel_3;
    TIMER4_BASE->CCR2 = channel_3;
    TIMER4_BASE->CCR3 = channel_3;
    TIMER4_BASE->CCR4 = channel_3; //ESC 1 2 3 4
    pinMode(PB6, PWM); // Chân PB6 nối với ESC 1
    pinMode(PB7, PWM);
    pinMode(PB8, PWM);
    pinMode(PB9, PWM);
  }
}
