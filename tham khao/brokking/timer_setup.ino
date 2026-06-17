///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//In this file the timers for reading the receiver pulses and for creating the output ESC pulses are set.
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//More information can be found in these two videos:
//STM32 for Arduino - Connecting an RC receiver via input capture mode: https://youtu.be/JFSFbSg0l2M
//STM32 for Arduino - Electronic Speed Controller (ESC) - STM32F103C8T6: https://youtu.be/Nju9rvZOjVQ
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//Timer2 và Timer3 đọc (Input Capture). Timer4 xuất (PWM Output). Cùng là Timer nhưng cấu hình ngược nhau.
void timer_setup(void) {
  //Cấu hình Timer2 — Đọc 4 kênh tay điều khiển
  Timer2.attachCompare1Interrupt(handler_channel_1);
  Timer2.attachCompare2Interrupt(handler_channel_2);
  Timer2.attachCompare3Interrupt(handler_channel_3);
  Timer2.attachCompare4Interrupt(handler_channel_4);
  TIMER2_BASE->CR1 = TIMER_CR1_CEN; // Bật (Enable) bộ đếm Timer 2. CEN = Counter Enable.
  TIMER2_BASE->CR2 = 0;
  TIMER2_BASE->SMCR = 0; //Bỏ qua chế độ nâng cao
  TIMER2_BASE->DIER = TIMER_DIER_CC1IE | TIMER_DIER_CC2IE | TIMER_DIER_CC3IE | TIMER_DIER_CC4IE; //bit "Capture/Compare x Interrupt Enable". Bật 4 bit cùng lúc bằng OR (|) = cho phép tất cả 4 kênh tạo ngắt khi có sự kiện capture.
  //Đây là dòng nối "phần cứng phát hiện mép" với "phần mềm gọi handler". Không có dòng này → mép có nhưng handler không được gọi.
  TIMER2_BASE->EGR = 0; //Event Generation Register — không tạo sự kiện thủ công. Đặt 0 để timer chạy tự nhiên.
  
  //đặt 4 kênh của Timer2 sang chế độ Input Capture.
  TIMER2_BASE->CCMR1 = 0b100000001; //Register is set like this due to a bug in the define table (30-09-2017) Cài đặt chân cắm kênh 1 2  thành "Input" để lắng nghe và đo đạc độ rộng xung.
  TIMER2_BASE->CCMR2 = 0b100000001; //Register is set like this due to a bug in the define table (30-09-2017) Cấu hình kênh 3 & 4 làm Input Capture tương tự như trên.
  TIMER2_BASE->CCER = TIMER_CCER_CC1E | TIMER_CCER_CC2E | TIMER_CCER_CC3E | TIMER_CCER_CC4E; //// Kích hoạt bộ đo Capture/Compare để bắt đầu ghi nhận tín hiệu.
  TIMER2_BASE->PSC = 71; //// Bộ chia tần số (Prescaler). Xung nhịp chip 72MHz chia cho (71+1) = 1MHz. Mỗi bước đếm = 1 microgiây (µs).
  TIMER2_BASE->ARR = 0xFFFF; //// Giá trị nạp lại tự động (Auto-Reload). Bằng 65535, tức là Timer đếm đến 65535 µs mới quay về 0.
  TIMER2_BASE->DCR = 0; // // Vô hiệu hóa DMA (Truy cập bộ nhớ trực tiếp) vì không dùng đến.

  Timer3.attachCompare1Interrupt(handler_channel_5);
  Timer3.attachCompare2Interrupt(handler_channel_6); //// Gắn ngắt cho kênh 5,6
  TIMER3_BASE->CR1 = TIMER_CR1_CEN; // Bật bộ đếm Timer 3
  TIMER3_BASE->CR2 = 0;
  TIMER3_BASE->SMCR = 0;
  TIMER3_BASE->DIER = TIMER_DIER_CC1IE | TIMER_DIER_CC2IE; // Bật bộ đếm Timer 3
  TIMER3_BASE->EGR = 0;
  TIMER3_BASE->CCMR1 = 0b100000001; //Register is set like this due to a bug in the define table (30-09-2017)
  TIMER3_BASE->CCMR2 = 0;
  TIMER3_BASE->CCER = TIMER_CCER_CC1E | TIMER_CCER_CC2E; // Cấu hình Input Capture cho kênh 1 & 2 của Timer 3
  TIMER3_BASE->PSC = 71;
  TIMER3_BASE->ARR = 0xFFFF;
  TIMER3_BASE->DCR = 0;


  //Cấu hình Timer4 — Xuất 4 xung PWM cho ESC 
  TIMER4_BASE->CR1 = TIMER_CR1_CEN | TIMER_CR1_ARPE; //CR1_CEN = bật bộ đếm (như Timer2). CR1_ARPE = "Auto-Reload Preload Enable" = khi cập nhật ARR giữa chừng, đợi đến cuối chu kỳ mới áp dụng, tránh xung bị méo.
  TIMER4_BASE->CR2 = 0;
  TIMER4_BASE->SMCR = 0;
  TIMER4_BASE->DIER = 0;  // KHÔNG cần ngắt (Output không cần) hardware tự tạo xung theo cấu hình, code không cần can thiệp mỗi chu kỳ.
  TIMER4_BASE->EGR = 0;
  TIMER4_BASE->CCMR1 = (0b110 << 4) | TIMER_CCMR1_OC1PE |(0b110 << 12) | TIMER_CCMR1_OC2PE;//Trong sách hướng dẫn (Datasheet) của STM32, quy định nhập mã 110 vào thanh ghi sẽ kích hoạt PWM Mode 1 TIMER_CCMR1_OC2PE: Bật tính năng Preload mượt mà cho Kênh 2.
    // TIMER_CCMR1_OC1PE: Chữ PE viết tắt của Preload Enable. Đây là một tính năng an toàn cực kỳ quan trọng.
    // dịch bit Nó đẩy chuỗi 110 sang trái 4 ô. Tại sao lại là 4? Vì theo thiết kế phần cứng của thanh ghi CCMR1, vị trí để cài đặt chế độ cho Kênh 1 nằm ở bit số 4, 5, 6.
    //Cấu hình cho Kênh 2 0b110 << 12: Vẫn là kích hoạt PWM Mode 1 (110), nhưng lần này dịch sang trái 12 ô. Vì vị trí để cài đặt cho Kênh 2 nằm ở bit số 12, 13, 14 của thanh ghi.    
    
  TIMER4_BASE->CCMR2 = (0b110 << 4) | TIMER_CCMR2_OC3PE |(0b110 << 12) | TIMER_CCMR2_OC4PE;  // kênh 3 và 4
  TIMER4_BASE->CCER = TIMER_CCER_CC1E | TIMER_CCER_CC2E | TIMER_CCER_CC3E | TIMER_CCER_CC4E; //// Mở cổng xuất tín hiệu cho cả 4 kênh
  TIMER4_BASE->PSC = 71;   //// 1 tick = 1µs (giống Timer2)
  TIMER4_BASE->ARR = 5000; // Chu kỳ = 5000µs = 5ms = 200Hz
  TIMER4_BASE->DCR = 0;
  //Đặt xung khởi đầu = 1000µs
  TIMER4_BASE->CCR1 = 1000;
  TIMER4_BASE->CCR1 = 1000;
  TIMER4_BASE->CCR2 = 1000;
  TIMER4_BASE->CCR3 = 1000;
  TIMER4_BASE->CCR4 = 1000;
  //Đặt chế độ PWM cho chân
  pinMode(PB6, PWM);
  pinMode(PB7, PWM);
  pinMode(PB8, PWM);
  pinMode(PB9, PWM);
}

//CR1Control Register 1Bật/tắt timer (bit CEN)
//CR2Control Register 2Cấu hình nâng cao (chế độ slave...)

//Timer2 (PSC=71, ARR=0xFFFF): Input Capture 4 kênh PA0-PA3, gắn ngắt → 4 handler đo xung tay điều khiển.
//Timer3 (PSC=71, ARR=0xFFFF): Input Capture 2 kênh PA6-PA7 → 2 handler cho kênh 5,6.
//Timer4 (PSC=71, ARR=5000): PWM Output 4 kênh PB6-PB9 → tạo xung 200Hz cho 4 ESC. Code chính ghi CCR1-4 mỗi vòng để cập nhật độ rộng xung.
//Hằng số chung PSC=71 → 1 tick = 1µs để đo/tạo xung theo đơn vị µs.

//Input Capture là chế độ hoạt động của Timer trên STM32, trong đó Timer tự động lưu giá trị bộ đếm vào thanh ghi CCRx khi phát hiện mép tín hiệu trên chân tương ứng. 
//Đồng thời nó kích hoạt ngắt để gọi hàm xử lý. Nhờ đó vi điều khiển có thể đo độ rộng xung PWM chính xác đến µs mà không cần lắng nghe chân liên tục bằng code, tiết kiệm tài nguyên CPU cho việc tính toán PID
