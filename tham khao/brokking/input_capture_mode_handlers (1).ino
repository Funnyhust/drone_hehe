void handler_channel_1(void) {                           // Hàm này được gọi khi kênh 1 bắt được tín hiệu (có sự thay đổi trạng thái).
  if (0b1 & GPIOA_BASE->IDR  >> 0) {                     // Nếu xung tín hiệu đầu vào của kênh 1 trên chân A0 ở mức CAO.
    channel_1_start = TIMER2_BASE->CCR1;                 // Ghi lại mốc thời gian bắt đầu của xung.
    TIMER2_BASE->CCER |= TIMER_CCER_CC1P;                // Đổi chế độ bắt tín hiệu (input capture) sang chờ bắt "sườn xuống" (khi điện áp rớt xuống).
  }
  else {                                                 // Nếu xung tín hiệu đầu vào của kênh 1 trên chân A0 ở mức THẤP.
    channel_1 = TIMER2_BASE->CCR1 - channel_1_start;     // Tính toán tổng thời gian của xung (độ rộng xung).
    if (channel_1 < 0)channel_1 += 0xFFFF;               // Nếu bộ đếm timer vừa bị tràn (đếm lố qua 65535 rồi quay về 0), cần cộng bù thêm để ra số dương.
    TIMER2_BASE->CCER &= ~TIMER_CCER_CC1P;               // Đổi chế độ bắt tín hiệu quay lại chờ bắt "sườn lên" (để chuẩn bị cho chu kỳ tiếp theo).
  }
}

void handler_channel_2(void) {                           // Hàm này được gọi khi kênh 2 bắt được tín hiệu.
  if (0b1 & GPIOA_BASE->IDR >> 1) {                      // Nếu xung tín hiệu đầu vào của kênh 2 trên chân A1 ở mức CAO.
    channel_2_start = TIMER2_BASE->CCR2;                 // Ghi lại mốc thời gian bắt đầu của xung.
    TIMER2_BASE->CCER |= TIMER_CCER_CC2P;                // Đổi chế độ bắt tín hiệu sang chờ bắt "sườn xuống".
  }
  else {                                                 // Nếu xung tín hiệu đầu vào của kênh 2 trên chân A1 ở mức THẤP.
    channel_2 = TIMER2_BASE->CCR2 - channel_2_start;     // Tính toán tổng thời gian của xung.
    if (channel_2 < 0)channel_2 += 0xFFFF;               // Bù trừ nếu timer bị tràn vòng.
    TIMER2_BASE->CCER &= ~TIMER_CCER_CC2P;               // Đổi chế độ bắt tín hiệu quay lại chờ bắt "sườn lên".
  }
}

void handler_channel_3(void) {                           // Hàm này được gọi khi kênh 3 bắt được tín hiệu.
  if (0b1 & GPIOA_BASE->IDR >> 2) {                      // Nếu xung tín hiệu đầu vào của kênh 3 trên chân A2 ở mức CAO.
    channel_3_start = TIMER2_BASE->CCR3;                 // Ghi lại mốc thời gian bắt đầu của xung.
    TIMER2_BASE->CCER |= TIMER_CCER_CC3P;                // Đổi chế độ bắt tín hiệu sang chờ bắt "sườn xuống".
  }
  else {                                                 // Nếu xung tín hiệu đầu vào của kênh 3 trên chân A2 ở mức THẤP.
    channel_3 = TIMER2_BASE->CCR3 - channel_3_start;     // Tính toán tổng thời gian của xung.
    if (channel_3 < 0)channel_3 += 0xFFFF;               // Bù trừ nếu timer bị tràn vòng.
    TIMER2_BASE->CCER &= ~TIMER_CCER_CC3P;               // Đổi chế độ bắt tín hiệu quay lại chờ bắt "sườn lên".
  }
}

void handler_channel_4(void) {                           // Hàm này được gọi khi kênh 4 bắt được tín hiệu.
  if (0b1 & GPIOA_BASE->IDR >> 3) {                      // Nếu xung tín hiệu đầu vào của kênh 4 trên chân A3 ở mức CAO.
    channel_4_start = TIMER2_BASE->CCR4;                 // Ghi lại mốc thời gian bắt đầu của xung.
    TIMER2_BASE->CCER |= TIMER_CCER_CC4P;                // Đổi chế độ bắt tín hiệu sang chờ bắt "sườn xuống".
  }
  else {                                                 // Nếu xung tín hiệu đầu vào của kênh 4 trên chân A3 ở mức THẤP.
    channel_4 = TIMER2_BASE->CCR4 - channel_4_start;     // Tính toán tổng thời gian của xung.
    if (channel_4 < 0)channel_4 += 0xFFFF;               // Bù trừ nếu timer bị tràn vòng.
    TIMER2_BASE->CCER &= ~TIMER_CCER_CC4P;               // Đổi chế độ bắt tín hiệu quay lại chờ bắt "sườn lên".
  }
}

void handler_channel_5(void) {                           // Hàm này được gọi khi kênh 5 bắt được tín hiệu.
  if (0b1 & GPIOA_BASE->IDR >> 6) {                      // Nếu xung tín hiệu đầu vào của kênh 5 trên chân A6 ở mức CAO.
    channel_5_start = TIMER3_BASE->CCR1;                 // Ghi lại mốc thời gian bắt đầu của xung.
    TIMER3_BASE->CCER |= TIMER_CCER_CC1P;                // Đổi chế độ bắt tín hiệu sang chờ bắt "sườn xuống".
  }
  else {                                                 // Nếu xung tín hiệu đầu vào của kênh 5 trên chân A6 ở mức THẤP.
    channel_5 = TIMER3_BASE->CCR1 - channel_5_start;     // Tính toán tổng thời gian của xung.
    if (channel_5 < 0)channel_5 += 0xFFFF;               // Bù trừ nếu timer bị tràn vòng.
    TIMER3_BASE->CCER &= ~TIMER_CCER_CC1P;               // Đổi chế độ bắt tín hiệu quay lại chờ bắt "sườn lên".
  }
}

void handler_channel_6(void) {                           // Hàm này được gọi khi kênh 6 bắt được tín hiệu.
  if (0b1 & GPIOA_BASE->IDR >> 7) {                      // Nếu xung tín hiệu đầu vào của kênh 6 trên chân A7 ở mức CAO.
    channel_6_start = TIMER3_BASE->CCR2;                 // Ghi lại mốc thời gian bắt đầu của xung.
    TIMER3_BASE->CCER |= TIMER_CCER_CC2P;                // Đổi chế độ bắt tín hiệu sang chờ bắt "sườn xuống".
  }
  else {                                                 // Nếu xung tín hiệu đầu vào của kênh 6 trên chân A7 ở mức THẤP.
    channel_6 = TIMER3_BASE->CCR2 - channel_6_start;     // Tính toán tổng thời gian của xung.
    if (channel_6 < 0)channel_6 += 0xFFFF;               // Bù trừ nếu timer bị tràn vòng.
    TIMER3_BASE->CCER &= ~TIMER_CCER_CC2P;               // Đổi chế độ bắt tín hiệu quay lại chờ bắt "sườn lên".
  }
}
//Tay điều khiển RC của bạn không gửi các con số như "tốc độ 50" hay "rẽ trái 30 độ" về cho máy bay. Thay vào đó, nó giao tiếp bằng mã Morse
//của thế giới điện tử: Độ rộng xung (PWM). Toàn bộ ý nghĩa của 6 hàm ngắt này là để đo đạc chính xác độ dài của các xung điện đó tính bằng microgiây (µs).

// Toàn bộ khối code này là một thiết kế hoàn hảo để đo đạc ý muốn của bạn một cách thầm lặng, chính xác đến từng phần triệu giây, mà không hề làm máy bay bị "lag" hay mất thăng bằng trong lúc bay.

//Timer của STM32 là một bộ đếm chỉ đếm được tối đa đến 65535 (0xFFFF) rồi sẽ tự động reset về 0 
///Giả sử tín hiệu bật lên lúc Timer đang chỉ số 65000.
//Tín hiệu kéo dài 1500µs. Vậy lúc tín hiệu tắt, Timer sẽ chạy qua số 65535, quay về 0 và chạy tiếp đến số 964.
//Nếu bạn lấy thời gian kết thúc trừ thời gian bắt đầu: 964 - 65000 = -64036 (ra một số âm vô lý).
//Dòng lệnh trên phát hiện số âm, lập tức cộng thêm một vòng của đồng hồ (65535 + 1 = 65536): -64036 + 65536 = 1500. Kết quả lại trở về con số 1500µs chuẩn xác

//Kết quả phép trừ ở trên (biến channel_1, channel_2...) sẽ luôn dao động trong khoảng từ 1000 đến 2000:
//1000µs: Nghĩa là bạn đang gạt cần điều khiển xuống kịch sàn (hoặc kịch trái).
//1500µs: Nghĩa là bạn đang để cần điều khiển nằm tự do ở chính giữa.
//2000µs: Nghĩa là bạn đang đẩy cần lên kịch trần (hoặc kịch phải).
//Vi điều khiển cần những con số này để biết bạn đang muốn máy bay bay nhanh hay chậm, nghiêng trái hay phải.
