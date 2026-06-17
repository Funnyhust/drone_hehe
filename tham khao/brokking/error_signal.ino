///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//In this part the error LED signal is generated.
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void error_signal(void){
  if (error >= 100) red_led(HIGH);                                        //When the error is 100 the LED is always on.
  else if (error_timer < millis()){                                       //If the error_timer value is smaller that the millis() function.
    error_timer = millis() + 250;                                         //Set the next error_timer interval at 250ms.
    if(error > 0 && error_counter > error + 3) error_counter = 0;         //If there is an error to report and the error_counter > error +3 reset the error.
    if (error_counter < error && error_led == 0 && error > 0){            //If the error flash sequence isn't finisched (error_counter < error) and the LED is off.
      red_led(HIGH);                                                      //Turn the LED on.
      error_led = 1;                                                      //Set the LED flag to indicate that the LED is on.
    }
    else{                                                                 //If the error flash sequence isn't finisched (error_counter < error) and the LED is on. 
      red_led(LOW);                                                       //Turn the LED off.
      error_counter++;                                                    //Increment the error_counter variable by 1 to keep trach of the flashes.
      error_led = 0;                                                      //Set the LED flag to indicate that the LED is off.
    }
  }
}

//0Không có lỗiBình thường 
//1 Pin yếu (<10V) 1 nháy, dài tối, 1 nháy, dài tối...
//2 Không nói chuyện được cảm biến 2 nháy nhanh, dài tối, 2 nháy nhanh, dài tối...
//3 Không có tín hiệu tay điều khiển 3 nháy nhanh, dài tối, lặp lại...
//4 Cần ga không thấp 4 nháy nhanh, dài tối, lặp lại...
//5 Vòng lặp quá giờ (>4050µs) 5 nháy nhanh, dài tối, lặp lại...
//≥100Lỗi nghiêm trọng
