#include "stm32f4xx.h"
#include <stdio.h>
#include <string.h>
/*
  PA0  : Chân vào analog (ADC1_IN0)
  PA2  : Chân truyền dữ liệu USART2 (TX)
*/
void GPIO_Init(void);
void ADC1_Init(void);
uint16_t ADC1_Read(void);
void USART2_Init(void);
void USART2_Transmit(char *data);
uint8_t Read_Button(void);

int main(void)
{
    uint16_t adc_value;
    uint8_t button_state;
    char msg[20];

    ADC1_Init();
    USART2_Init();
    GPIO_Init();

    while (1)
    {
        adc_value = ADC1_Read();
        button_state = Read_Button();

        // Định dạng chuỗi theo CSV
        sprintf(msg, "%d,%d\n", adc_value, button_state);
        USART2_Transmit(msg);

        for (volatile int i = 0; i < 100000; i++);
    }
}


void GPIO_Init(void)
{
    RCC->AHB1ENR |= RCC_AHB1ENR_GPIOAEN;   // Cấp xung clock cho GPIOA
    GPIOA->MODER &= ~GPIO_MODER_MODER1;    // PA1 là Input (00)
    GPIOA->PUPDR |= GPIO_PUPDR_PUPDR1_0;   // Kích hoạt Pull-up cho PA1
}

//ADC1 - PA0
void ADC1_Init(void)
{
    RCC->AHB1ENR |= RCC_AHB1ENR_GPIOAEN;   // Cho phép clock cho Port A (chân PA0)
    RCC->APB2ENR |= RCC_APB2ENR_ADC1EN;      // Cho phép clock cho ADC1
    // Cấu hình chân PA0 ở chế độ Analog (MODER0 = 11)
    GPIOA->MODER |= GPIO_MODER_MODER0;
    // Cấu hình ADC1:
    ADC1->CR2 = 0;                        // Reset cấu hình CR2
    ADC1->SQR3 = 0;                       // Chọn kênh 0 (ADC1_IN0) cho chuyển đổi đầu tiên
    ADC1->SMPR2 = ADC_SMPR2_SMP0;           // Chọn thời gian lấy mẫu (sampling time) tối đa cho độ chính xác cao
    ADC1->CR2 |= ADC_CR2_ADON;              // Bật ADC1
}
uint16_t ADC1_Read(void)
{
    ADC1->CR2 |= ADC_CR2_SWSTART;       // Bắt đầu chuyển đổi ADC
    while (!(ADC1->SR & ADC_SR_EOC));   // Chờ đến khi quá trình chuyển đổi kết thúc (EOC: End Of Conversion)
    return ADC1->DR;                    // Đọc và trả về giá trị số của ADC
}
// USART
void USART2_Init(void)
{
    RCC->AHB1ENR |= RCC_AHB1ENR_GPIOAEN;   // Cho phép clock cho Port A (PA2, PA3)
    RCC->APB1ENR |= RCC_APB1ENR_USART2EN;    // Cho phép clock cho USART2

    GPIOA->MODER |= GPIO_MODER_MODER2_1 | GPIO_MODER_MODER3_1; // Cấu hình AF mode cho PA2 và PA3
    GPIOA->AFR[0] |= 0x7700;   // Gán AF7 cho PA2 và PA3

    USART2->BRR = 0x0683; //baud rate 9600
    USART2->CR1 = USART_CR1_TE | USART_CR1_UE;     // Bật chế độ truyền và USART2
}
// Hàm truyền chuỗi ký tự qua USART2
void USART2_Transmit(char *data)
{
    while (*data)
    {
        // Chờ đến khi thanh ghi truyền TXE rỗng
        while (!(USART2->SR & USART_SR_TXE));
        // Gửi ký tự (8 bit)
        USART2->DR = (*data++ & 0xFF);
    }
}

uint8_t Read_Button(void)
{
    return (GPIOA->IDR & GPIO_IDR_ID1) ? 1 : 0; // Đọc trạng thái của PA1
}
