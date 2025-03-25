#include "stm32f4xx.h"

void GPIO_Init(void) {
    RCC->AHB1ENR |= RCC_AHB1ENR_GPIOAEN;
    GPIOA->MODER |= (3 << 0);  // PA0: Analog mode (ADC input)
    GPIOA->MODER |= (2 << 2);  // PA1: Alternate function mode (PWM output)
    GPIOA->AFR[0] |= (1 << 4); // AF1: TIM2_CH2 cho PA1
}

void ADC_Init(void) {
    RCC->APB2ENR |= RCC_APB2ENR_ADC1EN;
    ADC1->SQR3 = 0;  // Kênh 0 (PA0)
    ADC1->CR2 |= ADC_CR2_ADON;
}

uint16_t ADC_Read(void) {
    ADC1->CR2 |= ADC_CR2_SWSTART;
    while (!(ADC1->SR & ADC_SR_EOC));
    return ADC1->DR;
}

void UART2_Init(void) {
    RCC->APB1ENR |= RCC_APB1ENR_USART2EN;
    RCC->AHB1ENR |= RCC_AHB1ENR_GPIOAEN;
    GPIOA->MODER |= (2 << 4) | (2 << 6);  // PA2, PA3: AF mode
    GPIOA->AFR[0] |= (7 << 8) | (7 << 12); // AF7: UART2
    USART2->BRR = 0x8B;  // 115200 baud (HCLK = 16MHz)
    USART2->CR1 = USART_CR1_TE | USART_CR1_UE;
}

void UART2_SendChar(uint8_t c) {
    while (!(USART2->SR & USART_SR_TXE));
    USART2->DR = c;
}

void UART2_SendString(char *str) {
    while (*str) UART2_SendChar(*str++);
}

void int_to_string(uint16_t num, char *str) {
    int i = 0;
    if (num == 0) {
        str[i++] = '0';
    } else {
        while (num > 0) {
            str[i++] = (num % 10) + '0';
            num /= 10;
        }
    }
    int j = 0;
    char temp;
    for (j = 0; j < i / 2; j++) {
        temp = str[j];
        str[j] = str[i - 1 - j];
        str[i - 1 - j] = temp;
    }
    str[i] = '\0';
}

void adc_to_string(uint16_t adc_value, char *str) {
    uint32_t voltage_x100 = (adc_value * 330) / 4095;
    uint16_t integer_part = voltage_x100 / 100;
    uint16_t decimal_part = voltage_x100 % 100;

    char int_str[5];
    int_to_string(integer_part, int_str);

    int i = 0;
    while (int_str[i] != '\0') {
        str[i] = int_str[i];
        i++;
    }
    str[i++] = '.';
    if (decimal_part < 10) {
        str[i++] = '0';
        str[i++] = decimal_part + '0';
    } else {
        str[i++] = (decimal_part / 10) + '0';
        str[i++] = (decimal_part % 10) + '0';
    }
    str[i++] = '\n';
    str[i] = '\0';
}

void PWM_Init(void) {
    // Bật clock cho TIM2
    RCC->APB1ENR |= RCC_APB1ENR_TIM2EN;

    // Cấu hình TIM2: tần số 1Hz, HCLK = 16MHz
    TIM2->ARR = 63999;  // Auto-reload: 16MHz / (250 * 64000) = 1Hz
    TIM2->PSC = 249;    // Prescaler = 249 (250 sau +1)

    // Cấu hình kênh 2 (PA1) cho PWM mode 1
    TIM2->CCMR1 |= (6 << 12);  // PWM mode 1 (110)
    TIM2->CCER |= TIM_CCER_CC2E;  // Bật kênh 2 output

    // Đặt duty cycle (50%: 32000/64000)
    TIM2->CCR2 = 32000;

    // Bật timer
    TIM2->CR1 |= TIM_CR1_CEN;
}

int main(void) {
    GPIO_Init();
    ADC_Init();
    UART2_Init();
    PWM_Init();

    char buffer[10];
    while (1) {
        uint16_t adc_value = ADC_Read();  // Đo tín hiệu tại PA0
        adc_to_string(adc_value, buffer);
        UART2_SendString(buffer);
        for (volatile int i = 0; i < 10000; i++);  // Giữ delay để lấy mẫu
    }
}
