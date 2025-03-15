#include "stm32f4xx.h"

#define PWM_MAX 1000

void TIM2_PWM_Init(void) {
    RCC->APB1ENR |= RCC_APB1ENR_TIM2EN;  // Bật clock cho TIM2
    TIM2->PSC = 84 - 1;                  // Chia tần số 84MHz xuống 1MHz
    TIM2->ARR = PWM_MAX - 1;             // Định chu kỳ PWM
    TIM2->CCMR1 |= TIM_CCMR1_OC1M_1 | TIM_CCMR1_OC1M_2; // PWM mode 1
    TIM2->CCMR1 |= TIM_CCMR1_OC1PE;      // Cho phép preload
    TIM2->CCER |= TIM_CCER_CC1E;         // Kích hoạt đầu ra PWM
    TIM2->CR1 |= TIM_CR1_CEN;            // Bắt đầu timer
}

void GPIO_Init(void) {
    RCC->AHB1ENR |= RCC_AHB1ENR_GPIOAEN; // Bật clock GPIOA

    // Cấu hình PA0 (PWM - TIM2_CH1) chế độ AF1
    GPIOA->MODER |= GPIO_MODER_MODER0_1;
    GPIOA->AFR[0] |= (1 << (4 * 0));     // AF1 cho PA0

    // Cấu hình PA1 (IN2 - Điều khiển chiều quay) là Output
    GPIOA->MODER |= GPIO_MODER_MODER1_0;
    GPIOA->OTYPER &= ~GPIO_OTYPER_OT1;
    GPIOA->OSPEEDR |= GPIO_OSPEEDER_OSPEEDR1;
}

void Motor_SetSpeed(uint16_t speed) {
    if (speed > PWM_MAX) speed = PWM_MAX;
    TIM2->CCR1 = speed;
}

void Motor_SetDirection(uint8_t dir) {
    if (dir == 1) {
        GPIOA->BSRR = GPIO_BSRR_BS1; // PA1 = 1 (Quay thuận)
    } else {
        GPIOA->BSRR = GPIO_BSRR_BR1; // PA1 = 0 (Quay ngược)
    }
}

void delay_ms(uint32_t ms) {
    for (volatile uint32_t i = 0; i < ms * 4000; i++);
}

int main(void) {
    GPIO_Init();
    TIM2_PWM_Init();

    while (1) {
        Motor_SetDirection(1);
        Motor_SetSpeed(500);
        delay_ms(2000);

        Motor_SetDirection(0);
        Motor_SetSpeed(800);
        delay_ms(2000);

        Motor_SetSpeed(0);
        delay_ms(2000);
    }
}
