#include "stm32f4xx.h"

void delay(void) {
    for (volatile int i = 0; i < 100000; i++);
}

void USART1_Init(void) {
    RCC->APB2ENR |= RCC_APB2ENR_USART1EN;
    RCC->AHB1ENR |= RCC_AHB1ENR_GPIOAEN;

    // PA9 (TX)
    GPIOA->MODER &= ~(3 << (9 * 2));
    GPIOA->MODER |= (2 << (9 * 2)); // AF mode
    GPIOA->AFR[1] |= (7 << ((9 - 8) * 4)); // AF7 (USART1)

    USART1->BRR = 16000000 / 9600;
    USART1->CR1 |= USART_CR1_TE | USART_CR1_UE;
}

void USART1_Send(char c) {
    while (!(USART1->SR & USART_SR_TXE));
    USART1->DR = c;
}

void USART1_Send_String(const char *str) {
    while (*str) {
        USART1_Send(*str++);
    }
}

void GPIO_Init(void) {
    RCC->AHB1ENR |= RCC_AHB1ENR_GPIOAEN | RCC_AHB1ENR_GPIOBEN;

    // PA0-7: D0-D7 output
    GPIOA->MODER &= ~(0xFFFF); // Clear mode
    GPIOA->MODER |= 0x5555;    // Set PA0-7 as output

    // PB0: WR, PB1: RD, PB5: READY (input for master)
    GPIOB->MODER &= ~(0x3F | (3 << 10));      // Clear PB0, PB1, PB5
    GPIOB->MODER |= (1 << 0) | (1 << 2);      // PB0, PB1 as output
    // PB5 as input to read READY signal

    // WR, RD default high
    GPIOB->BSRR = GPIO_BSRR_BS0 | GPIO_BSRR_BS1;
}

void Set_Data_Output(void) {
    GPIOA->MODER &= ~(0xFFFF);
    GPIOA->MODER |= 0x5555; // output
}

void Set_Data_Input(void) {
    GPIOA->MODER &= ~(0xFFFF); // input
}

void PPI_Send(char c) {
    Set_Data_Output();
    GPIOA->ODR = (GPIOA->ODR & 0xFFFFFF00) | c;
    delay();
    GPIOB->BSRR = GPIO_BSRR_BR0; // WR low
    delay();
    GPIOB->BSRR = GPIO_BSRR_BS0; // WR high
}

char PPI_Receive(void) {
    Set_Data_Input();
    delay();
    GPIOB->BSRR = GPIO_BSRR_BR1; // RD low
    delay();
    char data = GPIOA->IDR & 0xFF;
    GPIOB->BSRR = GPIO_BSRR_BS1; // RD high
    return data;
}

int main(void) {
    USART1_Init();
    GPIO_Init();

    char c = 'A';

    while (1) {
        // Master sends character
        USART1_Send_String("[Master Send] ");
        USART1_Send(c);
        USART1_Send('\r');
        USART1_Send('\n');

        PPI_Send(c);

        while (!(GPIOB->IDR & GPIO_IDR_ID5)); // Wait for READY signal from slave
        delay();

        char res = PPI_Receive();
        USART1_Send_String("[Master Receive] ");
//        res = c + 32;
        USART1_Send(res);
        USART1_Send('\r');
        USART1_Send('\n');

        c++; // Increment character
        if (c > 'Z') c = 'A'; // Reset to 'A' after 'Z'

        delay();
    }
}
