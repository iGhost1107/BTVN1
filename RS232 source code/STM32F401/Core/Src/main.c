#include "stm32f4xx.h"  // Include STM32F4 device header

void USART2_Init(void);
char USART2_Read(void);

int main(void) {
    char received_data; // Variable to store received character

    USART2_Init(); // Initialize USART2 for UART communication

    while (1) {
        received_data = USART2_Read(); // Read the received character
        // Add any processing for `received_data` here
    }
}

void USART2_Init(void) {
    // Enable clocks for GPIOA and USART2
    RCC->AHB1ENR |= RCC_AHB1ENR_GPIOAEN;  // Enable GPIOA clock
    RCC->APB1ENR |= RCC_APB1ENR_USART2EN; // Enable USART2 clock

    // Configure PA3 (USART2 RX) as alternate function
    GPIOA->MODER &= ~(3U << (3 * 2)); // Clear mode bits for PA3
    GPIOA->MODER |= (2U << (3 * 2));  // Set alternate function mode for PA3
    GPIOA->AFR[0] |= (7U << (4 * 3)); // Set AF7 (USART2) for PA3

    // USART2 configuration
    USART2->BRR = 0x683;              // Set baud rate to 9600 (assuming 16MHz clock)
    USART2->CR1 |= USART_CR1_RE;     // Enable receiver
    USART2->CR1 |= USART_CR1_UE;     // Enable USART2
}

char USART2_Read(void) {
    while (!(USART2->SR & USART_SR_RXNE)) {} // Wait until data is available to read
    return (char)(USART2->DR);               // Return the received character
}
