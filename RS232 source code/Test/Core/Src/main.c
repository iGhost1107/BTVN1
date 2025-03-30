#include "stm32f1xx.h"  // Include STM32F1 device header
void USART2_Init(void);
void USART2_Write(char data);
void Delay_ms(uint32_t ms);
const char* message = "Never give up!"; // Define the message to be sent
int main(void) {
   USART2_Init(); // Initialize USART2 for UART communication
   while (1) {
       for (int i = 0; message[i] != '\0'; i++) {
           USART2_Write(message[i]); // Send each character
           Delay_ms(500);            // Wait for 500 milliseconds
       }
   }
}
void USART2_Init(void) {
   // Enable clocks for GPIOA and USART2
   RCC->APB2ENR |= RCC_APB2ENR_IOPAEN;   // Enable GPIOA clock
   RCC->APB1ENR |= RCC_APB1ENR_USART2EN; // Enable USART2 clock
   // Configure PA2 (USART2 TX) as alternate function push-pull
   GPIOA->CRL &= ~(0xF << (2 * 4)); // Clear configuration for PA2
   GPIOA->CRL |= (0xB << (2 * 4));  // Set PA2 as Alternate Function Push-Pull
   // Configure USART2 settings
   USART2->BRR = 0x1D4C;            // Set baud rate to 9600 (assuming 72 MHz clock)
   USART2->CR1 |= USART_CR1_TE;     // Enable transmitter
   USART2->CR1 |= USART_CR1_UE;     // Enable USART2
}
void USART2_Write(char data) {
   while (!(USART2->SR & USART_SR_TXE)) {} // Wait until transmit register is empty
   USART2->DR = (uint8_t)data;             // Send character
}
void Delay_ms(uint32_t ms) {
   // Simple delay loop (assuming 72 MHz system clock)
   for (uint32_t i = 0; i < ms * 7200; i++) {
       __NOP(); // Perform a no-operation (waste time)
   }
}

