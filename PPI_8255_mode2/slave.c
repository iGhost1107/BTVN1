#include "stm32f4xx.h"

// Function prototypes
void GPIO_Config(void);
void UART_Config(void);
void delay(void);
char PPI_Receive(void);
void PPI_Send(char c);
void send_char(char c);
void send_ready_signal(void);
void set_PA0_7_as_input(void);
void set_PA0_7_as_output(void);
void send_char_string(const char *str);

int main(void) {
    GPIO_Config();
    UART_Config();

    char recv, send;
    while (1) {
        recv = PPI_Receive();       // Receive from Master

        // Convert uppercase to lowercase
        send = (recv >= 'A' && recv <= 'Z') ? (recv)+ 32 : recv;

        // Slave debug output
        send_char_string("[Slave Receive] ");
        send_char(recv);
        send_char('\r');
        send_char('\n');

        send_char_string("[Slave Send] ");
        send_char(send);
        send_char('\r');
        send_char('\n');

//        delay();
        PPI_Send(send);             // Send transformed character back to Master
        send_ready_signal();        // Signal readiness to Master
        delay();

        // Reset READY signal
        GPIOB->BSRR = GPIO_BSRR_BR5; // PB5 LOW - Reset READY
    }
}

void GPIO_Config(void) {
    RCC->AHB1ENR |= RCC_AHB1ENR_GPIOAEN | RCC_AHB1ENR_GPIOBEN;

    // PB0 (WR), PB1 (RD): output
    GPIOB->MODER &= ~((3 << (0 * 2)) | (3 << (1 * 2)));
    GPIOB->MODER |= (1 << (0 * 2)) | (1 << (1 * 2));

    // PB5: output (READY signal)
    GPIOB->MODER &= ~(3 << (5 * 2));
    GPIOB->MODER |= (1 << (5 * 2));

    // Initialize PA0-7 as input to receive data
    set_PA0_7_as_input();

    // Default states for control lines
    GPIOB->BSRR = GPIO_BSRR_BS0 | GPIO_BSRR_BS1; // WR and RD high
    GPIOB->BSRR = GPIO_BSRR_BR5; // READY low initially
}

void UART_Config(void) {
    RCC->APB2ENR |= RCC_APB2ENR_USART1EN;
    RCC->AHB1ENR |= RCC_AHB1ENR_GPIOAEN;

    // PA9 (TX)
    GPIOA->MODER &= ~(3 << (9 * 2));
    GPIOA->MODER |= (2 << (9 * 2));  // AF mode
    GPIOA->AFR[1] &= ~(0xF << ((9 - 8) * 4));
    GPIOA->AFR[1] |= (7 << ((9 - 8) * 4));  // AF7 for USART1

    USART1->BRR = 16000000 / 9600; // Baudrate 9600 @16MHz
    USART1->CR1 |= USART_CR1_TE | USART_CR1_UE; // Corrected to USART_CR1_UE
}

void delay(void) {
    for (volatile int i = 0; i < 300000; i++);
}

void send_char(char c) {
    while (!(USART1->SR & USART_SR_TXE));
    USART1->DR = c;
}

void send_char_string(const char *str) {
    while (*str) {
        send_char(*str++);
    }
}

void set_PA0_7_as_input(void) {
    GPIOA->MODER &= ~(0xFFFF); // Clear PA0-7 (input)
}

void set_PA0_7_as_output(void) {
    GPIOA->MODER &= ~(0xFFFF); // Clear first
    GPIOA->MODER |= 0x5555;    // Output mode
}

char PPI_Receive(void) {
    set_PA0_7_as_input();
    delay();

    GPIOB->BSRR = GPIO_BSRR_BR1; // RD low
    delay();

    char val = GPIOA->IDR & 0xFF;

    GPIOB->BSRR = GPIO_BSRR_BS1; // RD high
    delay();

    return val;
}

void PPI_Send(char c) {
    set_PA0_7_as_output();
    delay();  // Thêm delay để đảm bảo cấu hình mode ổn định

    // Ghi dữ liệu vào PA0-7
    GPIOA->ODR = (GPIOA->ODR & 0xFFFFFF00) | (uint8_t)c;
    delay();

    // Tín hiệu ghi (WR) xuống LOW rồi lên HIGH để kích hoạt ghi
    GPIOB->BSRR = GPIO_BSRR_BR0; // WR low
    delay();
    GPIOB->BSRR = GPIO_BSRR_BS0; // WR high
    delay();

    // Xoá dữ liệu khỏi ODR để tránh nhiễu
    GPIOA->ODR &= 0xFFFFFF00;
    delay();

    set_PA0_7_as_input(); // Trở lại chế độ input sau khi gửi
    delay();
}

void send_ready_signal(void) {
    GPIOB->BSRR = GPIO_BSRR_BS5; // PB5 HIGH - Signal ready
}
