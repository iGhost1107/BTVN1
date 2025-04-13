#include "stm32f4xx.h"

void GPIO_Init(void);
void SPI1_Slave_Init(void);
void USART2_Init(void);
void USART2_SendChar(char c);
void USART2_SendString(char *str);
uint8_t SPI1_ReceiveData(void);

int main(void) {
    GPIO_Init();
    SPI1_Slave_Init();
    USART2_Init();

    while (1) {
        char data[100];      // Mảng lưu chuỗi nhận được
        int i = 0;

        // Nhận dữ liệu từ Master cho đến khi gặp ký tự '\0' (kết thúc chuỗi)
        while (1) {
            uint8_t received_byte = SPI1_ReceiveData();  // Nhận dữ liệu từ SPI
            if (received_byte == '\0') {
                break;  // Dừng khi nhận được ký tự kết thúc chuỗi
            }
            data[i++] = received_byte;  // Lưu ký tự vào mảng
            if (i >= sizeof(data) - 1) {  // Đảm bảo không tràn mảng
                break;
            }
        }

        // Null-terminate the string
        data[i] = '\0';

        // Gửi chuỗi qua UART
        USART2_SendString(data);
    }
}

/* ==== Cấu hình GPIO ==== */
void GPIO_Init(void) {
    RCC->AHB1ENR |= RCC_AHB1ENR_GPIOAEN; // Bật clock GPIOA

    // PA4 - NSS (Alternate Function)
    GPIOA->MODER |= (2 << (4 * 2));
    GPIOA->AFR[0] |= (5 << (4 * 4));

    // PA5, PA6, PA7 - SPI1 (Alternate Function 5)
    GPIOA->MODER |= (2 << (5 * 2)) | (2 << (6 * 2)) | (2 << (7 * 2));
    GPIOA->AFR[0] |= (5 << (5 * 4)) | (5 << (6 * 4)) | (5 << (7 * 4));

    // PA2 - TX cho USART2 (chỉ cần cấu hình chân này cho UART)
    GPIOA->MODER |= (2 << (2 * 2));         // PA2 (TX) là Alternate Function
    GPIOA->AFR[0] |= (7 << (2 * 4));        // PA2 (TX) kết nối USART2_AF7
}

/* ==== Cấu hình SPI1 ở chế độ Slave ==== */
void SPI1_Slave_Init(void) {
    RCC->APB2ENR |= RCC_APB2ENR_SPI1EN; // Bật clock SPI1

    SPI1->CR1 |= SPI_CR1_SPE;   // Bật SPI ở chế độ Slave
}

/* ==== Cấu hình USART2 để gửi dữ liệu lên Serial Monitor ==== */
void USART2_Init(void) {
    RCC->AHB1ENR |= RCC_AHB1ENR_GPIOAEN;   // Bật clock GPIOA
    RCC->APB1ENR |= RCC_APB1ENR_USART2EN;  // Bật clock USART2

    // Cấu hình Baud Rate = 115200, 8N1
    USART2->BRR = 16000000 / 115200;        // Tốc độ baud = 115200
    USART2->CR1 |= USART_CR1_TE;            // Bật truyền (Transmitter)
    USART2->CR1 |= USART_CR1_UE;            // Bật USART
}

void USART2_SendChar(char c) {
    while (!(USART2->SR & USART_SR_TXE));  // Đợi bộ đệm TX trống
    USART2->DR = c;                        // Gửi ký tự
}

/* ==== Gửi chuỗi qua UART2 ==== */
void USART2_SendString(char *str) {
    while (*str) {
        USART2_SendChar(*str);  // Gửi từng ký tự trong chuỗi
        str++;
    }
}

/* ==== Nhận dữ liệu từ SPI1 ==== */
uint8_t SPI1_ReceiveData(void) {
    while (!(SPI1->SR & SPI_SR_RXNE));  // Đợi dữ liệu vào bộ đệm
    return SPI1->DR;                     // Lấy dữ liệu ra
}
