#include "stm32f4xx.h"

#define BUTTON_PIN (1 << 0)   // PA0 - Nút nhấn
#define LED_PIN (1 << 5)      // PA5 - LED (nếu cần debug)
#define NSS_PIN (1 << 4)      // PA4 - Slave Select (NSS)

void GPIO_Init(void);
void SPI1_Master_Init(void);
void SPI1_SendData(uint8_t data);
void SPI1_SendString(const char *str);  // Hàm gửi chuỗi
uint8_t Read_Button(void);

int main(void) {
    GPIO_Init();
    SPI1_Master_Init();

    while (1) {
        const char *data = Read_Button() ? "Hello\n" : "Bye\n";  // Nhấn gửi "Hello\n", không nhấn gửi "Bye\n"

        GPIOA->BSRR = NSS_PIN << 16; // Kéo NSS xuống (chọn Slave)
        SPI1_SendString(data);       // Gửi chuỗi
        GPIOA->BSRR = NSS_PIN;       // Kéo NSS lên (bỏ chọn Slave)

        for (volatile int i = 0; i < 100000; i++); // Delay tránh gửi quá nhanh
    }
}

/* ==== Cấu hình GPIO ==== */
void GPIO_Init(void) {
    RCC->AHB1ENR |= RCC_AHB1ENR_GPIOAEN; // Bật clock GPIOA

    // PA0 - Nút nhấn (Input, Pull-up)
    GPIOA->MODER &= ~(3 << (0 * 2));
    GPIOA->PUPDR |= (1 << (0 * 2));

    // PA4 - NSS (Output)
    GPIOA->MODER |= (1 << (4 * 2));
    GPIOA->BSRR = NSS_PIN; // NSS mặc định = 1 (không chọn Slave)

    // PA5, PA6, PA7 - SPI1 (Alternate Function 5)
    GPIOA->MODER |= (2 << (5 * 2)) | (2 << (6 * 2)) | (2 << (7 * 2));
    GPIOA->AFR[0] |= (5 << (5 * 4)) | (5 << (6 * 4)) | (5 << (7 * 4));
}

/* ==== Đọc trạng thái nút nhấn ==== */
uint8_t Read_Button(void) {
    return (GPIOA->IDR & BUTTON_PIN) ? 0 : 1; // Nhấn trả về 1, không nhấn trả về 0
}

/* ==== Cấu hình SPI1 ở chế độ Master ==== */
void SPI1_Master_Init(void) {
    RCC->APB2ENR |= RCC_APB2ENR_SPI1EN; // Bật clock SPI1

    SPI1->CR1 |= SPI_CR1_MSTR;  // Chế độ Master
    SPI1->CR1 |= SPI_CR1_BR_1;  // Baud rate = fPCLK/8
    SPI1->CR1 |= SPI_CR1_SSM | SPI_CR1_SSI; // Điều khiển NSS bằng phần mềm
    SPI1->CR1 |= SPI_CR1_SPE;   // Bật SPI
}

/* ==== Gửi dữ liệu SPI ==== */
void SPI1_SendData(uint8_t data) {
    while (!(SPI1->SR & SPI_SR_TXE)); // Chờ bộ đệm TX trống
    SPI1->DR = data;
    while (!(SPI1->SR & SPI_SR_RXNE)); // Chờ dữ liệu được gửi xong
    (void)SPI1->DR; // Đọc DR để xóa cờ RXNE
}

/* ==== Gửi chuỗi qua SPI ==== */
void SPI1_SendString(const char *str) {
    while (*str) {              // Lặp qua từng ký tự trong chuỗi
        SPI1_SendData(*str);     // Gửi từng ký tự qua SPI
        str++;                   // Tăng con trỏ để gửi ký tự tiếp theo
    }
}
