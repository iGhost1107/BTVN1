/*******************************************************************
 * File         : main.c
 * Mục đích     : Giao tiếp 8255 Mode 0 với STM32F103C6
 *                - Port A: 8 LED
 *                - Port B: 8 LED
 *                Thực hiện nháy Port A, sau đó nháy Port B, lặp lại
 *******************************************************************/

#include "stm32f1xx.h"  // File CMSIS cho STM32F1

/* Định nghĩa các chân điều khiển trên Port B (STM32) */
#define PPI_RD_PIN    (1 << 0)  // PB0
#define PPI_WR_PIN    (1 << 1)  // PB1
#define PPI_A0_PIN    (1 << 2)  // PB2
#define PPI_A1_PIN    (1 << 3)  // PB3
#define PPI_CS_PIN    (1 << 4)  // PB4

/*
 * Delay đơn giản bằng vòng lặp. Có thể điều chỉnh
 * để phù hợp tốc độ mô phỏng.
 */
void Delay(volatile uint32_t nCount)
{
    for(; nCount != 0; nCount--);
}

/*
 * Cấu hình GPIOA, GPIOB:
 *  - PA0..PA7 làm output push-pull (bus dữ liệu 8 bit)
 *  - PB0..PB4 làm output push-pull (điều khiển RD,WR,A0,A1,CS)
 */
void GPIO_Config(void)
{
    /* Bật clock cho GPIOA và GPIOB */
    RCC->APB2ENR |= (RCC_APB2ENR_IOPAEN | RCC_APB2ENR_IOPBEN);

    /* ------------------ Cấu hình PA0 - PA7 ------------------ */
    // Mỗi chân: MODE=0x2 (output 2MHz), CNF=0x0 (push-pull)
    // => CRL = 0x22222222 cho 8 chân
    GPIOA->CRL &= ~0xFFFFFFFF;
    GPIOA->CRL |=  0x22222222;

    /* ------------------ Cấu hình PB0 - PB4 ------------------ */
    // Tương tự, PB0..PB4 là output push-pull, 2MHz
    GPIOB->CRL &= ~(0xFFFFF);        // Clear 5 chân đầu
    GPIOB->CRL |=  (0x22222);        // Đặt mode=0x2 cho PB0..PB4

    // Đưa các chân điều khiển lên mức HIGH (logic nghỉ)
    GPIOB->BSRR = PPI_RD_PIN | PPI_WR_PIN | PPI_A0_PIN | PPI_A1_PIN | PPI_CS_PIN;
}

/*
 * Ghi Control Word vào 8255
 * controlWord = 0x80 => Mode 0, Port A/B/C đều output
 */
void PPI_WriteControlWord(uint8_t controlWord)
{
    // RD = 1 (không đọc)
    GPIOB->BSRR = PPI_RD_PIN;

    // Đưa controlWord lên bus dữ liệu (PA0-PA7)
    GPIOA->ODR = (GPIOA->ODR & 0xFF00) | controlWord;

    // Chọn thanh ghi điều khiển: A1=1, A0=1
    GPIOB->BSRR = (PPI_A0_PIN | PPI_A1_PIN);

    // CS = 0 (chọn 8255)
    GPIOB->BRR = PPI_CS_PIN;

    // Tạo xung WR
    GPIOB->BRR = PPI_WR_PIN;    // WR = 0
    Delay(1000);
    GPIOB->BSRR = PPI_WR_PIN;   // WR = 1

    // Kết thúc
    GPIOB->BSRR = PPI_CS_PIN;   // CS = 1
}

/*
 * Ghi data ra Port A của 8255
 * Port A => A1=0, A0=0
 */
void PPI_WritePortA(uint8_t data)
{
    GPIOB->BSRR = PPI_RD_PIN;   // RD = 1

    // Bus dữ liệu
    GPIOA->ODR = (GPIOA->ODR & 0xFF00) | data;

    // A1=0, A0=0 => Port A
    GPIOB->BRR = (PPI_A0_PIN | PPI_A1_PIN);

    // CS=0
    GPIOB->BRR = PPI_CS_PIN;

    // Xung WR
    GPIOB->BRR = PPI_WR_PIN;
    Delay(1000);
    GPIOB->BSRR = PPI_WR_PIN;

    // Kết thúc
    GPIOB->BSRR = PPI_CS_PIN;
}

/*
 * Ghi data ra Port B của 8255
 * Port B => A1=0, A0=1
 */
void PPI_WritePortB(uint8_t data)
{
    GPIOB->BSRR = PPI_RD_PIN;   // RD=1

    // Bus
    GPIOA->ODR = (GPIOA->ODR & 0xFF00) | data;

    // A1=0, A0=1 => Port B
    GPIOB->BSRR = PPI_A0_PIN;   // A0=1
    GPIOB->BRR  = PPI_A1_PIN;   // A1=0

    // CS=0
    GPIOB->BRR = PPI_CS_PIN;

    // Xung WR
    GPIOB->BRR = PPI_WR_PIN;
    Delay(1000);
    GPIOB->BSRR = PPI_WR_PIN;

    // Kết thúc
    GPIOB->BSRR = PPI_CS_PIN;
}

/*
 * Hàm nháy toàn bộ LED 8 bit (bật / tắt) k lần trên 1 port
 *  (Ví dụ: data=0xFF => bật tất cả; data=0x00 => tắt tất cả)
 */
void BlinkPortA(uint8_t k, uint32_t delayTime)
{
    for(uint8_t i=0; i<k; i++)
    {
        // Bật
        PPI_WritePortA(0xFF);
        Delay(delayTime);
        // Tắt
        PPI_WritePortA(0x00);
        Delay(delayTime);
    }
}

void BlinkPortB(uint8_t k, uint32_t delayTime)
{
    for(uint8_t i=0; i<k; i++)
    {
        // Bật
        PPI_WritePortB(0xFF);
        Delay(delayTime);
        // Tắt
        PPI_WritePortB(0x00);
        Delay(delayTime);
    }
}

int main(void)
{
    // Cấu hình GPIO
    GPIO_Config();

    // Thiết lập 8255: Mode 0, Port A/B/C output
    PPI_WriteControlWord(0x80);

    while(1)
    {
        // Nháy Port A 5 lần
        BlinkPortA(5, 500000);

        // Nháy Port B 5 lần
        BlinkPortB(5, 500000);

        // Lặp lại
    }
}
