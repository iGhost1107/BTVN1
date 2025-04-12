#include "stm32f4xx.h"
#include <stdint.h>
#include <stdbool.h>

// --- Định nghĩa chân (Tất cả trên GPIOA) ---
#define PPI_PORT         GPIOA   // Tất cả tín hiệu PPI và Handshake trên GPIOA

// Chân dữ liệu (D0-D7 -> PA0-PA7)
#define PPI_DATA_MASK    0x00FF  // Mặt nạ cho PA0-PA7
#define PPI_DATA_OFFSET  0       // Dữ liệu bắt đầu từ chân 0

// Chân điều khiển PPI trên Port A
#define PPI_RD_PIN       8       // PA8  -> /RD
#define PPI_WR_PIN       9       // PA9  -> /WR
#define PPI_A0_PIN       10      // PA10 -> A0
#define PPI_A1_PIN       11      // PA11 -> A1
#define PPI_CS_PIN       13      // PA13 -> /CS (PA12 không dùng)

// Chân STM32 tạo/đọc Handshake (kết nối với Port C của 8255)
#define SIM_STB_B_PIN    12      // Output -> PA12 -> 8255 PC2 (STB_B input for 8255)
#define READ_IBF_B_PIN   14      // Input  <- PA14 <- 8255 PC1 (IBF_B output from 8255)

void delay_ms(volatile uint32_t ms) {
    uint32_t iterations = ms * 8000;
    for (uint32_t i = 0; i < iterations; i++) {
        __NOP(); // Lệnh không làm gì, dùng để tạo trễ
    }
}

// --- Hàm tạo trễ ngắn ---
__STATIC_INLINE void short_delay() {
    // Trễ rất ngắn, dùng giữa các thao tác tín hiệu điều khiển
    for(volatile int i=0; i<10; i++) {__NOP();}
}

// --- Cấu hình Hướng Dữ liệu cho Chân Dữ liệu GPIOA (PA0-PA7) ---
void PPI_SetDataDirection(bool is_input) {
    // Xóa cấu hình mode hiện tại của PA0-PA7
    PPI_PORT->MODER &= ~(0xFFFF << (PPI_DATA_OFFSET * 2)); // Mask for PA0-PA7 MODER bits

    if (is_input) {
        // PA0-PA7 là Input (00) - Không cần làm gì thêm sau khi xóa
    } else {
        // PA0-PA7 là Output (01)
        PPI_PORT->MODER |= (0x5555 << (PPI_DATA_OFFSET * 2)); // 0101... for PA0-PA7
    }
    short_delay(); // Chờ một chút để hướng ổn định (có thể không cần thiết)
}

// --- Cấu hình GPIO ---
void GPIO_Config() {
    // 1. Cấp clock cho GPIOA
    RCC->AHB1ENR |= RCC_AHB1ENR_GPIOAEN;

    // Chờ clock ổn định (không bắt buộc nhưng nên có)
    volatile uint32_t dummy_read = RCC->AHB1ENR;
    (void)dummy_read; // Tránh cảnh báo unused variable

    // 2. Cấu hình Chân Điều khiển (PA8, 9, 10, 11, 13) -> Output
    uint32_t ctrl_mode = (GPIO_MODER_MODER8_0)  | (GPIO_MODER_MODER9_0) |
                         (GPIO_MODER_MODER10_0) | (GPIO_MODER_MODER11_0)|
                         (GPIO_MODER_MODER13_0);
    uint32_t ctrl_mask = (0b11 << (PPI_RD_PIN * 2)) | (0b11 << (PPI_WR_PIN * 2)) |
                         (0b11 << (PPI_A0_PIN * 2)) | (0b11 << (PPI_A1_PIN * 2)) |
                         (0b11 << (PPI_CS_PIN * 2));

    PPI_PORT->MODER &= ~ctrl_mask; // Xóa mode cũ
    PPI_PORT->MODER |= ctrl_mode;  // Đặt là Output

    // Đặt tốc độ Medium cho chân điều khiển
    uint32_t ctrl_speed = (GPIO_OSPEEDER_OSPEEDR8_0)  | (GPIO_OSPEEDER_OSPEEDR9_0) |
                          (GPIO_OSPEEDER_OSPEEDR10_0) | (GPIO_OSPEEDER_OSPEEDR11_0)|
                          (GPIO_OSPEEDER_OSPEEDR13_0);
    PPI_PORT->OSPEEDR &= ~ctrl_mask; // Xóa tốc độ cũ
    PPI_PORT->OSPEEDR |= ctrl_speed; // Đặt tốc độ Medium (01)

    // Đặt trạng thái ban đầu không tích cực (High) cho /CS, /RD, /WR
    PPI_PORT->BSRR = (1 << PPI_CS_PIN) | (1 << PPI_RD_PIN) | (1 << PPI_WR_PIN);

    // 3. Cấu hình Chân Dữ liệu (PA0-PA7) -> Ban đầu là Output
    // Tốc độ Medium cho chân dữ liệu
    PPI_PORT->OSPEEDR &= ~(0xFFFF << (PPI_DATA_OFFSET * 2)); // Xóa tốc độ cũ PA0-7
    PPI_PORT->OSPEEDR |= (0x5555 << (PPI_DATA_OFFSET * 2));  // Đặt tốc độ Medium PA0-7
    PPI_SetDataDirection(false); // Đặt PA0-PA7 là Output

    // 4. Cấu hình Chân Handshake STM32
    // PA15 (SIM_STB_B) -> Output
    PPI_PORT->MODER &= ~(0b11 << (SIM_STB_B_PIN * 2)); // Xóa mode cũ PA15
    PPI_PORT->MODER |= (GPIO_MODER_MODER15_0);         // Đặt PA15 là Output (01)
    PPI_PORT->OSPEEDR &= ~(0b11 << (SIM_STB_B_PIN * 2)); // Xóa tốc độ cũ PA15
    PPI_PORT->OSPEEDR |= (GPIO_OSPEEDER_OSPEEDR15_0);    // Đặt tốc độ Medium PA15 (01)
    PPI_PORT->BSRR = (1 << SIM_STB_B_PIN);             // Đặt STB_B ban đầu là High

    // PA14 (READ_IBF_B) -> Input
    PPI_PORT->MODER &= ~(0b11 << (READ_IBF_B_PIN * 2)); // Đặt PA14 là Input (00) - Xóa là đủ
    // Cấu hình Pull-down cho PA14 (IBF_B thường là active high, nên chờ nó lên cao)
    // Nếu không có điện trở kéo ngoài, có thể cần pull-down để tránh trạng thái lơ lửng khi 8255 chưa tích cực.
    PPI_PORT->PUPDR &= ~(0b11 << (READ_IBF_B_PIN * 2)); // Xóa cấu hình pull cũ
    PPI_PORT->PUPDR |= (GPIO_PUPDR_PUPDR14_1);        // Bật Pull-down cho PA14 (10)
                                                    // Nếu IBF_B là active low, dùng Pull-up.
                                                    // Dựa trên code gốc chờ IBF_B lên cao -> IBF_B là Active High -> Pull-down là hợp lý.
}

// --- Ghi Control Word vào 8255 ---
void PPI_WriteControl(uint8_t controlWord) {
    PPI_SetDataDirection(false); // PA0-PA7 Output

    // Chọn Control Register (A1=1, A0=1) và kích hoạt chip (/CS=0)
    PPI_PORT->BSRR = (1 << (PPI_CS_PIN + 16)) | // CS = 0 (Set bit 13+16)
                     (1 << PPI_A1_PIN)        | // A1 = 1 (Set bit 11)
                     (1 << PPI_A0_PIN);         // A0 = 1 (Set bit 10)
    short_delay();

    // Đặt dữ liệu lên bus (PA0-PA7)
    // Ghi an toàn: chỉ thay đổi PA0-PA7
    PPI_PORT->ODR = (PPI_PORT->ODR & ~(PPI_DATA_MASK << PPI_DATA_OFFSET)) | (controlWord << PPI_DATA_OFFSET);
    short_delay();

    // Tạo xung ghi (/WR = 0 rồi /WR = 1)
    PPI_PORT->BSRR = (1 << (PPI_WR_PIN + 16)); // WR = 0 (Set bit 9+16)
    short_delay(); short_delay(); // Độ rộng xung WR
    PPI_PORT->BSRR = (1 << PPI_WR_PIN);       // WR = 1 (Set bit 9)
    short_delay();

    // Hủy kích hoạt chip (/CS=1) và bỏ chọn địa chỉ (có thể không cần thiết nhưng an toàn)
    PPI_PORT->BSRR = (1 << PPI_CS_PIN) |        // CS = 1
                     (1 << (PPI_A1_PIN + 16)) | // A1 = 0 (optional)
                     (1 << (PPI_A0_PIN + 16));  // A0 = 0 (optional)
    short_delay();
}

// --- Ghi dữ liệu vào Port A của 8255 (Mode 0 Output) ---
void PPI_WritePortA(uint8_t data) {
    PPI_SetDataDirection(false); // PA0-PA7 Output

    // Chọn Port A (A1=0, A0=0) và kích hoạt chip (/CS=0)
    PPI_PORT->BSRR = (1 << (PPI_CS_PIN + 16)) | // CS = 0
                     (1 << (PPI_A1_PIN + 16)) | // A1 = 0
                     (1 << (PPI_A0_PIN + 16));  // A0 = 0
    short_delay();

    // Đặt dữ liệu lên bus (PA0-PA7)
    PPI_PORT->ODR = (PPI_PORT->ODR & ~(PPI_DATA_MASK << PPI_DATA_OFFSET)) | (data << PPI_DATA_OFFSET);
    short_delay();

    // Tạo xung ghi (/WR = 0 rồi /WR = 1)
    PPI_PORT->BSRR = (1 << (PPI_WR_PIN + 16)); // WR = 0
    short_delay(); short_delay();
    PPI_PORT->BSRR = (1 << PPI_WR_PIN);       // WR = 1
    short_delay();

    // Hủy kích hoạt chip (/CS=1)
    PPI_PORT->BSRR = (1 << PPI_CS_PIN);
    short_delay();
}

// --- Đọc dữ liệu từ một Port (A, B, hoặc C) ---
uint8_t PPI_ReadPort(uint8_t port) {
    uint8_t data = 0xFF;        // Giá trị lỗi mặc định
    PPI_SetDataDirection(true); // ****** PA0-PA7 Input ******

    // Kích hoạt chip (/CS=0)
    PPI_PORT->BSRR = (1 << (PPI_CS_PIN + 16)); // CS = 0
    short_delay();

    // Chọn Port dựa trên A1, A0
    if (port == 'A') { // A1=0, A0=0
        PPI_PORT->BSRR = (1 << (PPI_A1_PIN + 16)) | (1 << (PPI_A0_PIN + 16));
    } else if (port == 'B') { // A1=0, A0=1
        PPI_PORT->BSRR = (1 << (PPI_A1_PIN + 16)) | (1 << PPI_A0_PIN);
    } else if (port == 'C') { // A1=1, A0=0
        PPI_PORT->BSRR = (1 << PPI_A1_PIN) | (1 << (PPI_A0_PIN + 16));
    } else {
        PPI_PORT->BSRR = (1 << PPI_CS_PIN); // Lỗi port -> Hủy CS
        PPI_SetDataDirection(false);       // Trả PA0-PA7 về Output
        return data; // Trả về lỗi
    }
    short_delay();

    // Tạo xung đọc (/RD = 0 rồi /RD = 1)
    PPI_PORT->BSRR = (1 << (PPI_RD_PIN + 16)); // RD = 0
    short_delay(); short_delay(); // Đợi Access Time của 8255 + Setup time cho STM32
    data = (uint8_t)((PPI_PORT->IDR >> PPI_DATA_OFFSET) & PPI_DATA_MASK); // Đọc PA0-PA7
    PPI_PORT->BSRR = (1 << PPI_RD_PIN);       // RD = 1
    short_delay(); // Giữ CS thấp một chút sau RD (Hold time)

    // Hủy kích hoạt chip (/CS=1)
    PPI_PORT->BSRR = (1 << PPI_CS_PIN);
    short_delay();

    PPI_SetDataDirection(false); // ****** Trả PA0-PA7 về Output ******
    return data;
}


// --- Tạo xung STB_B (trên PA15 nối vào 8255 PC2) ---
void PPI_Generate_STB_B() {
    PPI_PORT->BSRR = (1 << (SIM_STB_B_PIN + 16)); // STB = 0 (Active Low Pulse)
    short_delay(); short_delay();                // Độ rộng xung STB
    PPI_PORT->BSRR = (1 << SIM_STB_B_PIN);       // STB = 1
    short_delay();
}

// --- Chờ IBF_B (đọc qua PA14 <- 8255 PC1) lên cao ---
bool PPI_Wait_IBF_B_High() {
    // Cần thêm cơ chế timeout phức tạp hơn cho ứng dụng thực tế
    volatile uint32_t timeout = 200000; // Tăng timeout một chút
    // Chờ đến khi bit PA14 đọc được là 1 HOẶC hết timeout
    while (((PPI_PORT->IDR & (1 << READ_IBF_B_PIN)) == 0) && (timeout > 0)) {
         __NOP();
         timeout--;
    }
    // Trả về true nếu không hết timeout (IBF_B đã lên cao)
    return (timeout > 0);
}

// --- Khởi tạo 8255 ---
// Cấu hình: Port A = Mode 0 Output, Port B = Mode 1 Input
// Control word = 1000 1011 = 0x8B
// Bit 7=1: Mode Set flag
// Bit 6,5=00: Port A Mode 0
// Bit 4=0: Port A Output
// Bit 3=0: Port C Upper (PC4-7) Output
// Bit 2=1: Port B Mode 1
// Bit 1=1: Port B Input
// Bit 0=1: Port C Lower (PC0-3) Input (PC1, PC2 được dùng làm handshake IBF_B, STB_B)
void PPI_Init_Mode0A_Out_Mode1B_In() {
    PPI_WriteControl(0x8B); // Gửi Control Word cấu hình 8255
    // Ở Mode 1 Input, IBF (PC1) và INTR (PC0 cho Port B) được điều khiển bởi 8255.
    // STB (PC2) là input cho 8255.
    // INTE không được set trong control word này, nên INTR sẽ không hoạt động.
}


// --- Chương trình chính ---
int main() {
    uint8_t button_state = 0;
    uint8_t led_state = 0;

    // 1. Cấu hình hệ thống cơ bản (Clock, v.v... - thường được gọi trong SystemInit)
    // SystemInit(); // Đảm bảo hàm này được gọi (thường tự động bởi startup code)

    // 2. Cấu hình chân GPIO của STM32
    GPIO_Config();

    // 3. Cấu hình chip 8255
    PPI_Init_Mode0A_Out_Mode1B_In();

    while (1) {
        // --- Giai đoạn đọc nút nhấn từ 8255 Port B (Mode 1 Input) ---

        // 1. Tạo xung STB_B (trên PA15) để yêu cầu 8255 chốt dữ liệu Port B
        PPI_Generate_STB_B();
        // Khi STB_B xuống thấp, 8255 sẽ đọc dữ liệu từ các chân Port B của nó.

        // 2. Chờ 8255 xác nhận đã chốt xong dữ liệu bằng cách đưa IBF_B (PC1) lên cao.
        // STM32 đọc tín hiệu này qua PA14.
        if (PPI_Wait_IBF_B_High()) {
            // IBF_B đã lên cao, dữ liệu đã sẵn sàng trong bộ đệm của 8255 Port B.

            // 3. Đọc giá trị đã được 8255 chốt từ Port B.
            // STM32 thực hiện chu trình đọc từ địa chỉ Port B.
            // Quan trọng: Hành động đọc (/RD xuống thấp) này sẽ tự động xóa cờ IBF_B (kéo PC1 xuống thấp).
            button_state = PPI_ReadPort('B');

            // --- Giai đoạn xử lý và điều khiển LED qua 8255 Port A (Mode 0 Output) ---

            // 4. Xử lý trạng thái nút nhấn đọc được.
            // Giả sử nút nhấn nối với GND -> nhấn là 0, không nhấn là 1.
            // Đảo bit để LED sáng khi nhấn, và chỉ lấy 3 bit thấp (giả sử 3 LED nối PA0-PA2).
            led_state = (~button_state) & 0x07; // Lấy PA0, PA1, PA2

            // 5. Ghi trạng thái LED mới ra Port A của 8255.
            // STM32 thực hiện chu trình ghi đến địa chỉ Port A.
            PPI_WritePortA(led_state);

        } else {
             // Xử lý lỗi timeout: Không nhận được tín hiệu IBF_B từ 8255.
             // Có thể do lỗi kết nối, lỗi cấu hình 8255, hoặc 8255 không hoạt động.
             // Ví dụ: Nhấp nháy tất cả 3 LED để báo lỗi
             for(int i=0; i<3; i++) {
                 PPI_WritePortA(0x07); // Bật 3 LED
                 delay_ms(100);
                 PPI_WritePortA(0x00); // Tắt 3 LED
                 delay_ms(100);
             }
        }

        // Delay ngắn giữa các lần lặp để giảm tải CPU và tránh đọc quá nhanh
        delay_ms(50);
    }
    // return 0; // main không bao giờ thoát trong ứng dụng nhúng
}
