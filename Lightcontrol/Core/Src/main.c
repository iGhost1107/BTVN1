#include "stm32f4xx.h"

void ADC_Init(void);
void PWM_Init(void);

uint16_t Read_ADC(void);
void Set_PWM_Duty(uint16_t duty);

int main(void) {
ADC_Init(); // Khởi tạo ADC
PWM_Init(); // Khởi tạo PWM trên PA2 (A2)

while (1) {
uint16_t adc_value = Read_ADC(); // Đọc giá trị ADC (0-4095)
uint16_t pwm_duty = 999 - (adc_value * 999) / 4095; // Chuyển đổi sang duty cycle 0-100%
Set_PWM_Duty(pwm_duty);
}
}

/* Khởi tạo ADC trên PA0 */
void ADC_Init(void) {
RCC->AHB1ENR |= RCC_AHB1ENR_GPIOAEN; // Bật clock GPIOA
RCC->APB2ENR |= RCC_APB2ENR_ADC1EN; // Bật clock ADC1
GPIOA->MODER |= (3 << (0 * 2)); // PA0 ở chế độ analog

ADC1->CR1 = 0; // Cấu hình mặc định
ADC1->CR2 = ADC_CR2_ADON; // Bật ADC1
ADC1->SQR3 = 0; // Chọn kênh 0 (PA0)
ADC1->SMPR2 |= (7 << 0); // Thời gian lấy mẫu dài hơn (480 cycles)
ADC1->CR2 |= ADC_CR2_CONT; // Chế độ liên tục
}

/* Khởi tạo PWM trên PA2 (TIM2_CH3) */
void PWM_Init(void) {
RCC->AHB1ENR |= RCC_AHB1ENR_GPIOAEN; // Bật clock GPIOA
RCC->APB1ENR |= RCC_APB1ENR_TIM2EN; // Bật clock TIM2

GPIOA->MODER |= (2 << (2 * 2)); // PA2 ở chế độ Alternate Function
GPIOA->AFR[0] |= (1 << (2 * 4)); // PA2 sử dụng AF1 (TIM2_CH3)

TIM2->PSC = 84 - 1; // Chia tần số (PSC+1) -> 84MHz / 84 = 1MHz
TIM2->ARR = 999; // Định kỳ PWM 1ms -> Tần số PWM 1kHz
TIM2->CCR3 = 0; // Duty cycle ban đầu = 0%

TIM2->CCMR2 |= (6 << 4); // PWM mode 1
TIM2->CCMR2 |= TIM_CCMR2_OC3PE; // Cho phép preload
TIM2->CCER |= TIM_CCER_CC3E; // Kích hoạt PWM trên CH3
TIM2->CR1 |= TIM_CR1_CEN; // Bật Timer 2
}

/* Đọc giá trị ADC từ PA0 */
uint16_t Read_ADC(void) {
ADC1->CR2 |= ADC_CR2_SWSTART; // Bắt đầu chuyển đổi ADC
while (!(ADC1->SR & ADC_SR_EOC)); // Chờ hoàn thành
return ADC1->DR; // Trả về giá trị ADC
}

/* Thiết lập độ rộng xung PWM */
void Set_PWM_Duty(uint16_t duty) {
TIM2->CCR3 = duty; // Cập nhật duty cycle
}


#include "stm32f4xx.h"

void ADC_Init(void);
void PWM_Init(void);

uint16_t Read_ADC(void);
void Set_PWM_Duty(uint16_t duty);

int main(void) {
ADC_Init(); // Khởi tạo ADC
PWM_Init(); // Khởi tạo PWM trên PA2 (A2)

while (1) {
uint16_t adc_value = Read_ADC(); // Đọc giá trị ADC (0-4095)
uint16_t pwm_duty = 999 - (adc_value * 999) / 4095; // Chuyển đổi sang duty cycle 0-100%
Set_PWM_Duty(pwm_duty);
}
}

/* Khởi tạo ADC trên PA0 */
void ADC_Init(void) {
RCC->AHB1ENR |= RCC_AHB1ENR_GPIOAEN; // Bật clock GPIOA
RCC->APB2ENR |= RCC_APB2ENR_ADC1EN; // Bật clock ADC1
GPIOA->MODER |= (3 << (0 * 2)); // PA0 ở chế độ analog

ADC1->CR1 = 0; // Cấu hình mặc định
ADC1->CR2 = ADC_CR2_ADON; // Bật ADC1
ADC1->SQR3 = 0; // Chọn kênh 0 (PA0)
ADC1->SMPR2 |= (7 << 0); // Thời gian lấy mẫu dài hơn (480 cycles)
ADC1->CR2 |= ADC_CR2_CONT; // Chế độ liên tục
}

/* Khởi tạo PWM trên PA2 (TIM2_CH3) */
void PWM_Init(void) {
RCC->AHB1ENR |= RCC_AHB1ENR_GPIOAEN; // Bật clock GPIOA
RCC->APB1ENR |= RCC_APB1ENR_TIM2EN; // Bật clock TIM2

GPIOA->MODER |= (2 << (2 * 2)); // PA2 ở chế độ Alternate Function
GPIOA->AFR[0] |= (1 << (2 * 4)); // PA2 sử dụng AF1 (TIM2_CH3)

TIM2->PSC = 84 - 1; // Chia tần số (PSC+1) -> 84MHz / 84 = 1MHz
TIM2->ARR = 999; // Định kỳ PWM 1ms -> Tần số PWM 1kHz
TIM2->CCR3 = 0; // Duty cycle ban đầu = 0%

TIM2->CCMR2 |= (6 << 4); // PWM mode 1
TIM2->CCMR2 |= TIM_CCMR2_OC3PE; // Cho phép preload
TIM2->CCER |= TIM_CCER_CC3E; // Kích hoạt PWM trên CH3
TIM2->CR1 |= TIM_CR1_CEN; // Bật Timer 2
}

/* Đọc giá trị ADC từ PA0 */
uint16_t Read_ADC(void) {
ADC1->CR2 |= ADC_CR2_SWSTART; // Bắt đầu chuyển đổi ADC
while (!(ADC1->SR & ADC_SR_EOC)); // Chờ hoàn thành
return ADC1->DR; // Trả về giá trị ADC
}

/* Thiết lập độ rộng xung PWM */
void Set_PWM_Duty(uint16_t duty) {
TIM2->CCR3 = duty; // Cập nhật duty cycle
}
