#include "stm32f4xx_hal.h"

#define SLA_ADDRESS 0x08

I2C_HandleTypeDef hi2c1;
char receiveBuffer[20];
uint8_t receiveIndex = 0;
char replyBuffer[] = "Hi Master";
uint8_t replyIndex = 0;
uint8_t transmissionComplete = 0;

void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_I2C1_Init(void);

int main(void)
{
  HAL_Init();
  SystemClock_Config();
  MX_GPIO_Init();
  MX_I2C1_Init();

  // Kích hoạt chế độ interrupt cho I2C
  HAL_I2C_EnableListen_IT(&hi2c1);

  while (1)
  {
    // STM32 xử lý I2C qua ngắt, không cần code trong vòng lặp chính
    HAL_Delay(100);
  }
}

void MX_I2C1_Init(void)
{
  hi2c1.Instance = I2C1;
  hi2c1.Init.ClockSpeed = 100000;
  hi2c1.Init.DutyCycle = I2C_DUTYCYCLE_2;
  hi2c1.Init.OwnAddress1 = SLA_ADDRESS << 1;
  hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c1.Init.OwnAddress2 = 0;
  hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;

  if (HAL_I2C_Init(&hi2c1) != HAL_OK)
  {
    Error_Handler();
  }
}

// Callback khi địa chỉ slave phù hợp được gọi bởi master
void HAL_I2C_AddrCallback(I2C_HandleTypeDef *hi2c, uint8_t TransferDirection, uint16_t AddrMatchCode)
{
  if (TransferDirection == I2C_DIRECTION_RECEIVE) // Master muốn gửi dữ liệu
  {
    receiveIndex = 0;
    HAL_I2C_Slave_Receive_IT(&hi2c1, (uint8_t*)&receiveBuffer[receiveIndex], 1);
  }
  else // Master muốn nhận dữ liệu
  {
    replyIndex = 0;
    HAL_I2C_Slave_Transmit_IT(&hi2c1, (uint8_t*)&replyBuffer[replyIndex], 1);
  }
}

// Callback khi nhận dữ liệu
void HAL_I2C_SlaveRxCpltCallback(I2C_HandleTypeDef *hi2c)
{
  receiveIndex++;

  if (receiveBuffer[receiveIndex-1] == '\0' || receiveIndex >= sizeof(receiveBuffer)-1)
  {
    // Chuỗi đã nhận xong
    receiveBuffer[receiveIndex] = '\0';
    // In ra debug console nếu có
    // printf("Slave nhận từ Master: %s\n", receiveBuffer);

    // Kích hoạt lại chế độ lắng nghe cho lần tiếp theo
    HAL_I2C_EnableListen_IT(&hi2c1);
  }
  else
  {
    // Tiếp tục nhận byte tiếp theo
    HAL_I2C_Slave_Receive_IT(&hi2c1, (uint8_t*)&receiveBuffer[receiveIndex], 1);
  }
}

// Callback khi gửi dữ liệu
void HAL_I2C_SlaveTxCpltCallback(I2C_HandleTypeDef *hi2c)
{
  replyIndex++;

  if (replyBuffer[replyIndex-1] == '\0' || replyIndex >= strlen(replyBuffer))
  {
    // Đã gửi toàn bộ chuỗi phản hồi
    // printf("Slave: Đã gửi -> Hi Master\n");

    // Kích hoạt lại chế độ lắng nghe cho lần tiếp theo
    HAL_I2C_EnableListen_IT(&hi2c1);
  }
  else
  {
    // Tiếp tục gửi byte tiếp theo
    HAL_I2C_Slave_Transmit_IT(&hi2c1, (uint8_t*)&replyBuffer[replyIndex], 1);
  }
}

// Callback khi ngắt I2C xảy ra lỗi
void HAL_I2C_ErrorCallback(I2C_HandleTypeDef *hi2c)
{
  // Xử lý lỗi I2C
  // Có thể reset lại I2C và kích hoạt lại chế độ lắng nghe
  HAL_I2C_DeInit(&hi2c1);
  HAL_I2C_Init(&hi2c1);
  HAL_I2C_EnableListen_IT(&hi2c1);
}

void HAL_I2C_ListenCpltCallback(I2C_HandleTypeDef *hi2c)
{
  // Kích hoạt lại chế độ lắng nghe khi một phiên giao tiếp kết thúc
  HAL_I2C_EnableListen_IT(&hi2c1);
}

// Các hàm cấu hình hệ thống
void SystemClock_Config(void)
{
  // [Mã cấu hình clock cho STM32F401]
}

static void MX_GPIO_Init(void)
{
  // [Mã cấu hình GPIO cho STM32F401]
}

void Error_Handler(void)
{
  while(1)
  {
    // Xử lý khi xảy ra lỗi
  }
}
