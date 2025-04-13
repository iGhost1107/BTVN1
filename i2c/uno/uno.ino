/* Arduino Master Code */

#define SLA 0x08

void I2C_Init() {
    TWSR = 0x00;
    TWBR = ((16000000 / 100000) - 16) / 2;
    TWCR = (1 << TWEN);
}

void I2C_Start() {
    TWCR = (1 << TWSTA) | (1 << TWEN) | (1 << TWINT);
    while (!(TWCR & (1 << TWINT)));
}

void I2C_Stop() {
    TWCR = (1 << TWSTO) | (1 << TWEN) | (1 << TWINT);
}

void I2C_Write(uint8_t data) {
    TWDR = data;
    TWCR = (1 << TWEN) | (1 << TWINT);
    while (!(TWCR & (1 << TWINT)));
}

uint8_t I2C_Read(uint8_t ack) {
    TWCR = (1 << TWEN) | (1 << TWINT) | (ack << TWEA);
    while (!(TWCR & (1 << TWINT)));
    return TWDR;
}

void setup() {
    Serial.begin(9600);
    I2C_Init();
}

void loop() {
    Serial.println("Master: Gửi -> Hello Slave");
    
    I2C_Start();
    I2C_Write(SLA << 1);  // Địa chỉ Slave (bit W = 0)
    I2C_Write('H'); I2C_Write('e'); I2C_Write('l'); I2C_Write('l'); I2C_Write('o'); 
    I2C_Write(' '); I2C_Write('S'); I2C_Write('l'); I2C_Write('a'); I2C_Write('v'); I2C_Write('e');
    I2C_Write('\0');  // Ký tự kết thúc chuỗi
    I2C_Stop();
    delay(500);
    
    Serial.println("Master: Chờ phản hồi từ Slave...");
    I2C_Start();
    I2C_Write((SLA << 1) | 1);  // Địa chỉ Slave (bit R = 1)
    char receiveBuffer[10];
    uint8_t i = 0;
    do {
        receiveBuffer[i] = I2C_Read(1);  // Đọc dữ liệu với ACK
    } while (receiveBuffer[i++] != '\0' && i < sizeof(receiveBuffer) - 1);
    receiveBuffer[i] = '\0';
    
    I2C_Read(0);  // Gửi NACK để kết thúc
    I2C_Stop();
    Serial.print("Master nhận từ Slave: ");
    Serial.println(receiveBuffer);
    delay(2000);
}