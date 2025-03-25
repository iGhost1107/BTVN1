#include <avr/io.h>

char c;
String num;

void setup() {
    // Cấu hình các chân D2 -> D9 làm OUTPUT
    DDRD |= 0b11111100; // Chân D2 -> D7 (PORTD) là OUTPUT
    DDRB |= 0b00000011; // Chân D8, D9 (PORTB) là OUTPUT
    
    // Khởi tạo UART baud rate 9600
    UBRR0H = 0;
    UBRR0L = 103; // Giá trị tương ứng với baud rate 9600 ở 16MHz
    UCSR0B = (1 << RXEN0) | (1 << TXEN0); // Bật UART RX và TX
    UCSR0C = (1 << UCSZ01) | (1 << UCSZ00); // Cấu hình UART 8-bit
}

char uartRead() {
    while (!(UCSR0A & (1 << RXC0))); // Chờ dữ liệu nhận được
    return UDR0; // Đọc dữ liệu từ UART
}

void loop() {
    if (UCSR0A & (1 << RXC0)) { // Kiểm tra nếu có dữ liệu UART
        c = uartRead();
        num += c;

        if (c == '\n') { // Khi gặp ký tự xuống dòng
            int value = num.toInt(); // Chuyển chuỗi sang số nguyên
            num = ""; // Xóa chuỗi để chuẩn bị dữ liệu mới

            // Xuất dữ liệu ra các chân D2 -> D9
            PORTD = (PORTD & 0x03) | ((value << 2) & 0xFC); // Xuất 6 bit cao vào PORTD (D2->D7)
            PORTB = (PORTB & 0xFC) | ((value >> 6) & 0x03); // Xuất 2 bit thấp vào PORTB (D8, D9)
        }
    }
}