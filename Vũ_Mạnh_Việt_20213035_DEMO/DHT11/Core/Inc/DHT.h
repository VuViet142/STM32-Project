#ifndef __DHT_H
#define __DHT_H

#include "main.h"

// Khai báo các chân GPIO của DHT11
#define DHT11_PORT GPIOB
#define DHT11_PIN GPIO_PIN_9

// Khai báo các hàm đọc dữ liệu từ cảm biến DHT11
void DHT_Init(void);                    // Hàm khởi tạo DHT11
uint8_t DHT11_Start(void);              // Hàm bắt đầu giao tiếp với DHT11
uint8_t DHT11_Read(void);               // Hàm đọc dữ liệu từ DHT11
float DHT_ReadHumi(void);
float DHT_ReadTemp(void);    // Hàm đọc và xử lý dữ liệu từ DHT11
void microDelay(uint16_t delay);        // Hàm delay sử dụng Timer

// Biến toàn cục để lưu trữ dữ liệu nhiệt độ và độ ẩm
extern float temp;   // Nhiệt độ (Celsius)
extern float humi;   // Độ ẩm (%)

#endif /* __DHT_H */
