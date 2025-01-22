#include "DHT.h"
#include "delay_timer.h"  // Thêm thư viện delay_timer
#include "main.h"

// Định nghĩa các biến toàn cục
uint8_t RHI, RHD, TCI, TCD, SUM;
uint32_t pMillis, cMillis;
float temp = -1, humi = -1;  // Biến nhiệt độ và độ ẩm

// Hàm khởi tạo cho DHT11
void DHT_Init(void)
{
    // Cấu hình GPIO hoặc các thiết lập khác có thể thêm vào đây
    // Ví dụ, khởi tạo Timer nếu chưa thực hiện
    TIM_HandleTypeDef htim1;
    DELAY_TIM_Init(&htim1);  // Khởi tạo Timer cho delay
}

// Hàm bắt đầu giao tiếp với cảm biến DHT11


uint8_t DHT11_Start(void)
{
    uint8_t Response = 0;
    GPIO_InitTypeDef GPIO_InitStructPrivate = {0};

    // Cấu hình chân GPIO của DHT11
    GPIO_InitStructPrivate.Pin = DHT11_PIN;
    GPIO_InitStructPrivate.Mode = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStructPrivate.Speed = GPIO_SPEED_FREQ_LOW;
    GPIO_InitStructPrivate.Pull = GPIO_NOPULL;
    HAL_GPIO_Init(DHT11_PORT, &GPIO_InitStructPrivate); // Cấu hình chân như output

    HAL_GPIO_WritePin(DHT11_PORT, DHT11_PIN, 0);  // Kéo chân xuống thấp
    HAL_Delay(20);  // Chờ 20ms
    HAL_GPIO_WritePin(DHT11_PORT, DHT11_PIN, 1);  // Kéo chân lên cao
    microDelay(30);  // Chờ 30us

    // Đặt chân GPIO làm input
    GPIO_InitStructPrivate.Mode = GPIO_MODE_INPUT;
    GPIO_InitStructPrivate.Pull = GPIO_PULLUP;
    HAL_GPIO_Init(DHT11_PORT, &GPIO_InitStructPrivate); // Cấu hình chân làm input
    microDelay(40);

    if (!(HAL_GPIO_ReadPin(DHT11_PORT, DHT11_PIN)))
    {
        microDelay(80);
        if ((HAL_GPIO_ReadPin(DHT11_PORT, DHT11_PIN))) Response = 1;
    }

    pMillis = HAL_GetTick();
    cMillis = HAL_GetTick();
    while ((HAL_GPIO_ReadPin(DHT11_PORT, DHT11_PIN)) && pMillis + 2 > cMillis)
    {
        cMillis = HAL_GetTick();
    }

    return Response;
}

// Hàm đọc dữ liệu từ cảm biến DHT11
uint8_t DHT11_Read(void)
{
    uint8_t a, b = 0;
    for (a = 0; a < 8; a++)
    {
        pMillis = HAL_GetTick();
        cMillis = HAL_GetTick();
        while (!(HAL_GPIO_ReadPin(DHT11_PORT, DHT11_PIN)) && pMillis + 2 > cMillis)
        {  // Chờ chân lên cao
            cMillis = HAL_GetTick();
        }
        microDelay(40);  // Chờ 40us
        if (!(HAL_GPIO_ReadPin(DHT11_PORT, DHT11_PIN)))  // Nếu chân xuống thấp
            b &= ~(1 << (7 - a));
        else
            b |= (1 << (7 - a));

        pMillis = HAL_GetTick();
        cMillis = HAL_GetTick();
        while ((HAL_GPIO_ReadPin(DHT11_PORT, DHT11_PIN)) && pMillis + 2 > cMillis)
        {  // Chờ chân xuống thấp
            cMillis = HAL_GetTick();
        }
    }
    return b;
}

// Hàm đọc và xử lý dữ liệu từ cảm biến DHT11
float DHT_ReadHumi(void)
{
    if (DHT11_Start())  // Bắt đầu giao tiếp với DHT11
    {
        // Đọc dữ liệu từ cảm biến
        RHI = DHT11_Read();  // Đọc độ ẩm phần nguyên
        RHD = DHT11_Read();  // Đọc độ ẩm phần thập phân
        TCI = DHT11_Read();  // Đọc nhiệt độ phần nguyên (bỏ qua)
        TCD = DHT11_Read();  // Đọc nhiệt độ phần thập phân (bỏ qua)
        SUM = DHT11_Read();  // Đọc checksum

        // Kiểm tra dữ liệu hợp lệ bằng cách kiểm tra tổng kiểm tra
        if (RHI + RHD + TCI + TCD == SUM)
        {
            // Trả về giá trị độ ẩm tính từ phần nguyên và phần thập phân
            return (float)RHI + (float)(RHD / 10.0);  // Độ ẩm
        }
        else
        {
            // Nếu checksum không hợp lệ, trả về -1 để chỉ ra lỗi
            return -1.0;
        }
    }
    else
    {
        // Nếu không nhận được phản hồi từ cảm biến, trả về -1
        return -1.0;
    }
}
// Hàm chỉ đọc và trả về nhiệt độ từ DHT11
float DHT_ReadTemp(void)
{
    if (DHT11_Start())  // Bắt đầu giao tiếp với DHT11
    {
        // Đọc dữ liệu từ cảm biến
        RHI = DHT11_Read();  // Đọc độ ẩm phần nguyên (bỏ qua)
        RHD = DHT11_Read();  // Đọc độ ẩm phần thập phân (bỏ qua)
        TCI = DHT11_Read();  // Đọc nhiệt độ phần nguyên
        TCD = DHT11_Read();  // Đọc nhiệt độ phần thập phân
        SUM = DHT11_Read();  // Đọc checksum

        // Kiểm tra dữ liệu hợp lệ bằng cách kiểm tra tổng kiểm tra
        if (RHI + RHD + TCI + TCD == SUM)
        {
            // Trả về giá trị nhiệt độ tính từ phần nguyên và phần thập phân
            return (float)TCI + (float)(TCD / 10.0);  // Nhiệt độ
        }
        else
        {
            // Nếu checksum không hợp lệ, trả về -1 để chỉ ra lỗi
            return -1.0;
        }
    }
    else
    {
        // Nếu không nhận được phản hồi từ cảm biến, trả về -1
        return -1.0;
    }
}
