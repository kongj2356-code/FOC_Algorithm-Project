/****************************************************************************************************
 * 文件功能说明：
 * 1. 串口DMA环形缓冲区与printf重定向
 * 2. Justfloat浮点数串口协议发送
 * 3. 软件I2C驱动
 * 4. AS5600磁编码器角度读取
 * 5. FOC电机电角度计算
 ****************************************************************************************************/
#include "Upper.h"

/*Justfloat格式发送解析*/
//要点提示
//1.float和unsigned long具有相同的数据结构长度4Byte(32Bit)
//2.union据类型里的数据存放在相同的物理空间


/* 串口DMA发送缓冲区大小 */
#define UART_TX_BUFFER_SIZE 1024



/**
 * @brief 串口DMA环形发送缓冲区结构体 // 循环队列 head 队尾写入数据，tail 队头取出数据FIFO
 * buffer：数据缓存区
 * head：写指针（新数据放入的位置）
 * tail：读指针（DMA发送的位置）
 * dma_busy：DMA是否正在发送
 */
typedef struct{
    uint8_t buffer[UART_TX_BUFFER_SIZE];
    volatile uint32_t head;
    volatile uint32_t tail;
    volatile uint8_t dma_busy;
} uart_tx_buffer_t;
uart_tx_buffer_t uart_tx_buf;



/**
 * @brief  串口TX DMA初始化
 * @return 无
 */
void UART_TX_DMA_Init(void)
{
    uart_tx_buf.head = 0;
    uart_tx_buf.tail = 0;
    uart_tx_buf.dma_busy = 0;
    
    // 使能DMA传输中断
    __HAL_DMA_ENABLE_IT(&hdma_usart1_tx, DMA_IT_TC);
}



/**
 * @brief  重定向printf到串口DMA（不阻塞）
 * @param  ch 要发送的字符
 * @param  f  文件流（用不到）
 * @return 发送的字符
 */
int fputc(int ch, FILE *f)
{
    // 计算下一个写指针位置（环形）
    uint32_t next_head = (uart_tx_buf.head + 1) % UART_TX_BUFFER_SIZE;
    
    // 如果缓冲区满，直接返回（不阻塞CPU）
    while(next_head == uart_tx_buf.tail) {
        return ch;
    }
    
    // 把字符放入缓冲区
    uart_tx_buf.buffer[uart_tx_buf.head] = (uint8_t)ch;
    uart_tx_buf.head = next_head;
    
    // 如果DMA空闲，立即启动发送
    if(!uart_tx_buf.dma_busy) {
        UART_Start_DMA_Transfer();
    }
    return ch;
}



/**
 * @brief  启动串口DMA发送数据
 * @return 无
 */
void UART_Start_DMA_Transfer(void)
{
    if(uart_tx_buf.head == uart_tx_buf.tail) {
        return; // 缓冲区为空
    }
    
    uart_tx_buf.dma_busy = 1;
    
    uint32_t bytes_to_send;
    uint32_t start_index = uart_tx_buf.tail;
    
    // 计算连续可发送的数据长度
    if(uart_tx_buf.head > uart_tx_buf.tail) {
        bytes_to_send = uart_tx_buf.head - uart_tx_buf.tail;
    } else {
        bytes_to_send = UART_TX_BUFFER_SIZE - uart_tx_buf.tail;
    }
    
    // 启动DMA传输
    HAL_UART_Transmit_DMA(&huart1, &uart_tx_buf.buffer[uart_tx_buf.tail], bytes_to_send);
}



/**
 * @brief  重定向scanf，从串口读1个字节
 */
int fgetc(FILE *f)
{
	uint8_t ch;
	HAL_UART_Receive(&huart1,(uint8_t*)&ch,1,10);
	return ch;
}



/**
 * @brief  浮点数4字节 共用体
 * 用于把float拆成4个字节发送，或反过来解析
 */
//union = 同一块内存，把 4 字节 float 放进内存，用 unsigned long 把这 4 字节读出来
typedef union
{
    float fdata;
    unsigned long ldata;
} FloatLongType;



//将浮点数f转化为4个字节数据存放在byte[4]中
//传递指针
void Float_to_Byte(float f,unsigned char byte[])
{
    FloatLongType fl;
    fl.fdata=f;
    byte[0]=(unsigned char)fl.ldata;
    byte[1]=(unsigned char)(fl.ldata>>8);
    byte[2]=(unsigned char)(fl.ldata>>16);
    byte[3]=(unsigned char)(fl.ldata>>24);
}




/**
 * @brief  Justfloat协议发送浮点数数组
 * @param  data_array 浮点数数组指针
 * @param  CH_COUNT   要发送的浮点数个数
 */
void Float_send(float *data_array , uint8_t CH_COUNT)
{
    //参数为浮点数组
    unsigned char tail[4] = {0x00, 0x00, 0x80, 0x7f};//定义帧尾
    //按照通道数发送数据，CH_COUNT为发送数据的通道数
    for (int i = 0; i < CH_COUNT; i++)
   {
        unsigned char byte[4];
        Float_to_Byte(data_array[i],byte);//将每个float转换为4个字节
        Send_array(byte,4);               //发送这4个字节
    }
    Send_array(tail,4);//发送数据帧尾
}



/**
 * @brief  串口发送字节数组
 * @param  byte    数据指针
 * @param  Number  数据长度
 */
void Send_array(unsigned char* byte, uint8_t Number)
{
    // 增加基本参数校验
    if(byte == NULL || Number == 0){
        return;
    }
    HAL_StatusTypeDef status = HAL_UART_Transmit(&huart1, byte, Number, 10); // 阻塞发送，超时10ms
    if (status != HAL_OK){
    }
}



/* ================================================== 软件I2C驱动（AS5600专用）================================================== */
// GPIO定义 - 根据您的配置修改
#define SW_I2C_SCL_PIN    GPIO_PIN_8
#define SW_I2C_SCL_PORT   GPIOA
#define SW_I2C_SDA_PIN    GPIO_PIN_9
#define SW_I2C_SDA_PORT   GPIOC

// 直接寄存器操作（最快速度）
#define SW_I2C_SCL_HIGH()     (SW_I2C_SCL_PORT->BSRR = SW_I2C_SCL_PIN)
#define SW_I2C_SCL_LOW()      (SW_I2C_SCL_PORT->BSRR = (uint32_t)SW_I2C_SCL_PIN << 16)
#define SW_I2C_SDA_HIGH()     (SW_I2C_SDA_PORT->BSRR = SW_I2C_SDA_PIN)
#define SW_I2C_SDA_LOW()      (SW_I2C_SDA_PORT->BSRR = (uint32_t)SW_I2C_SDA_PIN << 16)
#define SW_I2C_SDA_READ()     ((SW_I2C_SDA_PORT->IDR & SW_I2C_SDA_PIN) ? 1 : 0)

static inline void SW_I2C_Delay(void){
    for(volatile int i = 0; i < 5; i++){
        __NOP(); __NOP(); __NOP(); __NOP();
        __NOP(); __NOP(); __NOP(); __NOP();
    }
}

// 起始条件
static void SW_I2C_Start(void){
    SW_I2C_SDA_HIGH();
    SW_I2C_SCL_HIGH();
    SW_I2C_Delay();
    SW_I2C_SDA_LOW();
    SW_I2C_Delay();
    SW_I2C_SCL_LOW();
}

// 停止条件
static void SW_I2C_Stop(void){
    SW_I2C_SDA_LOW();
    SW_I2C_Delay();
    SW_I2C_SCL_HIGH();
    SW_I2C_Delay();
    SW_I2C_SDA_HIGH();
    SW_I2C_Delay();
}

// 写入字节并检查ACK
static uint8_t SW_I2C_WriteByte(uint8_t data){
    uint8_t ack;
    
    for(int i = 0; i < 8; i++) {
        if(data & 0x80) {
            SW_I2C_SDA_HIGH();
        } else {
            SW_I2C_SDA_LOW();
        }
        data <<= 1;
        
        SW_I2C_Delay();
        SW_I2C_SCL_HIGH();
        SW_I2C_Delay();
        SW_I2C_SCL_LOW();
    }
    
    // 读取ACK
    SW_I2C_SDA_HIGH();  // 释放SDA线
    SW_I2C_Delay();
    SW_I2C_SCL_HIGH();
    SW_I2C_Delay();
    ack = SW_I2C_SDA_READ();  // 读取ACK位（0表示ACK）
    SW_I2C_SCL_LOW();
    
    return ack;  // 返回0表示成功
}

// 读取字节
static uint8_t SW_I2C_ReadByte(uint8_t ack)
{
    uint8_t data = 0;
    
    SW_I2C_SDA_HIGH();  // 释放SDA线用于输入
    
    for(int i = 0; i < 8; i++) {
        data <<= 1;
        SW_I2C_Delay();
        SW_I2C_SCL_HIGH();
        SW_I2C_Delay();
        
        if(SW_I2C_SDA_READ()) {
            data |= 1;
        }
        
        SW_I2C_SCL_LOW();
    }
    
    // 发送ACK/NACK
    if(ack) {
        SW_I2C_SDA_LOW();  // 发送ACK
    } else {
        SW_I2C_SDA_HIGH(); // 发送NACK
    }
    
    SW_I2C_Delay();
    SW_I2C_SCL_HIGH();
    SW_I2C_Delay();
    SW_I2C_SCL_LOW();
    SW_I2C_SDA_HIGH();  // 释放SDA线
    
    return data;
}

// 读取AS5600寄存器
static uint8_t AS5600_ReadRegister(uint8_t reg_addr, uint8_t* data, uint8_t len)
{
    //uint8_t ret = 0;
    
    SW_I2C_Start();
    
    // 发送设备地址 + 写
    if(SW_I2C_WriteByte(AS5600_I2C_ADDRESS << 1)) {
        SW_I2C_Stop();
        return 0;
    }
    
    // 发送寄存器地址
    if(SW_I2C_WriteByte(reg_addr)) {
        SW_I2C_Stop();
        return 0;
    }
    
    // 重复起始条件
    SW_I2C_Start();
    
    // 发送设备地址 + 读
    if(SW_I2C_WriteByte((AS5600_I2C_ADDRESS << 1) | 0x01)) {
        SW_I2C_Stop();
        return 0;
    }
    
    // 读取数据
    for(uint8_t i = 0; i < len; i++) {
        data[i] = SW_I2C_ReadByte(i < (len - 1));  // 最后一个字节发送NACK
    }
    
    SW_I2C_Stop();
    return 1;
}


// 读取所有AS5600数据
uint8_t AS5600_Read_All_Data(Encoder_t* data)
{
    uint8_t buffer[2];
    
    if(data == NULL) return 0;
    
    // 读取状态寄存器
    if(!AS5600_ReadRegister(AS5600_REG_STATUS, &data->status, 1)) {
        return 0;
    }
    
    // 读取原始角度
    if(!AS5600_ReadRegister(AS5600_REG_RAW_ANGLE_H, buffer, 2)) {
        return 0;
    }
    data->raw_angle = (buffer[0] << 8) | buffer[1];
    data->raw_angle &= 0x0FFF;
    
    // 读取处理后的角度
    if(!AS5600_ReadRegister(AS5600_REG_ANGLE_H, buffer, 2)) {
        return 0;
    }
    data->angle = (buffer[0] << 8) | buffer[1];
    data->angle &= 0x0FFF;
    
    // 读取AGC
    if(!AS5600_ReadRegister(AS5600_REG_AGC, &data->agc, 1)) {
        return 0;
    }
    
    // 读取磁场强度
    if(!AS5600_ReadRegister(AS5600_REG_MAGNITUDE_H, buffer, 2)) {
        return 0;
    }
    data->magnitude = (buffer[0] << 8) | buffer[1];
    
    // 检查磁铁状态
    data->magnet_detected = (data->status & AS5600_STATUS_MD) ? 1 : 0;
    
    return 1;
}



// 快速读取角度(仅角度值)
uint16_t AS5600_Read_Angle(void)
{
    uint8_t buffer[2];
    if(AS5600_ReadRegister(AS5600_REG_ANGLE_H, buffer, 2)) {
        return (((buffer[0] << 8) | buffer[1]) & 0x0FFF);
    }
    return 0xFFFF; //错误值
}


// 检查磁铁状态
uint8_t AS5600_Check_Magnet(Encoder_t* data)
{
    if(data == NULL) return 0;
    // 读取状态寄存器
    if(!AS5600_ReadRegister(AS5600_REG_STATUS, &data->status, 1)){
        return 0;
    }
    data->magnet_detected = (data->status & AS5600_STATUS_MD) ? 1 : 0;
    if(data->status & AS5600_STATUS_ML) {
        return 2; // 磁铁太弱
    }
    if(data->status & AS5600_STATUS_MH) {
        return 3; // 磁铁太强
    }
    if(data->magnet_detected) {
        return 1; // 磁铁正常
    }
    return 0; // 无磁铁
}


// 获取角度（度）
float AS5600_Get_Angle_Degrees(void)
{
    uint16_t angle = AS5600_Read_Angle();
    if(angle == 0xFFFF) return -1.0f; // 错误
    
    return (angle * 360.0f) / 4096.0f;
}


// 获取角度（弧度）
float AS5600_Get_Angle_Radians(void)
{
    uint16_t angle = AS5600_Read_Angle();
    if(angle == 0xFFFF) return -1.0f; // 错误
    
    return (angle * 2.0f * 3.1415926535f) / 4096.0f;
}



/**
 * @brief  获取电机电角度（FOC用）
 * @param  svpm          SVPWM结构体
 * @param  as5600_data   编码器数据
 * @param  motor         电机参数
 */
void Get_angle(SVPWM_t* svpm , Encoder_t* as5600_data, Motor_t* motor){
    // 读取机械角度
    as5600_data->mec_angle = AS5600_Get_Angle_Degrees();
    
    // 减去零点偏置，得到校正后的机械角
    as5600_data->mec_angle -= motor->zero_encoder_angle;
    
    // 角度限幅 0~360
    if(as5600_data->mec_angle > 360.0f)  as5600_data->mec_angle -= 360.0f;
    if(as5600_data->mec_angle < 0.0f)    as5600_data->mec_angle += 360.0f;
    
    // 机械角 × 极对数 = 电角度（FOC核心）
    svpm->elec_angle = as5600_data->mec_angle * svpm->pole_pairs;
}



/**
 * @brief 电机速度计算（基于编码器角度差分 + 一阶低通滤波）
 * @param veclo: 速度位置结构体指针
 * @param as5600: AS5600编码器结构体指针
 * @note  解决360°-0°过零跳变问题，输出平滑转速
 */
void Get_speed(Vel_Pos_t* veclo, Encoder_t* as5600)
{
    // 保存当前机械角度，用于速度计算
    veclo->now_angle = as5600->mec_angle;

    // 计算相邻采样周期的角度差值
    float angle_diff = veclo->now_angle - veclo->last_angle;

    // 处理编码器360°→0°过零跳变，保证速度计算连续
    if (angle_diff > 180.0f)
    {
        angle_diff -= 360.0f;  // 正向过零修正
    }
    else if (angle_diff < -180.0f)
    {
        angle_diff += 360.0f;  // 反向过零修正
    }

    // 原始速度：角度差/采样周期 → 转换为 °/s
    float raw_speed = angle_diff;
    float speed_deg_per_sec = raw_speed * 100;  // 速度缩放（根据采样频率调整）

    // 一阶低通滤波：抑制噪声，输出平滑速度
    veclo->speed = 0.1f * speed_deg_per_sec + (1 - 0.1f) * veclo->last_speed;

    // 更新历史值，为下一次计算做准备
    veclo->last_angle = veclo->now_angle;
    veclo->last_speed = veclo->speed;
}

/**
 * @brief 电机多圈位置计算（角度差分累加，支持多圈）
 * @param pos: 位置结构体指针
 * @param as5600: AS5600编码器结构体指针
 * @note  适用于多圈绝对位置控制，解决360°过零问题
 */
void Get_position(Vel_Pos_t* pos, Encoder_t* as5600)
{
    // 获取当前机械角度
    pos->now_angle = as5600->mec_angle;

    // 计算角度差值
    float angle_diff = pos->now_angle - pos->last_angle;

    // 360°过零处理，保证位置累加连续
    if (angle_diff > 180.0f)
    {
        angle_diff -= 360.0f;
    }
    else if (angle_diff < -180.0f)
    {
        angle_diff += 360.0f;
    }

    // 多圈位置累加：实时角度 = 历史角度 + 角度增量
    pos->position += angle_diff;

    // 更新上一次角度值
    pos->last_angle = pos->now_angle;
}

// /**
//  * @brief 电机单圈位置获取（直接读取编码器绝对角度）
//  * @param pos: 位置结构体指针
//  * @param as5600: AS5600编码器结构体指针
//  * @note  仅适用于0~360°单圈控制
//  */
// void Get_position(Vel_Pos_t* pos, Encoder_t* as5600)
// {
//     // 直接使用绝对角度作为位置反馈
//     pos->position = as5600->mec_angle;
// }

