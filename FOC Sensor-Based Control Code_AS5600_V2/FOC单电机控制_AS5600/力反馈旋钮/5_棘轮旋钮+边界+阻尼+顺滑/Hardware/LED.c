#include "stm32f10x.h"                  // Device header

// 核心修正：纯二进制映射（挡位=二进制值，PA4=23, PA5=22, PA6=21, PA7=2?）
// 挡位n → 二进制值n → PA7(2?)、PA6(21)、PA5(22)、PA4(23) 对应位亮
static const uint8_t led_mode_map[8] = {
    0x00,   // 挡位0: 0000 → 全灭
    0x01,   // 挡位1: 0001 → PA7亮
    0x02,   // 挡位2: 0010 → PA6亮
    0x03,   // 挡位3: 0011 → PA7+PA6亮（核心修正：原0x04改为0x03）
    0x04,   // 挡位4: 0100 → PA5亮
    0x05,   // 挡位5: 0101 → PA5+PA7亮
    0x06,   // 挡位6: 0110 → PA5+PA6亮
    0x07    // 挡位7: 0111 → PA5+PA6+PA7亮
};

void LED_Init(void)
{
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE);
    
    GPIO_InitTypeDef GPIO_InitStructure;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
    // 保留A1/A2，新增A4/A5/A6/A7引脚
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_1 | GPIO_Pin_2 | 
                                  GPIO_Pin_4 | GPIO_Pin_5 | GPIO_Pin_6 | GPIO_Pin_7;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(GPIOA, &GPIO_InitStructure);
    
    // 初始化所有LED为熄灭状态（高电平灭，低电平亮）
    GPIO_SetBits(GPIOA, GPIO_Pin_1 | GPIO_Pin_2 | 
                 GPIO_Pin_4 | GPIO_Pin_5 | GPIO_Pin_6 | GPIO_Pin_7);
}

// 原有A1/A2控制函数保留（高灭低亮）
void LED1_ON(void) { GPIO_ResetBits(GPIOA, GPIO_Pin_1); }
void LED1_OFF(void) { GPIO_SetBits(GPIOA, GPIO_Pin_1); }
void LED1_Turn(void)
{
    if (GPIO_ReadOutputDataBit(GPIOA, GPIO_Pin_1) == 0)
        GPIO_SetBits(GPIOA, GPIO_Pin_1);
    else
        GPIO_ResetBits(GPIOA, GPIO_Pin_1);
}

void LED2_ON(void) { GPIO_ResetBits(GPIOA, GPIO_Pin_2); }
void LED2_OFF(void) { GPIO_SetBits(GPIOA, GPIO_Pin_2); }
void LED2_Turn(void)
{
    if (GPIO_ReadOutputDataBit(GPIOA, GPIO_Pin_2) == 0)
        GPIO_SetBits(GPIOA, GPIO_Pin_2);
    else
        GPIO_ResetBits(GPIOA, GPIO_Pin_2);
}

// 新增：A4-A7独立控制函数
void LED4_OFF(void) { GPIO_ResetBits(GPIOA, GPIO_Pin_4); }
void LED4_ON(void) { GPIO_SetBits(GPIOA, GPIO_Pin_4); }

void LED5_OFF(void) { GPIO_ResetBits(GPIOA, GPIO_Pin_5); }
void LED5_ON(void) { GPIO_SetBits(GPIOA, GPIO_Pin_5); }

void LED6_OFF(void) { GPIO_ResetBits(GPIOA, GPIO_Pin_6); }
void LED6_ON(void) { GPIO_SetBits(GPIOA, GPIO_Pin_6); }

void LED7_OFF(void) { GPIO_ResetBits(GPIOA, GPIO_Pin_7); }
void LED7_ON(void) { GPIO_SetBits(GPIOA, GPIO_Pin_7); }

// 核心控制函数：PA4(23)、PA5(22)、PA6(21)、PA7(2?) 二进制映射
void LED_SetByKnobMode(uint8_t mode)
{
    // 限制挡位范围在0-7，防止越界
    if (mode >= 8) mode = 0;
    
    uint8_t led_cfg = led_mode_map[mode];
    
    // 按二进制位映射：2?→PA7，21→PA6，22→PA5，23→PA4
    (led_cfg & 0x01) ? LED7_ON() : LED7_OFF();  // 2? → PA7
    (led_cfg & 0x02) ? LED6_ON() : LED6_OFF();  // 21 → PA6
    (led_cfg & 0x04) ? LED5_ON() : LED5_OFF();  // 22 → PA5
    (led_cfg & 0x08) ? LED4_ON() : LED4_OFF();  // 23 → PA4
}
