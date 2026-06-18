#include "lwbtn.h"
#include "stdio.h"
#include "stdint.h"
#include "header.h"
#include "FOC_Algorithm.h"
#include "Control.h"
#include "FOC_Struct.h"

//同时需要打开use micro lib并进行串口重定向,否则无法使用串口功能

/**
 * @brief 全局变量，记录当前触发的按键编号
 * 
 * - 0: 无按键
 * - 1: KEY0
 * - 2: KEY1
 * - 3: KEY2
 * - 4: KEY_UP
 */

/*-------------------------Key的定义------------------------*/
//修改为自己的那件引脚
//E4, E3
//KEY0,KEY1
//ms为单位
//其中lwbtn_keys代表着那个按键被按下,同时在回调函数
//button_event_handler中进行处理
uint8_t lwbtn_keys;
#define KEY0_GPIO_Port    GPIOE
#define KEY0_Pin          GPIO_PIN_4

#define KEY1_GPIO_Port    GPIOE
#define KEY1_Pin          GPIO_PIN_3

//-----------------------------------------------------------------------------------------
// 移植设置
//-----------------------------------------------------------------------------------------
/*为每个按键定义GPIO参数*/
lwbtn_argdata_port_pin_state_t key0_gpio = {
    .port = KEY0_GPIO_Port,  /* KEY0的GPIO端口 */
    .pin = KEY0_Pin,         /* KEY0的GPIO引脚 */
    .state = 0               /* 低电平触发 */
};

lwbtn_argdata_port_pin_state_t key1_gpio = {
    .port = KEY1_GPIO_Port,  /* KEY1的GPIO端口 */
    .pin = KEY1_Pin,         /* KEY1的GPIO引脚 */
    .state = 0               /* 低电平触发 */
};

/* 定义按钮数组，每个按钮绑定对应的GPIO参数 */
lwbtn_btn_t buttons[] = {
    { .arg = &key0_gpio },    /* KEY0 */
    { .arg = &key1_gpio },    /* KEY1 */
};

/**
 * @brief 获取按钮的当前状态
 * 
 * 通过读取GPIO引脚的电平，判断按钮是否处于激活状态。
 * 
 * @param lwobj: LwBTN实例（未使用）
 * @param btn: 按钮实例，包含GPIO参数
 * @return uint8_t: 1表示按钮激活，0表示按钮未激活
 */
uint8_t get_button_state(lwbtn_t* lwobj, lwbtn_btn_t* btn)
{
    lwbtn_argdata_port_pin_state_t* cfg = btn->arg;
    uint8_t pin_state = HAL_GPIO_ReadPin(cfg->port, cfg->pin);
    return (pin_state == cfg->state) ? 1 : 0; // 返回激活状态
}
/**
 * @brief 按钮事件处理函数
 * 根据LwBTN库触发的事件类型，处理按钮的按下、释放、单击、双击和长按事件。
 * 每一个事件都会触发一个回调 *
 * @param lwobj: LwBTN实例
 * @param btn: 触发事件的按钮实例
 * @param evt: 事件类型（按下、释放、单击、长按等）
 **/

uint8_t enumerate = 0;

void button_event_handler(lwbtn_t* lwobj,lwbtn_btn_t* btn, lwbtn_evt_t evt)
{//根据按钮索引识别具体按键
    uint8_t btn_index = (btn - lwobj->btns);
    switch (evt) {
        /* 按键按下事件 */
        case LWBTN_EVT_ONPRESS:
					if(lwbtn_keys==1){//如果是按键一的单击
						Cur_Uq.Ref += 2;
//						enumerate++;
//						enumerate%=2;
//						if(enumerate){
//							svpwm.Vq = 1;//这里就是开始
//						}
//						else {
//							svpwm.Vq = 0;//这里就是停止
//						}
					}
					else if(lwbtn_keys==2){//如果是按键二的单击
						Cur_Uq.Ref -= 2;
					}
					lwbtn_keys = btn_index + 1;
            break;
        /* 按键释放事件 */
        case LWBTN_EVT_ONRELEASE:
					
            break;
        /* 单击/双击事件 */
        case LWBTN_EVT_ONCLICK:/*双击*/
            if (btn->click.cnt == 2){
								if(lwbtn_keys==1){//如果是按键一的双击
									
								}
								else if(lwbtn_keys==2){
									
								}
                lwbtn_keys = btn_index + 1; //记录按键编号
                btn->click.cnt = 0; 				//重置计数器
            }else{/*单击*/
                lwbtn_keys = btn_index + 1; // 记录按键编号
            }
            break;
        /*长按事件*/
        case LWBTN_EVT_KEEPALIVE:
							if(lwbtn_keys==1){//如果是按键一的长按
							}
							else if(lwbtn_keys==2){
							}
            lwbtn_keys = btn_index+1; // 记录按键编号
            break;
    }
}

/**
 * @brief 初始化按钮管理器
 * 
 * 使用LwBTN库初始化按钮管理器，绑定按钮数组、状态读取函数和事件处理函数。
 */
void button_init(void)
{
    lwbtn_init_ex(NULL, buttons, BUTTON_COUNT, get_button_state, button_event_handler);
}

/**
 * @brief 处理按键状态
 * 
 * 定期调用此函数以处理按键状态，触发事件回调
 */
void get_btn(void)
{
    uint32_t tick = 0;
    tick = HAL_GetTick();//获取当前系统时间
    lwbtn_process(tick);//处理按键状态
}
