/**
 * \file            lwbtn.h
 * \brief           轻量级按钮管理器
 */
#ifndef LWBTN_H
#define LWBTN_H

#include "lwbtn_opt.h"

#ifdef __cplusplus
extern "C" {
#endif /* __cplusplus */

/**
 * \defgroup        LWBTN 轻量级按钮管理器
 * \brief           轻量级按钮管理器
 * \{
 */

/**
 * \brief           自定义用户参数数据结构
 * 
 * 这是一个简单的预定义结构，用户可以用它来定义嵌入式系统中常见的功能，
 * 例如 GPIO 端口、GPIO 引脚以及按钮被认为是激活状态时的 GPIO 电平。
 * 
 * 用户可以将这个结构作为参数附加到按钮结构中。
 */
typedef struct {
     GPIO_TypeDef* port;    /*!< 用户定义的 GPIO 端口信息 */
     uint16_t pin;     /*!< 用户定义的 GPIO 引脚信息 */
    uint8_t state; /*!< 用户定义的按钮被认为激活时的 GPIO 电平 */
} lwbtn_argdata_port_pin_state_t;

/* 前向声明 */
struct lwbtn_btn;
struct lwbtn;

/**
 * \brief           时间变量类型
 */
typedef LWBTN_CFG_TYPE_VARTYPE lwbtn_time_t;

/**
 * \brief           按钮事件列表
 * 
 */
typedef enum {
    LWBTN_EVT_ONPRESS = 0x00, /*!< 按下事件 - 在检测到有效按下（如果启用了消抖）后发送 */
    LWBTN_EVT_ONRELEASE, /*!< 释放事件 - 在检测到有效释放事件（从激活到非激活）时发送 */
#if LWBTN_CFG_USE_CLICK || __DOXYGEN__
    LWBTN_EVT_ONCLICK, /*!< 点击事件 - 在有效的按下和释放事件序列发生时发送 */
#endif                 /* LWBTN_CFG_USE_CLICK || __DOXYGEN__ */
#if LWBTN_CFG_USE_KEEPALIVE || __DOXYGEN__
    LWBTN_EVT_KEEPALIVE, /*!< 保持活动事件 - 当按钮处于活动状态时定期发送 */
#endif                   /* LWBTN_CFG_USE_KEEPALIVE || __DOXYGEN__ */
} lwbtn_evt_t;


/**
 * \brief           按钮事件函数回调原型
 * \param[in]       lwobj: LwBTN 实例
 * \param[in]       btn: 事件发生时的按钮实例，来自按钮数组
 * \param[in]       evt: 事件类型
 */
typedef void (*lwbtn_evt_fn)(struct lwbtn* lwobj, struct lwbtn_btn* btn, lwbtn_evt_t evt);

/**
 * \brief           获取按钮/输入状态回调函数
 * \param[in]       lwobj: LwBTN 实例
 * \param[in]       btn: 从数组中读取状态的按钮实例
 * \return          按钮被认为“激活”时返回 `1`，否则返回 `0`
 */
typedef uint8_t (*lwbtn_get_state_fn)(struct lwbtn* lwobj, struct lwbtn_btn* btn);

/**
 * \brief           按钮/输入结构体
 */
typedef struct lwbtn_btn {
    uint16_t flags; /*!< 按钮的私有标志管理 */
#if LWBTN_CFG_GET_STATE_MODE != LWBTN_GET_STATE_MODE_CALLBACK || __DOXYGEN__
    uint8_t curr_state;             /*!< 当前按钮状态，用于处理。当应用程序手动设置按钮状态时使用 */
#endif                              /* LWBTN_CFG_GET_STATE_MODE != LWBTN_GET_STATE_MODE_CALLBACK || __DOXYGEN__ */
    uint8_t last_state;             /*!< 上一个按钮状态 - `1` 表示激活，`0` 表示非激活 */
    lwbtn_time_t time_change;       /*!< 按钮状态上次变化的时间（单位：毫秒），经过有效消抖后 */
    lwbtn_time_t time_state_change; /*!< 按钮状态最后一次变化的时间（单位：毫秒） */

#if LWBTN_CFG_USE_KEEPALIVE || __DOXYGEN__
    struct {
        lwbtn_time_t last_time; /*!< 上次发送保持活动事件的时间（单位：毫秒） */
        uint16_t cnt;           /*!< 成功检测到按下事件后发送的保持活动事件数量。按释放事件后重置 */
    } keepalive;                /*!< 保持活动结构体 */
#endif                          /* LWBTN_CFG_USE_KEEPALIVE || __DOXYGEN__ */

#if LWBTN_CFG_USE_CLICK || __DOXYGEN__
    struct {
        lwbtn_time_t last_time; /*!< 上次成功检测到（未发送）点击事件的时间（单位：毫秒） */
        uint8_t cnt;            /*!< 检测到的连续点击次数，遵循点击之间的最大超时限制 */
    } click;                    /*!< 点击事件结构体 */
#endif                          /* LWBTN_CFG_USE_CLICK || __DOXYGEN__ */


    void* arg; /*!< 用户定义的自定义参数，用于回调函数 */
    
#if LWBTN_CFG_TIME_DEBOUNCE_PRESS_DYNAMIC || __DOXYGEN__
    uint16_t time_debounce; /*!< 去抖动时间（单位：毫秒），用于按下事件 */
#endif                      /* LWBTN_CFG_TIME_DEBOUNCE_PRESS_DYNAMIC || __DOXYGEN__ */
#if LWBTN_CFG_TIME_DEBOUNCE_RELEASE_DYNAMIC || __DOXYGEN__
    uint16_t time_debounce_release; /*!< 去抖动时间（单位：毫秒），用于释放事件 */
#endif                              /* LWBTN_CFG_TIME_DEBOUNCE_RELEASE */
#if LWBTN_CFG_TIME_CLICK_MIN_DYNAMIC || __DOXYGEN__
    uint16_t time_click_pressed_min; /*!< 有效点击事件的最小按下时间（单位：毫秒） */
#endif                               /* LWBTN_CFG_TIME_CLICK_MIN_DYNAMIC || __DOXYGEN__ */
#if LWBTN_CFG_TIME_CLICK_MAX_DYNAMIC || __DOXYGEN__
    uint16_t time_click_pressed_max; /*!< 有效点击事件的最大按下时间（单位：毫秒） */
#endif                               /* LWBTN_CFG_TIME_CLICK_MAX_DYNAMIC || __DOXYGEN__ */
#if LWBTN_CFG_TIME_CLICK_MULTI_MAX_DYNAMIC || __DOXYGEN__
    uint16_t time_click_multi_max; /*!< 被认为是连续点击的最大时间间隔（单位：毫秒） */
#endif                             /* LWBTN_CFG_TIME_CLICK_MULTI_MAX_DYNAMIC || __DOXYGEN__ */
#if LWBTN_CFG_TIME_KEEPALIVE_PERIOD_DYNAMIC || __DOXYGEN__
    uint16_t time_keepalive_period; /*!< 周期性保持活动事件的时间间隔（单位：毫秒） */
#endif                              /* LWBTN_CFG_TIME_KEEPALIVE_PERIOD_DYNAMIC || __DOXYGEN__ */
#if LWBTN_CFG_CLICK_MAX_CONSECUTIVE_DYNAMIC || __DOXYGEN__
    uint16_t max_consecutive; /*!< 允许的最大连续点击次数 */
#endif                        /* LWBTN_CFG_CLICK_MAX_CONSECUTIVE_DYNAMIC || __DOXYGEN__ */
} lwbtn_btn_t;

/**
 * \brief           LwBTN 按钮组结构体
 */
typedef struct lwbtn {
    lwbtn_btn_t* btns;   /*!< 按钮数组的指针 */
    uint16_t btns_cnt;   /*!< 数组中按钮的数量 */
    lwbtn_evt_fn evt_fn; /*!< 事件处理函数的指针 */
#if LWBTN_CFG_GET_STATE_MODE != LWBTN_GET_STATE_MODE_MANUAL || __DOXYGEN__
    lwbtn_get_state_fn get_state_fn; /*!< 获取按钮状态的函数指针 */
#endif                               /* LWBTN_CFG_GET_STATE_MODE != LWBTN_GET_STATE_MODE_MANUAL || __DOXYGEN__ */
}lwbtn_t;

/**
 * \brief           LwBTN 库初始化（扩展版本）
 * \param[in]       lwobj: LwBTN 对象指针
 * \param[in]       btns: 按钮数组
 * \param[in]       btns_cnt: 按钮数组中的按钮数量
 * \param[in]       get_state_fn: 提供按钮状态的函数指针。如果 \ref LWBTN_CFG_GET_STATE_MODE 未设置为 \ref LWBTN_GET_STATE_MODE_CALLBACK，则可以设置为 `NULL`
 * \param[in]       evt_fn: 按钮事件回调函数指针
 * \return          返回初始化状态，0表示成功
 */
uint8_t lwbtn_init_ex(lwbtn_t* lwobj, lwbtn_btn_t* btns, uint16_t btns_cnt, lwbtn_get_state_fn get_state_fn,
                      lwbtn_evt_fn evt_fn);

/**
 * \brief           定期读取按钮状态并采取相应的操作。
 *                  它处理默认按钮实例组。
 * \param[in]       mstime: 当前系统时间（以毫秒为单位）
 * \return          返回处理结果，0表示成功
 */
uint8_t lwbtn_process_ex(lwbtn_t* lwobj, lwbtn_time_t mstime);

/**
 * \brief           处理特定按钮的状态
 * \param[in]       btn: 要处理的按钮实例
 * \param[in]       mstime: 当前系统时间（以毫秒为单位）
 * \return          返回处理结果，0表示成功
 */
uint8_t lwbtn_process_btn_ex(lwbtn_t* lwobj, lwbtn_btn_t* btn, lwbtn_time_t mstime);

/**
 * \brief           设置按钮的状态
 * \param[in]       btn: 要设置状态的按钮实例
 * \param[in]       state: 要设置的状态值
 * \return          返回设置结果，0表示成功
 */
uint8_t lwbtn_set_btn_state(lwbtn_btn_t* btn, uint8_t state);

/**
 * \brief           检查按钮是否处于激活状态
 * \param[in]       btn: 要检查的按钮实例
 * \return          返回按钮状态，1表示激活，0表示未激活
 */
uint8_t lwbtn_is_btn_active(const lwbtn_btn_t* btn);

/**
 * \brief           重置按钮状态
 * \param[in]       lwobj: LwBTN 对象指针
 * \param[in]       btn: 要重置的按钮实例
 * \return          返回重置结果，0表示成功
 */
uint8_t lwbtn_reset(lwbtn_t* lwobj, lwbtn_btn_t* btn);

/**
 * \brief           使用默认按钮组初始化 LwBTN 库
 * \param[in]       btns: 按钮数组
 * \param[in]       btns_cnt: 按钮数量
 * \param[in]       get_state_fn: 提供按钮状态的函数指针。如果 \ref LWBTN_CFG_GET_STATE_MODE 未设置为 \ref LWBTN_GET_STATE_MODE_CALLBACK，则可以设置为 `NULL`
 * \param[in]       evt_fn: 按钮事件回调函数
 * \sa              lwbtn_init_ex
 */
#define lwbtn_init(btns, btns_cnt, get_state_fn, evt_fn) lwbtn_init_ex(NULL, btns, btns_cnt, get_state_fn, evt_fn)

/**
 * \brief           定期读取按钮状态并采取适当的操作。
 * 它处理默认按钮实例组。
 * \param[in]       mstime: 当前系统时间（以毫秒为单位）
 * \sa              lwbtn_process_ex
 */
#define lwbtn_process(mstime)                            lwbtn_process_ex(NULL, mstime)

/**
 * \brief           处理特定按钮在默认 LwBTN 实例中的状态
 * \param[in]       btn: 要处理的按钮实例
 * \param[in]       mstime: 当前系统时间（以毫秒为单位）
 */
#define lwbtn_process_btn(btn, mstime)                   lwbtn_process_btn_ex(NULL, (btn), (mstime))

#if LWBTN_CFG_USE_KEEPALIVE || __DOXYGEN__
#if LWBTN_CFG_TIME_KEEPALIVE_PERIOD_DYNAMIC || __DOXYGEN__

/**
 * \brief           获取特定按钮的保持活动周期
 * \param[in]       btn: 要获取保持活动周期的按钮实例
 * \return          保持活动周期，单位为毫秒
 */
#define lwbtn_keepalive_get_period(btn) ((btn)->time_keepalive_period)

#else
/* 默认配置 */
#define lwbtn_keepalive_get_period(btn) (LWBTN_CFG_TIME_KEEPALIVE_PERIOD)
#endif /* LWBTN_CFG_TIME_KEEPALIVE_PERIOD_DYNAMIC || __DOXYGEN__ */

/**
 * \brief           获取自上次按下事件以来的保持活动计数
 *                  如果按钮未按下，则返回 `0`
 * \param[in]       btn: 要获取保持活动计数的按钮实例
 * \return          自上次按下事件以来的保持活动计数
 * \sa              lwbtn_keepalive_get_count_for_time
 */
#define lwbtn_keepalive_get_count(btn)                   ((btn)->keepalive.cnt)

/**
 * \brief           获取特定时间（以毫秒为单位）内的保持活动计数
 *                  计算在所请求时间内，特定按钮应进行的保持活动计数
 * 
 * 该函数的结果可以与 \ref lwbtn_keepalive_get_count 一起使用，后者返回自上次按下事件以来的实际保持活动计数。
 * 
 * \note            结果始终是整数，并且具有一个保持活动周期的粒度
 * \note            作为宏实现，以便在使用静态保持活动时，编译器能够进行优化
 * 
 * \param[in]       btn: 用于检查的按钮
 * \param[in]       ms_time: 以毫秒为单位的时间，用于计算保持活动计数
 * \return          保持活动计数
 * \sa              lwbtn_keepalive_get_count
 */
#define lwbtn_keepalive_get_count_for_time(btn, ms_time) ((ms_time) / lwbtn_keepalive_get_period(btn))

#endif /* LWBTN_CFG_USE_KEEPALIVE || __DOXYGEN__ */

/**
 * \brief           获取按钮的连续点击次数
 * \param[in]       btn: 要获取点击次数的按钮实例
 * \return          按钮的连续点击次数
 */
#define lwbtn_click_get_count(btn) ((btn)->click.cnt)

/**
 * \}
 */

#ifdef __cplusplus
}
#endif /* __cplusplus */

#endif /* LWBTN_HDR_H */
