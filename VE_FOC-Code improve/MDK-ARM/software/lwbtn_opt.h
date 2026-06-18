/**
 * \file            lwbtn_opt.h
 * \brief           LwBTN 选项
 */
#ifndef LWBTN_OPT_H
#define LWBTN_OPT_H

/* 取消注释以忽略用户选项（或在编译器标志中设置宏） */
/* #define LWBTN_IGNORE_USER_OPTS */

/* 包含应用程序选项 */
#ifndef LWBTN_IGNORE_USER_OPTS
#include "lwbtn_opts.h"
#endif /* lwbtn_IGNORE_USER_OPTS */

#ifdef __cplusplus
extern "C" {
#endif /* __cplusplus */

/**
 * \defgroup        LWBTN_OPT 配置
 * \brief           默认配置设置
 * \{
 */

/**
 * \brief           内存设置函数
 * 
 * \note            函数的功能与 \ref memset 相同
 */
#ifndef LWBTN_MEMSET
#define LWBTN_MEMSET(dst, val, len) memset((dst), (val), (len))
#endif


/**
 * \brief           内存复制函数
 * 
 * \note            函数的功能与 \ref memcpy 相同
 */
#ifndef LWBTN_MEMCPY
#define LWBTN_MEMCPY(dst, src, len) memcpy((dst), (src), (len))
#endif

/**
 * \brief           启用 `1` 或禁用 `0` 周期性保持活动事件
 * 
 * 保持活动事件会周期性地发送到应用程序，
 * 当输入保持在活动状态时。
 * 
 * 默认的保持活动周期由 \ref LWBTN_CFG_TIME_KEEPALIVE_PERIOD 宏设置
 */
#ifndef LWBTN_CFG_USE_KEEPALIVE
#define LWBTN_CFG_USE_KEEPALIVE 1
#endif


/**
 * \brief           启用 `1` 或禁用 `0` 点击事件管理
 * 
 * 点击事件是指按钮经过去抖动后，在有效时间内按下和释放的事件。
 * 
 * 当此功能被禁用时，LwBTN 仅作为一个简单的去抖动库，
 * 只会向用户提供“按下”和“释放”事件等。
 */
#ifndef LWBTN_CFG_USE_CLICK
#define LWBTN_CFG_USE_CLICK 1
#endif

/**
 * \brief           按下事件的最小去抖动时间（单位：毫秒）
 * 
 * 这是输入保持稳定活动状态以检测有效 *按下* 事件所需的时间。
 * 
 * 当值设置为 `> 0` 时，输入必须至少保持活动状态最小毫秒时间，
 * 才能检测到有效的 *按下* 事件。
 * 
 * \note            如果值设置为 `0`，则不使用去抖动，并且在输入状态变为 *非活动* 状态时，
 *                  *按下* 事件会立即触发。
 *  
 *                  为了安全起见，如果不使用此功能，外部逻辑必须确保输入状态的稳定过渡。
 * 
 * \sa              LWBTN_CFG_TIME_DEBOUNCE_PRESS_DYNAMIC
 */
#ifndef LWBTN_CFG_TIME_DEBOUNCE_PRESS
#define LWBTN_CFG_TIME_DEBOUNCE_PRESS 20
#endif


/**
 * \brief           启用 `1` 或禁用 `0` 动态可设置去抖动时间
 * 
 * 启用时，会向按钮结构中添加额外的字段，允许每个按钮设置自己独有的按下事件去抖动时间。
 * 
 * **如果未使用此功能，默认将使用 \ref LWBTN_CFG_TIME_DEBOUNCE_PRESS 配置为去抖动时间。**
 */
#ifndef LWBTN_CFG_TIME_DEBOUNCE_PRESS_DYNAMIC
#define LWBTN_CFG_TIME_DEBOUNCE_PRESS_DYNAMIC 0
#endif

/**
 * \brief           释放事件的最小去抖动时间（单位：毫秒）
 * 
 * 这是输入必须保持最低稳定释放电平的时间，以便检测有效的 *释放* 事件。
 * 
 * 如果应用程序希望防止输入被认为“活动”时出现不必要的波动，可以使用此设置。
 * 
 * 当值设置为 `> 0` 时，输入必须保持低电平（非活动状态）至少最小毫秒时间，
 * 才能检测到有效的 *释放* 事件。
 * 
 * \note            如果值设置为 `0`，则不使用去抖动，并且当输入状态变为 *非活动* 状态时，
 *                  *释放* 事件会立即触发。
 * 
 * \sa              LWBTN_CFG_TIME_DEBOUNCE_RELEASE_DYNAMIC
 */
#ifndef LWBTN_CFG_TIME_DEBOUNCE_RELEASE
#define LWBTN_CFG_TIME_DEBOUNCE_RELEASE 20
#endif

/**
 * \brief           启用 `1` 或禁用 `0` 动态可设置释放事件的去抖动时间
 * 
 * 启用时，会向按钮结构中添加额外的字段，允许每个按钮设置自己独有的释放事件去抖动时间。
 * 
 * **如果未使用此功能，默认将使用 \ref LWBTN_CFG_TIME_DEBOUNCE_RELEASE 配置为去抖动时间。**
 */
#ifndef LWBTN_CFG_TIME_DEBOUNCE_RELEASE_DYNAMIC
#define LWBTN_CFG_TIME_DEBOUNCE_RELEASE_DYNAMIC 0
#endif

/**
 * \brief           有效点击事件的最小活动输入时间，单位为毫秒
 * 
 * 输入在活动状态（去抖动后）至少持续该时间，才能考虑其为潜在的有效点击事件。
 * 设置该值为 `0` 以禁用此功能。
 * 
 * \sa              LWBTN_CFG_TIME_CLICK_MIN_DYNAMIC
 */
#ifndef LWBTN_CFG_TIME_CLICK_MIN
#define LWBTN_CFG_TIME_CLICK_MIN 20
#endif


/**
 * \brief           启用 `1` 或禁用 `0` 动态可设置最小点击时间
 * 
 * 启用时，会向按钮结构中添加额外的字段，允许应用程序为每个按钮实例手动设置最小按下时间以触发点击。
 * **默认值设置为 \ref LWBTN_CFG_TIME_CLICK_MIN**
 */
#ifndef LWBTN_CFG_TIME_CLICK_MIN_DYNAMIC
#define LWBTN_CFG_TIME_CLICK_MIN_DYNAMIC 0
#endif

/**
 * \brief           有效点击事件的最大活动输入时间，单位为毫秒
 * 
 * 输入按下的时间不得超过此最大值，才能触发有效的点击事件。
 * 设置为 `-1` 可允许任意时间触发点击事件。
 * 
 * 当输入处于活动状态超过配置的时间时，点击事件将不会被检测到并将被忽略。
 * 
 * \sa              LWBTN_CFG_TIME_CLICK_MAX_DYNAMIC
 */
#ifndef LWBTN_CFG_TIME_CLICK_MAX
#define LWBTN_CFG_TIME_CLICK_MAX 300
#endif

/**
 * \brief           启用 `1` 或禁用 `0` 动态可设置最大点击时间
 * 
 * 启用时，会向按钮结构中添加额外的字段，允许应用程序为每个按钮实例手动设置最大按下时间以触发点击。
 * **默认值设置为 \ref LWBTN_CFG_TIME_CLICK_MAX**
 */
#ifndef LWBTN_CFG_TIME_CLICK_MAX_DYNAMIC
#define LWBTN_CFG_TIME_CLICK_MAX_DYNAMIC 0
#endif

/**
 * \brief           最长允许的时间，介于最后一次释放事件和下一次有效按下事件之间，
 *                  以允许多次点击事件，单位为毫秒
 * 
 * 此值也用作超时长度，将 *onclick* 事件发送给应用程序，基于之前检测到的有效点击事件。
 * 
 * 如果应用程序依赖于连续的多次点击，这将是允许用户触发潜在新点击的最大时间，
 * 否则结构会在发送给用户之前重置（如果已检测到任何点击）。
 * 
 * \sa              LWBTN_CFG_TIME_CLICK_MULTI_MAX_DYNAMIC
 */
#ifndef LWBTN_CFG_TIME_CLICK_MULTI_MAX
#define LWBTN_CFG_TIME_CLICK_MULTI_MAX 400
#endif

/**
 * \brief           启用 `1` 或禁用 `0` 动态可设置多次点击的最大时间
 * 
 * 启用时，会向按钮结构中添加额外的字段，允许应用程序为每个按钮实例手动设置多次点击的最大时间。
 * **默认值设置为 \ref LWBTN_CFG_TIME_CLICK_MULTI_MAX**
 */
#ifndef LWBTN_CFG_TIME_CLICK_MULTI_MAX_DYNAMIC
#define LWBTN_CFG_TIME_CLICK_MULTI_MAX_DYNAMIC 0
#endif


/**
 * \brief           允许的最大连续点击事件数，
 *                  超过此次数后，结构将重置为默认值。
 * 
 * \note            当连续点击次数达到设定值时，应用程序将收到点击通知。
 *                  这可以在检测到最后一次点击后立即执行，
 *                  或在标准超时后执行（除非已经检测到下一个按下事件，
 *                  那么将在有效的下一个按下事件之前发送给应用程序）。
 *                  配置可以通过 \ref LWBTN_CFG_CLICK_MAX_CONSECUTIVE_SEND_IMMEDIATELY 宏进行更改。
 * 
 * \sa              LWBTN_CFG_CLICK_MAX_CONSECUTIVE_DYNAMIC, LWBTN_CFG_CLICK_MAX_CONSECUTIVE_SEND_IMMEDIATELY
 */
#ifndef LWBTN_CFG_CLICK_MAX_CONSECUTIVE
#define LWBTN_CFG_CLICK_MAX_CONSECUTIVE 2
#endif

/**
 * \brief           启用 `1` 或禁用 `0` 动态可设置最大连续点击次数
 * 
 * 启用时，会向按钮结构中添加额外的字段，允许应用程序为每个按钮实例手动设置最大连续点击次数。
 * **默认值设置为 \ref LWBTN_CFG_CLICK_MAX_CONSECUTIVE**
 */
#ifndef LWBTN_CFG_CLICK_MAX_CONSECUTIVE_DYNAMIC
#define LWBTN_CFG_CLICK_MAX_CONSECUTIVE_DYNAMIC 0
#endif

/**
 * \brief           保持活动事件周期，单位为毫秒
 * 
 * 当输入处于活动状态时，保持活动事件将在此时间段内发送。
 * 第一个保持活动事件将在输入被认为处于活动状态后 \ref LWBTN_CFG_TIME_KEEPALIVE_PERIOD 毫秒发送。
 * 
 * \sa              LWBTN_CFG_TIME_KEEPALIVE_PERIOD_DYNAMIC
 */
#ifndef LWBTN_CFG_TIME_KEEPALIVE_PERIOD
#define LWBTN_CFG_TIME_KEEPALIVE_PERIOD 500
#endif

/**
 * \brief           启用 `1` 或禁用 `0` 动态可设置保持活动周期
 * 
 * 启用时，会向按钮结构中添加额外的字段，允许应用程序为每个按钮实例手动设置保持活动周期。
 * **默认值设置为 \ref LWBTN_CFG_TIME_KEEPALIVE_PERIOD**
 */
#ifndef LWBTN_CFG_TIME_KEEPALIVE_PERIOD_DYNAMIC
#define LWBTN_CFG_TIME_KEEPALIVE_PERIOD_DYNAMIC 0
#endif

/**
 * \brief           启用 `1` 或禁用 `0` 在连续点击次数达到最大值后，立即发送点击事件 
 *                  在释放事件后。
 * 
 * 当此模式被禁用时，点击事件将在以下两种情况下发送：
 * 
 * - 发生了点击超时
 * - 在超时到期之前发生了下一个按下事件
 */
#ifndef LWBTN_CFG_CLICK_MAX_CONSECUTIVE_SEND_IMMEDIATELY
#define LWBTN_CFG_CLICK_MAX_CONSECUTIVE_SEND_IMMEDIATELY 1
#endif

/**
 * \brief           获取按钮状态选项
 * \name            LWBTN_CFG_GET_STATE_MODE_GROUP
 * \anchor          LWBTN_CFG_GET_STATE_MODE_GROUP
 * \{
 * 
 * \ref LWBTN_CFG_GET_STATE_MODE 配置的配置选项
 */

#define LWBTN_GET_STATE_MODE_CALLBACK           0 /*!< 仅回调状态模式 */
#define LWBTN_GET_STATE_MODE_MANUAL             1 /*!< 仅手动状态模式 */
#define LWBTN_GET_STATE_MODE_CALLBACK_OR_MANUAL 2 /*!< 回调或手动状态模式 */


/**
 * \}
 */

/**
 * \brief           设置如何获取按钮状态的模式。
 * 
 * 提供了不同的模式，可以通过级别编号进行设置：
 * 
 * - `LWBTN_GET_STATE_MODE_CALLBACK`：通过 *获取状态* 回调函数检查按钮状态
 * - `LWBTN_GET_STATE_MODE_MANUAL`：仅启用手动状态设置。应用程序必须使用 API 函数设置按钮状态。
 *          不使用回调 API。
 * - `LWBTN_GET_STATE_MODE_CALLBACK_OR_MANUAL`：通过 *获取状态* 回调函数检查按钮状态（默认情况）。
 *          它允许应用程序通过适当的函数调用手动设置按钮状态。
 *          在调用手动状态 API 函数之前，按钮状态至少会通过回调检查。
 * 
 *          这允许为不同的按钮类型提供多个构建配置。
 * 
 * \note            在手动模式下，应用程序有责任
 *                  使用 \ref lwbtn_set_btn_state 函数手动将初始状态设置为“非活动”。
 */
#ifndef LWBTN_CFG_GET_STATE_MODE
#define LWBTN_CFG_GET_STATE_MODE LWBTN_GET_STATE_MODE_CALLBACK
#endif

/**
 * \brief           启用 `1` 或禁用 `0` 保留连续点击事件组，如果最后一次
 *                  *按下* 和 *释放* 的时间间隔过短。
 * 
 */
#ifndef LWBTN_CFG_CLICK_CONSECUTIVE_KEEP_AFTER_SHORT_PRESS
#define LWBTN_CFG_CLICK_CONSECUTIVE_KEEP_AFTER_SHORT_PRESS 0
#endif

/**
 * \brief           默认的时间值变量类型。
 * 
 *                  该值定义了时间类型（时间始终以毫秒为单位），
 *                  并允许用户根据运行库的系统使用 32 位或 16 位时间。
 * 
 * \note            如果系统使用 `16 位` 的定时器进行系统时钟计时，
 *                  可以将此配置设置为 `uint16_t`。
 * 
 */
#ifndef LWBTN_CFG_TIME_VARTYPE
#define LWBTN_CFG_TYPE_VARTYPE uint32_t
#endif


/**
 * \}
 */

#ifdef __cplusplus
}
#endif /* __cplusplus */

#endif /* LWBTN_OPT_HDR_H */
