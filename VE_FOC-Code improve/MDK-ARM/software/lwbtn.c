#include "lwbtn.h"
#include "stdio.h"
#include "stdint.h"
#include "header.h"

#if LWBTN_CFG_GET_STATE_MODE > 2
#error "无效的 LWBTN_GET_STATE_MODE_CALLBACK 配置"
#endif

#define LWBTN_FLAG_ONPRESS_SENT ((uint16_t)0x0001) /*!< 标志表示已发送按下事件 */
#define LWBTN_FLAG_MANUAL_STATE                                                                                        \
    ((uint16_t)0x0002) /*!< 标志表示用户希望手动设置按钮状态。不要调用 "get_state" 函数 */
#define LWBTN_FLAG_FIRST_INACTIVE_RCVD                                                                                 \
    ((uint16_t)0x0004)                      /*!< 表示等待第一个非活动状态，才继续进行 */
#define LWBTN_FLAG_RESET ((uint16_t)0x0008) /*!< 按钮已调用重置 */

#if LWBTN_CFG_TIME_DEBOUNCE_PRESS_DYNAMIC
#define LWBTN_TIME_DEBOUNCE_PRESS_GET_MIN(btn) ((lwbtn_time_t)((btn)->time_debounce))
#else
#define LWBTN_TIME_DEBOUNCE_PRESS_GET_MIN(btn) ((lwbtn_time_t)LWBTN_CFG_TIME_DEBOUNCE_PRESS)
#endif /* LWBTN_CFG_TIME_DEBOUNCE_PRESS_DYNAMIC */

#if LWBTN_CFG_TIME_DEBOUNCE_RELEASE_DYNAMIC
#define LWBTN_TIME_DEBOUNCE_RELEASE_GET_MIN(btn) ((lwbtn_time_t)((btn)->time_debounce_release))
#else
#define LWBTN_TIME_DEBOUNCE_RELEASE_GET_MIN(btn) ((lwbtn_time_t)LWBTN_CFG_TIME_DEBOUNCE_RELEASE)
#endif /* LWBTN_CFG_TIME_DEBOUNCE_RELEASE_DYNAMIC */

#if LWBTN_CFG_TIME_CLICK_MIN_DYNAMIC
#define LWBTN_TIME_CLICK_GET_PRESSED_MIN(btn) ((lwbtn_time_t)((btn)->time_click_pressed_min))
#else
#define LWBTN_TIME_CLICK_GET_PRESSED_MIN(btn) ((lwbtn_time_t)LWBTN_CFG_TIME_CLICK_MIN)
#endif /* LWBTN_CFG_TIME_CLICK_MIN_DYNAMIC */

#if LWBTN_CFG_TIME_CLICK_MAX_DYNAMIC
#define LWBTN_TIME_CLICK_GET_PRESSED_MAX(btn) ((lwbtn_time_t)((btn)->time_click_pressed_max))
#else
#define LWBTN_TIME_CLICK_GET_PRESSED_MAX(btn) ((lwbtn_time_t)LWBTN_CFG_TIME_CLICK_MAX)
#endif /* LWBTN_CFG_TIME_CLICK_MAX_DYNAMIC */

#if LWBTN_CFG_TIME_CLICK_MULTI_MAX_DYNAMIC
#define LWBTN_TIME_CLICK_MAX_MULTI(btn) ((lwbtn_time_t)((btn)->time_click_multi_max))
#else
#define LWBTN_TIME_CLICK_MAX_MULTI(btn) ((lwbtn_time_t)LWBTN_CFG_TIME_CLICK_MULTI_MAX)
#endif /* LWBTN_CFG_TIME_CLICK_MULTI_MAX_DYNAMIC */

#if LWBTN_CFG_TIME_KEEPALIVE_PERIOD_DYNAMIC
#define LWBTN_TIME_KEEPALIVE_PERIOD(btn) ((lwbtn_time_t)((btn)->time_keepalive_period))
#else
#define LWBTN_TIME_KEEPALIVE_PERIOD(btn) ((lwbtn_time_t)LWBTN_CFG_TIME_KEEPALIVE_PERIOD)
#endif /* LWBTN_CFG_TIME_KEEPALIVE_PERIOD_DYNAMIC */

#if LWBTN_CFG_CLICK_MAX_CONSECUTIVE_DYNAMIC
#define LWBTN_CLICK_MAX_CONSECUTIVE(btn) ((btn)->max_consecutive)
#else
#define LWBTN_CLICK_MAX_CONSECUTIVE(btn) LWBTN_CFG_CLICK_MAX_CONSECUTIVE
#endif /* LWBTN_CFG_CLICK_MAX_CONSECUTIVE_DYNAMIC */

/* 获取按钮状态 */
#if LWBTN_CFG_GET_STATE_MODE == LWBTN_GET_STATE_MODE_CALLBACK
#define LWBTN_BTN_GET_STATE(lwobj, btn) ((lwobj)->get_state_fn((lwobj), (btn)))
#elif LWBTN_CFG_GET_STATE_MODE == LWBTN_GET_STATE_MODE_MANUAL
#define LWBTN_BTN_GET_STATE(lwobj, btn) ((btn)->curr_state)
#elif LWBTN_CFG_GET_STATE_MODE == LWBTN_GET_STATE_MODE_CALLBACK_OR_MANUAL
#define LWBTN_BTN_GET_STATE(lwobj, btn)                                                                                \
    (((btn)->flags & LWBTN_FLAG_MANUAL_STATE)                                                                          \
         ? ((btn)->curr_state)                                                                                         \
         : (((lwobj)->get_state_fn != NULL) ? ((lwobj)->get_state_fn((lwobj), (btn))) : 0))
#endif

/* 默认按钮组实例 */
static lwbtn_t lwbtn_default;
#define LWBTN_GET_LWOBJ(in_lwobj) ((in_lwobj) != NULL ? (in_lwobj) : (&lwbtn_default))

/**
 * \brief           处理按钮信息和状态
 * 
 * \param[in]       lwobj: LwBTN 实例。如果设置为 `NULL`，则使用默认实例
 * \param[in]       btn: 要处理的按钮实例
 * \param[in]       mstime: 当前的毫秒系统时间
 */
static void
prv_process_btn(lwbtn_t* lwobj, lwbtn_btn_t* btn, lwbtn_time_t mstime) {
    uint8_t new_state;

    /* 获取按钮状态 */
    new_state = LWBTN_BTN_GET_STATE(lwobj, btn);

    /* 
     * 第一个状态必须是“非活动”状态，
     * 才能进一步处理按钮状态。
     * 
     * 这是为了防止在硬件故障时检测到初始状态，
     * 或者在系统/库重置后按钮一直被按下的情况。
     * 
     * 当用户使用手动状态设置（没有回调系统）
时，
     * 用户需要先调用“设置状态”函数并将状态设置为非活动状态。
     * 
     * 这个特性也用于“按钮重置”。
     */
    if (!(btn->flags & LWBTN_FLAG_FIRST_INACTIVE_RCVD)) {
        if (new_state) {
            return;
        }

        /* 重置所有状态 */
        btn->last_state = 0;
        btn->flags = LWBTN_FLAG_FIRST_INACTIVE_RCVD;
    }

#if 0
    /*
     * 当按钮被按下时，用户可以手动
     * 重置按钮
     */
    if ((btn->flags) & LWBTN_FLAG_RESET) {
        btn->last_state = 0;             /* 禁用状态 */
        btn->flags &= LWBTN_FLAG_RESET; /* 保留重置标志，删除其他标志 */
        if (new_state) {
            return;
        }
        btn->flags &= ~LWBTN_FLAG_RESET; /* 重新开始 */
    }
#endif

    /* 按钮状态刚刚发生了变化 */
    if (new_state != btn->last_state) {
        btn->time_state_change = mstime;
    }

    /* 按钮仍然被按下 */
    else if (new_state) {
        /* 
         * 处理去抖动并发送按下事件
         *
         * 这是当我们检测到有效按下时
         */
        if (!(btn->flags & LWBTN_FLAG_ONPRESS_SENT)) {
            /*
             * 在以下情况下执行 if 语句：
             *
             * - 启用运行时模式 -> 用户设置了自己的去抖动配置
             * - 配置的按下去抖动时间大于 `0`
             */
#if LWBTN_CFG_TIME_DEBOUNCE_PRESS_DYNAMIC || LWBTN_CFG_TIME_DEBOUNCE_PRESS > 0
            if ((lwbtn_time_t)(mstime - btn->time_state_change) >= LWBTN_TIME_DEBOUNCE_PRESS_GET_MIN(btn))
#endif /* LWBTN_CFG_TIME_DEBOUNCE_PRESS_DYNAMIC || LWBTN_CFG_TIME_DEBOUNCE_PRESS> 0 */
            {
#if !LWBTN_CFG_CLICK_MAX_CONSECUTIVE_SEND_IMMEDIATELY
                /*
                 * 根据配置，
                 * 当达到最大连续点击次数时，在下一个按下释放之前发送点击事件。
                 */
                if (btn->click.cnt > 0 && btn->click.cnt == LWBTN_CLICK_MAX_CONSECUTIVE(btn)) {
                    lwobj->evt_fn(lwobj, btn, LWBTN_EVT_ONCLICK);
                    btn->click.cnt = 0;
                }
#endif /* !LWBTN_CFG_CLICK_MAX_CONSECUTIVE_SEND_IMMEDIATELY */

                /* 开始新的按下事件 */
                btn->flags |= LWBTN_FLAG_ONPRESS_SENT;
                lwobj->evt_fn(lwobj, btn, LWBTN_EVT_ONPRESS);
#if LWBTN_CFG_USE_KEEPALIVE
                /* 设置保持活动时间 */
                btn->keepalive.last_time = mstime;
                btn->keepalive.cnt = 0;
#endif /* LWBTN_CFG_USE_KEEPALIVE */

                btn->time_change = mstime; /* 按钮状态已发生变化 */
            }
#if LWBTN_CFG_USE_KEEPALIVE
        } else {
            /*
             * 处理保持活动，只在发送按下事件后执行
             *
             * 当检测到有效按下时发送保持活动事件
             */
            while ((lwbtn_time_t)(mstime - btn->keepalive.last_time) >= LWBTN_TIME_KEEPALIVE_PERIOD(btn)) {
                btn->keepalive.last_time += LWBTN_TIME_KEEPALIVE_PERIOD(btn);
                ++btn->keepalive.cnt;
                lwobj->evt_fn(lwobj, btn, LWBTN_EVT_KEEPALIVE);
            }
#endif /* LWBTN_CFG_USE_KEEPALIVE */
        }
    }

    /* 按钮仍然处于释放状态 */
    else {
        /*
         * 只有在按下事件已经开始时，我们才需要做出反应。
         *
         * 如果没有按下事件，则不做任何操作
         */
        if (btn->flags & LWBTN_FLAG_ONPRESS_SENT) {
            /*
             * 在以下情况下执行 if 语句：
             *
             * - 启用运行时模式 -> 用户设置了自己的去抖动配置
             * - 配置的释放去抖动时间大于 `0`
             */
#if LWBTN_CFG_TIME_DEBOUNCE_RELEASE_DYNAMIC || LWBTN_CFG_TIME_DEBOUNCE_RELEASE > 0
            if ((mstime - btn->time_state_change) >= LWBTN_TIME_DEBOUNCE_RELEASE_GET_MIN(btn))
#endif
 /* LWBTN_CFG_TIME_DEBOUNCE_RELEASE_DYNAMIC || LWBTN_CFG_TIME_DEBOUNCE_RELEASE > 0 */
            {
                /* Handle on-release event */
                btn->flags &= ~LWBTN_FLAG_ONPRESS_SENT;
                lwobj->evt_fn(lwobj, btn, LWBTN_EVT_ONRELEASE);

#if LWBTN_CFG_USE_CLICK
                /* 检查点击事件的时间有效性 */
                if ((lwbtn_time_t)(mstime - btn->time_change) >= LWBTN_TIME_CLICK_GET_PRESSED_MIN(btn)
                    && (lwbtn_time_t)(mstime - btn->time_change) <= LWBTN_TIME_CLICK_GET_PRESSED_MAX(btn)) {

                    /*
                     * 如果最大连续点击次数没有达到，
                     * 并且两次点击之间的时间没有太长，
                     * 则增加连续点击次数。
                     * 
                     * 否则将点击视为新的点击事件。
                     */
                    if (btn->click.cnt > 0 && btn->click.cnt < LWBTN_CLICK_MAX_CONSECUTIVE(btn)
                        && (lwbtn_time_t)(mstime - btn->click.last_time) < LWBTN_TIME_CLICK_MAX_MULTI(btn)) {
                        ++btn->click.cnt;
                    } else {
                        /*
                         * 如果新点击不再符合当前上下文，处理之前的点击事件。
                         *
                         * 这种情况只有在按下事件开始时间早于最大连续点击时间，
                         * 而释放事件发生在最大连续点击时间之后时才会发生。
                         * 
                         * 在这种情况下，简单地报告前一个状态，然后设置新的点击。
                         */
                        if (btn->click.cnt > 0) {
                            lwobj->evt_fn(lwobj, btn, LWBTN_EVT_ONCLICK);
                        }
                        btn->click.cnt = 1;
                    }
                    btn->click.last_time = mstime;
                } else {
#if LWBTN_CFG_CLICK_CONSECUTIVE_KEEP_AFTER_SHORT_PRESS
                    /* 如果上次按下时间过短，并且前一个点击序列是有效的，向用户发送事件 */
                    if (btn->click.cnt > 0
                        && (lwbtn_time_t)(mstime - btn->time_change) < LWBTN_TIME_CLICK_GET_PRESSED_MIN(btn)) {
                        lwobj->evt_fn(lwobj, btn, LWBTN_EVT_ONCLICK);
                    }
#endif /* LWBTN_CFG_CLICK_CONSECUTIVE_KEEP_AFTER_SHORT_PRESS */
                    /*
                     * 如果存在释放事件，但点击事件的检测时间超出了允许的窗口，
                     * 则重置点击计数器 -> 此序列不再有效作为点击事件。
                     */
                    btn->click.cnt = 0;
                }

#if LWBTN_CFG_CLICK_MAX_CONSECUTIVE_SEND_IMMEDIATELY
                /* 
                 * 根据配置，当达到最大连续点击次数时，
                 * 在释放事件之后立即发送点击事件。
                 */
                if (btn->click.cnt > 0 && btn->click.cnt == LWBTN_CLICK_MAX_CONSECUTIVE(btn)) {
                    lwobj->evt_fn(lwobj, btn, LWBTN_EVT_ONCLICK);
                    btn->click.cnt = 0;
                }
#endif /* LWBTN_CFG_CLICK_MAX_CONSECUTIVE_SEND_IMMEDIATELY */
#endif /* LWBTN_CFG_USE_CLICK */

                btn->time_change = mstime; /* 按钮状态已发生变化 */
            }
#if LWBTN_CFG_USE_CLICK

        } else {
            /* 
             * 根据配置，这部分代码将在特定超时时间后发送点击事件。
             * 
             * 如果用户希望多次点击功能，并且只有在最后一次点击事件发生后才报告，
             * 包括用户所做的点击次数，这个特性将非常有用。
             */
            if (btn->click.cnt > 0) {
                if ((lwbtn_time_t)(mstime - btn->click.last_time) >= LWBTN_TIME_CLICK_MAX_MULTI(btn)) {
                    lwobj->evt_fn(lwobj, btn, LWBTN_EVT_ONCLICK);
                    btn->click.cnt = 0;
                }
            }
#endif /* LWBTN_CFG_USE_CLICK */
        }
    }

    btn->last_state = new_state;
}

/**
 * \brief           初始化按钮管理器
 * \param[in]       lwobj: LwBTN 实例。如果设置为 `NULL`，则使用默认实例
 * \param[in]       btns: 要处理的按钮数组
 * \param[in]       btns_cnt: 要处理的按钮数量
 * \param[in]       get_state_fn: 提供按钮状态的函数指针。如果启用了 \ref LWBTN_CFG_GET_STATE_MODE 为手动模式，则该参数可设置为 `NULL`。
 * \param[in]       evt_fn: 按钮事件回调函数
 * \return          成功时返回 `1`，否则返回 `0`
 */
uint8_t
lwbtn_init_ex(lwbtn_t* lwobj, lwbtn_btn_t* btns, uint16_t btns_cnt, lwbtn_get_state_fn get_state_fn,
              lwbtn_evt_fn evt_fn) {
    lwobj = LWBTN_GET_LWOBJ(lwobj);

    if (btns == NULL || btns_cnt == 0 || evt_fn == NULL
#if LWBTN_CFG_GET_STATE_MODE == LWBTN_GET_STATE_MODE_CALLBACK
        || get_state_fn == NULL /* 仅在回调模式下必须提供此参数 */
#endif                          /* LWBTN_CFG_GET_STATE_MODE == LWBTN_GET_STATE_MODE_CALLBACK */
    ) {
        return 0;
    }

    LWBTN_MEMSET(lwobj, 0x00, sizeof(*lwobj));
    lwobj->btns = btns;
    lwobj->btns_cnt = btns_cnt;
    lwobj->evt_fn = evt_fn;
#if LWBTN_CFG_GET_STATE_MODE != LWBTN_GET_STATE_MODE_MANUAL
    lwobj->get_state_fn = get_state_fn;
#else
    (void)get_state_fn; /* 可能未使用 */
#endif /* LWBTN_CFG_GET_STATE_MODE != LWBTN_GET_STATE_MODE_MANUAL */

    for (size_t i = 0; i < btns_cnt; ++i) {
#if LWBTN_CFG_TIME_DEBOUNCE_PRESS_DYNAMIC
        btns[i].time_debounce = LWBTN_CFG_TIME_DEBOUNCE_PRESS;
#endif /* LWBTN_CFG_TIME_DEBOUNCE_PRESS_DYNAMIC */
#if LWBTN_CFG_TIME_DEBOUNCE_RELEASE_DYNAMIC
        btns[i].time_debounce_release = LWBTN_CFG_TIME_DEBOUNCE_RELEASE;
#endif /* LWBTN_CFG_TIME_DEBOUNCE_RELEASE_DYNAMIC */
#if LWBTN_CFG_TIME_CLICK_MIN_DYNAMIC
        btns[i].time_click_pressed_min = LWBTN_CFG_TIME_CLICK_MIN;
#endif /* LWBTN_CFG_TIME_CLICK_MIN_DYNAMIC */
#if LWBTN_CFG_TIME_CLICK_MAX_DYNAMIC
        btns[i].time_click_pressed_max = LWBTN_CFG_TIME_CLICK_MAX;
#endif /* LWBTN_CFG_TIME_CLICK_MAX_DYNAMIC */
#if LWBTN_CFG_TIME_CLICK_MULTI_MAX_DYNAMIC
        btns[i].time_click_multi_max = LWBTN_CFG_TIME_CLICK_MULTI_MAX;
#endif /* LWBTN_CFG_TIME_CLICK_MULTI_MAX_DYNAMIC */
#if LWBTN_CFG_TIME_KEEPALIVE_PERIOD_DYNAMIC
        btns[i].time_keepalive_period = LWBTN_CFG_TIME_KEEPALIVE_PERIOD;
#endif /* LWBTN_CFG_TIME_KEEPALIVE_PERIOD_DYNAMIC */
#if LWBTN_CFG_CLICK_MAX_CONSECUTIVE_DYNAMIC
        btns[i].max_consecutive = LWBTN_CFG_CLICK_MAX_CONSECUTIVE;
#endif /* LWBTN_CFG_CLICK_MAX_CONSECUTIVE_DYNAMIC */
    }

    return 1;
}

/**
 * \brief           按钮处理函数，用于读取输入并执行相应动作
 * 
 * 该函数检查所有与指定LwBTN实例(组)关联的按钮状态。
 * 
 * \param[in]       lwobj: LwBTN实例。设为`NULL`使用默认实例
 * \param[in]       mstime: 当前系统时间（毫秒）
 * \return          `1` 成功, `0` 失败
 */
uint8_t lwbtn_process_ex(lwbtn_t* lwobj, lwbtn_time_t mstime) {
    lwobj = LWBTN_GET_LWOBJ(lwobj);

    /* 处理所有按钮 */
    for (size_t index = 0; index < lwobj->btns_cnt; ++index) {
        prv_process_btn(lwobj, &lwobj->btns[index], mstime);
    }
    return 1;
}

/**
 * \brief           处理指定LwOBJ实例（组）中的单个按钮。
 * 
 * 当应用需要在中断触发时处理按钮事件时，可使用此功能。
 * 它为用户提供了更高的自主权来决定何时处理特定按钮。
 * 
 * \param[in]       lwobj: LwBTN实例。设为`NULL`使用默认实例
 * \param[in]       btn: 按钮对象。不可为`NULL`
 * \param[in]       mstime: 当前系统时间（毫秒）
 * \return          `1` 成功, `0` 失败
 */
uint8_t
lwbtn_process_btn_ex(lwbtn_t* lwobj, lwbtn_btn_t* btn, lwbtn_time_t mstime) {
    if (btn != NULL) {
        prv_process_btn(LWBTN_GET_LWOBJ(lwobj), btn, mstime);
        return 1;
    }
    return 0;
}

/**
 * \brief           设置按钮状态为"激活"或"未激活"。
 * \param[in]       btn: 按钮实例
 * \param[in]       state: 新状态。`1`为激活（按下），`0`为未激活（释放）
 * \return          `1` 成功（非回调模式时），`0` 失败（回调模式时）
 */
uint8_t
lwbtn_set_btn_state(lwbtn_btn_t* btn, uint8_t state) {
#if LWBTN_CFG_GET_STATE_MODE != LWBTN_GET_STATE_MODE_CALLBACK
    btn->curr_state = state;
    btn->flags |= LWBTN_FLAG_MANUAL_STATE;
    return 1;
#else  /* 非回调模式时才允许手动设置状态 */
    (void)btn;
    (void)state;
    return 0;
#endif
}

/**
 * \brief           检查按钮是否处于激活状态。
 * 激活状态指经过初始消抖周期后的持续按下状态（介于按下事件和释放事件之间）。
 * 
 * \param[in]       btn: 要检查的按钮句柄
 * \return          `1` 激活状态, `0` 未激活或无效句柄
 */
uint8_t
lwbtn_is_btn_active(const lwbtn_btn_t* btn) {
    return btn != NULL && (btn->flags & LWBTN_FLAG_ONPRESS_SENT);
}

/**
 * \brief           重置按钮状态到初始值
 * 
 *                  当按钮处于按下状态时调用重置，将停止后续事件触发，
 *                  直到检测到新的有效按下动作。
 *
 *                  典型用例：当应用需要处理长按操作（多个保持事件）时，
 *                  例如GUI界面中长按切换屏幕后，新界面需要不同的按钮处理逻辑。
 *                  调用此函数可强制用户必须松开并重新按下按钮才能触发新事件。
 *
 * \note            重置处于激活状态的按钮后，该按钮将不再触发任何事件，
 *                  直到检测到新的有效按下动作
 *
 * \param           lwobj: 要重置的LwBTN对象（设为非NULL时重置该对象所有按钮）
 * \param           btn: 要重置的单个按钮对象（可选参数，非NULL时生效）
 * \return          `1` 成功, `0` 失败
 */
uint8_t
lwbtn_reset(lwbtn_t* lwobj, lwbtn_btn_t* btn) {
    for (size_t idx = 0; idx < (lwobj != NULL ? lwobj->btns_cnt : 0); ++idx) {
        lwobj->btns[idx].flags &= ~LWBTN_FLAG_FIRST_INACTIVE_RCVD;
    }
    if (btn != NULL) {
        btn->flags &= ~LWBTN_FLAG_FIRST_INACTIVE_RCVD;
    }
    return 1;
}
