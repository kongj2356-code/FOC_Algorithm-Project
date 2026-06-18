#ifndef LWBTN_OPTS_H
#define LWBTN_OPTS_H

/* 将此文件重命名为 "lwbtn_opts.h" 以适应您的应用程序 */

/*
 * 打开 "include/lwbtn/lwbtn_opt.h" 并
 * 复制并在此处替换您希望更改的设置值
 */

#include "main.h"

#define BUTTON_COUNT (sizeof(buttons) / sizeof(buttons[0]))

extern void button_init(void);
void get_btn(void);

extern uint8_t lwbtn_keys;

#endif /* LWBTN_OPTS_H */
