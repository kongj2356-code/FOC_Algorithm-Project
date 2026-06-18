#include "stm32f10x.h"
#include "Delay.h"
#include "bsp_foc.h"
#include "Serial.h"
#include <math.h>

#define PI 3.1415926f

extern float Ta,Tb,Tc;

float melody[] = {
  261.63,261.63,392.00,392.00,440.00,440.00,392.00,
  349.23,349.23,329.63,329.63,293.66,293.66,261.63
};

int duration[] = {
  500,500,500,500,500,500,800,
  500,500,500,500,500,500,800
};




int noteIndex = 0;
float note_time = 0;

uint32_t last_ms = 0;

int main(void)
{
    PWM_Init();
    Serial_Init();

    last_ms = SysTick->VAL;

    while (1)
    {
        /* 当前音符频率 -> 电角速度 */
        float freq = melody[noteIndex];
        float omega_e = 2.0f * PI * freq;

        /* 驱动 FOC */
        FOC_VelocityOpenLoop(omega_e);

        /* 时间累计 */
        Delay_ms(1);
        note_time += 1;

        if (note_time >= duration[noteIndex])
        {
            note_time = 0;
            noteIndex++;

            if (noteIndex >= sizeof(melody)/sizeof(melody[0]))
                noteIndex = 0;

            printf("Note=%d  Freq=%.2fHz\r\n", noteIndex, melody[noteIndex]);
        }
    }
}