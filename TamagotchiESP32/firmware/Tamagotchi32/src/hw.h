
#ifndef _HW_H_
#define _HW_H_

#include "hal.h"

#define LCD_WIDTH			32
#define LCD_HEIGHT			16

#define ICON_NUM			8

typedef enum {
	BTN_STATE_RELEASED = 0,
	BTN_STATE_PRESSED,
} btn_state_t;

typedef enum {
	BTN_LEFT = 0,
	BTN_MIDDLE,
	BTN_RIGHT,
} button_t;


#ifdef __cplusplus
 extern "C" {
#endif

bool_t hw_init(void);
void hw_release(void);

void hw_set_lcd_pin(u8_t seg, u8_t com, u8_t val);
void hw_set_button(button_t btn, btn_state_t state);

void hw_set_buzzer_freq(u4_t freq);
void hw_enable_buzzer(bool_t en);

#ifdef __cplusplus
}
#endif

#endif /* _HW_H_ */
