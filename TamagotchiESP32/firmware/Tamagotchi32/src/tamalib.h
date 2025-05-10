
#ifndef _TAMALIB_H_
#define _TAMALIB_H_

#include "cpu.h"
#include "hw.h"
#include "hal.h"

#define tamalib_set_button(btn, state)			hw_set_button(btn, state)

#define tamalib_set_speed(speed)			cpu_set_speed(speed)

//#define tamalib_get_state()				cpu_get_state()
#define tamalib_refresh_hw()				cpu_refresh_hw()

#define tamalib_reset()					cpu_reset()

//#define tamalib_add_bp(list, addr)			cpu_add_bp(list, addr)
//#define tamalib_free_bp(list)				cpu_free_bp(list)

typedef enum {
	EXEC_MODE_PAUSE,
	EXEC_MODE_RUN,
	EXEC_MODE_STEP,
	EXEC_MODE_NEXT,
	EXEC_MODE_TO_CALL,
	EXEC_MODE_TO_RET,
} exec_mode_t;


#ifdef __cplusplus
 extern "C" {
#endif

//void tamalib_release(void);
bool_t tamalib_init(u32_t freq);
//bool_t tamalib_init( breakpoint_t *breakpoints, u32_t freq);


void tamalib_set_framerate(u8_t framerate);
//u8_t tamalib_get_framerate(void);

void tamalib_register_hal(hal_t *hal);

//void tamalib_set_exec_mode(exec_mode_t mode);

/* NOTE: Only one of these two functions must be used in the main application
 * (tamalib_step() should be used only if tamalib_mainloop() does not fit the
 * main application execution flow).
 */
//void tamalib_step(void);
//void tamalib_mainloop(void);
void tamalib_mainloop_step_by_step(void);
#ifdef __cplusplus
}
#endif

#endif /* _TAMALIB_H_ */
