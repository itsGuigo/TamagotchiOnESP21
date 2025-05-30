
#include "tamalib.h"
#include "hw.h"
#include "cpu.h"
#include "hal.h"

#define DEFAULT_FRAMERATE				3// fps

static exec_mode_t exec_mode = EXEC_MODE_RUN;

static u32_t step_depth = 0;

static timestamp_t screen_ts = 0;

static u32_t ts_freq;

static u8_t g_framerate = DEFAULT_FRAMERATE;

hal_t *g_hal;


bool_t tamalib_init(u32_t freq)
//bool_t tamalib_init(breakpoint_t *breakpoints, u32_t freq)
{
	bool_t res = 0;
  res |= cpu_init( freq);

//	res |= cpu_init(program, breakpoints, freq);
	res |= hw_init();

	ts_freq = freq;

	return res;
}

/*
void tamalib_release(void)
{
	hw_release();
	cpu_release();
}*/


void tamalib_set_framerate(u8_t framerate)
{
	g_framerate = framerate;
}
/*
u8_t tamalib_get_framerate(void)
{
 
	//return g_framerate;
  return DEFAULT_FRAMERATE;
}
*/
void tamalib_register_hal(hal_t *hal)
{
	g_hal = hal;
}
/*
void tamalib_set_exec_mode(exec_mode_t mode)
{
	exec_mode = mode;
	step_depth = cpu_get_depth();
	cpu_sync_ref_timestamp();
}
*/

/*
void tamalib_step(void)
{
	if (exec_mode == EXEC_MODE_PAUSE) {
		return;
	}

	if (cpu_step()) {
		exec_mode = EXEC_MODE_PAUSE;
		step_depth = cpu_get_depth();
	}  else {
		switch (exec_mode) {
			case EXEC_MODE_PAUSE:
			case EXEC_MODE_RUN:
				break;

			case EXEC_MODE_STEP:
				exec_mode = EXEC_MODE_PAUSE;
				break;

			case EXEC_MODE_NEXT:
				if (cpu_get_depth() <= step_depth) {
					exec_mode = EXEC_MODE_PAUSE;
					step_depth = cpu_get_depth();
				}
				break;

			case EXEC_MODE_TO_CALL:
				if (cpu_get_depth() > step_depth) {
					exec_mode = EXEC_MODE_PAUSE;
					step_depth = cpu_get_depth();
				}
				break;

			case EXEC_MODE_TO_RET:
				if (cpu_get_depth() < step_depth) {
					exec_mode = EXEC_MODE_PAUSE;
					step_depth = cpu_get_depth();
				}
				break;
		}
	} 
}
*/
/*
void tamalib_mainloop(void)
{
	timestamp_t ts;

	while (!g_hal->handler()) {
		tamalib_step();

		ts = g_hal->get_timestamp();
		if (ts - screen_ts >= ts_freq/g_framerate) {
			screen_ts = ts;
			g_hal->update_screen();
		}
	}
} */

void tamalib_mainloop_step_by_step(void)
{
  timestamp_t ts;

  if (!g_hal->handler()) {
    //tamalib_step();

    if (exec_mode == EXEC_MODE_RUN) {
      if (cpu_step()) {
        exec_mode = EXEC_MODE_PAUSE;
        step_depth = cpu_get_depth();
      }
    }


    /* Update the screen @ g_framerate fps */
    ts = g_hal->get_timestamp();
    
    if (ts - screen_ts >= ts_freq/g_framerate) {
    //if (ts - screen_ts >= ts_freq/DEFAULT_FRAMERATE) {
      screen_ts = ts;
      g_hal->update_screen();
    }
  }
}
