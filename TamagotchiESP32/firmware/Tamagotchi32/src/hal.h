
#ifndef _HAL_H_
#define _HAL_H_

#include "hal_types.h"

#ifndef NULL
	#define NULL 0
#endif

typedef enum {
	LOG_ERROR	= 0x1,
	LOG_INFO	= (0x1 << 1),
	LOG_MEMORY	= (0x1 << 2),
	LOG_CPU		= (0x1 << 3),
} log_level_t;

/* The Hardware Abstraction Layer
 * NOTE: This structure acts as an abstraction layer between TamaLIB and the OS/SDK.
 * All pointers MUST be implemented, but some implementations can be left empty.
 */
typedef struct {
	/* Memory allocation functions
	 * NOTE: Needed only if breakpoints support is required.
	 */
	//void * (*malloc)(u32_t size);
	//void (*free)(void *ptr);

	/* What to do if the CPU has halted
	 */
	void (*halt)(void);

	/* Log related function
	 * NOTE: Needed only if log messages are required.
	 */
	//bool_t (*is_log_enabled)(log_level_t level);
	void (*log)(log_level_t level, char *buff, ...);

	/* Clock related functions
	 * NOTE: Timestamps granularity is configured with tamalib_init(), an accuracy
	 * of ~30 us (1/32768) is required for a cycle accurate emulation.
	 */
	void (*sleep_until)(timestamp_t ts);
	timestamp_t (*get_timestamp)(void);

	/* Screen related functions
	 * NOTE: In case of direct hardware access to pixels, the set_XXXX() functions
	 * (called for each pixel/icon update) can directly drive them, otherwise they
	 * should just store the data in a buffer and let update_screen() do the actual
	 * rendering (at 30 fps).
	 */
	void (*update_screen)(void);
	void (*set_lcd_matrix)(u8_t x, u8_t y, bool_t val);
	void (*set_lcd_icon)(u8_t icon, bool_t val);

	/* Sound related functions
	 * NOTE: set_frequency() changes the output frequency of the sound, while
	 * play_frequency() decides whether the sound should be heard or not.
	 */
	void (*set_frequency)(u32_t freq);
	void (*play_frequency)(bool_t en);

	/* Event handler from the main app (if any)
	 * NOTE: This function usually handles button related events, states loading/saving ...
	 */
	int (*handler)(void);
} hal_t;

extern hal_t *g_hal;

#endif /* _HAL_H_ */
