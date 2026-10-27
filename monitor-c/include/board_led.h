#ifndef ZXJB14A_BOARD_LED_C_H
#define ZXJB14A_BOARD_LED_C_H
#include "monitor.h"

/* 仅--board-leds时访问/dev/mem；默认完全不碰GPIO寄存器。 */
typedef struct {bool enabled,level,blinking;mc_millis activity,next_toggle;} mc_led_channel;
typedef struct {
    int fd;void *system_page,*gpio_page;bool ready,normal,heartbeat;
    mc_millis next_heartbeat;
    mc_led_channel channels[8];
} mc_led;
int mc_led_init(mc_led *led,bool enabled);
int mc_led_enable(mc_led *led,unsigned physical,bool enabled);
int mc_led_activity(mc_led *led,unsigned physical,mc_millis now);
int mc_led_normal(mc_led *led,bool normal,mc_millis now);
int mc_led_update(mc_led *led,mc_millis now);
void mc_led_close(mc_led *led);
#endif
