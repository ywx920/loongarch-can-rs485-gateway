#define _GNU_SOURCE
#include "board_led.h"
#include <fcntl.h>
#include <stdint.h>
#include <string.h>
#include <sys/mman.h>
#include <unistd.h>

/* 沿用已用C++板端模块的LS2K0300寄存器和原理图GPIO映射。
 * OEN为低有效；先输出低，再配置方向，避免上电闪烁。 */
static const unsigned serial_gpio[8]={17,16,15,14,3,2,1,12};
static const unsigned panel_gpio[4]={21,20,25,26}; /* CAN0/CAN1/RUN红/RUN绿 */
static int write_gpio(mc_led *l,unsigned gpio,bool level) {
    volatile uint8_t *reg=(volatile uint8_t *)l->gpio_page+0x900+gpio;
    *reg=level?1:0;__sync_synchronize();return ((*reg&1)==(unsigned)level)?0:-1;
}
static int mux(mc_led *l,unsigned gpio) {
    volatile uint32_t *reg=(volatile uint32_t *)((uint8_t *)l->system_page+0x490+(gpio/16)*4);
    uint32_t mask=(uint32_t)3<<((gpio%16)*2);*reg&=~mask;__sync_synchronize();if(*reg&mask) return -1;
    if(write_gpio(l,gpio,false)) return -1;
    volatile uint8_t *oen=(volatile uint8_t *)l->gpio_page+0x800+gpio;*oen=0;__sync_synchronize();return (*oen&1)?-1:0;
}
static int serial_level(mc_led *l,unsigned i,bool level) {
    if(l->channels[i].level==level) return 0;
    if(write_gpio(l,serial_gpio[i],level)) return -1;
    l->channels[i].level=level;return 0;
}
static int panel(mc_led *l,bool normal,bool level) {
    return write_gpio(l,21,normal&&level)||write_gpio(l,20,normal&&level)||write_gpio(l,25,!normal)||write_gpio(l,26,normal&&level)?-1:0;
}
int mc_led_init(mc_led *l,bool enabled) {
    memset(l,0,sizeof(*l));l->fd=-1;if(!enabled) return 0;
    l->fd=open("/dev/mem",O_RDWR|O_SYNC|O_CLOEXEC);if(l->fd<0) return -1;
    l->system_page=mmap(NULL,0x1000,PROT_READ|PROT_WRITE,MAP_SHARED,l->fd,0x16000000);
    if(l->system_page==MAP_FAILED) {l->system_page=NULL;goto failed;}
    l->gpio_page=mmap(NULL,0x1000,PROT_READ|PROT_WRITE,MAP_SHARED,l->fd,0x16104000);
    if(l->gpio_page==MAP_FAILED) {l->gpio_page=NULL;goto failed;}
    for(unsigned i=0;i<8;++i) if(mux(l,serial_gpio[i])) goto failed;
    for(unsigned i=0;i<4;++i) if(mux(l,panel_gpio[i])) goto failed;
    l->ready=true;if(panel(l,false,false)) goto failed;return 0;
failed:
    mc_led_close(l);return -1;
}
int mc_led_enable(mc_led *l,unsigned physical,bool enabled) {
    if(!l->ready) return 0;
    if(physical<1||physical>8) return -1;
    mc_led_channel *c=&l->channels[physical-1];c->enabled=enabled;c->blinking=false;return serial_level(l,physical-1,enabled);
}
int mc_led_activity(mc_led *l,unsigned physical,mc_millis now) {
    if(!l->ready) return 0;
    if(physical<1||physical>8) return -1;
    mc_led_channel *c=&l->channels[physical-1];if(!c->enabled) return 0;c->activity=now;
    if(!c->blinking) {c->blinking=true;c->next_toggle=now+80;return serial_level(l,physical-1,false);}return 0;
}
int mc_led_normal(mc_led *l,bool normal,mc_millis now) {
    if(!l->ready||normal==l->normal) return 0;
    l->normal=normal;l->heartbeat=normal;l->next_heartbeat=normal?now+500:0;return panel(l,normal,normal);
}
int mc_led_update(mc_led *l,mc_millis now) {
    if(!l->ready) return 0;
    for(unsigned i=0;i<8;++i) {
        mc_led_channel *c=&l->channels[i];bool level=c->level;
        if(!c->enabled) level=false;
        else if(!c->blinking||now-c->activity>=400) {c->blinking=false;level=true;}
        else if(now>=c->next_toggle) {level=!level;do {c->next_toggle+=80;} while(now>=c->next_toggle);}
        if(serial_level(l,i,level)) return -1;
    }
    /* 双CAN及RUN绿同步1Hz闪烁；单路/控制器异常时红灯常亮。
     * 它表示本地控制器状态，不代表对端设备在线。 */
    if(l->normal&&now>=l->next_heartbeat) {
        l->heartbeat=!l->heartbeat;do {l->next_heartbeat+=500;} while(now>=l->next_heartbeat);
        return panel(l,true,l->heartbeat);
    }return 0;
}
void mc_led_close(mc_led *l) {
    if(l->ready) {
        for(unsigned i=0;i<8;++i) (void)write_gpio(l,serial_gpio[i],false);
        (void)panel(l,false,false); /* 正常退出或错误退出均留下RUN红。 */
    }
    if(l->gpio_page) munmap(l->gpio_page,0x1000);
    if(l->system_page) munmap(l->system_page,0x1000);
    if(l->fd>=0) close(l->fd);
    l->gpio_page=NULL;l->system_page=NULL;l->fd=-1;l->ready=false;
}
