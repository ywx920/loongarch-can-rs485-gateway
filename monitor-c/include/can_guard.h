#ifndef MC_CAN_GUARD_H
#define MC_CAN_GUARD_H
#include "monitor.h"
/* 每路仅1帧在途。write成功只代表入队；MSG_CONFIRM自己的回送才代表驱动完成。
 * 不探测未知设备、不发额外协议帧。无ACK时用正常待发业务做有限尝试。
 * 超时250ms，退避1/2/4/8/16/30秒；外部有效帧到来可提前唤醒，禁止重放485写。 */
typedef struct {
    unsigned confirm_ms,retry_max_ms;
    bool write_waiting;mc_millis write_wait_since;
    bool pending,blocked,reset_needed;mc_frame frame;
    mc_millis deadline,retry_at;unsigned failures;
    uint64_t submitted,completed,timeouts,recoveries,resets;
} mc_can_guard;
void mc_guard_sent(mc_can_guard *,const mc_frame *,mc_millis);
bool mc_guard_confirm(mc_can_guard *,const mc_frame *,mc_millis);
bool mc_guard_expire(mc_can_guard *,mc_millis);
void mc_guard_fail(mc_can_guard *,mc_millis);
bool mc_guard_ready(const mc_can_guard *,mc_millis);
void mc_guard_external(mc_can_guard *,mc_millis);
const char *mc_guard_state(const mc_can_guard *);
void mc_guard_pressure(mc_can_guard *,mc_millis);
#endif
