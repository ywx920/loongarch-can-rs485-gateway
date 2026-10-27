#include "can_guard.h"
#include <string.h>
bool mc_guard_ready(const mc_can_guard *g,mc_millis now){return !g->pending&&!g->reset_needed&&now>=g->retry_at;}
void mc_guard_sent(mc_can_guard *g,const mc_frame *f,mc_millis now){g->write_waiting=false;g->frame=*f;g->pending=true;g->deadline=now+(g->confirm_ms?g->confirm_ms:250);++g->submitted;}
bool mc_guard_confirm(mc_can_guard *g,const mc_frame *f,mc_millis now){
    (void)now;
    if(!g->pending||g->frame.id!=f->id||g->frame.length!=f->length||memcmp(g->frame.data,f->data,f->length))return false;
    g->pending=false;++g->completed;if(g->blocked)++g->recoveries;
    g->write_waiting=false;g->blocked=false;g->failures=0;g->retry_at=0;return true;
}
void mc_guard_fail(mc_can_guard *g,mc_millis now){
    g->write_waiting=false;g->pending=false;g->blocked=true;g->reset_needed=true;
    if(g->failures<10)++g->failures;
    unsigned seconds=1u<<(g->failures-1);
    unsigned delay=seconds*1000,cap=g->retry_max_ms?g->retry_max_ms:30000;if(delay>cap)delay=cap;g->retry_at=now+delay;
}
bool mc_guard_expire(mc_can_guard *g,mc_millis now){if(!g->pending||now<g->deadline)return false;++g->timeouts;mc_guard_fail(g,now);return true;}
void mc_guard_external(mc_can_guard *g,mc_millis now){if(g->blocked&&!g->pending&&!g->reset_needed)g->retry_at=now;}

/* 只是内核暂时忙时不要每1ms重试，也不要第一下就重置整个控制器。
 * 持续250ms不能入队才进入受控恢复。没有完成/取消发送能力时，
 * 真正的在途超时仍必须清理接口，不能假装释放驱动echo记录。 */
void mc_guard_pressure(mc_can_guard *g,mc_millis now){
    if(!g->write_waiting){g->write_waiting=true;g->write_wait_since=now;}
    if(now-g->write_wait_since>=(g->confirm_ms?g->confirm_ms:250))mc_guard_fail(g,now);
    else g->retry_at=now+20;
}
const char *mc_guard_state(const mc_can_guard *g){
    if(g->reset_needed)return "recovering";
    if(g->pending)return g->blocked?"probing":"transmitting";
    if(g->write_waiting)return "backpressure";
    if(g->blocked)return "waiting_peer";
    return g->completed?"online":"waiting_first_completion";
}
