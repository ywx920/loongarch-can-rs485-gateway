#ifndef MC_CAN_RATE_H
#define MC_CAN_RATE_H
#include "monitor.h"
/* 07速率切换扩展：旧速率接受回复→延迟1秒→新速率同帧确认→持久化。
 * 只认完整匹配的请求，双路模式必须分别在两路确认；10秒未确认恢复旧速率。
 * 与业务引擎解耦，以回调测试驱动失败/掉电前未提交/持久化失败等路径。 */
typedef struct {
    unsigned rates[2],old[2],count,mask,confirmed,target,phase;
    bool both,fault;mc_frame request;mc_millis due,expires;
} mc_rate;
typedef bool (*mc_rate_apply)(void *,unsigned,unsigned);
typedef bool (*mc_rate_save)(void *,const unsigned [2]);
typedef void (*mc_rate_reply)(void *,const mc_frame *,bool);
bool mc_rate_valid(unsigned);
void mc_rate_init(mc_rate *,unsigned,const unsigned [2],bool);
bool mc_rate_receive(mc_rate *,const mc_frame *,unsigned,mc_millis,mc_rate_reply,void *);
void mc_rate_tick(mc_rate *,mc_millis,bool,mc_rate_apply,mc_rate_save,mc_rate_reply,void *);
void mc_rate_cancel(mc_rate *,mc_rate_apply,void *);
#endif
