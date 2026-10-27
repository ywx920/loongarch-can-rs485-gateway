#include "io_runtime.h"
#include <errno.h>
#include <string.h>
/* 复制一次完整RTU请求到固定缓冲。上一条尚未结束时拒绝覆盖。
 * 请求一般8或13字节，16字节容量足够当前03/04/06/固定双字10。 */
bool mc_serial_begin(mc_serial_tx *t,const uint8_t *data,size_t n,mc_millis deadline) {
    if(t->active||!n||n>sizeof(t->data)) return false;
    memcpy(t->data,data,n);t->length=n;t->offset=0;t->deadline=deadline;t->active=true;return true;
}
/* 从offset继续写，成功多少就推进多少；绝不回到0重新发送完整帧。
 * EAGAIN留给下次poll循环，EINTR只在本轮有界重试，其余错误上报失败。
 * deadline约束提交阶段，不与设备响应timeout_ms混用。 */
int mc_serial_progress(mc_serial_tx *t,mc_millis now,mc_write_fn write_fn,void *context) {
    if(!t->active) return 0;
    if(now>=t->deadline) {t->active=false;return -1;}
    for(unsigned i=0;i<16;++i) {
        ssize_t n=write_fn(context,t->data+t->offset,t->length-t->offset);
        if(n>0&&(size_t)n<=t->length-t->offset) {
            t->offset+=(size_t)n;
            if(t->offset==t->length) {t->active=false;return 1;}
        } else if(n<0&&errno==EINTR) continue;
        else if(n<0&&(errno==EAGAIN||errno==EWOULDBLOCK)) return 0;
        else {t->active=false;return -1;}
    }
    return 0;
}
static void remove_group(mc_can_tx *q,size_t i) {
    --q->count;memmove(q->groups+i,q->groups+i+1,(q->count-i)*sizeof(q->groups[0]));
}
/* 队列单位是“业务组”而非单帧。相同优先级FIFO；高优先级只插在
 * 尚未开始的低优先级组之前，已开始组发送完才能切换。
 * 预留应答容量，周期流量不能挤满所有空间。内核CAN队列仍有独立背压。 */
bool mc_can_enqueue(mc_can_tx *q,const mc_frame *f,size_t n,mc_millis now,unsigned stale_ms) {
    if(!n||n>15) {++q->dropped;return false;}
    unsigned type=mc_decode_id(f[0].id).type;
    unsigned priority=(type==1||type==7||type==8)?0:(type==3?1:(type==4?2:3));
    /* 留16组给控制/参数/授权应答，再留16组给查询，低优先级不能占满队列。满时不截断旧组。 */
    if(q->count>=MC_TX_GROUPS||(priority>0&&q->count>=MC_TX_GROUPS-16)||(priority>1&&q->count>=MC_TX_GROUPS-32)) {++q->dropped;return false;}
    size_t position=q->count;
    for(size_t i=0;i<q->count;++i) if(!q->groups[i].offset&&q->groups[i].priority>priority) {position=i;break;}
    memmove(q->groups+position+1,q->groups+position,(q->count-position)*sizeof(q->groups[0]));
    mc_tx_group *g=&q->groups[position];memset(g,0,sizeof(*g));
    memcpy(g->frames,f,n*sizeof(*f));g->count=(unsigned)n;g->priority=priority;
    /* 应答排队最多250ms；周期报文沿用采样时刻的真实过期时间，不能出队后续命。 */
    g->expires=type==2?(f[0].expires_at?f[0].expires_at:now+stale_ms):now+250;++q->count;return true;
}
/* 读队头时清理已过期组，包括已经发出一部分的剩余帧。
 * 无法撤回已上线的CAN帧，因此对端也必须有多帧组包超时。 */
const mc_frame *mc_can_front(mc_can_tx *q,mc_millis now) {
    while(q->count&&now>=q->groups[0].expires) {++q->dropped;remove_group(q,0);}
    if(!q->count||now<q->next) return NULL;
    return &q->groups[0].frames[q->groups[0].offset];
}
void mc_can_advance(mc_can_tx *q,bool success,mc_millis now,unsigned gap) {
    if(!q->count) return;
    if(!success) {++q->dropped;remove_group(q,0);}
    else if(++q->groups[0].offset==q->groups[0].count) remove_group(q,0);
    q->next=now+gap;
}
