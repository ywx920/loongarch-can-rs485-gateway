#include "monitor.h"
#include "sha256.h"
#include <stdio.h>
#include <limits.h>
#include <stdlib.h>
#include <string.h>

enum { PERIODIC,CONTROL,QUERY };
/* 一个job对应一条完整RTU事务，origin用于响应回原CAN入口。
 * sent_at只在请求提交完整后用于响应超时；expires_at限制控制/查询的排队等待。 */
typedef struct {
    unsigned kind,slave,function,start,count,timeout_ms,retries_left;
    uint8_t request[16];size_t length;
    mc_frame origin;
    mc_millis sent_at,expires_at;
    uint64_t authority_generation; /* 绑定受理时的租约，重新申请不能复活旧控制。 */
} job;
typedef struct {bool valid;uint16_t words[60];size_t count;mc_millis updated;} cache;
typedef struct {
    job queue[MC_QUEUE],active;size_t queued;
    bool pending,transmitting,unavailable;
    uint8_t input[512];size_t input_length;
    mc_millis quiet_until,next_poll[16];
    cache values[16];
    /* 按从站地址1～15直接索引；诊断历史不与周期缓存一起清零。 */
    mc_device_status diagnostics[16];
} port_state;
typedef struct {
    bool used,completed;
    uint32_t id;uint8_t data[8];mc_millis first_seen;
} duplicate;
/* 模拟IO通道：真实板卡接入前只保存寄存器镜像，方便验证DO/DI/AI的CAN打包。
 * 该状态不持久化、不代表现场输出；进程重启后由tick用确定序列重新播种。 */
typedef struct {bool valid;uint16_t regs[60];mc_millis updated;} io_state;
/* 08授权租约：channel=0表示全局租约；到期即失效，不依赖日历时间。 */
typedef struct {bool used;unsigned source,channel;mc_millis expires;uint64_t generation;} lease;
/* 授权请求没有事务号：固定去重窗口内的同ID/同载荷视为同一请求。
 * 独立于01去重，跨CAN入口复用结果；不续租、不再次抢占或写盘。 */
typedef struct {
    bool used;mc_frame request;mc_millis first_seen;unsigned reply;uint64_t generation;
} authority_duplicate;
struct mc_engine {
    mc_config config;
    mc_persist_fn persist;void *persist_context;
    mc_stats stats;
    bool async_tx;mc_event_fn event;void *event_context;
    port_state states[MC_PORTS]; /* 按physical-1索引，不依赖配置排列。 */
    duplicate duplicates[512];
    io_state io[MC_ROUTES];
    lease leases[MC_LEASES];
    uint64_t next_lease_generation;
    authority_duplicate authority_duplicates[64];
    unsigned authority_last_reply;
    bool io_simulation_paused; /* 测试注入：暂停采样，保留缓存以验证过期停报。 */
    mc_frame output[MC_OUTPUT];size_t out_head,out_count;
    mc_millis next_report,next_heartbeat;
    uint8_t heartbeat;
    unsigned heartbeat_channel;
    mc_millis heartbeat_due;
    uint8_t channel_counter[8];
    mc_device gateway_device; /* 单次接收使用的临时描述；队列只保存值，不持有此指针。 */
};

/* 失败计数按“事务失败”增加。CRC错误只增加错误字节流计数，
 * 因为同一次事务可能随后收到正确帧，不能每个错字节都算一次失败。 */
static void failed(mc_engine *e,unsigned physical,const job *j,mc_result reason,mc_millis now,unsigned detail) {
    mc_device_status *d=&e->states[physical-1].diagnostics[j->slave];
    d->last_result=reason;
    if(d->consecutive_failures<UINT_MAX) ++d->consecutive_failures;
    if(reason==MC_RESULT_TIMEOUT) ++d->timeouts;
    else if(reason==MC_RESULT_TX_ERROR) ++d->tx_errors;
    else if(reason==MC_RESULT_DISCONNECTED) ++d->disconnects;
    else if(reason==MC_RESULT_EXCEPTION) {
        ++d->exceptions;d->last_exception=detail;d->has_response=true;d->last_response_ms=now;
    }
}
static void succeeded(mc_engine *e,unsigned physical,const job *j,mc_millis now) {
    mc_device_status *d=&e->states[physical-1].diagnostics[j->slave];
    ++d->successes;d->consecutive_failures=0;d->last_result=MC_RESULT_OK;d->last_exception=0;
    d->has_success=d->has_response=true;d->last_success_ms=d->last_response_ms=now;
    if(j->kind==PERIODIC) {d->has_sample=true;d->last_sample_ms=now;}
}
const char *mc_result_name(mc_result result) {
    switch(result) {
    case MC_RESULT_OK:return "ok";case MC_RESULT_TIMEOUT:return "timeout";
    case MC_RESULT_EXCEPTION:return "modbus_exception";case MC_RESULT_TX_ERROR:return "serial_tx_error";
    case MC_RESULT_DISCONNECTED:return "disconnected";default:return "none";
    }
}
int mc_device_snapshot(const mc_engine *e,unsigned physical,unsigned slave,mc_millis now,mc_device_status *out) {
    if(!e||!out||physical<1||physical>MC_PORTS||slave<1||slave>MC_DEVICES) return -1;
    const mc_port *p=NULL;bool found=false;
    for(size_t i=0;i<e->config.port_count;++i) if(e->config.ports[i].physical==physical) p=&e->config.ports[i];
    if(!p) return -1;
    for(size_t i=0;i<p->device_count;++i) if(p->devices[i].slave==slave) found=true;
    if(!found) return -1;
    const port_state *s=&e->states[physical-1];*out=s->diagnostics[slave];
    out->physical=physical;out->channel=p->channel;out->slave=slave;out->enabled=p->enabled;
    out->port_available=p->enabled&&!s->unavailable;
    out->communication_online=out->port_available&&out->has_response&&now>=out->last_response_ms&&now-out->last_response_ms<=e->config.stale_ms;
    out->cache_fresh=out->port_available&&s->values[slave].valid&&now>=s->values[slave].updated&&now-s->values[slave].updated<=e->config.stale_ms;
    out->pending=s->pending&&s->active.slave==slave;out->transmitting=out->pending&&s->transmitting;out->queued=0;
    for(size_t i=0;i<s->queued;++i) if(s->queue[i].slave==slave) ++out->queued;
    return 0;
}

/* 诊断仅进入本地日志，不改变DW报文格式。 */
static void event(mc_engine *e,const char *reason,unsigned physical,const job *j,mc_millis now,unsigned detail) {
    if(!e->event) return;
    unsigned channel=0;
    for(size_t i=0;i<e->config.port_count;++i) if(e->config.ports[i].physical==physical) channel=e->config.ports[i].channel;
    mc_event v={reason,physical,channel,j?j->slave:0,j?j->function:0,j?j->start:0,j?j->count:0,j?j->kind:0,detail,now};
    e->event(e->event_context,&v);
}
void mc_engine_events(mc_engine *e,mc_event_fn callback,void *context) {e->event=callback;e->event_context=context;}
void mc_engine_async(mc_engine *e,bool enabled) {e->async_tx=enabled;}
/* 异步发送完成通知：完成仅指TTY已接收整帧，尚不代表从站处理成功。
 * 失败必须终止本次事务，不把完整写命令重新排队，避免部分已发情况下重复动作。 */
void mc_serial_tx_complete(mc_engine *e,unsigned physical,bool success,mc_millis now) {
    if(physical<1||physical>MC_PORTS) return;
    port_state *s=&e->states[physical-1];
    if(!s->pending||!s->transmitting) return;
    s->transmitting=false;s->active.sent_at=now;
    if(!success) {
        failed(e,physical,&s->active,MC_RESULT_TX_ERROR,now,0);
        ++e->stats.serial_tx_error;event(e,"serial_tx_error",physical,&s->active,now,0);
        s->pending=false;s->input_length=0;s->values[s->active.slave].valid=false;
        s->quiet_until=now+s->active.timeout_ms;
    } else event(e,"sent",physical,&s->active,now,0);
}
void mc_port_available(mc_engine *e,unsigned physical,bool available,mc_millis now) {
    if(physical<1||physical>MC_PORTS) return;
    port_state *s=&e->states[physical-1];
    if(s->unavailable==!available) return;
    /* 保留历史成功时间，但断开后必须等新回复才能重新标记在线。
     * 只对被中断的在途事务增加失败，未发出的队列撤销不伪造超时。 */
    if(!available) {
        if(s->pending) {failed(e,physical,&s->active,MC_RESULT_DISCONNECTED,now,0);event(e,"aborted_disconnect",physical,&s->active,now,0);}
        for(unsigned i=1;i<=MC_DEVICES;++i) s->diagnostics[i].has_response=false;
    }
    s->unavailable=!available;s->pending=false;s->transmitting=false;s->queued=0;s->input_length=0;
    memset(s->values,0,sizeof(s->values));memset(s->next_poll,0,sizeof(s->next_poll));s->quiet_until=now+500;
    event(e,available?"port_online":"port_offline",physical,NULL,now,0);
}
static bool contains(const unsigned *items,size_t count,unsigned value) {
    for(size_t i=0;i<count;++i) if(items[i]==value) return true;
    return false;
}
static bool in_ranges(const mc_range *ranges,size_t count,unsigned start,unsigned n) {
    for(size_t i=0;i<count;++i) if(start>=ranges[i].start&&start+n<=ranges[i].start+ranges[i].count) return true;
    return false;
}
/* 有界队列使用数组移动实现优先插入；最多256项，避免链表分配及生命周期问题。 */
static void insert_job(port_state *s,size_t position,const job *j) {
    memmove(s->queue+position+1,s->queue+position,(s->queued-position)*sizeof(job));
    s->queue[position]=*j;++s->queued;
}
static job pop_job(port_state *s) {
    job j=s->queue[0];--s->queued;memmove(s->queue,s->queue+1,s->queued*sizeof(job));return j;
}
static size_t bus_count(const mc_engine *e) {return e->config.primary_only?1:e->config.can_count;}
static void emit(mc_engine *e,mc_id id,const uint8_t data[8],int only_bus) {
    /* 冗余链路主动双发，某一路故障不改变业务目标；写操作仍跨路去重。 */
    if(e->config.customer_gateway&&(id.type==1||id.type==3)) only_bus=-1;
    size_t n=only_bus>=0?1:bus_count(e);uint32_t encoded;
    /* 一次双网发送整体预留容量，满时整组拒绝，不越界也不写半组。 */
    if(e->out_count+n>MC_OUTPUT||mc_encode_id(id,&encoded)) {++e->stats.queue_full;return;}
    for(size_t i=0;i<n;++i) {
        mc_frame *f=&e->output[(e->out_head+e->out_count)%MC_OUTPUT];
        f->expires_at=0;f->bus=only_bus>=0?only_bus:(int)i;f->id=encoded;f->length=8;memcpy(f->data,data,8);++e->out_count;
    }
}
mc_engine *mc_engine_create(const mc_config *c,mc_persist_fn persist,void *context,char *err,size_t cap) {
    if(mc_config_validate(c,err,cap)) return NULL;
    mc_engine *e=calloc(1,sizeof(*e));
    if(!e) {if(cap) snprintf(err,cap,"allocate protocol engine failed");return NULL;}
    e->config=*c;e->persist=persist;e->persist_context=context;return e;
}
bool mc_engine_busy(const mc_engine *e) {for(unsigned i=0;i<MC_PORTS;++i)if(e->states[i].pending||e->states[i].queued)return true;return false;}
void mc_engine_destroy(mc_engine *e) {free(e);}
const mc_config *mc_engine_config(const mc_engine *e) {return &e->config;}
mc_stats *mc_engine_stats(mc_engine *e) {return &e->stats;}
size_t mc_output_count(const mc_engine *e) {return e->out_count;}
bool mc_take_output(mc_engine *e,mc_frame *f) {
    if(!e->out_count) return false;
    *f=e->output[e->out_head];e->out_head=(e->out_head+1)%MC_OUTPUT;--e->out_count;return true;
}
static const mc_device *find_device(mc_engine *e,mc_id id,const mc_port **port) {
    *port=NULL;
    for(size_t i=0;i<e->config.port_count;++i) {
        const mc_port *p=&e->config.ports[i];
        if(p->enabled&&p->channel==id.channel) {
            *port=p;
            if(e->config.customer_gateway&&id.slave>=1&&id.slave<=15) {
                mc_device *d=&e->gateway_device;memset(d,0,sizeof(*d));
                d->slave=id.slave;d->timeout_ms=500;d->control_enabled=d->query_enabled=true;
                d->allowed_write_functions[0]=6;d->allowed_write_functions[1]=16;d->allowed_write_count=2;
                d->readable[0]=(mc_range){0,65536};d->writable[0]=(mc_range){0,65536};
                d->readable_count=d->writable_count=1;return d;
            }
            for(size_t k=0;k<p->device_count;++k) if(p->devices[k].slave==id.slave) return &p->devices[k];
        }
    }
    return NULL;
}
static bool same_key(const mc_frame *a,const mc_frame *b) {return a->id==b->id&&memcmp(a->data,b->data,8)==0;}
static int duplicate_index(mc_engine *e,const mc_frame *f) {
    for(unsigned i=0;i<512;++i) if(e->duplicates[i].used&&e->duplicates[i].id==f->id&&memcmp(e->duplicates[i].data,f->data,8)==0) return (int)i;
    return -1;
}
static void control_ack(mc_engine *e,const mc_frame *origin) {
    mc_id id=mc_decode_id(origin->id);id.destination=id.source;id.source=e->config.address;id.sequence=1;
    emit(e,id,origin->data,origin->bus);++e->stats.control_acks;
}
/* ---- 1.4.0：通道路由、模拟IO与08授权 ---- */
static int route_find(const mc_engine *e,unsigned channel,unsigned kind) {
    for(size_t i=0;i<e->config.route_count;++i) {
        const mc_route *r=&e->config.routes[i];
        if(r->channel==channel&&r->kind==kind) return (int)i;
    }
    return -1;
}
/* 全局范围与任意通道重叠；覆盖判定与冲突判定不能混用。 */
static bool lease_overlaps(unsigned a,unsigned b) {return !a||!b||a==b;}
static int lease_holder(const mc_engine *e,unsigned channel,mc_millis now) {
    for(unsigned i=0;i<MC_LEASES;++i) {
        const lease *l=&e->leases[i];
        if(l->used&&now<l->expires&&lease_overlaps(l->channel,channel)) return (int)i;
    }
    return -1;
}
static uint64_t authority_generation(const mc_engine *e,unsigned source,unsigned channel,mc_millis now) {
    for(unsigned i=0;i<MC_LEASES;++i) {
        const lease *l=&e->leases[i];
        if(l->used&&now<l->expires&&l->source==source&&(!l->channel||l->channel==channel)) return l->generation;
    }
    return 0;
}
/* 未启用授权保持原行为；启用后无有效租约即拒绝。 */
bool mc_authority_allows(const mc_engine *e,unsigned source,unsigned channel,mc_millis now) {
    if(!e) return false;
    return !e->config.authorization_enabled||authority_generation(e,source,channel,now)!=0;
}
static bool job_authorized(const mc_engine *e,const job *j,mc_millis now) {
    if(j->kind!=CONTROL||!e->config.authorization_enabled) return true;
    mc_id id=mc_decode_id(j->origin.id);
    return j->authority_generation!=0&&j->authority_generation==authority_generation(e,id.source,id.channel,now);
}
/* 只撤销尚未交给串口适配层的控制；已经提交/部分写出的事务继续收尾，
 * 绝不整帧重试。保留去重记录至窗口结束，避免旧报文重传复活被撤销动作。 */
static void authority_prune(mc_engine *e,mc_millis now) {
    if(!e->config.authorization_enabled) return;
    for(size_t i=0;i<e->config.port_count;++i) {
        port_state *s=&e->states[e->config.ports[i].physical-1];
        size_t kept=0;
        for(size_t k=0;k<s->queued;++k) {
            if(!job_authorized(e,&s->queue[k],now)) {
                event(e,"authority_revoked",e->config.ports[i].physical,&s->queue[k],now,0);++e->stats.rejected;++e->stats.controls_cancelled;
            } else s->queue[kept++]=s->queue[k];
        }
        s->queued=kept;
    }
}
static bool lease_grant(mc_engine *e,unsigned source,unsigned channel,mc_millis now,bool force) {
    /* 不允许代次回绕后再次匹配旧任务。实际寿命内不可能用尽uint64_t。 */
    if(e->next_lease_generation==UINT64_MAX) return false;
    if(force) for(unsigned i=0;i<MC_LEASES;++i)
        if(e->leases[i].used&&lease_overlaps(e->leases[i].channel,channel)) e->leases[i].used=false;
    int slot=-1;
    for(unsigned i=0;i<MC_LEASES&&slot<0;++i) if(!e->leases[i].used||now>=e->leases[i].expires) slot=(int)i;
    if(slot<0) return false;
    lease *l=&e->leases[slot];
    l->used=true;l->source=source;l->channel=channel;l->generation=++e->next_lease_generation;
    l->expires=now+(mc_millis)e->config.authorization_lease_ms;
    authority_prune(e,now);return true;
}
static bool authority_code_matches(const mc_engine *e,const mc_frame *f) {
    char digest[65];mc_sha256_hex(f->data+2,6,digest);
    return e->config.authorization_sha256[0]&&!strcmp(digest,e->config.authorization_sha256);
}
static void authority_reply(mc_engine *e,mc_id id,const mc_frame *f,unsigned code) {
    e->authority_last_reply=code;
    if(code==4||code==8) ++e->stats.authority_denied;
    uint8_t data[8];memcpy(data,f->data,8);data[0]=(uint8_t)code;
    mc_id reply=id;reply.source=id.destination?id.destination:e->config.address;reply.destination=id.source;
    emit(e,reply,data,f->bus);
}
/* 08骨架：授权码取Byte2..7六字节ASCII；1/2/9用最小租约，3/4/7/8/10是应答码。
 * 仲裁、租期、抢占规则尚未由协议方定版，默认关闭；不要据此声称08已验收。 */
static void authority_execute(mc_engine *e,mc_id id,const mc_frame *f,mc_millis now) {
    if(!id.destination||id.sequence||(!e->config.customer_gateway&&!contains(e->config.control_sources,e->config.control_source_count,id.source))) {++e->stats.rejected;return;}
    unsigned function=f->data[0];
    switch(function) {
    case 1:
        if(lease_holder(e,id.channel,now)>=0) {authority_reply(e,id,f,4);break;}
        if(lease_grant(e,id.source,id.channel,now,false)) authority_reply(e,id,f,3);else authority_reply(e,id,f,4);
        break;
    case 2:
        /* 功能2必须随本次请求校验六字节授权码；功能5成功不能替代它。 */
        if(!e->config.authorization_allow_force||!authority_code_matches(e,f)) {authority_reply(e,id,f,4);break;}
        if(lease_grant(e,id.source,id.channel,now,true)) authority_reply(e,id,f,3);else authority_reply(e,id,f,4);
        break;
    case 5: {
        bool ok=authority_code_matches(e,f);
        authority_reply(e,id,f,ok?7:8);break;
    }
    case 6: {
        if(!mc_authority_allows(e,id.source,id.channel,now)) {authority_reply(e,id,f,8);break;}
        mc_config candidate=e->config;mc_sha256_hex(f->data+2,6,candidate.authorization_sha256);
        char error[160];
        if(mc_config_validate(&candidate,error,sizeof(error))||!e->persist||e->persist(e->persist_context,&candidate,error,sizeof(error))) authority_reply(e,id,f,8);
        else {e->config=candidate;authority_reply(e,id,f,7);}
        break;
    }
    case 9: {
        /* 释放必须指定原租约范围，子通道请求不能暗中释放全局租约。 */
        int slot=lease_holder(e,id.channel,now);
        if(slot>=0&&e->leases[slot].source==id.source&&e->leases[slot].channel==id.channel) {
            e->leases[slot].used=false;authority_prune(e,now);authority_reply(e,id,f,10);
        }
        else authority_reply(e,id,f,4);
        break;
    }
    case 3:case 4:case 7:case 8:case 10:break; /* 应答码，不是请求 */
    default:++e->stats.unsupported;break;
    }
}
static void authority(mc_engine *e,mc_id id,const mc_frame *f,mc_millis now) {
    ++e->stats.authority_requests;
    /* 先检查来源与寻址，缓存命中也不能绕过准入。 */
    if(!id.destination||id.sequence||(!e->config.customer_gateway&&!contains(e->config.control_sources,e->config.control_source_count,id.source))) {++e->stats.rejected;return;}
    unsigned function=f->data[0];
    bool mutating=function==1||function==2||function==6||function==9;
    if(!mutating) {authority_execute(e,id,f,now);return;}
    int free_slot=-1;
    for(unsigned i=0;i<64;++i) {
        authority_duplicate *d=&e->authority_duplicates[i];
        if(d->used&&now-d->first_seen>=e->config.dedup_ms) d->used=false;
        if(!d->used) {if(free_slot<0) free_slot=(int)i;continue;}
        if(d->request.id!=f->id||memcmp(d->request.data,f->data,8)) continue;
        unsigned reply=d->reply;
        uint64_t current=authority_generation(e,id.source,id.channel,now);
        /* 已释放/被抢占/到期的成功不能重放为新授权；更新旧码也不能回滚。 */
        if(function==1||function==2) {if(!current||current!=d->generation) reply=4;}
        else if(function==6) {if(!current||current!=d->generation||!authority_code_matches(e,f)) reply=8;}
        else if(function==9&&current) reply=4; /* 新租约存在，旧释放报文不释放它。 */
        ++e->stats.authority_replays;authority_reply(e,id,f,reply);return;
    }
    if(free_slot<0) {++e->stats.queue_full;authority_reply(e,id,f,function==6?8:4);return;}
    e->authority_last_reply=0;authority_execute(e,id,f,now);
    /* 拒绝结果不缓存，条件改变后允许重新申请；成功必须固定窗口，不能滑动续期。 */
    if(e->authority_last_reply==3||e->authority_last_reply==7||e->authority_last_reply==10) {
        authority_duplicate *d=&e->authority_duplicates[free_slot];
        d->used=true;d->request=*f;d->first_seen=now;d->reply=e->authority_last_reply;
        d->generation=authority_generation(e,id.source,id.channel,now);
    }
}
size_t mc_authority_snapshot(const mc_engine *e,mc_millis now,mc_authority_status *out,size_t capacity) {
    if(!e||!out) return 0;
    size_t n=0;
    for(unsigned i=0;i<MC_LEASES&&n<capacity;++i) {
        const lease *l=&e->leases[i];if(!l->used||now>=l->expires) continue;
        out[n++]=(mc_authority_status){l->source,l->channel,l->expires,l->generation};
    }
    return n;
}
/* DO控制使用通用01布局：start(2)+count(2)+value(2)+value(2)，CAN ID从站字段为0。
 * 模拟后端直接写寄存器镜像并立即ACK；真实板卡必须替换为“写后读回/驱动确认”。 */
static void do_control(mc_engine *e,mc_id id,const mc_frame *f,mc_millis now,int route_index) {
    const mc_route *r=&e->config.routes[route_index];
    if(!id.destination&&!e->config.allow_broadcast_control) {++e->stats.rejected;return;}
    if(id.sequence||(!e->config.customer_gateway&&!contains(e->config.control_sources,e->config.control_source_count,id.source))) {++e->stats.rejected;return;}
    if(!mc_authority_allows(e,id.source,id.channel,now)) {++e->stats.rejected;return;}
    unsigned start=mc_read_be(f->data),count=mc_read_be(f->data+2);
    if(id.slave||count<1||count>2||start<r->reg_start||start+count>r->reg_start+r->reg_count) {++e->stats.rejected;return;}
    int index=duplicate_index(e,f),free_index=-1;size_t used=0;
    if(index>=0&&now-e->duplicates[index].first_seen<e->config.dedup_ms) {
        ++e->stats.duplicate;if(e->duplicates[index].completed) control_ack(e,f);return;
    }
    for(unsigned i=0;i<512;++i) {if(e->duplicates[i].used) ++used;else if(free_index<0) free_index=(int)i;}
    if(used>=512) {++e->stats.queue_full;return;}
    for(unsigned i=0;i<count;++i) e->io[route_index].regs[start-r->reg_start+i]=mc_read_be(f->data+4+i*2);
    e->io[route_index].valid=true;e->io[route_index].updated=now;
    duplicate *dup=&e->duplicates[index>=0?index:free_index];
    dup->used=true;dup->completed=true;dup->id=f->id;memcpy(dup->data,f->data,8);dup->first_seen=now;
    ++e->stats.controls;++e->stats.io_controls;
    control_ack(e,f); /* 复用01应答：交换源/目的、seq=1、原8字节不变 */
}
/* 模拟采样独立于上报，100ms采样一次；值可不变，成功采样时间必须更新。
 * 暂停后不更新时间戳，让report_io正常检测陈旧数据，而非在上报时续命。 */
void mc_io_simulation_pause(mc_engine *e,bool paused) {
    if(e) e->io_simulation_paused=paused;
}
static void io_seed(mc_engine *e,mc_millis now) {
    if(e->config.io_backend!=MC_IO_SIMULATED||e->io_simulation_paused) return;
    for(size_t i=0;i<e->config.route_count;++i) {
        const mc_route *r=&e->config.routes[i];io_state *sample=&e->io[i];
        if(r->kind==MC_ROUTE_DO&&sample->valid) continue; /* 保留DO写入镜像。 */
        if(sample->valid&&now>=sample->updated&&now-sample->updated<100) continue;
        for(unsigned k=0;k<r->reg_count;++k) sample->regs[k]=(uint16_t)((r->channel*257u+k*37u)&0xffffu);
        sample->valid=true;sample->updated=now;
    }
}
int mc_io_snapshot(const mc_engine *e,unsigned channel,unsigned kind,uint16_t *words,size_t capacity,mc_millis now) {
    (void)now;
    if(!e||!words||e->config.io_backend!=MC_IO_SIMULATED) return -1;
    int index=route_find(e,channel,kind);if(index<0) return -1;
    const mc_route *r=&e->config.routes[index];
    size_t count=r->reg_count<capacity?r->reg_count:capacity;
    memcpy(words,e->io[index].regs,count*sizeof(uint16_t));
    return (int)count;
}
int mc_io_status_snapshot(const mc_engine *e,unsigned channel,unsigned kind,mc_millis now,mc_io_status *out) {
    if(!e||!out||e->config.io_backend!=MC_IO_SIMULATED) return -1;
    int index=route_find(e,channel,kind);if(index<0) return -1;
    const io_state *sample=&e->io[index];const mc_route *r=&e->config.routes[index];
    memset(out,0,sizeof(*out));out->valid=sample->valid;out->updated_ms=sample->updated;
    out->fresh=sample->valid&&now>=sample->updated&&now-sample->updated<=e->config.stale_ms;
    out->count=r->reg_count;memcpy(out->words,sample->regs,r->reg_count*sizeof(uint16_t));return 0;
}
static bool same_control(const job *a,const job *b) {
    return a->kind==CONTROL&&a->length==b->length&&!memcmp(a->request,b->request,a->length)&&same_key(&a->origin,&b->origin);
}
/* 01控制接收：先校验权限、功能和地址范围，再按“原ID+原8字节”去重。
 * 去重键不含入口bus，从而双CAN重复也只写一次；只有已完成事务可补ACK。
 * 排到队列前面并不抢断正在收发的RTU，因为半双工总线不能同时做两个事务。 */
static void control(mc_engine *e,mc_id id,const mc_frame *f,mc_millis now) {
    if((!id.destination&&!e->config.allow_broadcast_control)||id.sequence||(!e->config.customer_gateway&&!contains(e->config.control_sources,e->config.control_source_count,id.source))) {++e->stats.rejected;return;}
    /* 08未启用时恒为true；启用后无租约来源不得写485/DO。 */
    if(!mc_authority_allows(e,id.source,id.channel,now)) {++e->stats.rejected;return;}
    const mc_port *p;const mc_device *d=find_device(e,id,&p);
    if(!d||!d->control_enabled) {++e->stats.rejected;return;}
    unsigned start=0,count=0,function=0;
    /* 新示例没有数量：06写一个，10固定写两个；不能按载荷猜协议版本。 */
    function=f->data[1];start=mc_read_be(f->data+2);count=function==6?1:2;
    bool allowed=d->allowed_write_count?contains(d->allowed_write_functions,d->allowed_write_count,function):function==d->write_function;
    if(f->data[0]!=id.slave||(function!=6&&function!=16)||!allowed) {++e->stats.rejected;return;}
    if(count<1||count>2||start+count>65536||!in_ranges(d->writable,d->writable_count,start,count)||(function==6&&count!=1)) {++e->stats.rejected;return;}
    int index=duplicate_index(e,f),free_index=-1;size_t used=0;
    if(index>=0&&now-e->duplicates[index].first_seen<e->config.dedup_ms) {
        ++e->stats.duplicate;
        /* 仅已完成的DW事务允许重发ACK；在途重复不能提前宣告成功。 */
        if(e->duplicates[index].completed) control_ack(e,f);
        return;
    }
    for(unsigned i=0;i<512;++i) {if(e->duplicates[i].used) ++used;else if(free_index<0) free_index=(int)i;}
    port_state *s=&e->states[p->physical-1];
    if(s->unavailable) {++e->stats.rejected;event(e,"port_unavailable",p->physical,NULL,now,id.slave);return;}
    if(s->queued>=e->config.queue_limit||used>=512) {++e->stats.queue_full;return;}
    job j={0};j.kind=CONTROL;j.slave=id.slave;j.function=function;j.start=start;j.count=count;
    j.timeout_ms=d->timeout_ms;j.expires_at=now+e->config.dedup_ms;j.origin=*f;
    j.authority_generation=authority_generation(e,id.source,id.channel,now);
    j.request[0]=(uint8_t)id.slave;j.request[1]=(uint8_t)function;mc_write_be(j.request+2,start);
    if(function==6) {memcpy(j.request+4,f->data+4,2);j.length=mc_append_crc(j.request,6);}
    else {mc_write_be(j.request+4,count);j.request[6]=(uint8_t)(count*2);memcpy(j.request+7,f->data+4,count*2);j.length=mc_append_crc(j.request,7+count*2);}
    if(s->pending&&same_control(&s->active,&j)) {++e->stats.duplicate;return;}
    for(size_t i=0;i<s->queued;++i) if(same_control(&s->queue[i],&j)) {++e->stats.duplicate;return;}
    size_t position=0;while(position<s->queued&&s->queue[position].kind==CONTROL) ++position;
    insert_job(s,position,&j);
    duplicate *dup=&e->duplicates[index>=0?index:free_index];
    dup->used=true;dup->completed=false;dup->id=f->id;memcpy(dup->data,f->data,8);dup->first_seen=now;++e->stats.controls;
}
static bool same_query(const job *j,const mc_frame *f) {return j->kind==QUERY&&same_key(&j->origin,f);}
/* CAN03载荷本身含完整RTU03 CRC，必须验证后才转发。
 * 相同入口、同一在途请求合并；不同入口各自保留返回路径。
 * 查询任务只产生03回复，不能把不同地址块写进周期02缓存。 */
static void query(mc_engine *e,mc_id id,const mc_frame *f,mc_millis now) {
    if(!id.destination||id.sequence||(!e->config.customer_gateway&&!contains(e->config.query_sources,e->config.query_source_count,id.source))||f->data[0]!=id.slave||f->data[1]!=3||!mc_valid_crc(f->data,8)) {++e->stats.rejected;return;}
    const mc_port *p;const mc_device *d=find_device(e,id,&p);
    unsigned start=mc_read_be(f->data+2),count=mc_read_be(f->data+4);
    if(!d||!d->query_enabled||count<1||count>60||start+count>65536||!in_ranges(d->readable,d->readable_count,start,count)) {++e->stats.rejected;return;}
    port_state *s=&e->states[p->physical-1];
    if(s->unavailable) {++e->stats.rejected;event(e,"port_unavailable",p->physical,NULL,now,id.slave);return;}
    if(s->queued>=e->config.queue_limit) {++e->stats.queue_full;return;}
    if(s->pending&&same_query(&s->active,f)) {++e->stats.duplicate;return;}
    for(size_t i=0;i<s->queued;++i) if(same_query(&s->queue[i],f)) {++e->stats.duplicate;return;}
    job j={0};j.kind=QUERY;j.slave=id.slave;j.function=3;j.start=start;j.count=count;j.timeout_ms=d->timeout_ms;
    j.expires_at=now+e->config.dedup_ms;j.origin=*f;j.length=8;memcpy(j.request,f->data,8);
    size_t position=0;while(position<s->queued&&s->queue[position].kind!=PERIODIC) ++position;
    insert_job(s,position,&j);++e->stats.queries;
}
/* 每帧4个大端16位字。1帧用序号0；多帧1～15，尾帧空位补0。
 * 先预留整个返回块容量，避免输出环形队列中留下半条查询回复。 */
static void words_reply(mc_engine *e,const job *j,const uint16_t *words) {
    unsigned frames=(j->count+3)/4;
    if(e->out_count+frames*(e->config.customer_gateway?bus_count(e):1)>MC_OUTPUT) {++e->stats.queue_full;return;}
    mc_id id=mc_decode_id(j->origin.id);id.destination=id.source;id.source=e->config.address;
    for(unsigned n=0;n<frames;++n) {
        uint8_t data[8]={0};for(unsigned k=0;k<4;++k) if(n*4+k<j->count) mc_write_be(data+k*2,words[n*4+k]);
        id.sequence=frames==1?0:n+1;emit(e,id,data,j->origin.bus);
    }
    ++e->stats.query_replies;
}
/* 07采用候选配置事务：校验→落盘→提交内存→回复结果。
 * 改地址/通道时必须无在途和排队任务，防止旧任务回到新身份。
 * 目前未定义双方协同切速率/回退流程，故该参数仍明确返回失败。 */
static void parameter(mc_engine *e,mc_id id,const mc_frame *f) {
    /* 先修改候选，再校验/持久化，成功后提交内存；不需要异常处理机制。 */
    mc_config candidate=e->config;unsigned result=0,kind=f->data[0],value=mc_read_be(f->data+1);char err[160];
    if(!e->config.parameters_enabled||!id.destination||(!e->config.customer_gateway&&!contains(e->config.control_sources,e->config.control_source_count,id.source))) result=3;
    else if(id.sequence) result=1;
    else {
        bool busy=false;
        for(unsigned i=0;i<MC_PORTS;++i) if(e->states[i].pending||e->states[i].queued) busy=true;
        if(kind==1) result=2;
        else if(busy&&(kind==2||kind==3)) result=5;
        else if(kind==2) {
            bool found=false;
            for(size_t i=0;i<candidate.port_count;++i) if(candidate.ports[i].channel==id.channel) {candidate.ports[i].channel=value;found=true;}
            if(!found) result=1;
        } else if(kind==3) candidate.address=value;
        else if(kind==4) candidate.heartbeat_ms=value;
        else if(kind==5) candidate.report_ms=value;
        else result=2;
        if(!result) {
            if(mc_config_validate(&candidate,err,sizeof(err))) result=1;
            else if(!e->persist||e->persist(e->persist_context,&candidate,err,sizeof(err))) result=4;
            else {e->config=candidate;e->next_report=0;e->next_heartbeat=0;}
        }
    }
    uint8_t data[8];memcpy(data,f->data,8);data[3]=(uint8_t)(!result?1:0);
    mc_id reply=id;reply.source=id.destination?id.destination:e->config.address;reply.destination=id.source;
    emit(e,reply,data,f->bus);
}
/* Linux层只负责CAN基本帧过滤；这里再次做bus/长度/ID边界和目的地址检查。
 * 未定义业务不由收到的数据“猜测实现”。本地拒绝事件不扩展线上失败报文。 */
void mc_receive_can(mc_engine *e,const mc_frame *f,mc_millis now) {
    ++e->stats.can_rx;
    if(f->bus<0||(size_t)f->bus>=e->config.can_count||f->id>0x1fffffff||f->length!=8) {++e->stats.rejected;return;}
    mc_id id=mc_decode_id(f->id);
    if(id.destination!=e->config.address&&id.destination) return;
    uint64_t rejected=e->stats.rejected,full=e->stats.queue_full,unsupported=e->stats.unsupported;
    if(id.type==1) {
        int route=id.slave?-1:route_find(e,id.channel,MC_ROUTE_DO);
        if(route>=0) do_control(e,id,f,now,route);else control(e,id,f,now);
    }
    else if(id.type==7) parameter(e,id,f);
    else if(id.type==3) query(e,id,f,now);
    else if(id.type==8) {if(e->config.authorization_enabled) authority(e,id,f,now);else ++e->stats.unsupported;}
    else if(id.type==15) ++e->stats.unsupported;
    if(e->stats.rejected!=rejected||e->stats.queue_full!=full||e->stats.unsupported!=unsupported) {
        const mc_port *p;find_device(e,id,&p);unsigned physical=p?p->physical:0;
        if(!physical&&id.type==1) {int route=id.slave?-1:route_find(e,id.channel,MC_ROUTE_DO);if(route>=0) physical=e->config.routes[route].physical;}
        job j={0};j.slave=id.slave;j.function=f->data[1];j.start=mc_read_be(f->data+2);
        event(e,e->stats.queue_full!=full?"queue_full":(e->stats.unsupported!=unsupported?"unsupported":"request_rejected"),physical,&j,now,id.source);
    }
}
static void consume(port_state *s,size_t n) {s->input_length-=n;memmove(s->input,s->input+n,s->input_length);}
/* 字节流入口允许任意拆帧/粘包，使用当前唯一在途job决定应答格式。
 * 先匹配从站/功能/字节数，再校验CRC和写回显，满足全部条件才提交成功。
 * 错CRC可继续等待正确帧；只有超时/异常/发送失败才增加事务失败连续计数。 */
void mc_receive_serial(mc_engine *e,unsigned physical,const uint8_t *data,size_t n,mc_millis now) {
    if(physical<1||physical>8) return;
    const mc_port *port=NULL;
    for(size_t i=0;i<e->config.port_count;++i) if(e->config.ports[i].enabled&&e->config.ports[i].physical==physical) port=&e->config.ports[i];
    if(!port) return;
    port_state *s=&e->states[physical-1];
    if(!s->pending||s->transmitting) {s->input_length=0;return;}
    if(n>sizeof(s->input)-s->input_length) {s->input_length=0;++e->stats.mismatch;++s->diagnostics[s->active.slave].mismatches;return;}
    memcpy(s->input+s->input_length,data,n);s->input_length+=n;
    /* 按当前唯一在途事务识别半帧/错帧，CRC正确且业务字段匹配才算成功。 */
    while(s->pending&&s->input_length) {
        job *j=&s->active;
        if(s->input[0]!=j->slave) {consume(s,1);++e->stats.mismatch;++s->diagnostics[s->active.slave].mismatches;continue;}
        if(s->input_length<2) break;
        bool error=s->input[1]==(j->function|0x80);
        if(s->input[1]!=j->function&&!error) {consume(s,1);++e->stats.mismatch;++s->diagnostics[s->active.slave].mismatches;continue;}
        size_t length=error?5:(j->kind==CONTROL?8:0);
        if(!error&&j->kind!=CONTROL) {
            if(s->input_length<3) break;
            if(s->input[2]!=j->count*2) {consume(s,1);++e->stats.mismatch;++s->diagnostics[s->active.slave].mismatches;continue;}
            length=s->input[2]+5;
        }
        if(s->input_length<length) break;
        if(!mc_valid_crc(s->input,length)) {event(e,"crc_error",physical,j,now,0);consume(s,1);++e->stats.crc_error;++s->diagnostics[j->slave].crc_errors;continue;}
        if(!error&&j->kind==CONTROL&&memcmp(s->input,j->request,6)) {consume(s,length);++e->stats.mismatch;++s->diagnostics[s->active.slave].mismatches;continue;}
        if(error) failed(e,physical,j,MC_RESULT_EXCEPTION,now,s->input[2]);
        else succeeded(e,physical,j,now);
        event(e,error?"modbus_exception":"serial_ok",physical,j,now,error?s->input[2]:0);
        cache *c=&s->values[j->slave];
        if(error) {++e->stats.exception;c->valid=false;}
        else if(j->kind==QUERY) {
            uint16_t words[60];for(unsigned i=0;i<j->count;++i) words[i]=mc_read_be(s->input+3+i*2);
            words_reply(e,j,words);++e->stats.serial_ok;
            /* 查询可能读另一地址块，因此绝不覆盖周期采集缓存。 */
        } else if(j->kind==PERIODIC) {
            for(unsigned i=0;i<j->count;++i) c->words[i]=mc_read_be(s->input+3+i*2);
            c->count=j->count;c->valid=true;c->updated=now;++e->stats.serial_ok;
        } else {
            ++e->stats.serial_ok;
            control_ack(e,&j->origin);int index=duplicate_index(e,&j->origin);
            if(index>=0&&now-e->duplicates[index].first_seen<e->config.dedup_ms) e->duplicates[index].completed=true;
        
            c->valid=false;s->next_poll[j->slave]=now;
        }
        unsigned bits=9+port->stopbits+(port->parity!='N');
        unsigned gap=(3500*bits+port->baud-1)/port->baud;if(gap<5) gap=5;
        s->pending=false;s->input_length=0;s->quiet_until=now+gap;
    }
}
/* DI压缩为4字节位图（Byte0-1高16位、Byte2-3低16位），AI按16路分4帧、每帧4值。
 * 模拟后端只验证打包与序号，不代表真实板卡采样/标定/极性。 */
static void report_io(mc_engine *e,mc_millis now) {
    if(e->config.io_backend!=MC_IO_SIMULATED) return;
    for(size_t i=0;i<e->config.route_count;++i) {
        const mc_route *r=&e->config.routes[i];
        if(r->kind==MC_ROUTE_DO) continue;
        io_state *s=&e->io[i];
        if(!s->valid||now<s->updated||now-s->updated>e->config.stale_ms) {++e->stats.stale_skipped;continue;}
        unsigned frames=r->kind==MC_ROUTE_DI?1:4;
        if(e->out_count+frames*e->config.target_count*bus_count(e)>MC_OUTPUT) {++e->stats.queue_full;continue;}
        for(size_t t=0;t<e->config.target_count;++t) for(unsigned n=0;n<frames;++n) {
            uint8_t data[8]={0};
            if(r->kind==MC_ROUTE_DI) {mc_write_be(data,s->regs[0]);mc_write_be(data+2,s->regs[1]);}
            else for(unsigned k=0;k<4;++k) if(n*4+k<r->reg_count) mc_write_be(data+k*2,s->regs[n*4+k]);
            mc_id id={2,e->config.address,e->config.targets[t],r->channel,0,frames==1?0:n+1};
            size_t before=e->out_count;emit(e,id,data,-1);
            for(size_t q=before;q<e->out_count;++q) e->output[(e->out_head+q)%MC_OUTPUT].expires_at=s->updated+e->config.stale_ms;
            e->stats.io_reports+=e->out_count-before;
        }
    }
}
/* 周期上报只使用该设备周期读缓存。过期/无样本直接跳过，不重复旧值。
 * 本地expires_at保留采样截止时间，让Linux CAN队列继续执行相同过期边界。 */
static void report(mc_engine *e,mc_millis now) {
    for(size_t i=0;i<e->config.port_count;++i) {
        const mc_port *p=&e->config.ports[i];if(!p->enabled) continue;
        port_state *s=&e->states[p->physical-1];
        for(size_t k=0;k<p->device_count;++k) {
            const mc_device *d=&p->devices[k];cache *c=&s->values[d->slave];
            if(!c->valid||now-c->updated>e->config.stale_ms) {++e->stats.stale_skipped;continue;}
            size_t frames=(c->count+3)/4;
            if(e->out_count+frames*e->config.target_count*bus_count(e)>MC_OUTPUT) {++e->stats.queue_full;continue;}
            for(size_t t=0;t<e->config.target_count;++t) for(unsigned n=0;n<frames;++n) {
                uint8_t data[8]={0};for(unsigned w=0;w<4;++w) if(n*4+w<c->count) mc_write_be(data+w*2,c->words[n*4+w]);
                mc_id id={2,e->config.address,e->config.targets[t],p->channel,d->slave,frames==1?0:n+1};
                size_t before=e->out_count;emit(e,id,data,-1);
                for(size_t q=before;q<e->out_count;++q) e->output[(e->out_head+q)%MC_OUTPUT].expires_at=c->updated+e->config.stale_ms;
            }
        }
    }
}
/* 单线程调度器，每轮做超时、补周期任务、启动空闲端口和定时CAN上报。
 * 控制/查询排队超时直接撤销；仅周期读可以有限重试。
 * 定时、缓存、去重都基于调用者传入的单调毫秒，禁止改用系统日历时间。 */
void mc_tick(mc_engine *e,mc_millis now,mc_send_fn send,void *context) {
    for(unsigned i=0;i<512;++i) if(e->duplicates[i].used&&now-e->duplicates[i].first_seen>=e->config.dedup_ms) e->duplicates[i].used=false;
    for(unsigned i=0;i<MC_LEASES;++i) if(e->leases[i].used&&now>=e->leases[i].expires) e->leases[i].used=false;
    authority_prune(e,now);
    io_seed(e,now);
    for(size_t i=0;i<e->config.port_count;++i) {
        const mc_port *p=&e->config.ports[i];if(!p->enabled) continue;
        port_state *s=&e->states[p->physical-1];
        if(s->unavailable) continue;
        if(s->pending&&!s->transmitting&&now-s->active.sent_at>=s->active.timeout_ms) {
            failed(e,p->physical,&s->active,MC_RESULT_TIMEOUT,now,0);
            event(e,"timeout",p->physical,&s->active,now,0);
            ++e->stats.timeout;s->pending=false;s->input_length=0;s->values[s->active.slave].valid=false;
            s->quiet_until=now+s->active.timeout_ms; /* 超时后隔离迟到回复。 */
            if(s->active.kind==PERIODIC&&s->active.retries_left) {
                --s->active.retries_left;
                if(s->queued<MC_QUEUE) insert_job(s,0,&s->active);else ++e->stats.queue_full;
            }
        }
        for(size_t k=0;!e->config.customer_gateway&&k<p->device_count;++k) {
            const mc_device *d=&p->devices[k];if(now<s->next_poll[d->slave]) continue;
            bool exists=s->pending&&s->active.kind==PERIODIC&&s->active.slave==d->slave;
            for(size_t n=0;n<s->queued;++n) if(s->queue[n].kind==PERIODIC&&s->queue[n].slave==d->slave) exists=true;
            if(!exists&&s->queued<e->config.queue_limit) {
                job j={0};j.slave=d->slave;j.function=d->function;j.start=d->start;j.count=d->count;j.timeout_ms=d->timeout_ms;j.retries_left=d->retries;
                j.request[0]=(uint8_t)d->slave;j.request[1]=(uint8_t)d->function;mc_write_be(j.request+2,d->start);mc_write_be(j.request+4,d->count);
                j.length=mc_append_crc(j.request,6);insert_job(s,s->queued,&j);
            }
            s->next_poll[d->slave]=now+d->poll_ms;
        }
        if(!s->pending&&s->queued&&now>=s->quiet_until) {
            while(s->queued&&s->queue[0].kind!=PERIODIC&&(now>=s->queue[0].expires_at||!job_authorized(e,&s->queue[0],now))) {event(e,"queue_expired",p->physical,&s->queue[0],now,0);(void)pop_job(s);++e->stats.rejected;}
            if(!s->queued) continue;
            s->active=pop_job(s);s->input_length=0;
            if(send&&send(context,p->physical,s->active.request,s->active.length)) {
                s->pending=true;s->transmitting=e->async_tx;s->active.sent_at=now;
                event(e,"accepted",p->physical,&s->active,now,0);if(s->active.kind==PERIODIC) ++e->stats.polls;
            } else {failed(e,p->physical,&s->active,MC_RESULT_TX_ERROR,now,0);event(e,"serial_tx_error",p->physical,&s->active,now,0);++e->stats.serial_tx_error;s->values[s->active.slave].valid=false;s->quiet_until=now+s->active.timeout_ms;}
        }
    }
    /* 客户模式每轮8包，每个物理口对应一个协议通道；未接设备也发心跳。
     * 每次tick最多推进一个通道，避免一次性无间隔提交8包。
     * 同一通道在两路CAN使用相同计数，计数按通道独立递增。 */
    if(e->config.customer_gateway&&e->config.heartbeat_enabled&&now>=e->next_heartbeat&&now>=e->heartbeat_due) {
        const mc_port *p=&e->config.ports[e->heartbeat_channel];
        uint8_t data[8]={0};data[0]=e->channel_counter[e->heartbeat_channel]++;
        mc_id id={4,e->config.address,0,p->channel,0,0};emit(e,id,data,-1);
        e->heartbeat_due=now+1;
        if(++e->heartbeat_channel==8) {e->heartbeat_channel=0;e->next_heartbeat=now+e->config.heartbeat_ms;}
    }
    if(!e->config.customer_gateway&&e->config.heartbeat_enabled&&now>=e->next_heartbeat) {
        uint8_t data[8]={0};
        for(size_t t=0;t<e->config.target_count;++t) {
            mc_id id={4,e->config.address,e->config.targets[t],0,0,0};
            for(size_t b=0;b<bus_count(e);++b) {data[0]=e->heartbeat++;emit(e,id,data,(int)b);}
        }
        e->next_heartbeat=now+e->config.heartbeat_ms;
    }
    if(!e->config.customer_gateway&&now>=e->next_report) {report(e,now);report_io(e,now);e->next_report=now+e->config.report_ms;}
}
