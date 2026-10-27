#include "monitor_diagnostics.h"
#include <math.h>

/* 离线检查只使用有效配置（含已保存07参数），从不open串口/CAN。
 * 预算仅含线速和帧间隔下限，未包含设备处理、错误、重试及现场竞争，
 * 因而“通过”不能当成现场接线、地址或设备语义已确认。 */
int mc_site_check(const mc_config *c,FILE *out,bool strict) {
    unsigned warnings=0,blockers=0,enabled=0,devices=0;
    double report_frames=0;
    fprintf(out,"SITE protocol=%s address=%u mapping_confirmed=%d parameters_enabled=%d broadcast_control=%d\n",MC_PROTOCOL,c->address,c->site_mapping_confirmed,c->parameters_enabled,c->allow_broadcast_control);
    if(!c->site_mapping_confirmed) {fputs("BLOCK mapping_unconfirmed: confirm physical wiring and protocol channels before running\n",out);++blockers;}
    fprintf(out,"CAN bitrate=%u report_ms=%u heartbeat_ms=%u stale_ms=%u frame_gap_ms=%u\n",c->baud,c->report_ms,c->heartbeat_ms,c->stale_ms,c->frame_gap_ms);
    for(size_t i=0;i<c->can_count;++i) fprintf(out,"CAN_BUS index=%zu interface=%s reports=%d\n",i,c->can_interfaces[i],!c->primary_only||i==0);
    fprintf(out,"SOURCES control=");for(size_t i=0;i<c->control_source_count;++i) fprintf(out,"%s%u",i?",":"",c->control_sources[i]);
    fputs(" query=",out);for(size_t i=0;i<c->query_source_count;++i) fprintf(out,"%s%u",i?",":"",c->query_sources[i]);
    fputs(" targets=",out);for(size_t i=0;i<c->target_count;++i) fprintf(out,"%s%u",i?",":"",c->targets[i]);fputc('\n',out);
    fprintf(out,"HEARTBEAT owner=%s\n",c->heartbeat_enabled?"this_process":"disabled");
    if(c->dedup_ms<4000) {fputs("WARN dedup_window_below_4000ms: six 500ms repeats need margin\n",out);++warnings;}
    if(c->io_backend==MC_IO_SIMULATED) {fputs("WARN simulated_io_backend: DO/DI/AI are software mirrors, no physical output\n",out);++warnings;}
    for(size_t i=0;i<c->route_count;++i) {
        const mc_route *r=&c->routes[i];
        const char *kind=r->kind==MC_ROUTE_DO?"do":(r->kind==MC_ROUTE_DI?"di":"ai");
        fprintf(out,"IO_ROUTE channel=%u kind=%s physical=%u slave=%u reg_start=%u reg_count=%u\n",r->channel,kind,r->physical,r->slave,r->reg_start,r->reg_count);
    }
    if(c->authorization_enabled) {
        fputs("WARN authorization_enabled: provisional lease policy, confirm arbitration rules before field use\n",out);++warnings;
        if(!c->authorization_sha256[0]) {fputs("WARN authorization_without_code: verify(5) will always fail\n",out);++warnings;}
    }
    for(size_t i=0;i<c->port_count;++i) {
        const mc_port *p=&c->ports[i];double utilization=0,cycle=0;
        fprintf(out,"PORT physical=%u channel=%u path=%s enabled=%d baud=%u format=8%c%u devices=%zu\n",p->physical,p->channel,p->path,p->enabled,p->baud,p->parity,p->stopbits,p->device_count);
        if(!p->enabled) continue;
        ++enabled;devices+=(unsigned)p->device_count;
        if(!p->device_count) {fputs("WARN enabled_port_without_devices\n",out);++warnings;}
        double bits=9+p->stopbits+(p->parity!='N');
        for(size_t k=0;k<p->device_count;++k) {
            const mc_device *d=&p->devices[k];
            /* 请求8字节，响应5+2count字节；两次3.5字符间隔合计7字符。 */
            double wire=(8+5+2*d->count+7)*bits*1000/p->baud;
            double response=(5+2*d->count)*bits*1000/p->baud;
            utilization+=wire/d->poll_ms;cycle+=wire;report_frames+=(d->count+3)/4;
            fprintf(out,"DEVICE physical=%u channel=%u slave=%u read_function=%u read_start_0based=%u read_count=%u poll_ms=%u timeout_ms=%u retries=%u control=%d query=%d wire_floor_ms=%.2f\n",p->physical,p->channel,d->slave,d->function,d->start,d->count,d->poll_ms,d->timeout_ms,d->retries,d->control_enabled,d->query_enabled,wire);
            fprintf(out," WRITE_FUNCTION default=%u allowed=",d->write_function);
            for(size_t j=0;j<d->allowed_write_count;++j) fprintf(out,"%s%u",j?",":"",d->allowed_write_functions[j]);
            fputc('\n',out);
            for(size_t j=0;j<d->readable_count;++j) fprintf(out," READ_RANGE start=%u count=%u\n",d->readable[j].start,d->readable[j].count);
            for(size_t j=0;j<d->writable_count;++j) fprintf(out," WRITE_RANGE start=%u count=%u\n",d->writable[j].start,d->writable[j].count);
            if(d->control_enabled) {fputs("WARN control_enabled: require device-specific safe-write approval\n",out);++warnings;}
            if(d->control_enabled&&!c->control_source_count) {fputs("WARN control_without_sources\n",out);++warnings;}
            if(d->query_enabled&&!c->query_source_count) {fputs("WARN query_without_sources\n",out);++warnings;}
            if(d->timeout_ms<response) {fputs("WARN timeout_below_response_wire_time\n",out);++warnings;}
            if(d->poll_ms<wire) {fputs("WARN poll_below_transaction_wire_time\n",out);++warnings;}
        }
        fprintf(out,"SERIAL_BUDGET physical=%u utilization_floor=%.3f cycle_floor_ms=%.2f\n",p->physical,utilization,cycle);
        if(utilization>=0.7) {fprintf(out,"WARN serial_%s physical=%u\n",utilization>1?"budget_exceeded":"headroom_low",p->physical);++warnings;}
        if(cycle+c->report_ms>c->stale_ms) {fputs("WARN stale_budget_low\n",out);++warnings;}
    }
    if(!enabled||!devices) {fputs("BLOCK no_enabled_devices\n",out);++blockers;}
    report_frames*=c->target_count;
    double fps=report_frames*1000/c->report_ms+c->target_count*1000.0/c->heartbeat_ms;
    /* 经典扩展CAN DLC8按160位保守估算。查询/控制尚未计入，不能打满预算。 */
    fprintf(out,"CAN_BUDGET per_report_frames=%.0f per_reporting_bus_fps=%.2f utilization_estimate=%.3f local_send_ms=%.0f\n",report_frames,fps,fps*160/c->baud,report_frames*c->frame_gap_ms);
    if(fps*160/c->baud>=0.7) {fputs("WARN can_headroom_low\n",out);++warnings;}
    if(fps*c->frame_gap_ms>=1000) {fputs("WARN local_can_queue_budget_exceeded\n",out);++warnings;}
    if(devices*c->target_count>224) {fputs("WARN periodic_group_burst_exceeds_reserved_capacity\n",out);++warnings;}
    if(c->allow_broadcast_control) {fputs("WARN broadcast_control_enabled\n",out);++warnings;}
    if(c->parameters_enabled) {fputs("WARN parameters_enabled: verify ownership and persistence directory\n",out);++warnings;}
    fputs("NOTE register units/scaling/signedness/word order, wiring, CAN ACK and safe actions require external confirmation\n",out);
    fprintf(out,"SITE_RESULT blockers=%u warnings=%u strict=%d\n",blockers,warnings,strict);
    return blockers||(strict&&warnings)?2:0;
}
