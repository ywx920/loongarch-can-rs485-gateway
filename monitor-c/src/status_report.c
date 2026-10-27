#define _XOPEN_SOURCE 700
#include "monitor_diagnostics.h"
#include <errno.h>
#include <fcntl.h>
#include <inttypes.h>
#include <limits.h>
#include <stdlib.h>
#include <string.h>
#include <sys/stat.h>
#include <unistd.h>

/* 字符串只用于路径等本地信息；JSON控制字符必须转义，UTF-8字节直接保留。 */
static void json_string(FILE *out,const char *s) {
    fputc('"',out);
    for(const unsigned char *p=(const unsigned char *)s;*p;++p) {
        if(*p=='"'||*p=='\\') {fputc('\\',out);fputc(*p,out);}
        else if(*p<32) fprintf(out,"\\u%04x",*p);
        else fputc(*p,out);
    }
    fputc('"',out);
}
static const char *boolean(bool value) {return value?"true":"false";}
/* 时间0本身可能合法，例如虚拟时钟首次事务。必须用has_*区分“未知”。 */
static void timestamp(FILE *out,bool known,mc_millis value) {
    if(known) fprintf(out,"%" PRId64,value);else fputs("null",out);
}
void mc_status_print(const mc_engine *e,mc_millis now,FILE *out) {
    const mc_config *c=mc_engine_config(e);
    for(size_t i=0;i<c->port_count;++i) for(size_t k=0;k<c->ports[i].device_count;++k) {
        mc_device_status d;
        if(mc_device_snapshot(e,c->ports[i].physical,c->ports[i].devices[k].slave,now,&d)) continue;
        fprintf(out,"DEVICE physical=%u channel=%u slave=%u enabled=%d port_available=%d online=%d cache_fresh=%d failures=%u result=%s exception=%u pending=%d queued=%zu last_success_ms=",
                d.physical,d.channel,d.slave,d.enabled,d.port_available,d.communication_online,d.cache_fresh,d.consecutive_failures,mc_result_name(d.last_result),d.last_exception,d.pending,d.queued);
        timestamp(out,d.has_success,d.last_success_ms);fputc('\n',out);
    }
    fflush(out);
}
/* 取绝对规范路径，既识别同inode硬链接，也识别尚未创建文件的相对路径别名。 */
static bool canonical(const char *path,char out[PATH_MAX]) {
    if(!path||!path[0]) return false;
    if(realpath(path,out)) return true;
    char copy[PATH_MAX],parent[PATH_MAX];
    if(strlen(path)>=sizeof(copy)) return false;
    strcpy(copy,path);char *slash=strrchr(copy,'/');const char *name=copy;
    if(slash) {*slash=0;name=slash+1;if(!realpath(copy[0]?copy:"/",parent)) return false;}
    else if(!realpath(".",parent)) return false;
    int n=snprintf(out,PATH_MAX,"%s/%s",parent,name);return n>0&&n<PATH_MAX;
}
static bool same_path(const char *a,const char *b) {
    if(!a||!b||!a[0]||!b[0]) return false;
    struct stat sa,sb;
    if(!stat(a,&sa)&&!stat(b,&sb)&&sa.st_dev==sb.st_dev&&sa.st_ino==sb.st_ino) return true;
    char ca[PATH_MAX],cb[PATH_MAX];
    return !strcmp(a,b)||(canonical(a,ca)&&canonical(b,cb)&&!strcmp(ca,cb));
}
/* runtime只允许写普通文件。所有路径在runtime_load前检查，避免把现场
 * 主配置当运行参数再次加载，或把串口设备当参数目标进行rename覆盖。 */
int mc_config_paths_safe(const char *config_path,const mc_config *c,char *err,size_t cap) {
    if(c->runtime_settings[0]) {
        if(same_path(config_path,c->runtime_settings)) {snprintf(err,cap,"runtime path conflicts with source config");return -1;}
        struct stat st;
        if(!lstat(c->runtime_settings,&st)&&!S_ISREG(st.st_mode)) {snprintf(err,cap,"runtime target must be a regular file, not a symlink/device");return -1;}
    }
    for(size_t i=0;i<c->port_count;++i) {
        const mc_port *p=&c->ports[i];
        if(same_path(p->path,config_path)||same_path(p->path,c->runtime_settings)) {snprintf(err,cap,"serial path conflicts with config/runtime");return -1;}
        if(!p->enabled) continue;
        struct stat a;
        for(size_t k=0;k<i;++k) if(c->ports[k].enabled) {
            struct stat b;
            bool alias=same_path(p->path,c->ports[k].path);
            if(!stat(p->path,&a)&&!stat(c->ports[k].path,&b)&&S_ISCHR(a.st_mode)&&S_ISCHR(b.st_mode)&&a.st_rdev==b.st_rdev) alias=true;
            if(alias) {snprintf(err,cap,"enabled serial paths alias one device: physical %u and %u",c->ports[k].physical,p->physical);return -1;}
        }
    }
    return 0;
}
int mc_status_path_safe(const char *path,const char *config_path,const mc_config *c,char *err,size_t cap) {
    if(!path||!path[0]||strlen(path)>=MC_PATH) {snprintf(err,cap,"invalid or too long status path");return -1;}
    if(same_path(path,config_path)||same_path(path,c->runtime_settings)) goto collision;
    for(size_t i=0;i<c->port_count;++i) if(same_path(path,c->ports[i].path)) goto collision;
    struct stat st;
    if(!lstat(path,&st)&&!S_ISREG(st.st_mode)) {snprintf(err,cap,"status destination must be a regular file, not a symlink/device/FIFO");return -1;}
    return 0;
collision:snprintf(err,cap,"status path conflicts with config/runtime/serial path");return -1;
}
/* PID可能复用，单调时钟可能跨重启变小；同时记录boot_id和进程启动ticks。
 * 不使用日历时间判断存活，避免NTP校时或板端RTC错误造成误判。 */
static void process_identity(char boot[80],unsigned long long *ticks) {
    boot[0]=0;*ticks=0;FILE *in=fopen("/proc/sys/kernel/random/boot_id","r");
    if(in) {if(fgets(boot,80,in)) boot[strcspn(boot,"\r\n")]=0;fclose(in);}
    in=fopen("/proc/self/stat","r");if(!in) return;
    char line[4096];if(fgets(line,sizeof(line),in)) {
        char *at=strrchr(line,')');
        if(at) {at+=2;for(unsigned field=3;field<22&&*at;++field) {at=strchr(at,' ');if(!at) break;++at;}if(at) *ticks=strtoull(at,NULL,10);}
    }
    fclose(in);
}
int mc_status_write(const char *path,const mc_engine *e,mc_millis now,const int *can_fds,char *err,size_t cap) {
    const mc_runtime_status runtime={.running=true,.started_ms=now,.interval_ms=5000};
    return mc_status_write_ex(path,e,now,can_fds,&runtime,err,cap);
}
int mc_status_write_ex(const char *path,const mc_engine *e,mc_millis now,const int *can_fds,const mc_runtime_status *runtime,char *err,size_t cap) {
    char temp[MC_PATH+48];int n=snprintf(temp,sizeof(temp),"%s.tmp.%ld",path,(long)getpid());
    if(n<0||(size_t)n>=sizeof(temp)) {snprintf(err,cap,"status path too long");return -1;}
    /* O_EXCL避免跟随同名临时文件；0600限制本地状态信息访问。 */
    int fd=open(temp,O_WRONLY|O_CREAT|O_EXCL,0600);if(fd<0) goto failure;
    FILE *out=fdopen(fd,"w");if(!out) {int saved=errno;close(fd);unlink(temp);errno=saved;goto failure;}
    const mc_config *c=mc_engine_config(e);
    fprintf(out,"{\"schema\":2,\"version\":\"%s\",\"protocol\":\"%s\",\"monotonic_ms\":%" PRId64 ",\"device_address\":%u,\"can\":[",MC_VERSION,MC_PROTOCOL,now,c->address);
    /* 生命周期字段单独输出在顶层。正常退出running=false；异常退出文件可能
     * 保留true，读取方必须同时验证boot_id/PID-start-ticks及截止时间。 */
    for(size_t i=0;i<c->can_count;++i) {
        if(i) fputc(',',out);
        fputs("{\"interface\":",out);json_string(out,c->can_interfaces[i]);
        fprintf(out,",\"socket_open\":%s,\"queued_groups\":%zu,\"dropped_groups\":%" PRIu64,boolean(can_fds&&can_fds[i]>=0),runtime?runtime->can_queued_groups[i]:0,runtime?runtime->can_dropped_groups[i]:0);
        fprintf(out,",\"tx_pending\":%s,\"waiting_peer\":%s,\"tx_completed\":%" PRIu64 ",\"ack_timeouts\":%" PRIu64 ",\"recoveries\":%" PRIu64 ",\"resets\":%" PRIu64 ",\"retry_at_ms\":%" PRId64 "}",boolean(runtime&&runtime->can_pending[i]),boolean(runtime&&runtime->can_waiting_peer[i]),runtime?runtime->can_completed[i]:0,runtime?runtime->can_noack[i]:0,runtime?runtime->can_recoveries[i]:0,runtime?runtime->can_resets[i]:0,runtime?runtime->can_retry_at[i]:0);
    }
    char boot[80];unsigned long long ticks;process_identity(boot,&ticks);
    bool running=runtime&&runtime->running;
    unsigned interval=runtime&&runtime->interval_ms?runtime->interval_ms:5000;
    fprintf(out,"],\"running\":%s,\"pid\":%ld,\"process_start_ticks\":%llu,\"started_ms\":%" PRId64 ",\"status_interval_ms\":%u,\"fresh_until_ms\":%" PRId64 ",\"boot_id\":",boolean(running),(long)getpid(),ticks,runtime?runtime->started_ms:now,interval,running?now+3*(mc_millis)interval:now);
    json_string(out,boot);fputs(",\"stats\":{",out);
    const mc_stats *stats=mc_engine_stats((mc_engine *)e);
#define METRIC(name) fprintf(out,"\"" #name "\":%" PRIu64 ",",stats->name)
    METRIC(can_rx);METRIC(rejected);METRIC(duplicate);METRIC(queue_full);METRIC(polls);METRIC(controls);
    METRIC(serial_ok);METRIC(timeout);METRIC(crc_error);METRIC(mismatch);METRIC(exception);
    METRIC(stale_skipped);METRIC(can_tx_error);METRIC(serial_tx_error);METRIC(unsupported);METRIC(queries);METRIC(control_acks);
    METRIC(io_controls);METRIC(io_reports);METRIC(authority_requests);METRIC(authority_replays);METRIC(authority_denied);METRIC(controls_cancelled);
#undef METRIC
    fprintf(out,"\"query_replies\":%" PRIu64 "},\"authorization\":{\"enabled\":%s,\"leases\":[",stats->query_replies,boolean(c->authorization_enabled));
    mc_authority_status leases[MC_LEASES];size_t lease_count=mc_authority_snapshot(e,now,leases,MC_LEASES);
    for(size_t i=0;i<lease_count;++i) {
        if(i) fputc(',',out);
        fprintf(out,"{\"source\":%u,\"channel\":%u,\"expires_ms\":%" PRId64 ",\"generation\":%" PRIu64 "}",leases[i].source,leases[i].channel,leases[i].expires_ms,leases[i].generation);
    }
    fprintf(out,"]},\"io_backend\":\"%s\",\"io\":[",c->io_backend==MC_IO_SIMULATED?"simulated":"none");
    /* 用独立的first标志，不能按配置索引判断：某条路由快照失败时仍要输出合法JSON。 */
    bool first_io=true;
    for(size_t i=0;i<c->route_count;++i) {
        const mc_route *route=&c->routes[i];mc_io_status sample;
        if(mc_io_status_snapshot(e,route->channel,route->kind,now,&sample)) continue;
        if(!first_io) fputc(',',out);
        first_io=false;
        fprintf(out,"{\"channel\":%u,\"kind\":\"%s\",\"physical\":%u,\"slave\":%u,\"simulated\":true,\"valid\":%s,\"fresh\":%s,\"updated_ms\":",route->channel,route->kind==MC_ROUTE_DO?"do":(route->kind==MC_ROUTE_DI?"di":"ai"),route->physical,route->slave,boolean(sample.valid),boolean(sample.fresh));
        timestamp(out,sample.valid,sample.updated_ms);fputs(",\"words\":[",out);
        for(size_t k=0;k<sample.count;++k) fprintf(out,"%s%u",k?",":"",sample.words[k]);
        fputs("]}",out);
    }
    fputs("],\"devices\":[",out);
    /* 以下均为本地诊断，不发送授权码/摘要，不增加CAN字段。 */
    bool first=true;
    for(size_t i=0;i<c->port_count;++i) for(size_t k=0;k<c->ports[i].device_count;++k) {
        mc_device_status d;const mc_port *p=&c->ports[i];
        if(mc_device_snapshot(e,p->physical,p->devices[k].slave,now,&d)) continue;
        if(!first) fputc(',',out);
        first=false;
        fprintf(out,"{\"physical\":%u,\"channel\":%u,\"slave\":%u,\"path\":",d.physical,d.channel,d.slave);json_string(out,p->path);
        fprintf(out,",\"enabled\":%s,\"port_available\":%s,\"communication_online\":%s,\"cache_fresh\":%s,\"pending\":%s,\"transmitting\":%s,\"queued\":%zu,\"consecutive_failures\":%u,\"last_result\":\"%s\",\"last_exception\":%u,\"last_success_ms\":",
            boolean(d.enabled),boolean(d.port_available),boolean(d.communication_online),boolean(d.cache_fresh),boolean(d.pending),boolean(d.transmitting),d.queued,d.consecutive_failures,mc_result_name(d.last_result),d.last_exception);
        timestamp(out,d.has_success,d.last_success_ms);fputs(",\"last_response_ms\":",out);timestamp(out,d.has_response,d.last_response_ms);
        fputs(",\"last_sample_ms\":",out);timestamp(out,d.has_sample,d.last_sample_ms);
        fputs(",\"sample_age_ms\":",out);timestamp(out,d.has_sample&&now>=d.last_sample_ms,now-d.last_sample_ms);
        fprintf(out,",\"successes\":%" PRIu64 ",\"timeouts\":%" PRIu64 ",\"exceptions\":%" PRIu64 ",\"tx_errors\":%" PRIu64 ",\"crc_errors\":%" PRIu64 ",\"mismatches\":%" PRIu64 ",\"disconnects\":%" PRIu64 "}",
            d.successes,d.timeouts,d.exceptions,d.tx_errors,d.crc_errors,d.mismatches,d.disconnects);
    }
    fputs("]}\n",out);
    bool bad=ferror(out)!=0;
    if(fclose(out)) bad=true;
    if(bad||rename(temp,path)) {int saved=errno;unlink(temp);errno=saved;goto failure;}
    return 0;
failure:snprintf(err,cap,"status snapshot write failed: %s",strerror(errno));return -1;
}
