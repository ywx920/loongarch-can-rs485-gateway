#define _GNU_SOURCE
#include "monitor.h"
#include "can_rate.h"
#include "can_guard.h"
#include <sys/wait.h>
#include "monitor_diagnostics.h"
#include "board_led.h"
#include "io_runtime.h"
#include "service_notify.h"
#include <sys/un.h>
#include <errno.h>
#include <fcntl.h>
#include <inttypes.h>
#include <linux/can.h>
#include <linux/can/raw.h>
#include <linux/can/netlink.h>
#include <linux/netlink.h>
#include <linux/rtnetlink.h>
#include <net/if.h>
#include <poll.h>
#include <signal.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/file.h>
#include <sys/ioctl.h>
#include <sys/socket.h>
#include <sys/stat.h>
#include <sys/time.h>
#include <termios.h>
#include <time.h>
#include <unistd.h>

static volatile sig_atomic_t stopping=0;
static void stop_handler(int signal_number) {(void)signal_number;stopping=1;}
static mc_millis millis(void) {struct timespec t;clock_gettime(CLOCK_MONOTONIC,&t);return (mc_millis)t.tv_sec*1000+t.tv_nsec/1000000;}
static speed_t speed(unsigned baud) {
    switch(baud) {case 1200:return B1200;case 2400:return B2400;case 4800:return B4800;case 9600:return B9600;case 19200:return B19200;case 38400:return B38400;case 57600:return B57600;default:return 0;}
}
/* 只在初次连接/重连时设置串口；8位数据，按JSON选择校验与停止位。
 * 非阻塞fd交给poll循环，TIOCEXCL减少其它进程误打开；正式切换仍须停旧网关。
 * 配置检查模式不会调用本函数，避免探测时flush现场数据。 */
static int open_serial(const mc_port *p) {
    /* 每次重连也检查占用，防止掉线期间另一程序接管后被root误打开。
     * 专用测试构建明确跳过此保护：PTY模拟器会持有slave端作为夹具。
     * 生产程序没有测试参数入口，始终执行保护。 */
#ifndef MC_TEST_TRANSPORT
    char reason[160];int owner=mc_serial_owner(p->path,reason,sizeof(reason));
    if(owner) {errno=owner>0?EBUSY:EACCES;return -1;}
#endif
    int fd=open(p->path,O_RDWR|O_NOCTTY|O_NONBLOCK|O_CLOEXEC);if(fd<0) return -1;
    struct termios t;
    if(ioctl(fd,TIOCEXCL)<0||tcgetattr(fd,&t)<0) goto fail;
    cfmakeraw(&t);speed_t b=speed(p->baud);cfsetispeed(&t,b);cfsetospeed(&t,b);
    t.c_cflag&=~(CSIZE|PARENB|PARODD|CSTOPB|CRTSCTS);t.c_cflag|=CS8|CLOCAL|CREAD;
    if(p->parity=='E') t.c_cflag|=PARENB;
    if(p->parity=='O') t.c_cflag|=PARENB|PARODD;
    if(p->stopbits==2) t.c_cflag|=CSTOPB;
    t.c_cc[VMIN]=0;t.c_cc[VTIME]=0;
    if(tcsetattr(fd,TCSANOW,&t)<0||tcflush(fd,TCIOFLUSH)<0) goto fail;
    return fd;
fail: {int saved=errno;close(fd);errno=saved;return -1;}
}
#ifdef MC_TEST_TRANSPORT
static const char *test_can_dir;
static unsigned test_chunk,test_eagain,test_calls;
static int open_test_can(unsigned bus) {
    int fd=socket(AF_UNIX,SOCK_DGRAM|SOCK_NONBLOCK|SOCK_CLOEXEC,0);
    if(fd<0) return -1;
    struct sockaddr_un local={0},peer={0};local.sun_family=peer.sun_family=AF_UNIX;
    if(snprintf(local.sun_path,sizeof(local.sun_path),"%s/monitor-%u.sock",test_can_dir,bus)>=(int)sizeof(local.sun_path)||
       snprintf(peer.sun_path,sizeof(peer.sun_path),"%s/peer-%u.sock",test_can_dir,bus)>=(int)sizeof(peer.sun_path)) {close(fd);errno=ENAMETOOLONG;return -1;}
    unlink(local.sun_path);
    if(bind(fd,(struct sockaddr *)&local,sizeof(local))||connect(fd,(struct sockaddr *)&peer,sizeof(peer))) {int saved=errno;close(fd);errno=saved;return -1;}
    return fd;
}
#endif
/* 绑定已经由硬件准备服务配置好的CAN接口，不在应用里改波特率/UP状态。
 * 开启错误帧订阅便于记录bus-off等状态；socket可打开不等于线路有ACK。 */
static int open_can(const char *name) {
    int fd=socket(PF_CAN,SOCK_RAW|SOCK_NONBLOCK|SOCK_CLOEXEC,CAN_RAW);if(fd<0) return -1;
    unsigned index=if_nametoindex(name);if(!index) goto fail;
    struct sockaddr_can address={0};address.can_family=AF_CAN;address.can_ifindex=(int)index;
    if(bind(fd,(struct sockaddr *)&address,sizeof(address))<0) goto fail;
    int own=1;if(setsockopt(fd,SOL_CAN_RAW,CAN_RAW_RECV_OWN_MSGS,&own,sizeof(own))<0)goto fail;
    can_err_mask_t mask=CAN_ERR_MASK;if(setsockopt(fd,SOL_CAN_RAW,CAN_RAW_ERR_FILTER,&mask,sizeof(mask))<0) goto fail;return fd;
fail: {int saved=errno;close(fd);errno=saved;return -1;}
}
/* LED只复用预检的控制器查询；这仍不代表物理ACK正常。 */
static bool healthy(const char *name) {
    mc_can_health h;return !mc_can_inspect(name,&h)&&h.up&&h.state_known&&h.state==CAN_STATE_ERROR_ACTIVE;
}
/* Linux适配层：协议引擎不持有文件描述符，便于用PTY/Unix数据报做完整进程测试。 */
typedef struct {
    int serial[MC_PORTS];mc_serial_tx pending[MC_PORTS];mc_millis reconnect[MC_PORTS];
    mc_led *led;mc_millis now;bool led_failed;
} io_context;
static bool serial_send(void *context,unsigned physical,const uint8_t *data,size_t length) {
    io_context *io=context;unsigned i=physical-1;
    if(io->serial[i]<0||tcflush(io->serial[i],TCIFLUSH)<0) return false;
    /* 发送缓冲最多16字节；1秒只约束向TTY提交，响应超时从提交完整帧后开始。 */
    return mc_serial_begin(&io->pending[i],data,length,io->now+1000);
}
static ssize_t serial_write(void *context,const uint8_t *data,size_t length) {
    int fd=*(int *)context;
#ifdef MC_TEST_TRANSPORT
    if(test_eagain&&++test_calls%test_eagain==0) {errno=EAGAIN;return -1;}
    if(test_chunk&&length>test_chunk) length=test_chunk;
#endif
    return write(fd,data,length);
}
static int persist(void *unused,const mc_config *candidate,char *err,size_t cap) {(void)unused;return mc_runtime_save(candidate,err,cap);}
static void show_stats(const mc_stats *s) {
#define S(name) printf(" " #name "=%" PRIu64,s->name)
    printf("STATS");S(can_rx);S(polls);S(controls);S(serial_ok);S(rejected);S(duplicate);S(timeout);S(crc_error);S(mismatch);S(exception);S(stale_skipped);S(queue_full);S(can_tx_error);S(serial_tx_error);S(unsupported);S(queries);S(control_acks);S(query_replies);S(io_controls);S(io_reports);S(authority_requests);S(authority_replays);S(authority_denied);S(controls_cancelled);puts("");fflush(stdout);
#undef S
}
typedef struct {bool verbose;mc_millis window;unsigned logged,suppressed;} log_context;
/* 事件回调不修改引擎。默认省略正常高频事务，--verbose用于短期联调。
 * 每秒最多100条引擎事件；抑制数量仍报告，便于判断日志是否截流。 */
static void diagnostic(void *context,const mc_event *v) {
    log_context *log=context;
    if(!log->verbose&&(!strcmp(v->reason,"sent")||!strcmp(v->reason,"accepted")||!strcmp(v->reason,"serial_ok"))) return;
    if(v->now-log->window>=1000) {
        if(log->suppressed) fprintf(stderr,"EVENT reason=log_rate_limit suppressed=%u\n",log->suppressed);
        log->window=v->now;log->logged=log->suppressed=0;
    }
    if(log->logged++>=100) {++log->suppressed;return;}
    fprintf(stderr,"EVENT ms=%" PRId64 " reason=%s physical=%u channel=%u slave=%u function=%u start=%u count=%u kind=%u detail=%u\n",
            v->now,v->reason,v->physical,v->channel,v->slave,v->function,v->start,v->count,v->kind,v->detail);
}
/* 只影响该physical口。先关闭fd、取消待提交片段，再通知引擎撤销任务。
 * 下一轮重开后只重新采集，不恢复被撤销的控制队列。 */
static void serial_disconnect(io_context *io,mc_engine *e,unsigned physical,mc_millis now) {
    unsigned i=physical-1;
    if(io->serial[i]>=0) {ioctl(io->serial[i],TIOCNXCL);close(io->serial[i]);io->serial[i]=-1;}
    io->pending[i].active=false;io->reconnect[i]=now+1000;
    mc_port_available(e,physical,false,now);
}
/* 每条业务报文整组排队：从序号1到最后一帧，不与另一条报文交错。 */
static void transfer_output(mc_engine *e,mc_can_tx *tx,const mc_config *c,mc_millis now,const mc_can_guard *guards) {
    mc_frame groups[2][15],frame;size_t counts[2]={0};uint32_t keys[2]={0};
    while(mc_take_output(e,&frame)) {
        if(frame.bus<0||(size_t)frame.bus>=c->can_count) {++mc_engine_stats(e)->rejected;continue;}
        unsigned bus=(unsigned)frame.bus;
        if(guards[bus].reset_needed||now<guards[bus].retry_at)continue; /* 冷却期间丢弃过期业务，不积压或补发。 */
        mc_id id=mc_decode_id(frame.id);uint32_t key=frame.id&~15u;
        size_t n=counts[bus];
        if(n&&(key!=keys[bus]||!id.sequence||id.sequence==1||id.sequence!=mc_decode_id(groups[bus][n-1].id).sequence+1)) {
            if(!mc_can_enqueue(&tx[bus],groups[bus],n,now,c->stale_ms)) ++mc_engine_stats(e)->queue_full;
            n=0;
        }
        keys[bus]=key;groups[bus][n++]=frame;
        if(n==15||id.sequence==0) {
            if(!mc_can_enqueue(&tx[bus],groups[bus],n,now,c->stale_ms)) ++mc_engine_stats(e)->queue_full;
            n=0;
        }
        counts[bus]=n;
    }
    for(size_t bus=0;bus<c->can_count;++bus) if(counts[bus])
        if(!mc_can_enqueue(&tx[bus],groups[bus],counts[bus],now,c->stale_ms)) ++mc_engine_stats(e)->queue_full;
}
#include "can_rate_linux.h"
#include "can_guard_linux.h"
int main(int argc,char **argv) {
    /* CLI只控制本地检查/状态输出。协议和现场参数始终从JSON加载，
     * 不在命令行增加绕过来源/白名单的开关。 */
    const char *path=NULL,*status_file=NULL;
    bool check=false,check_site=false,check_hardware=false,strict_site=false,device_status=false,leds=false;
    unsigned status_interval=5000;log_context log={0};
    for(int i=1;i<argc;++i) {
        if(!strcmp(argv[i],"--config")&&i+1<argc) path=argv[++i];
        else if(!strcmp(argv[i],"--check-config")) check=true;
        else if(!strcmp(argv[i],"--check-hardware")) check_hardware=true;
        else if(!strcmp(argv[i],"--check-site")) check_site=true;
        else if(!strcmp(argv[i],"--strict-site")) strict_site=true;
        else if(!strcmp(argv[i],"--device-status")) device_status=true;
        else if(!strcmp(argv[i],"--status-file")&&i+1<argc) status_file=argv[++i];
        else if(!strcmp(argv[i],"--status-interval-ms")&&i+1<argc) {
            char *end;errno=0;const char *value=argv[++i];unsigned long n=strtoul(value,&end,10);
            if(errno||!*value||*end||n<100||n>3600000) {fprintf(stderr,"ERROR: status interval must be 100..3600000 ms\n");return 1;}
            status_interval=(unsigned)n;
        }
        else if(!strcmp(argv[i],"--board-leds")) leds=true;
        else if(!strcmp(argv[i],"--capabilities")) {mc_capabilities_print(stdout);return 0;}
        else if(!strcmp(argv[i],"--version")) {puts("zxjb14a_monitor_c " MC_VERSION " DW20261014 static");return 0;}
        else if(!strcmp(argv[i],"--verbose")) log.verbose=true;
#ifdef MC_TEST_TRANSPORT
        else if(!strcmp(argv[i],"--test-can-dir")&&i+1<argc) test_can_dir=argv[++i];
        else if(!strcmp(argv[i],"--test-write-chunk")&&i+1<argc) test_chunk=(unsigned)strtoul(argv[++i],NULL,10);
        else if(!strcmp(argv[i],"--test-eagain-every")&&i+1<argc) test_eagain=(unsigned)strtoul(argv[++i],NULL,10);
#endif
        else if(!strcmp(argv[i],"--help")) {puts("Usage: zxjb14a_monitor_c --config FILE [--check-config] [--check-hardware] [--check-site [--strict-site]] [--device-status] [--status-file FILE] [--status-interval-ms 100..3600000] [--board-leds] [--verbose] [--capabilities]");return 0;}
        else {fprintf(stderr,"ERROR: unknown or incomplete argument: %s\n",argv[i]);return 1;}
    }
    char error[256];mc_config *config=calloc(1,sizeof(*config));
    if(!config||!path) {fprintf(stderr,"ERROR: --config is required or allocation failed\n");free(config);return 1;}
    if(mc_config_load(path,config,error,sizeof(error))||mc_config_paths_safe(path,config,error,sizeof(error))||mc_runtime_load(config,error,sizeof(error))) {fprintf(stderr,"ERROR: %s\n",error);free(config);return 1;}
    if(config->can_rate_enabled&&(mc_status_path_safe(config->can_rate_state_file,path,config,error,sizeof(error))||(status_file&&!strcmp(status_file,config->can_rate_state_file)))) {fprintf(stderr,"ERROR: unsafe CAN rate state path\n");free(config);return 1;}
    if(strict_site&&!check_site) {fprintf(stderr,"ERROR: --strict-site requires --check-site\n");free(config);return 1;}
    /* 防止状态快照误覆盖现场配置/运行参数；即使检查模式也报告错误。 */
    if(status_file&&mc_status_path_safe(status_file,path,config,error,sizeof(error))) {fprintf(stderr,"ERROR: %s\n",error);free(config);return 1;}
    if(check||check_site||check_hardware) {
        int inspected=0;
        if(check_site) inspected=mc_site_check(config,stdout,strict_site);
        if(check_hardware&&mc_hardware_check(config,stdout)) inspected=2;
        printf("Protocol %s, address=%u, mapping_confirmed=%d\n",MC_PROTOCOL,config->address,config->site_mapping_confirmed);
        for(size_t i=0;i<config->port_count;++i) {mc_port *p=&config->ports[i];printf("physical %u -> protocol channel %u -> %s enabled=%d devices=%zu\n",p->physical,p->channel,p->path,p->enabled,p->device_count);}
        free(config);return inspected;
    }
    if(!config->site_mapping_confirmed) {fprintf(stderr,"ERROR: site_mapping_confirmed=false\n");free(config);return 1;}
#ifndef MC_TEST_TRANSPORT
    /* 在任何串口、CAN或GPIO初始化前先拒绝旧服务占用；绝不自动kill/stop。
     * 缺失串口允许进入原有重连流程；存在但无法检查占用则失败关闭。 */
    for(size_t i=0;i<config->port_count;++i) if(config->ports[i].enabled) {
        struct stat st;
        if(!stat(config->ports[i].path,&st)) {
            char reason[160];int owner=mc_serial_owner(config->ports[i].path,reason,sizeof(reason));
            if(owner) {fprintf(stderr,"ERROR: serial_startup_guard physical=%u %s\n",config->ports[i].physical,reason);free(config);return 2;}
        }
    }
#endif
    mc_service_notify notify={.fd=-1};
    int lock=-1,result=1;mc_engine *engine=NULL;mc_can_tx *tx=NULL;mc_led led={0};led.fd=-1;
    mc_can_guard guards[2]={{0}};
    for(unsigned b=0;b<config->can_count;++b){guards[b].confirm_ms=config->can_tx_confirm_ms;guards[b].retry_max_ms=config->can_retry_max_ms;mc_can_health h;if(mc_can_inspect(config->can_interfaces[b],&h)||!h.up||(h.state_known&&h.state!=CAN_STATE_ERROR_ACTIVE))guards[b].reset_needed=true;}
    guard_reset_state resets[2]={{0}};
    bool interrupted_reset[2]={false,false};
    guard_log_state guard_logs[2]={{0}};
    mc_millis next_can_summary=0;
    mc_rate rate;unsigned initial_rates[2]={250000,250000};mc_rate_init(&rate,(unsigned)config->can_count,initial_rates,config->can_rate_both);
    int can_fd[2]={-1,-1};mc_millis can_retry[2]={0};io_context io={0};io.led=&led;
    rate_io rate_context={config,can_fd,NULL,can_retry,&rate,guards};
    for(unsigned i=0;i<MC_PORTS;++i) io.serial[i]=-1;
    const char *lock_path="/run/lock/zxjb14a-monitor.lock";
#ifdef MC_TEST_TRANSPORT
    char test_lock[MC_PATH];
    if(test_can_dir) {snprintf(test_lock,sizeof(test_lock),"%s/monitor.lock",test_can_dir);lock_path=test_lock;}
#endif
    lock=open(lock_path,O_CREAT|O_RDWR|O_CLOEXEC,0600);
    if(lock<0||flock(lock,LOCK_EX|LOCK_NB)<0) {perror("ERROR: lock monitor");goto cleanup;}
    if(mc_led_init(&led,leds)) {perror("ERROR: initialize board LEDs");goto cleanup;}
    engine=mc_engine_create(config,persist,NULL,error,sizeof(error));tx=calloc(config->can_count,sizeof(*tx));
    if(!engine||!tx) {fprintf(stderr,"ERROR: allocation failed\n");goto cleanup;}
    rate_context.tx=tx;
    if(config->can_rate_enabled){
        FILE *state=fopen(config->can_rate_state_file,"r");
        if(state){unsigned a,b;char extra;int n=fscanf(state,"%u %u %c",&a,&b,&extra);fclose(state);
            if(n!=2||!mc_rate_valid(a)||!mc_rate_valid(b)){fprintf(stderr,"ERROR: invalid CAN rate state\n");goto cleanup;}
            rate.rates[0]=a;rate.rates[1]=b;
            for(unsigned bus=0;bus<config->can_count;++bus)if(!rate_apply(&rate_context,bus,rate.rates[bus]))goto cleanup;
        }else if(errno!=ENOENT){perror("ERROR: read CAN rate state");goto cleanup;}
        else for(unsigned bus=0;bus<config->can_count;++bus){mc_can_health h;if(mc_can_inspect(config->can_interfaces[bus],&h)||!h.bitrate_known){fprintf(stderr,"ERROR: cannot inspect CAN rate\n");goto cleanup;}rate.rates[bus]=h.bitrate;}
    }
    mc_engine_async(engine,true);mc_engine_events(engine,diagnostic,&log);
    for(size_t i=0;i<config->port_count;++i) {
        const mc_port *p=&config->ports[i];if(mc_led_enable(&led,p->physical,p->enabled)) goto cleanup;
        if(p->enabled) mc_port_available(engine,p->physical,false,millis());
    }
    signal(SIGINT,stop_handler);signal(SIGTERM,stop_handler);
    mc_millis next_status=0,next_stats=0,next_snapshot=0;
    mc_runtime_status runtime={.running=true,.started_ms=millis(),.interval_ms=status_interval};
    puts("READY: DW20261014 C11 " MC_VERSION " static production; reconnect enabled");fflush(stdout);
    mc_service_init(&notify,millis());
    if(notify.fd>=0&&!mc_service_send(&notify,"READY=1")) fprintf(stderr,"EVENT reason=service_ready_notify_failed\n");
    while(!stopping) {
        mc_millis now=millis();io.now=now;
        /* 一个端口掉线不会终止另外七路；每秒尝试重开，写命令不重放。 */
        for(size_t i=0;i<config->port_count;++i) {
            const mc_port *p=&config->ports[i];unsigned index=p->physical-1;
            if(!p->enabled||io.serial[index]>=0||now<io.reconnect[index]) continue;
            io.serial[index]=open_serial(p);io.reconnect[index]=now+1000;
            if(io.serial[index]>=0) mc_port_available(engine,p->physical,true,now);
            else fprintf(stderr,"EVENT ms=%" PRId64 " reason=serial_open_failed physical=%u path=%s errno=%d\n",now,p->physical,p->path,errno);
        }
        for(size_t bus=0;bus<config->can_count;++bus) if(can_fd[bus]<0&&!guards[bus].reset_needed&&now>=can_retry[bus]) {
#ifdef MC_TEST_TRANSPORT
            can_fd[bus]=test_can_dir?open_test_can((unsigned)bus):open_can(config->can_interfaces[bus]);
#else
            can_fd[bus]=open_can(config->can_interfaces[bus]);
#endif
            can_retry[bus]=now+1000;
            guard_log(&guard_logs[bus],now,(unsigned)bus,"can_open",can_fd[bus]<0?errno:0);
        }
        for(unsigned bus=0;bus<config->can_count;++bus){
            if(mc_guard_expire(&guards[bus],now))guard_log(&guard_logs[bus],now,bus,"can_noack",guards[bus].retry_at-now);
            if(guards[bus].reset_needed){
                if(can_fd[bus]>=0){close(can_fd[bus]);can_fd[bus]=-1;}
                memset(&tx[bus],0,sizeof(tx[bus]));
                int reset_result=0;
#ifdef MC_TEST_TRANSPORT
                if(test_can_dir)reset_result=1;else
#endif
                reset_result=guard_reset_tick(&resets[bus],config->can_interfaces[bus],now);
                if(reset_result){
                    ++guards[bus].resets;guards[bus].reset_needed=false;
                    if(reset_result<0)guards[bus].retry_at=now+30000;
                    can_retry[bus]=now;
                    guard_log(&guard_logs[bus],now,bus,"can_recover_reset",reset_result>0);
                }
            }
        }
        struct pollfd events[10];int buses[10];unsigned physicals[10];nfds_t endpoints=0;
        for(size_t bus=0;bus<config->can_count;++bus) if(can_fd[bus]>=0) {
            events[endpoints]=(struct pollfd){can_fd[bus],POLLIN,0};buses[endpoints]=(int)bus;physicals[endpoints++]=0;
        }
        for(unsigned i=0;i<MC_PORTS;++i) if(io.serial[i]>=0) {
            events[endpoints]=(struct pollfd){io.serial[i],POLLIN|(io.pending[i].active?POLLOUT:0),0};buses[endpoints]=-1;physicals[endpoints++]=i+1;
        }
        if(poll(events,endpoints,1)<0) {if(errno==EINTR) continue;perror("ERROR: poll");goto cleanup;}
        now=millis();io.now=now;
        for(nfds_t i=0;i<endpoints;++i) {
            if((events[i].revents&(POLLHUP|POLLNVAL))||((events[i].revents&POLLERR)&&buses[i]<0)) {
                if(buses[i]<0) serial_disconnect(&io,engine,physicals[i],now);
                else {close(can_fd[buses[i]]);can_fd[buses[i]]=-1;can_retry[buses[i]]=now+1000;}
                continue;
            }
            if(!(events[i].revents&POLLIN)) continue;
            for(unsigned k=0;k<64;++k) {
                if(buses[i]>=0) {
                    struct can_frame raw;struct iovec vector={&raw,sizeof(raw)};struct msghdr message={0};message.msg_iov=&vector;message.msg_iovlen=1;ssize_t n=recvmsg(events[i].fd,&message,0);
                    if(n<0&&(errno==EAGAIN||errno==EWOULDBLOCK)) break;
                    if(n<0&&errno==EINTR) continue;
                    if(n!=sizeof(raw)) {close(can_fd[buses[i]]);can_fd[buses[i]]=-1;can_retry[buses[i]]=now+1000;break;}
                    if(raw.can_id&CAN_ERR_FLAG) {guard_log(&guard_logs[buses[i]],now,(unsigned)buses[i],"can_error",raw.can_id&CAN_ERR_MASK);++mc_engine_stats(engine)->can_tx_error;continue;}
                    if(raw.can_id&CAN_RTR_FLAG||!(raw.can_id&CAN_EFF_FLAG)||raw.can_dlc!=8) {++mc_engine_stats(engine)->rejected;continue;}
                    mc_frame frame={0};frame.bus=buses[i];frame.id=raw.can_id&CAN_EFF_MASK;frame.length=8;memcpy(frame.data,raw.data,8);
                    /* 只消费本socket的发送完成回送，不把自己的心跳/ACK当外部指令。 */
                    if(message.msg_flags&MSG_CONFIRM){
                        bool recovered=guards[buses[i]].blocked;
                        if(mc_guard_confirm(&guards[buses[i]],&frame,now)&&recovered)fprintf(stderr,"EVENT ms=%" PRId64 " reason=can_peer_recovered bus=%d\n",now,buses[i]);
                        continue;
                    }
                    if(!(message.msg_flags&MSG_DONTROUTE))mc_guard_external(&guards[buses[i]],now);
                    mc_id incoming=mc_decode_id(frame.id);
                    if(config->can_rate_enabled&&incoming.type==7&&frame.data[0]==1&&incoming.destination==mc_engine_config(engine)->address){
                        /* 本地socket回送不能证明新速率外部通信成功。 */
                        if(rate.phase==2&&(message.msg_flags&MSG_DONTROUTE)){++mc_engine_stats(engine)->rejected;continue;}
                        if(!rate.phase&&mc_engine_busy(engine)){rate_reply(&rate_context,&frame,false);++mc_engine_stats(engine)->rejected;continue;}
                    }
                    if(rate.phase&&(rate.mask&(1u<<(unsigned)frame.bus))&&(incoming.type==1||incoming.type==3)){++mc_engine_stats(engine)->rejected;continue;}
                    bool rate_handled=config->can_rate_enabled&&mc_rate_receive(&rate,&frame,mc_engine_config(engine)->address,now,rate_reply,&rate_context);
                    if(rate_handled)++mc_engine_stats(engine)->can_rx;else mc_receive_can(engine,&frame,now);
                } else {
                    uint8_t buffer[256];ssize_t n=read(events[i].fd,buffer,sizeof(buffer));
                    if(!n||(n<0&&(errno==EAGAIN||errno==EWOULDBLOCK))) break;
                    if(n<0&&errno==EINTR) continue;
                    if(n<0) {serial_disconnect(&io,engine,physicals[i],now);break;}
                    if(mc_led_activity(&led,physicals[i],now)) goto cleanup;
                    mc_receive_serial(engine,physicals[i],buffer,(size_t)n,now);
                }
            }
        }
        mc_tick(engine,now,serial_send,&io);
        for(unsigned i=0;i<MC_PORTS;++i) if(io.pending[i].active&&io.serial[i]>=0) {
            int state=mc_serial_progress(&io.pending[i],now,serial_write,&io.serial[i]);
            if(state) {
                mc_serial_tx_complete(engine,i+1,state>0,now);
                if(state<0) serial_disconnect(&io,engine,i+1,now);
                else if(mc_led_activity(&led,i+1,now)) goto cleanup;
            }
        }
        const mc_config *current=mc_engine_config(engine);transfer_output(engine,tx,current,now,guards);
        for(size_t bus=0;bus<config->can_count;++bus) {
            /* 切换窗口暂停普通业务输出；07接受/确认/失败回复仍发送。 */
            if(rate.phase&&(rate.mask&(1u<<bus))){
                const mc_frame *queued;
                while((queued=mc_can_front(&tx[bus],now))&&mc_decode_id(queued->id).type!=7)mc_can_advance(&tx[bus],false,now,current->frame_gap_ms);
            }
            uint64_t dropped=tx[bus].dropped;
            const mc_frame *frame=mc_can_front(&tx[bus],now);
            if(frame&&can_fd[bus]>=0&&mc_guard_ready(&guards[bus],now)&&!(rate.fault&&(rate.mask&(1u<<bus)))) {
                struct can_frame raw={0};raw.can_id=frame->id|CAN_EFF_FLAG;raw.can_dlc=8;memcpy(raw.data,frame->data,8);
                ssize_t n=write(can_fd[bus],&raw,sizeof(raw));
                if(n==sizeof(raw)){
#ifdef MC_TEST_TRANSPORT
                    if(!test_can_dir)
#endif
                    mc_guard_sent(&guards[bus],frame,now);
                    mc_can_advance(&tx[bus],true,now,current->frame_gap_ms);
                }
                else if(n<0&&(errno==ENOBUFS||errno==EAGAIN||errno==EWOULDBLOCK))mc_guard_pressure(&guards[bus],now);
                else if(n>=0||errno!=EINTR) {
                    mc_can_advance(&tx[bus],false,now,current->frame_gap_ms);
                    close(can_fd[bus]);can_fd[bus]=-1;can_retry[bus]=now+1000;mc_guard_fail(&guards[bus],now);
                }
            }
            if(tx[bus].dropped!=dropped) {
                mc_engine_stats(engine)->can_tx_error+=tx[bus].dropped-dropped;
                guard_log(&guard_logs[bus],now,(unsigned)bus,"can_group_dropped",(int64_t)(tx[bus].dropped-dropped));
            }
        }
        if(config->can_rate_enabled){bool ready=true;
            for(unsigned b=0;b<config->can_count;++b)if((rate.mask&(1u<<b))&&(tx[b].count||guards[b].pending||guards[b].reset_needed))ready=false;
            mc_rate_tick(&rate,now,ready,rate_apply,rate_save,rate_reply,&rate_context);
        }
        /* 未启用LED时不执行阻塞Netlink检查；单路CAN也可正常工作。 */
        if(leds&&now>=next_status) {
            bool normal=true;for(size_t i=0;i<config->can_count;++i) normal=healthy(config->can_interfaces[i])&&normal;
            if(mc_led_normal(&led,normal,now)) goto cleanup;
            next_status=now+500;
        }
        if(mc_led_update(&led,now)) goto cleanup;
        /* 状态输出只复制诊断数据。可选文件输出失败只记日志，不能拖停采集。
         * 默认关闭文件写入；较长间隔可减少板端eMMC写入和日志量。 */
        if((device_status||status_file)&&now>=next_snapshot) {
            if(device_status) mc_status_print(engine,now,stdout);
            for(size_t bus=0;bus<config->can_count;++bus) {runtime.can_queued_groups[bus]=tx[bus].count;runtime.can_dropped_groups[bus]=tx[bus].dropped;runtime.can_pending[bus]=guards[bus].pending;runtime.can_waiting_peer[bus]=guards[bus].blocked;runtime.can_completed[bus]=guards[bus].completed;runtime.can_noack[bus]=guards[bus].timeouts;runtime.can_recoveries[bus]=guards[bus].recoveries;runtime.can_resets[bus]=guards[bus].resets;runtime.can_retry_at[bus]=guards[bus].retry_at;}
            if(status_file&&mc_status_write_ex(status_file,engine,now,can_fd,&runtime,error,sizeof(error))) fprintf(stderr,"EVENT reason=status_write_failed message=%s\n",error);
            next_snapshot=now+status_interval;
        }
        if(now>=next_can_summary){
            for(unsigned b=0;b<config->can_count;++b)fprintf(stderr,"CAN_SUMMARY bus=%u state=%s completed=%" PRIu64 " timeouts=%" PRIu64 " recoveries=%" PRIu64 " resets=%" PRIu64 "\n",b,mc_guard_state(&guards[b]),guards[b].completed,guards[b].timeouts,guards[b].recoveries,guards[b].resets);
            next_can_summary=now+60000;
        }
        mc_service_tick(&notify,now);
        if(now>=next_stats) {show_stats(mc_engine_stats(engine));next_stats=now+5000;}
    }
    /* 最后保存一次退出前快照，便于离线回查；它是历史快照，不表示进程仍在线。
     * 读者需结合monotonic_ms与当前系统运行时间检查文件是否停止更新。 */
    if(device_status) mc_status_print(engine,millis(),stdout);
    runtime.running=false;
    for(size_t bus=0;bus<config->can_count;++bus) {runtime.can_queued_groups[bus]=tx[bus].count;runtime.can_dropped_groups[bus]=tx[bus].dropped;runtime.can_pending[bus]=guards[bus].pending;runtime.can_waiting_peer[bus]=guards[bus].blocked;runtime.can_completed[bus]=guards[bus].completed;runtime.can_noack[bus]=guards[bus].timeouts;runtime.can_recoveries[bus]=guards[bus].recoveries;runtime.can_resets[bus]=guards[bus].resets;runtime.can_retry_at[bus]=guards[bus].retry_at;}
    if(status_file&&mc_status_write_ex(status_file,engine,millis(),can_fd,&runtime,error,sizeof(error))) fprintf(stderr,"EVENT reason=status_write_failed message=%s\n",error);
    show_stats(mc_engine_stats(engine));result=0;
cleanup:
    for(unsigned b=0;b<2;++b)if(resets[b].child>0){interrupted_reset[b]=true;kill(resets[b].child,SIGKILL);waitpid(resets[b].child,NULL,0);}
    if(config->can_rate_enabled&&rate.phase)mc_rate_cancel(&rate,rate_apply,&rate_context);
    (void)mc_service_send(&notify,"STOPPING=1");mc_service_close(&notify);
    mc_engine_destroy(engine);free(tx);
    for(unsigned i=0;i<MC_PORTS;++i) if(io.serial[i]>=0) {ioctl(io.serial[i],TIOCNXCL);close(io.serial[i]);}
    for(unsigned i=0;i<2;++i) if(can_fd[i]>=0) close(can_fd[i]);
    /* stop时取消仍悬挂的硬件发送，避免应用已退出而控制器继续重发。
     * 无取消发送API时仅关闭受影响接口；下次启动会按原速率恢复UP。
     * 不写新寄存器、不重放队列、不改变健康线路。 */
#ifdef MC_TEST_TRANSPORT
    if(!test_can_dir)
#endif
    for(unsigned b=0;b<config->can_count;++b)if(guards[b].pending||guards[b].reset_needed||interrupted_reset[b]){
        char *down[]={"ip","link","set",config->can_interfaces[b],"down",NULL};
        if(!rate_ip(down))fprintf(stderr,"EVENT reason=can_stop_cleanup_failed bus=%u\n",b);
    }
    mc_led_close(&led);if(lock>=0) close(lock);free(config);return result;
}
