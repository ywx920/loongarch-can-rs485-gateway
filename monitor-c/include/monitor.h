#ifndef ZXJB14A_MONITOR_C_H
#define ZXJB14A_MONITOR_C_H

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/* C11独立实现。容量均有界，越界配置明确报错，不静默截断。 */
#define MC_VERSION "1.9.1-dev"
#define MC_PROTOCOL "aux_monitor_dw_20261014"
#define MC_PORTS 8
#define MC_DEVICES 15
#define MC_RANGES 32
#define MC_SOURCES 32
#define MC_QUEUE 256
#define MC_OUTPUT 4096
#define MC_PATH 512
#define MC_ROUTES 64
#define MC_LEASES 16

/* 时间统一为CLOCK_MONOTONIC毫秒：只比较间隔，不作为日历时间显示。
 * 通过显式传入now可用虚拟时间测试超时，不依赖sleep或板卡时钟。 */
typedef int64_t mc_millis;
typedef struct { unsigned type,source,destination,channel,slave,sequence; } mc_id;
/* bus为配置中CAN接口下标；length保留，以便引擎拒绝非8字节输入。 */
typedef struct { int bus; uint32_t id; uint8_t data[8]; size_t length; mc_millis expires_at; } mc_frame; /* expires_at为本地采样期限，不上CAN线。 */
typedef struct { unsigned start,count; } mc_range;
/* 通道类型：协议规定DO/DI/AI板卡也占用一个通道号，且CAN ID里的从站字段为0。
 * 当前只有"模拟后端"可写，真实板卡驱动必须等硬件形态确认后接入。
 * MC_ROUTE_RTU不放进路由表，RS485仍走原有port/device路径，避免双重映射。 */
typedef enum { MC_ROUTE_RTU=0,MC_ROUTE_DO,MC_ROUTE_DI,MC_ROUTE_AI } mc_route_kind;
typedef enum { MC_IO_NONE=0,MC_IO_SIMULATED=1 } mc_io_backend;
typedef struct {
    unsigned channel;       /* 协议通道号1..63 */
    unsigned kind;          /* mc_route_kind，当前只允许DO/DI/AI */
    unsigned physical;      /* 映射到哪个物理口；模拟后端只用于归属统计 */
    unsigned slave;         /* 真实板卡在该485口上的Modbus从站地址1..15 */
    unsigned reg_start;     /* 板卡寄存器块0基起点 */
    unsigned reg_count;     /* DO=2、DI=2、AI=16 */
} mc_route;
typedef struct {
    /* start是Modbus PDU的0基地址，不直接填写文档中的40001式编号。
     * count是16位寄存器个数；retries只用于周期读，写控制绝不自动重试。 */
    unsigned slave,function,start,count,poll_ms,timeout_ms,retries,write_function;
    bool control_enabled,query_enabled;
    mc_range writable[MC_RANGES],readable[MC_RANGES];
    size_t writable_count,readable_count;
    unsigned allowed_write_functions[2];
    size_t allowed_write_count;
} mc_device;
typedef struct {
    /* physical用于Linux设备/本地状态索引；channel进入CAN ID，二者可不同。
     * 同一物理485总线上可有多个slave，不能为每个从站虚构一个串口。 */
    unsigned physical,channel,baud,stopbits;
    char path[MC_PATH],parity;
    bool enabled;
    mc_device devices[MC_DEVICES];
    size_t device_count;
} mc_port;
typedef struct {
    unsigned address,heartbeat_ms,report_ms,frame_gap_ms,dedup_ms,stale_ms,queue_limit,baud;
    bool site_mapping_confirmed,parameters_enabled,allow_broadcast_control;
    bool primary_only;
    unsigned can_tx_confirm_ms,can_retry_max_ms; /* CAN发送完成等待与离线重试上限，不是485响应时间。 */
    bool can_rate_enabled,can_rate_both;
    char can_rate_state_file[MC_PATH]; /* 独立原子保存两路已确认速率；未确认不落盘。 */
    bool customer_gateway; /* 客户模式：动态寻址、双CAN冗余、无08、无主动轮询。 */
    bool heartbeat_enabled; /* 多板共址时可显式关闭本进程心跳；默认开启。 */
    /* io_backend=none时routes必须为空；simulated仅用于无硬件时的协议验证，
     * 绝不能把它当作真实DO/DI/AI输出。authorization默认关闭。 */
    unsigned io_backend;
    bool authorization_enabled,authorization_allow_force;
    unsigned authorization_lease_ms;
    char authorization_sha256[65]; /* 6位ASCII授权码的SHA-256十六进制；空=未设置 */
    mc_route routes[MC_ROUTES];
    size_t route_count;
    char runtime_settings[MC_PATH],can_interfaces[2][16];
    size_t can_count;
    unsigned targets[4],control_sources[MC_SOURCES],query_sources[MC_SOURCES];
    size_t target_count,control_source_count,query_source_count;
    mc_port ports[MC_PORTS];
    size_t port_count;
} mc_config;
typedef struct {
    uint64_t can_rx,rejected,duplicate,queue_full,polls,controls,serial_ok,crc_error;
    uint64_t mismatch,timeout,exception,stale_skipped,can_tx_error,serial_tx_error,unsupported;
    uint64_t queries,control_acks,query_replies;
    /* 1.4.0本地统计：模拟IO控制/上报与08授权请求，不进入CAN载荷。 */
    uint64_t io_controls,io_reports,authority_requests;
    uint64_t authority_replays,authority_denied,controls_cancelled;
} mc_stats;

/* 下面的诊断结构只在本地使用，不添加CAN字段、不改变DW载荷。
 * “通信在线”=本口可用且stale_ms内收到过合法RTU正常/异常回复；
 * Modbus异常也证明有从站回应，但last_result与failures仍标记业务失败。
 * “缓存新鲜”单独判断：成功查询/控制不能冒充周期采样。 */
typedef enum {
    MC_RESULT_NONE,MC_RESULT_OK,MC_RESULT_TIMEOUT,MC_RESULT_EXCEPTION,
    MC_RESULT_TX_ERROR,MC_RESULT_DISCONNECTED
} mc_result;
typedef struct {
    unsigned physical,channel,slave;
    /* enabled来自JSON；port_available来自Linux适配层，不代表物理接线正常。
     * online按最近合法响应判定，cache_fresh只按周期缓存判定，互不代替。 */
    bool enabled,port_available,communication_online,cache_fresh;
    bool has_success,has_response,has_sample,pending,transmitting;
    mc_millis last_success_ms,last_response_ms,last_sample_ms;
    /* 连续失败跨周期读/查询/控制统计，成功归零；读重试失败也算一次。
     * last_exception保留最后异常码，直到后续正常成功；0表示未记录异常。 */
    unsigned consecutive_failures,last_exception;
    mc_result last_result;
    size_t queued;
    /* CRC/mismatch表示“等待该从站时观察到”的坏输入，不断言来源就是它。
     * disconnects只数因掉线被中断的在途事务，不把每次重开失败算事务失败。 */
    uint64_t successes,timeouts,exceptions,tx_errors,crc_errors,mismatches,disconnects;
} mc_device_status;

/* 错误返回-1并填充err；成功返回0。API不抛异常，也不终止调用进程。 */
int mc_encode_id(mc_id id,uint32_t *value);
mc_id mc_decode_id(uint32_t value);
uint16_t mc_read_be(const uint8_t *bytes);
void mc_write_be(uint8_t *bytes,unsigned value);
uint16_t mc_crc16(const uint8_t *data,size_t length);
bool mc_valid_crc(const uint8_t *data,size_t length);
size_t mc_append_crc(uint8_t *data,size_t length);
void mc_config_defaults(mc_config *config);
void mc_device_defaults(mc_device *device);
int mc_config_validate(const mc_config *config,char *err,size_t capacity);
int mc_config_load(const char *path,mc_config *config,char *err,size_t capacity);
int mc_runtime_load(mc_config *config,char *err,size_t capacity);
int mc_runtime_save(const mc_config *config,char *err,size_t capacity);

/* 内部任务/缓存隐藏在不透明结构中，创建时分配一次，收发过程中不动态分配。 */
typedef struct mc_engine mc_engine;
typedef bool (*mc_send_fn)(void *context,unsigned physical,const uint8_t *data,size_t length);
typedef int (*mc_persist_fn)(void *context,const mc_config *candidate,char *err,size_t capacity);
mc_engine *mc_engine_create(const mc_config *config,mc_persist_fn persist,void *context,char *err,size_t capacity);
/* Linux适配层使用异步发送：接纳请求不等于完整写出，完成后才启动响应超时。
 * 默认同步模式保留给纯协议测试和参考实现差分测试。 */
void mc_engine_async(mc_engine *engine,bool enabled);
void mc_serial_tx_complete(mc_engine *engine,unsigned physical,bool success,mc_millis now);
/* 端口断开时撤销在途/排队事务及缓存；恢复只重新采集，绝不重放控制。 */
void mc_port_available(mc_engine *engine,unsigned physical,bool available,mc_millis now);
typedef struct {
    const char *reason;
    unsigned physical,channel,slave,function,start,count,kind,detail;
    mc_millis now;
} mc_event;
typedef void (*mc_event_fn)(void *context,const mc_event *event);
void mc_engine_events(mc_engine *engine,mc_event_fn callback,void *context);
void mc_engine_destroy(mc_engine *engine);
bool mc_engine_busy(const mc_engine *engine); /* 切速率前不打断485在途事务。 */
/* 只复制快照，不调度任务、不刷新缓存、不读写硬件。未知physical/slave返回-1。
 * API与引擎同属单线程，外部并发读取需要由调用者同步。 */
int mc_device_snapshot(const mc_engine *engine,unsigned physical,unsigned slave,mc_millis now,mc_device_status *status);
const char *mc_result_name(mc_result result);
const mc_config *mc_engine_config(const mc_engine *engine);
mc_stats *mc_engine_stats(mc_engine *engine);
void mc_receive_can(mc_engine *engine,const mc_frame *frame,mc_millis now);
void mc_receive_serial(mc_engine *engine,unsigned physical,const uint8_t *data,size_t length,mc_millis now);
/* 仅模拟后端故障注入：暂停/恢复采样，不改变DO镜像或真实硬件。 */
void mc_io_simulation_pause(mc_engine *engine,bool paused);
/* 只读复制镜像，不保证新鲜；需要有效性和年龄时使用mc_io_status_snapshot。 */
int mc_io_snapshot(const mc_engine *engine,unsigned channel,unsigned kind,uint16_t *words,size_t capacity,mc_millis now);
typedef struct {
    unsigned source,channel;
    mc_millis expires_ms;
    uint64_t generation;
} mc_authority_status;
/* 只返回当前有效租约；不续租、不返回授权码或摘要。 */
size_t mc_authority_snapshot(const mc_engine *engine,mc_millis now,mc_authority_status *out,size_t capacity);
typedef struct {
    bool valid,fresh;
    mc_millis updated_ms;
    uint16_t words[16];
    size_t count;
} mc_io_status;
int mc_io_status_snapshot(const mc_engine *engine,unsigned channel,unsigned kind,mc_millis now,mc_io_status *out);
/* 只读查询授权租约：source在该通道（0=全局）是否有未过期持有权。 */
bool mc_authority_allows(const mc_engine *engine,unsigned source,unsigned channel,mc_millis now);
void mc_tick(mc_engine *engine,mc_millis now,mc_send_fn send,void *context);
/* 环形输出队列：有帧返回true；调用者应持续取到false，再进行下一轮调度。 */
bool mc_take_output(mc_engine *engine,mc_frame *frame);
size_t mc_output_count(const mc_engine *engine);

#ifdef __cplusplus
}
#endif
#endif
