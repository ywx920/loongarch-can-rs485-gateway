#ifndef MONITOR_DIAGNOSTICS_H
#define MONITOR_DIAGNOSTICS_H
#include "monitor.h"
#include <stdio.h>
/* 均为本地运维接口：不会打开真实CAN/串口，不修改配置或引擎状态。 */
/* 配置路径检查不打开设备：拒绝runtime覆盖配置/串口，拒绝串口真实设备别名重复。 */
int mc_config_paths_safe(const char *config_path,const mc_config *config,char *err,size_t capacity);
/* /proc只读扫描：>0为占用PID，0为本次未发现占用，-1为无法完成检查。
 * 它不是内核全局锁，仍需TIOCEXCL和共享锁，正式切换前也必须停冲突服务。 */
int mc_serial_owner(const char *path,char *err,size_t capacity);
typedef struct {bool present,up,state_known,bitrate_known;unsigned state,bitrate,restart_ms;} mc_can_health;
int mc_can_inspect(const char *name,mc_can_health *health);
/* 只读查询设备/驱动/占用/目录，不发送CAN帧、不open/flush串口、不改变接口。 */
int mc_hardware_check(const mc_config *config,FILE *out);
int mc_site_check(const mc_config *config,FILE *out,bool strict);
void mc_status_print(const mc_engine *engine,mc_millis now,FILE *out);
/* 拒绝把状态文件写到配置、参数文件、串口设备或已有非普通文件。 */
int mc_status_path_safe(const char *path,const char *config_path,const mc_config *config,char *err,size_t capacity);
/* 临时普通文件+rename保证读者只看到一份完整JSON；目录由使用者提前创建。
 * 状态可以丢失，不fsync，不承担运行参数的持久化职责。返回-1不应停止采集。 */
typedef struct {
    bool running;
    mc_millis started_ms;
    unsigned interval_ms;
    size_t can_queued_groups[2];
    uint64_t can_dropped_groups[2];
    bool can_pending[2],can_waiting_peer[2];
    uint64_t can_completed[2],can_noack[2],can_recoveries[2],can_resets[2];
    mc_millis can_retry_at[2];
} mc_runtime_status;
/* schema2增加运行标识、boot_id/PID启动时刻、快照截止时间和全局统计。
 * 保留旧接口供核心测试/嵌入调用；Linux主程序使用_ex显式提交运行/退出状态。 */
int mc_status_write_ex(const char *path,const mc_engine *engine,mc_millis now,const int *can_fds,const mc_runtime_status *runtime,char *err,size_t capacity);
int mc_status_write(const char *path,const mc_engine *engine,mc_millis now,const int *can_fds,char *err,size_t capacity);
void mc_capabilities_print(FILE *out);
#endif
