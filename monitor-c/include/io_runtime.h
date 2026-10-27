#ifndef MC_IO_RUNTIME_H
#define MC_IO_RUNTIME_H
#include "monitor.h"
#include <sys/types.h>
/* 单事务缓冲：短写仅继续剩余字节；完整帧不会被重新提交。 */
typedef struct {uint8_t data[16];size_t length,offset;mc_millis deadline;bool active;} mc_serial_tx;
typedef ssize_t (*mc_write_fn)(void *,const uint8_t *,size_t);
bool mc_serial_begin(mc_serial_tx *,const uint8_t *,size_t,mc_millis);
/* 0等待可写，1完成，-1失败。每次最多16次系统调用，避免饿死其他端口。 */
int mc_serial_progress(mc_serial_tx *,mc_millis,mc_write_fn,void *);
#define MC_TX_GROUPS 256
typedef struct {mc_frame frames[15];unsigned count,offset,priority;mc_millis expires;} mc_tx_group;
typedef struct {mc_tx_group groups[MC_TX_GROUPS];size_t count;mc_millis next;uint64_t dropped;} mc_can_tx;
/* 按同一业务报文分组；03/02多帧从1开始，连续序号。每组最多15帧。 */
bool mc_can_enqueue(mc_can_tx *,const mc_frame *,size_t,mc_millis,unsigned);
const mc_frame *mc_can_front(mc_can_tx *,mc_millis);
void mc_can_advance(mc_can_tx *,bool success,mc_millis,unsigned);
#endif
