#ifndef MC_SERVICE_NOTIFY_H
#define MC_SERVICE_NOTIFY_H
#include "monitor.h"
#include <sys/socket.h>
#include <sys/un.h>
typedef struct {int fd;struct sockaddr_un address;socklen_t length;mc_millis interval,next;} mc_service_notify;
/* 无NOTIFY_SOCKET时完全禁用；不依赖libsystemd，不启动后台喂狗线程。 */
void mc_service_init(mc_service_notify *state,mc_millis now);
bool mc_service_send(mc_service_notify *state,const char *message);
void mc_service_tick(mc_service_notify *state,mc_millis now);
void mc_service_close(mc_service_notify *state);
#endif
