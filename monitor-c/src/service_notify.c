#define _GNU_SOURCE
#include "service_notify.h"
#include <errno.h>
#include <limits.h>
#include <stddef.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>

/* 只给继承的systemd通知套接字发非阻塞消息，不查询或更改服务。 */
static bool number(const char *s,unsigned long long *value) {
    if(!s||!*s) return false;
    for(const char *p=s;*p;++p) if(*p<'0'||*p>'9') return false;
    char *end;errno=0;*value=strtoull(s,&end,10);return !errno&&!*end;
}
void mc_service_init(mc_service_notify *s,mc_millis now) {
    memset(s,0,sizeof(*s));s->fd=-1;
    const char *path=getenv("NOTIFY_SOCKET");
    if(!path||!path[0]) return;
    size_t n=strlen(path);
    if((path[0]!='/'&&path[0]!='@')||n>=sizeof(s->address.sun_path)) return;
    s->address.sun_family=AF_UNIX;memcpy(s->address.sun_path,path,n+1);
    s->length=(socklen_t)(offsetof(struct sockaddr_un,sun_path)+n+(path[0]=='/'?1:0));
    if(path[0]=='@') s->address.sun_path[0]=0; /* Linux抽象命名空间 */
    s->fd=socket(AF_UNIX,SOCK_DGRAM|SOCK_NONBLOCK|SOCK_CLOEXEC,0);
    unsigned long long usec,pid;const char *owner=getenv("WATCHDOG_PID");
    if(owner&&(!number(owner,&pid)||pid!=(unsigned long long)getpid())) return;
    if(!number(getenv("WATCHDOG_USEC"),&usec)||usec<2000||usec>LLONG_MAX) return;
    s->interval=(mc_millis)(usec/2000);s->next=now+s->interval;
}
bool mc_service_send(mc_service_notify *s,const char *message) {
    if(s->fd<0) return false;
    size_t n=strlen(message);
    return sendto(s->fd,message,n,MSG_DONTWAIT|MSG_NOSIGNAL,(struct sockaddr *)&s->address,s->length)==(ssize_t)n;
}
void mc_service_tick(mc_service_notify *s,mc_millis now) {
    if(!s->interval||now<s->next) return;
    /* 只有主循环实际推进才喂狗；失败不忙等，不把硬件离线当进程卡死。 */
    (void)mc_service_send(s,"WATCHDOG=1");s->next=now+s->interval;
}
void mc_service_close(mc_service_notify *s) {if(s->fd>=0) close(s->fd);s->fd=-1;}
