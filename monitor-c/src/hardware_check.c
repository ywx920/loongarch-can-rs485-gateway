#define _GNU_SOURCE
#include "monitor_diagnostics.h"
#include <dirent.h>
#include <errno.h>
#include <fcntl.h>
#include <limits.h>
#include <linux/can/netlink.h>
#include <linux/netlink.h>
#include <linux/rtnetlink.h>
#include <net/if.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/ioctl.h>
#include <sys/socket.h>
#include <sys/stat.h>
#include <sys/time.h>
#include <unistd.h>

/* 仅stat已有设备和/proc文件描述符，不open串口、不修改termios或丢弃输入。
 * root可能绕过TIOCEXCL，所以不能只依赖独占ioctl防止误抢旧网关。
 * 扫描仍有竞态；旧服务必须先停，共享锁/TIOCEXCL继续保留。
 * 遇到无法检查的进程fd目录返回未知，不把权限不足当作“空闲”。 */
int mc_serial_owner(const char *path,char *err,size_t cap) {
    struct stat target;
    if(stat(path,&target)||!S_ISCHR(target.st_mode)) {snprintf(err,cap,"serial path missing or not a character device");return -1;}
    DIR *proc=opendir("/proc");if(!proc) {snprintf(err,cap,"cannot inspect /proc");return -1;}
    bool unknown=false;struct dirent *entry;
    while((entry=readdir(proc))) {
        char *end;long pid=strtol(entry->d_name,&end,10);
        if(!entry->d_name[0]||*end||pid<=0||pid>INT_MAX||pid==(long)getpid()) continue;
        char directory[80];snprintf(directory,sizeof(directory),"/proc/%ld/fd",pid);
        DIR *fds=opendir(directory);
        if(!fds) {if(errno!=ENOENT&&errno!=ESRCH) unknown=true;continue;}
        struct dirent *fd;
        while((fd=readdir(fds))) {
            long number=strtol(fd->d_name,&end,10);if(!fd->d_name[0]||*end||number<0) continue;
            char name[128];snprintf(name,sizeof(name),"%s/%ld",directory,number);struct stat candidate;
            if(!stat(name,&candidate)) {
                if(S_ISCHR(candidate.st_mode)&&candidate.st_rdev==target.st_rdev) {
                    closedir(fds);closedir(proc);snprintf(err,cap,"serial device is owned by PID %ld",pid);return (int)pid;
                }
            }else if(errno==EACCES||errno==EPERM) unknown=true;
        }
        closedir(fds);
    }
    closedir(proc);
    if(unknown) {snprintf(err,cap,"serial ownership unknown: insufficient /proc access");return -1;}
    return 0;
}

/* Netlink只读查询接口，不绑定CAN_RAW、不发送CAN数据、不改UP/波特率。
 * 返回值仅是本机控制器状态；无法被动证明另一端有ACK或H/L/电阻正确。
 * 100ms接收上限使缺失/异常内核回复不会无限阻塞预检。 */
int mc_can_inspect(const char *name,mc_can_health *health) {
    memset(health,0,sizeof(*health));unsigned index=if_nametoindex(name);
    if(!index) return -1;
    char path[128];snprintf(path,sizeof(path),"/sys/class/net/%s/type",name);
    FILE *type=fopen(path,"r");unsigned link_type=0;
    if(!type) return -1;
    int read=fscanf(type,"%u",&link_type);fclose(type);
    if(read!=1||link_type!=280) {errno=ENODEV;return -1;} /* ARPHRD_CAN */
    health->present=true;
    int fd=socket(AF_NETLINK,SOCK_RAW|SOCK_CLOEXEC,NETLINK_ROUTE);if(fd<0) return -1;
    struct timeval timeout={0,100000};int result=-1;
    if(setsockopt(fd,SOL_SOCKET,SO_RCVTIMEO,&timeout,sizeof(timeout))) goto done;
    struct {struct nlmsghdr h;struct ifinfomsg i;} request={0};
    request.h.nlmsg_len=NLMSG_LENGTH(sizeof(request.i));request.h.nlmsg_type=RTM_GETLINK;request.h.nlmsg_flags=NLM_F_REQUEST;request.h.nlmsg_seq=1;request.i.ifi_family=AF_UNSPEC;request.i.ifi_index=(int)index;
    struct sockaddr_nl kernel={0};kernel.nl_family=AF_NETLINK;
    if(sendto(fd,&request,request.h.nlmsg_len,0,(void *)&kernel,sizeof(kernel))<0) goto done;
    union {struct nlmsghdr align;unsigned char bytes[8192];} buffer;
    ssize_t n=recv(fd,buffer.bytes,sizeof(buffer.bytes),0);if(n<=0) goto done;int remaining=(int)n;
    for(struct nlmsghdr *h=(void *)buffer.bytes;NLMSG_OK(h,remaining);h=NLMSG_NEXT(h,remaining)) {
        if(h->nlmsg_type!=RTM_NEWLINK||h->nlmsg_len<NLMSG_LENGTH(sizeof(struct ifinfomsg))) continue;
        struct ifinfomsg *info=NLMSG_DATA(h);if(info->ifi_index!=(int)index) continue;
        health->up=(info->ifi_flags&IFF_UP)!=0;result=0;
        int an=IFLA_PAYLOAD(h);
        for(struct rtattr *a=IFLA_RTA(info);RTA_OK(a,an);a=RTA_NEXT(a,an)) {
            if((a->rta_type&NLA_TYPE_MASK)!=IFLA_LINKINFO) continue;
            int ln=RTA_PAYLOAD(a);
            for(struct rtattr *l=RTA_DATA(a);RTA_OK(l,ln);l=RTA_NEXT(l,ln)) {
                if((l->rta_type&NLA_TYPE_MASK)!=IFLA_INFO_DATA) continue;
                int cn=RTA_PAYLOAD(l);
                for(struct rtattr *c=RTA_DATA(l);RTA_OK(c,cn);c=RTA_NEXT(c,cn)) {
                    unsigned kind=c->rta_type&NLA_TYPE_MASK;
                    if(kind==IFLA_CAN_STATE&&RTA_PAYLOAD(c)>=sizeof(unsigned)) {memcpy(&health->state,RTA_DATA(c),sizeof(unsigned));health->state_known=true;}
                    if(kind==IFLA_CAN_RESTART_MS&&RTA_PAYLOAD(c)>=sizeof(unsigned)) memcpy(&health->restart_ms,RTA_DATA(c),sizeof(unsigned));
                    if(kind==IFLA_CAN_BITTIMING&&RTA_PAYLOAD(c)>=sizeof(struct can_bittiming)) {struct can_bittiming bt;memcpy(&bt,RTA_DATA(c),sizeof(bt));health->bitrate=bt.bitrate;health->bitrate_known=true;}
                }
            }
        }
    }
done:close(fd);return result;
}
int mc_hardware_check(const mc_config *c,FILE *out) {
    unsigned blockers=0;
    for(size_t i=0;i<c->can_count;++i) {
        mc_can_health h;int result=mc_can_inspect(c->can_interfaces[i],&h);
        fprintf(out,"HARDWARE_CAN interface=%s present=%d up=%d state_known=%d state=%u bitrate_known=%d bitrate=%u expected_bitrate=%u restart_ms=%u\n",c->can_interfaces[i],h.present,h.up,h.state_known,h.state,h.bitrate_known,h.bitrate,c->baud,h.restart_ms);
        if(result||!h.up||!h.state_known||h.state!=CAN_STATE_ERROR_ACTIVE||!h.bitrate_known||h.bitrate!=c->baud) ++blockers;
    }
    for(size_t i=0;i<c->port_count;++i) if(c->ports[i].enabled) {
        const mc_port *p=&c->ports[i];char message[160]="";int owner=mc_serial_owner(p->path,message,sizeof(message));
        fprintf(out,"HARDWARE_SERIAL physical=%u path=%s ownership=%s pid=%d message=%s\n",p->physical,p->path,owner>0?"busy":(owner<0?"unknown":"not_observed_busy"),owner>0?owner:0,message);
        if(owner) ++blockers;
    }
    if(c->parameters_enabled&&!c->runtime_settings[0]) {fputs("HARDWARE_RUNTIME runtime_settings_empty\n",out);++blockers;}
    else if(c->parameters_enabled) {
        char parent[MC_PATH];snprintf(parent,sizeof(parent),"%s",c->runtime_settings);char *slash=strrchr(parent,'/');
        if(slash) {if(slash==parent) slash[1]=0;else *slash=0;}else strcpy(parent,".");
        struct stat st;bool writable=!stat(parent,&st)&&S_ISDIR(st.st_mode)&&access(parent,W_OK|X_OK)==0;
        fprintf(out,"HARDWARE_RUNTIME parent=%s accessible=%d\n",parent,writable);if(!writable) ++blockers;
    }
    fputs("NOTE passive inspection only: no CAN ACK test, no RS485 probe; ownership is a /proc observation, not a global reservation\n",out);
    fprintf(out,"HARDWARE_RESULT blockers=%u\n",blockers);return blockers?2:0;
}
