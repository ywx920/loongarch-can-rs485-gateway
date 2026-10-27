/* Linux适配：不通过shell拼命令，所有接口名均来自已校验配置。
 * 每次切换清空对应软件发送队列，避免把旧速率数据在新速率补发。 */
typedef struct {const mc_config *config;int *fds;mc_can_tx *tx;mc_millis *retry;mc_rate *rate;mc_can_guard *guards;} rate_io;
static bool rate_ip(char *const argv[]){
    pid_t child=fork();if(child<0)return false;
    if(!child){execv("/sbin/ip",argv);_exit(127);}
    int result=0;mc_millis end=millis()+3000;
    for(;;){pid_t done=waitpid(child,&result,WNOHANG);if(done==child)return WIFEXITED(result)&&WEXITSTATUS(result)==0;
        if(done<0&&errno!=EINTR)return false;
        if(millis()>=end){kill(child,SIGKILL);waitpid(child,&result,0);return false;}
        struct timespec delay={0,1000000};nanosleep(&delay,NULL);
    }
}
static bool rate_apply(void *ctx,unsigned bus,unsigned rate){
    rate_io *io=ctx;const char *name=io->config->can_interfaces[bus];
    if(io->fds[bus]>=0){close(io->fds[bus]);io->fds[bus]=-1;}
    if(io->guards){io->guards[bus].pending=false;io->guards[bus].reset_needed=false;io->guards[bus].retry_at=0;}
    memset(&io->tx[bus],0,sizeof(io->tx[bus]));io->retry[bus]=millis();
    char speed_text[16];snprintf(speed_text,sizeof(speed_text),"%u",rate);
    char *down[]={"ip","link","set",(char*)name,"down",NULL};
    char *set[]={"ip","link","set",(char*)name,"type","can","bitrate",speed_text,"restart-ms","100",NULL};
    char *up[]={"ip","link","set",(char*)name,"up",NULL};
    bool ok=rate_ip(down)&&rate_ip(set)&&rate_ip(up);mc_can_health h;
    ok=ok&&!mc_can_inspect(name,&h)&&h.up&&h.bitrate_known&&h.bitrate==rate;
    fprintf(stderr,"EVENT reason=can_rate_apply bus=%u bitrate=%u success=%d\n",bus,rate,ok);return ok;
}
static bool rate_save(void *ctx,const unsigned rates[2]){
    rate_io *io=ctx;char temp[MC_PATH+16];snprintf(temp,sizeof(temp),"%s.tmp.XXXXXX",io->config->can_rate_state_file);
    int fd=mkstemp(temp);if(fd<0)return false;
    char data[80];int n=snprintf(data,sizeof(data),"%u %u\n",rates[0],rates[1]);
    bool ok=write(fd,data,(size_t)n)==n&&fsync(fd)==0;if(close(fd))ok=false;
    if(ok)ok=rename(temp,io->config->can_rate_state_file)==0;
    if(!ok)unlink(temp);
    else {char dir[MC_PATH];snprintf(dir,sizeof(dir),"%s",io->config->can_rate_state_file);char *slash=strrchr(dir,'/');if(slash){if(slash==dir)slash[1]=0;else *slash=0;}else strcpy(dir,".");int d=open(dir,O_RDONLY|O_DIRECTORY);if(d>=0){if(fsync(d))fprintf(stderr,"WARNING: can rate directory sync failed\n");close(d);}}
    fprintf(stderr,"EVENT reason=can_rate_commit success=%d rates=%u,%u\n",ok,rates[0],rates[1]);return ok;
}
static void rate_reply(void *ctx,const mc_frame *request,bool ok){
    rate_io *io=ctx;mc_frame reply=*request;mc_id id=mc_decode_id(request->id);unsigned source=id.source;id.source=id.destination;id.destination=source;
    mc_encode_id(id,&reply.id);reply.data[3]=ok?1:0;reply.expires_at=0;
    if(!mc_can_enqueue(&io->tx[reply.bus],&reply,1,millis(),1000))fprintf(stderr,"EVENT reason=can_rate_reply_queue_full\n");
    fprintf(stderr,"EVENT reason=can_rate_reply bus=%d result=%u phase=%u\n",request->bus,ok?1u:0u,io->rate->phase);
}
