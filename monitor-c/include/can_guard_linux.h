/* 每路接口恢复使用独立子进程，主循环不阻塞等待ip命令。
 * 一个坏接口或命令卡住，不能拖停另一条CAN和8路485。 */
typedef struct {pid_t child;unsigned phase;mc_millis deadline;bool killed;} guard_reset_state;
static bool guard_reset_spawn(guard_reset_state *s,const char *name,const char *action,mc_millis now){
    pid_t child=fork();if(child<0)return false;
    if(!child){char *args[]={"ip","link","set",(char*)name,(char*)action,NULL};execv("/sbin/ip",args);_exit(127);}
    s->child=child;s->deadline=now+3000;s->killed=false;return true;
}
/* 0进行中、1完成、-1失败；单路最多一个恢复子进程。 */
static int guard_reset_tick(guard_reset_state *s,const char *name,mc_millis now){
    if(!s->child){s->phase=0;return guard_reset_spawn(s,name,"down",now)?0:-1;}
    int status=0;pid_t result=waitpid(s->child,&status,WNOHANG);
    if(result==0){if(now>=s->deadline&&!s->killed){kill(s->child,SIGKILL);s->killed=true;}return 0;}
    if(result<0&&errno==EINTR)return 0;
    s->child=0;
    if(result<0||s->killed||!WIFEXITED(status)||WEXITSTATUS(status))return -1;
    if(!s->phase){s->phase=1;return guard_reset_spawn(s,name,"up",now)?0:-1;}
    s->phase=0;return 1;
}

/* 应用诊断每路每分钟汇总一次；恢复成功仍由独立事件立即报告。
 * 只限应用日志，不能冒充已经修改内核驱动printk。 */
typedef struct {mc_millis next;uint64_t suppressed;} guard_log_state;
static void guard_log(guard_log_state *log,mc_millis now,unsigned bus,const char *reason,int64_t value){
    if(now<log->next){++log->suppressed;return;}
    fprintf(stderr,"EVENT ms=%" PRId64 " reason=%s bus=%u value=%" PRId64 " suppressed=%" PRIu64 "\n",now,reason,bus,value,log->suppressed);
    log->next=now+60000;log->suppressed=0;
}
