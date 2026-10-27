#define _POSIX_C_SOURCE 200809L
#include "monitor.h"
#include <ctype.h>
#include <errno.h>
#include <fcntl.h>
#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>

/* 受限JSON读取器：标准字符串转义/Unicode、数组、对象、布尔、数值。
 * token只引用输入文本，不复制子树。最大256KiB、深度64，过限明确失败。
 * 该模块只读配置，不执行文件内的任何命令。 */
enum {J_OBJECT,J_ARRAY,J_STRING,J_NUMBER,J_TRUE,J_FALSE,J_NULL};
typedef struct {int type,start,end,next;} token;
typedef struct {char *text;size_t size,pos;token *tokens;int used;char error[160];} json;
#define JSON_LIMIT (256u*1024u)
#define TOKEN_LIMIT 65536
static int fail(char *err,size_t cap,const char *message) {if(cap) snprintf(err,cap,"%s",message);return -1;}
static int jfail(json *j,const char *message) {if(!j->error[0]) snprintf(j->error,sizeof(j->error),"JSON byte %zu: %s",j->pos,message);return -1;}
static void space(json *j) {while(j->pos<j->size&&(j->text[j->pos]==' '||j->text[j->pos]=='\n'||j->text[j->pos]=='\r'||j->text[j->pos]=='\t')) ++j->pos;}
static int hex4(const char *s) {
    unsigned value=0;
    for(int k=0;k<4;++k) {unsigned char c=(unsigned char)s[k];unsigned v;
        if(c>='0'&&c<='9') v=c-'0';else if(c>='a'&&c<='f') v=c-'a'+10;else if(c>='A'&&c<='F') v=c-'A'+10;else return -1;
        value=value*16+v;
    }return (int)value;
}
/* 解码用于路径/键名的字符串，拒绝嵌入NUL和容量不足，不静默截断。 */
static int string_value(const json *j,int index,char *out,size_t cap) {
    const token *t=&j->tokens[index];size_t n=0;
    if(t->type!=J_STRING) return -1;
    for(int p=t->start+1;p<t->end-1;++p) {
        unsigned c=(unsigned char)j->text[p];
        if(c=='\\') {
            c=(unsigned char)j->text[++p];
            if(c=='u') {
                int v=hex4(j->text+p+1);if(v<0) return -1;p+=4;c=(unsigned)v;
                if(c>=0xd800&&c<=0xdbff) {
                    if(p+6>=t->end||j->text[p+1]!='\\'||j->text[p+2]!='u') return -1;
                    int low=hex4(j->text+p+3);if(low<0xdc00||low>0xdfff) return -1;
                    c=0x10000+((c-0xd800)<<10)+(unsigned)low-0xdc00;p+=6;
                } else if(c>=0xdc00&&c<=0xdfff) return -1;
                if(!c) return -1;
                unsigned char b[4];size_t bytes;
                if(c<0x80) {b[0]=(unsigned char)c;bytes=1;}
                else if(c<0x800) {b[0]=(unsigned char)(0xc0|(c>>6));b[1]=(unsigned char)(0x80|(c&63));bytes=2;}
                else if(c<0x10000) {b[0]=(unsigned char)(0xe0|(c>>12));b[1]=(unsigned char)(0x80|((c>>6)&63));b[2]=(unsigned char)(0x80|(c&63));bytes=3;}
                else {b[0]=(unsigned char)(0xf0|(c>>18));b[1]=(unsigned char)(0x80|((c>>12)&63));b[2]=(unsigned char)(0x80|((c>>6)&63));b[3]=(unsigned char)(0x80|(c&63));bytes=4;}
                if(n+bytes>=cap) return -1;
                memcpy(out+n,b,bytes);n+=bytes;continue;
            }
            if(c=='b') c='\b';else if(c=='f') c='\f';else if(c=='n') c='\n';else if(c=='r') c='\r';else if(c=='t') c='\t';
        }
        if(n+1>=cap||!c) return -1;
        out[n++]=(char)c;
    }
    if(!cap) return -1;
    out[n]=0;return 0;
}
static int parse_value(json *j,unsigned depth) {
    space(j);if(depth>64||j->used>=TOKEN_LIMIT||j->pos>=j->size) return jfail(j,"depth/token limit or missing value");
    int index=j->used++;token *t=&j->tokens[index];t->start=(int)j->pos;
    unsigned char c=(unsigned char)j->text[j->pos++];
    if(c=='{'||c=='[') {
        t->type=c=='{'?J_OBJECT:J_ARRAY;char closing=c=='{'?'}':']';space(j);
        if(j->pos<j->size&&j->text[j->pos]==closing) ++j->pos;
        else for(;;) {
            if(c=='{') {
                space(j);if(j->pos>=j->size||j->text[j->pos]!='"') return jfail(j,"object key must be a string");
                if(parse_value(j,depth+1)<0) return -1;
                space(j);if(j->pos>=j->size||j->text[j->pos++]!=':') return jfail(j,"missing colon");
            }
            if(parse_value(j,depth+1)<0) return -1;
            space(j);if(j->pos>=j->size) return jfail(j,"unclosed container");
            char sep=j->text[j->pos++];if(sep==closing) break;if(sep!=',') return jfail(j,"missing comma");
        }
    } else if(c=='"') {
        t->type=J_STRING;bool ended=false;
        while(j->pos<j->size) {
            c=(unsigned char)j->text[j->pos++];if(c=='"') {ended=true;break;}
            if(c<32) return jfail(j,"control byte in string");
            if(c=='\\') {
                if(j->pos>=j->size) return jfail(j,"bad escape");
                c=(unsigned char)j->text[j->pos++];
                if(c=='u') {if(j->pos+4>j->size||hex4(j->text+j->pos)<0) return jfail(j,"bad Unicode escape");j->pos+=4;}
                else if(!strchr("\"\\/bfnrt",c)) return jfail(j,"unknown escape");
            }
        }
        if(!ended) return jfail(j,"unclosed string");
    } else if(c=='t'||c=='f'||c=='n') {
        const char *word=c=='t'?"true":(c=='f'?"false":"null");size_t length=strlen(word);
        if((size_t)t->start+length>j->size||memcmp(j->text+t->start,word,length)) return jfail(j,"invalid literal");
        j->pos=(size_t)t->start+length;t->type=c=='t'?J_TRUE:(c=='f'?J_FALSE:J_NULL);
    } else {
        /* 先验证JSON数值语法，再由字段读取器做有限数/整数/取值范围检查。 */
        j->pos=(size_t)t->start;if(j->text[j->pos]=='-') ++j->pos;
        if(j->pos>=j->size) return jfail(j,"invalid number");
        if(j->text[j->pos]=='0') ++j->pos;
        else {if(j->text[j->pos]<'1'||j->text[j->pos]>'9') return jfail(j,"invalid number");while(j->pos<j->size&&isdigit((unsigned char)j->text[j->pos])) ++j->pos;}
        if(j->pos<j->size&&j->text[j->pos]=='.') {++j->pos;size_t first=j->pos;while(j->pos<j->size&&isdigit((unsigned char)j->text[j->pos])) ++j->pos;if(first==j->pos) return jfail(j,"missing fraction");}
        if(j->pos<j->size&&(j->text[j->pos]=='e'||j->text[j->pos]=='E')) {++j->pos;if(j->pos<j->size&&(j->text[j->pos]=='+'||j->text[j->pos]=='-')) ++j->pos;size_t first=j->pos;while(j->pos<j->size&&isdigit((unsigned char)j->text[j->pos])) ++j->pos;if(first==j->pos) return jfail(j,"missing exponent");}
        t->type=J_NUMBER;
    }
    t->end=(int)j->pos;t->next=j->used;return index;
}
static void json_free(json *j) {free(j->text);free(j->tokens);memset(j,0,sizeof(*j));}
static int json_load(const char *path,json *j,char *err,size_t cap) {
    memset(j,0,sizeof(*j));FILE *f=fopen(path,"rb");if(!f) {if(cap) snprintf(err,cap,"open %s: %s",path,strerror(errno));return -1;}
    j->text=malloc(JSON_LIMIT+1);j->tokens=calloc(TOKEN_LIMIT,sizeof(token));
    if(!j->text||!j->tokens) {fclose(f);json_free(j);return fail(err,cap,"allocate JSON parser failed");}
    j->size=fread(j->text,1,JSON_LIMIT,f);int extra=fgetc(f);bool io=ferror(f)!=0;fclose(f);j->text[j->size]=0;
    if(io||extra!=EOF) {json_free(j);return fail(err,cap,"JSON read failed or exceeds 256KiB");}
    if(parse_value(j,0)<0) {fail(err,cap,j->error);json_free(j);return -1;}
    space(j);if(j->pos!=j->size||j->tokens[0].type!=J_OBJECT) {json_free(j);return fail(err,cap,"JSON root must be one object");}
    return 0;
}
/* next直接跳过一个子树，查找仅限当前对象，避免误取子设备里的同名字段。 */
static int field(json *j,int object,const char *name) {
    if(object<0||j->tokens[object].type!=J_OBJECT) {jfail(j,"expected object");return -1;}
    for(int k=object+1;k<j->tokens[object].next;) {
        char key[128];int value=k+1;
        if(string_value(j,k,key,sizeof(key))) {jfail(j,"invalid/too long object key");return -1;}
        if(!strcmp(key,name)) return value;
        k=j->tokens[value].next;
    }return -1;
}
static unsigned numeric(json *j,int index,unsigned def) {
    if(index<0) return def;
    token *t=&j->tokens[index];char *end;double n=strtod(j->text+t->start,&end);
    if(t->type!=J_NUMBER||end!=j->text+t->end||!isfinite(n)||n<0||n>65535||floor(n)!=n) {jfail(j,"expected integer 0..65535");return def;}
    return (unsigned)n;
}
static unsigned num(json *j,int object,const char *name,unsigned def) {return numeric(j,field(j,object,name),def);}
static bool boolean(json *j,int object,const char *name,bool def) {
    int i=field(j,object,name);if(i<0) return def;
    if(j->tokens[i].type==J_TRUE) return true;
    if(j->tokens[i].type==J_FALSE) return false;
    jfail(j,"expected boolean");return def;
}
static void text(json *j,int object,const char *name,char *out,size_t cap) {
    int i=field(j,object,name);if(i>=0&&string_value(j,i,out,cap)) jfail(j,"expected string or string too long");
}
static int array(json *j,int object,const char *name,bool required) {
    int i=field(j,object,name);if(i<0) {if(required) jfail(j,"missing required array");return -1;}
    if(j->tokens[i].type!=J_ARRAY) {jfail(j,"expected array");return -1;}return i;
}
static size_t unsigned_list(json *j,int index,unsigned *out,size_t max) {
    size_t n=0;if(index<0) return 0;
    for(int k=index+1;k<j->tokens[index].next;k=j->tokens[k].next) {
        if(n>=max) {jfail(j,"array capacity exceeded");break;}out[n++]=numeric(j,k,0);
    }return n;
}
static size_t ranges(json *j,int index,mc_range *out) {
    size_t n=0;if(index<0) return 0;
    for(int k=index+1;k<j->tokens[index].next;k=j->tokens[k].next) {
        if(n>=MC_RANGES) {jfail(j,"at most 32 ranges allowed");break;}
        out[n].start=num(j,k,"start",0);out[n].count=num(j,k,"count",0);++n;
    }return n;
}
void mc_device_defaults(mc_device *d) {
    memset(d,0,sizeof(*d));d->slave=1;d->function=3;d->count=1;d->poll_ms=1900;d->timeout_ms=500;d->retries=1;d->write_function=6;
}
void mc_config_defaults(mc_config *c) {
    memset(c,0,sizeof(*c));c->address=11;c->heartbeat_ms=2000;c->report_ms=2000;c->frame_gap_ms=1;
    c->dedup_ms=3500;c->stale_ms=6000;c->queue_limit=32;c->baud=250000;c->can_tx_confirm_ms=250;c->can_retry_max_ms=30000;
    /* 默认不启用任何IO后端和授权，示例配置不会被误当成真实板卡。 */
    c->heartbeat_enabled=true;c->io_backend=MC_IO_NONE;c->authorization_lease_ms=30000;
}
/* 结构性硬约束：容量、地址位宽、唯一物理口/通道/路径、合法功能/范围。
 * 此函数决定配置是否可被引擎接受；site_check额外检查现场准备和预算，
 * 不把无法从JSON证明的接线、寄存器语义或客户授权当成已确认事实。 */
int mc_config_validate(const mc_config *c,char *err,size_t cap) {
#define REQUIRE(condition,message) do {if(!(condition)) return fail(err,cap,message);} while(0)
    REQUIRE(c,"null configuration");
    REQUIRE(c->address>=1&&c->address<=31,"device address must be 1..31");
    REQUIRE(c->can_count>=1&&c->can_count<=2,"one or two CAN interfaces required");
    for(size_t i=0;i<c->can_count;++i) REQUIRE(c->can_interfaces[i][0]&&memchr(c->can_interfaces[i],0,16),"invalid CAN interface");
    REQUIRE(c->can_count<2||strcmp(c->can_interfaces[0],c->can_interfaces[1]),"duplicate CAN interface");
    REQUIRE(!c->can_rate_enabled||c->can_rate_state_file[0],"CAN rate state file required");
    REQUIRE(c->can_tx_confirm_ms>=10&&c->can_tx_confirm_ms<=5000,"CAN confirmation timeout must be 10..5000ms");
    REQUIRE(c->can_retry_max_ms>=1000&&c->can_retry_max_ms<=300000,"CAN retry cap must be 1000..300000ms");
    REQUIRE(c->baud==250000,"initial CAN bitrate must be 250000");
    REQUIRE(c->heartbeat_ms>=100&&c->report_ms>=100&&c->frame_gap_ms>=1,"invalid reporting timers");
    REQUIRE(c->stale_ms>=c->report_ms&&c->queue_limit>=1&&c->queue_limit<=MC_QUEUE,"invalid stale/queue limit");
    REQUIRE(c->dedup_ms>=3000,"dedup window must cover six controls");
    REQUIRE(c->target_count>=1&&c->target_count<=4,"one to four report destinations required");
    REQUIRE(c->control_source_count<=MC_SOURCES&&c->query_source_count<=MC_SOURCES,"source list overflow");
    for(size_t i=0;i<c->target_count;++i) REQUIRE(c->targets[i]<=31,"invalid target");
    for(size_t i=0;i<c->control_source_count;++i) REQUIRE(c->control_sources[i]>=1&&c->control_sources[i]<=31,"invalid control source");
    for(size_t i=0;i<c->query_source_count;++i) REQUIRE(c->query_sources[i]>=1&&c->query_sources[i]<=31,"invalid query source");
    REQUIRE(c->port_count>=1&&c->port_count<=MC_PORTS,"one to eight ports required");
    for(size_t i=0;i<c->port_count;++i) {
        const mc_port *p=&c->ports[i];
        REQUIRE(p->physical>=1&&p->physical<=8&&p->channel>=1&&p->channel<=63,"invalid physical/channel");
        REQUIRE(p->path[0]&&memchr(p->path,0,sizeof(p->path)),"invalid serial path");
        REQUIRE(p->parity=='N'||p->parity=='E'||p->parity=='O',"parity must be N/E/O");
        REQUIRE(p->stopbits==1||p->stopbits==2,"stopbits must be 1/2");
        REQUIRE(p->baud==1200||p->baud==2400||p->baud==4800||p->baud==9600||p->baud==19200||p->baud==38400||p->baud==57600,"unsupported serial baud");
        for(size_t k=0;k<i;++k) REQUIRE(p->physical!=c->ports[k].physical&&p->channel!=c->ports[k].channel&&strcmp(p->path,c->ports[k].path),"duplicate physical/channel/path");
        REQUIRE(p->device_count<=MC_DEVICES,"at most 15 devices per port");
        for(size_t k=0;k<p->device_count;++k) {
            const mc_device *d=&p->devices[k];
            REQUIRE(d->slave>=1&&d->slave<=15,"slave must be 1..15");
            for(size_t n=0;n<k;++n) REQUIRE(d->slave!=p->devices[n].slave,"duplicate slave");
            REQUIRE(d->function==3||d->function==4,"read function must be 03/04");
            REQUIRE(d->count>=1&&d->count<=60&&d->start<=65535&&d->count<=65536-d->start,"invalid read range");
            REQUIRE(d->poll_ms>=100&&d->timeout_ms>=10&&d->retries<=3,"invalid device timers");
            REQUIRE(d->write_function==6||d->write_function==16,"write function must be 06/16");
            REQUIRE(d->writable_count<=MC_RANGES&&d->readable_count<=MC_RANGES&&d->allowed_write_count<=2,"device whitelist capacity exceeded");
            for(size_t n=0;n<d->allowed_write_count;++n) REQUIRE(d->allowed_write_functions[n]==6||d->allowed_write_functions[n]==16,"invalid allowed write function");
            for(size_t n=0;n<d->writable_count+d->readable_count;++n) {
                const mc_range *r=n<d->writable_count?&d->writable[n]:&d->readable[n-d->writable_count];
                REQUIRE(r->count&&r->start<=65535&&r->count<=65536-r->start,"invalid whitelist range");
            }
            REQUIRE(!d->control_enabled||d->writable_count,"enabled control needs writable whitelist");
            REQUIRE(!d->query_enabled||d->readable_count,"enabled query needs readable whitelist");
        }
    }
    /* 1.4.0：通道类型与授权属于本地扩展，不改变DW线上载荷。
     * 路由必须显式声明模拟后端；通道号不能与RS485口重复，否则无法唯一确定设备。 */
    REQUIRE(c->io_backend==MC_IO_NONE||c->io_backend==MC_IO_SIMULATED,"io_backend must be none/simulated");
    REQUIRE(c->route_count<=MC_ROUTES,"too many io routes");
    if(c->route_count) REQUIRE(c->io_backend==MC_IO_SIMULATED,"io routes require simulated backend");
    for(size_t i=0;i<c->route_count;++i) {
        const mc_route *r=&c->routes[i];
        REQUIRE(r->channel>=1&&r->channel<=63,"route channel must be 1..63");
        REQUIRE(r->kind==MC_ROUTE_DO||r->kind==MC_ROUTE_DI||r->kind==MC_ROUTE_AI,"route kind must be do/di/ai");
        REQUIRE(r->slave>=1&&r->slave<=15,"route slave must be 1..15");
        REQUIRE(r->reg_count==(r->kind==MC_ROUTE_AI?16u:2u),"route reg_count must be 2(DO/DI) or 16(AI)");
        REQUIRE(r->reg_start<=65535&&r->reg_count<=65536-r->reg_start,"invalid route register window");
        bool physical_ok=false;
        for(size_t k=0;k<c->port_count;++k) {
            REQUIRE(r->channel!=c->ports[k].channel,"route channel duplicates RS485 channel");
            if(c->ports[k].physical==r->physical) physical_ok=true;
        }
        REQUIRE(physical_ok,"route physical does not match any configured port");
        for(size_t k=0;k<i;++k) REQUIRE(r->channel!=c->routes[k].channel,"duplicate route channel");
    }
    if(c->authorization_sha256[0]) {
        REQUIRE(strlen(c->authorization_sha256)==64,"authorization_sha256 must be 64 hex characters");
        for(size_t i=0;i<64;++i) {
            char ch=c->authorization_sha256[i];
            REQUIRE((ch>='0'&&ch<='9')||(ch>='a'&&ch<='f'),"authorization_sha256 must be lowercase hex");
        }
    }
    REQUIRE(!c->customer_gateway||(!c->authorization_enabled&&!c->primary_only&&c->frame_gap_ms==1&&c->port_count==8),"customer gateway requires 8 ports, redundant CAN, 1ms gap and no authorization");
    if(c->authorization_enabled) {
        REQUIRE(c->authorization_lease_ms>=1000&&c->authorization_lease_ms<=86400000,"invalid authorization lease");
    }
    return 0;
#undef REQUIRE
}
/* 一次读入有界JSON并解码，失败保持明确错误，不截断字段或猜默认协议。
 * 示例JSON不等于现场参数；mapping_confirmed=false允许离线检查但禁止启动。 */
int mc_config_load(const char *path,mc_config *c,char *err,size_t cap) {
    json j;if(json_load(path,&j,err,cap)) return -1;mc_config_defaults(c);
    char protocol[64]="",reports[16]="both";text(&j,0,"protocol",protocol,sizeof(protocol));
    if(strcmp(protocol,MC_PROTOCOL)) jfail(&j,"wrong protocol mode");
    c->address=num(&j,0,"device_address",11);c->baud=num(&j,0,"can_bitrate_kbit",250)*1000;
    c->site_mapping_confirmed=boolean(&j,0,"site_mapping_confirmed",false);c->parameters_enabled=boolean(&j,0,"parameters_enabled",false);
    c->can_tx_confirm_ms=num(&j,0,"can_tx_confirm_ms",250);
    c->can_retry_max_ms=num(&j,0,"can_retry_max_ms",30000);
    c->can_rate_enabled=boolean(&j,0,"can_rate_enabled",false);
    c->can_rate_both=boolean(&j,0,"can_rate_both",false);
    text(&j,0,"can_rate_state_file",c->can_rate_state_file,sizeof(c->can_rate_state_file));
    c->allow_broadcast_control=boolean(&j,0,"allow_broadcast_control",false);
    char backend[16]="none";text(&j,0,"io_backend",backend,sizeof(backend));
    if(!strcmp(backend,"none")) c->io_backend=MC_IO_NONE;
    else if(!strcmp(backend,"simulated")) c->io_backend=MC_IO_SIMULATED;
    else jfail(&j,"io_backend must be none/simulated");
    c->customer_gateway=boolean(&j,0,"customer_gateway",false);
    c->heartbeat_enabled=boolean(&j,0,"heartbeat_enabled",true);
    c->authorization_enabled=boolean(&j,0,"authorization_enabled",false);
    c->authorization_allow_force=boolean(&j,0,"authorization_allow_force",false);
    c->authorization_lease_ms=num(&j,0,"authorization_lease_ms",30000);
    text(&j,0,"authorization_sha256",c->authorization_sha256,sizeof(c->authorization_sha256));
    c->heartbeat_ms=num(&j,0,"heartbeat_ms",2000);c->report_ms=num(&j,0,"report_ms",2000);c->frame_gap_ms=num(&j,0,"frame_gap_ms",1);
    c->dedup_ms=num(&j,0,"dedup_ms",3500);c->stale_ms=num(&j,0,"stale_ms",6000);c->queue_limit=num(&j,0,"queue_limit",32);
    text(&j,0,"report_buses",reports,sizeof(reports));text(&j,0,"runtime_settings",c->runtime_settings,sizeof(c->runtime_settings));
    if(strcmp(reports,"both")&&strcmp(reports,"primary")) jfail(&j,"invalid report_buses");
    c->primary_only=!strcmp(reports,"primary");
    int a=array(&j,0,"can_interfaces",true);
    if(a>=0) for(int k=a+1;k<j.tokens[a].next;k=j.tokens[k].next) {
        if(c->can_count>=2) {jfail(&j,"at most two CAN interfaces");break;}
        if(string_value(&j,k,c->can_interfaces[c->can_count++],16)) jfail(&j,"invalid CAN name");
    }
    c->target_count=unsigned_list(&j,array(&j,0,"report_destinations",true),c->targets,4);
    c->control_source_count=unsigned_list(&j,array(&j,0,"control_sources",true),c->control_sources,MC_SOURCES);
    a=array(&j,0,"query_sources",false);
    if(a>=0) c->query_source_count=unsigned_list(&j,a,c->query_sources,MC_SOURCES);
    else {c->query_source_count=c->control_source_count;memcpy(c->query_sources,c->control_sources,c->query_source_count*sizeof(unsigned));}
    a=array(&j,0,"ports",true);
    if(a>=0) for(int k=a+1;k<j.tokens[a].next;k=j.tokens[k].next) {
        if(c->port_count>=8) {jfail(&j,"at most eight ports");break;}
        mc_port *p=&c->ports[c->port_count++];p->physical=num(&j,k,"physical",0);p->channel=num(&j,k,"channel",0);
        p->enabled=boolean(&j,k,"enabled",true);p->baud=num(&j,k,"baudrate",9600);p->stopbits=num(&j,k,"stopbits",1);
        char parity[4]="N";text(&j,k,"parity",parity,sizeof(parity));if(strlen(parity)!=1) jfail(&j,"parity must be one character");p->parity=parity[0];
        text(&j,k,"path",p->path,sizeof(p->path));int devices=array(&j,k,"devices",true);
        if(devices>=0) for(int n=devices+1;n<j.tokens[devices].next;n=j.tokens[n].next) {
            if(p->device_count>=15) {jfail(&j,"at most 15 devices");break;}
            mc_device *d=&p->devices[p->device_count++];mc_device_defaults(d);
            d->slave=num(&j,n,"slave",1);d->function=num(&j,n,"read_function",3);d->start=num(&j,n,"read_start",0);d->count=num(&j,n,"read_count",1);
            d->poll_ms=num(&j,n,"poll_ms",1900);d->timeout_ms=num(&j,n,"timeout_ms",500);d->retries=num(&j,n,"retries",1);d->write_function=num(&j,n,"write_function",6);
            d->control_enabled=boolean(&j,n,"control_enabled",false);d->query_enabled=boolean(&j,n,"query_enabled",false);
            d->allowed_write_count=unsigned_list(&j,array(&j,n,"allowed_write_functions",false),d->allowed_write_functions,2);
            d->writable_count=ranges(&j,array(&j,n,"writable_ranges",false),d->writable);
            d->readable_count=ranges(&j,array(&j,n,"readable_ranges",false),d->readable);
        }
    }
    a=array(&j,0,"io_routes",false);
    if(a>=0) for(int k=a+1;k<j.tokens[a].next;k=j.tokens[k].next) {
        if(c->route_count>=MC_ROUTES) {jfail(&j,"at most 64 io routes");break;}
        mc_route *r=&c->routes[c->route_count++];
        r->channel=num(&j,k,"channel",0);r->physical=num(&j,k,"physical",0);
        r->slave=num(&j,k,"slave",1);r->reg_start=num(&j,k,"reg_start",0);r->reg_count=num(&j,k,"reg_count",0);
        char kind[8]="";text(&j,k,"kind",kind,sizeof(kind));
        if(!strcmp(kind,"do")) r->kind=MC_ROUTE_DO;
        else if(!strcmp(kind,"di")) r->kind=MC_ROUTE_DI;
        else if(!strcmp(kind,"ai")) r->kind=MC_ROUTE_AI;
        else jfail(&j,"route kind must be do/di/ai");
    }
    int result=j.error[0]?fail(err,cap,j.error):mc_config_validate(c,err,cap);json_free(&j);return result;
}
/* 运行参数是07允许修改的少数字段，不允许借运行文件覆盖整份设备配置。
 * 先加载到candidate再整体校验，避免半份坏文件改变当前配置。 */
int mc_runtime_load(mc_config *c,char *err,size_t cap) {
    if(!c->runtime_settings[0]) return 0;
    FILE *probe=fopen(c->runtime_settings,"rb");if(!probe) {if(errno==ENOENT) return 0;return fail(err,cap,"cannot read runtime settings");}fclose(probe);
    json j;if(json_load(c->runtime_settings,&j,err,cap)) return -1;mc_config candidate=*c;
    candidate.address=num(&j,0,"device_address",c->address);candidate.heartbeat_ms=num(&j,0,"heartbeat_ms",c->heartbeat_ms);candidate.report_ms=num(&j,0,"report_ms",c->report_ms);
    text(&j,0,"authorization_sha256",candidate.authorization_sha256,sizeof(candidate.authorization_sha256));
    int a=array(&j,0,"channels",false);
    if(a>=0) for(int k=a+1;k<j.tokens[a].next;k=j.tokens[k].next) {
        unsigned physical=num(&j,k,"physical",0);bool found=false;
        for(size_t i=0;i<candidate.port_count;++i) if(candidate.ports[i].physical==physical) {candidate.ports[i].channel=num(&j,k,"channel",candidate.ports[i].channel);found=true;}
        if(!found) jfail(&j,"runtime settings refer to unknown physical port");
    }
    int result=j.error[0]?fail(err,cap,j.error):mc_config_validate(&candidate,err,cap);json_free(&j);
    if(!result) *c=candidate;
    return result;
}
/* 参数保存与普通状态快照不同：完整写临时文件并fsync，再rename提交。
 * 重命名后内存必须跟已提交文件一致；不能因目录fsync告警假装未提交。 */
int mc_runtime_save(const mc_config *c,char *err,size_t cap) {
    if(!c->runtime_settings[0]) return fail(err,cap,"runtime_settings path is required");
    char data[2048];int length=snprintf(data,sizeof(data),"{\n\"device_address\":%u,\n\"heartbeat_ms\":%u,\n\"report_ms\":%u,\n\"authorization_sha256\":\"%s\",\n\"channels\":[",c->address,c->heartbeat_ms,c->report_ms,c->authorization_sha256);
    for(size_t i=0;i<c->port_count;++i) length+=snprintf(data+length,sizeof(data)-(size_t)length,"%s{\"physical\":%u,\"channel\":%u}",i?",":"",c->ports[i].physical,c->ports[i].channel);
    length+=snprintf(data+length,sizeof(data)-(size_t)length,"]\n}\n");
    char temp[MC_PATH+16],dir[MC_PATH];snprintf(temp,sizeof(temp),"%s.tmp.XXXXXX",c->runtime_settings);
    snprintf(dir,sizeof(dir),"%s",c->runtime_settings);char *slash=strrchr(dir,'/');if(!slash) strcpy(dir,".");else if(slash==dir) slash[1]=0;else *slash=0;
    int fd=mkstemp(temp);if(fd<0) return fail(err,cap,"create runtime settings temp file failed");
    size_t done=0;bool ok=true;
    while(done<(size_t)length) {ssize_t n=write(fd,data+done,(size_t)length-done);if(n<0&&errno==EINTR) continue;if(n<=0) {ok=false;break;}done+=(size_t)n;}
    if(fsync(fd)<0) ok=false;
    if(close(fd)<0) ok=false;
    int dfd=open(dir,O_RDONLY|O_DIRECTORY);
    if(dfd<0||!ok||rename(temp,c->runtime_settings)<0) {if(dfd>=0) close(dfd);unlink(temp);return fail(err,cap,"commit runtime settings failed");}
    /* rename已提交，不因目录fsync失败回滚内存，保持与磁盘当前文件一致。 */
    if(fsync(dfd)<0) fprintf(stderr,"WARNING: settings saved but directory fsync failed\n");
    close(dfd);return 0;
}
