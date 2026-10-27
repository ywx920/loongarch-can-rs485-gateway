#include "can_rate.h"
#include <string.h>
bool mc_rate_valid(unsigned n){return n==50000||n==100000||n==125000||n==250000||n==500000||n==1000000;}
void mc_rate_init(mc_rate *r,unsigned count,const unsigned rates[2],bool both){memset(r,0,sizeof(*r));r->count=count;r->both=both;memcpy(r->rates,rates,sizeof(r->rates));}
static void answer_all(mc_rate *r,bool ok,mc_rate_reply reply,void *ctx){
    for(unsigned b=0;b<r->count;++b)if(r->mask&(1u<<b)){mc_frame f=r->request;f.bus=(int)b;reply(ctx,&f,ok);}
}
/* apply失败也可能已将接口DOWN，回滚必须覆盖整个目标集合。 */
void mc_rate_cancel(mc_rate *r,mc_rate_apply apply,void *ctx){
    if(r->phase==2)for(unsigned b=0;b<r->count;++b)if(r->mask&(1u<<b)){
        if(!apply(ctx,b,r->old[b]))r->fault=true;
        else r->rates[b]=r->old[b];
    }
    r->phase=0;
}
bool mc_rate_receive(mc_rate *r,const mc_frame *f,unsigned address,mc_millis now,mc_rate_reply reply,void *ctx){
    mc_id id=mc_decode_id(f->id);
    if(id.type!=7||f->data[0]!=1)return false;
    if(f->length!=8||f->bus<0||(unsigned)f->bus>=r->count)return true;
    if(id.destination!=address)return true; /* 不响应其它箱或广播。 */
    unsigned target=mc_read_be(f->data+1)*1000u;
    if(id.sequence||!mc_rate_valid(target)||r->fault||(r->both&&r->count!=2)){reply(ctx,f,false);return true;}
    if(r->phase){
        if(f->id!=r->request.id||memcmp(f->data,r->request.data,8)||!(r->mask&(1u<<(unsigned)f->bus))){reply(ctx,f,false);return true;}
        if(r->phase==1)reply(ctx,f,true); /* 重发不延迟切换，也不延长超时。 */
        else if(now<r->expires)r->confirmed|=1u<<(unsigned)f->bus;
        return true;
    }
    r->mask=r->both?3u:1u<<(unsigned)f->bus;
    bool same=true;for(unsigned b=0;b<r->count;++b)if((r->mask&(1u<<b))&&r->rates[b]!=target)same=false;
    if(same){reply(ctx,f,true);return true;}
    r->request=*f;r->target=target;r->confirmed=0;memcpy(r->old,r->rates,sizeof(r->old));
    r->phase=1;r->due=now+1000;r->expires=now+3000;reply(ctx,f,true);
    return true;
}
void mc_rate_tick(mc_rate *r,mc_millis now,bool ready,mc_rate_apply apply,mc_rate_save save,mc_rate_reply reply,void *ctx){
    if(r->phase==1&&now>=r->due){
        if(!ready){if(now>=r->expires){answer_all(r,false,reply,ctx);r->phase=0;}return;}
        r->phase=2; /* 先进入可回滚状态，再尝试改硬件。 */
        for(unsigned b=0;b<r->count;++b)if(r->mask&(1u<<b)){
            if(!apply(ctx,b,r->target)){mc_rate_cancel(r,apply,ctx);answer_all(r,false,reply,ctx);return;}
            r->rates[b]=r->target;
        }
        r->expires=now+10000;return;
    }
    if(r->phase==2){
        if(now>=r->expires){mc_rate_cancel(r,apply,ctx);answer_all(r,false,reply,ctx);}
        else if(r->confirmed==r->mask){
            if(!save(ctx,r->rates))mc_rate_cancel(r,apply,ctx);else r->phase=0;
            answer_all(r,!r->phase&&!r->fault&&r->rates[r->request.bus]==r->target,reply,ctx);
        }
    }
}
