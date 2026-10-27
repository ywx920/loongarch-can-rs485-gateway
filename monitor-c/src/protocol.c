#include "monitor.h"

/* ID只含29位协议内容；CAN_EFF_FLAG由Linux收发层处理。 */
int mc_encode_id(mc_id id,uint32_t *value) {
    if(!value||id.type>31||id.source>31||id.destination>31||id.channel>63||id.slave>15||id.sequence>15) return -1;
    *value=(id.type<<24)|(id.source<<19)|(id.destination<<14)|(id.channel<<8)|(id.slave<<4)|id.sequence;
    return 0;
}
/* [28:24]类型，[23:19]源，[18:14]目的，[13:8]通道，
 * [7:4]从站，[3:0]序号。CAN_EFF_FLAG不属于此布局。 */
mc_id mc_decode_id(uint32_t v) {
    mc_id id={(v>>24)&31,(v>>19)&31,(v>>14)&31,(v>>8)&63,(v>>4)&15,v&15};
    return id;
}
uint16_t mc_read_be(const uint8_t *b) {return (uint16_t)(((unsigned)b[0]<<8)|b[1]);}
void mc_write_be(uint8_t *b,unsigned value) {b[0]=(uint8_t)(value>>8);b[1]=(uint8_t)value;}
/* 标准Modbus RTU CRC16：初值FFFF，多项式A001，逐字节低位先算。
 * 寄存器数据是大端，附加CRC却低字节先发，两者必须分开处理。 */
uint16_t mc_crc16(const uint8_t *data,size_t n) {
    uint16_t crc=0xffff;
    for(size_t i=0;i<n;++i) {
        crc^=data[i];
        for(unsigned k=0;k<8;++k) crc=(uint16_t)((crc>>1)^((crc&1)?0xa001:0));
    }
    return crc;
}
bool mc_valid_crc(const uint8_t *data,size_t n) {
    return n>=4&&mc_crc16(data,n-2)==(uint16_t)(data[n-2]|((unsigned)data[n-1]<<8));
}
/* 调用者保证缓冲区余量至少2字节。RTU CRC低字节在前，不能套用大端规则。 */
size_t mc_append_crc(uint8_t *data,size_t n) {
    uint16_t crc=mc_crc16(data,n);data[n]=(uint8_t)crc;data[n+1]=(uint8_t)(crc>>8);return n+2;
}
