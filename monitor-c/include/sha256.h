#ifndef MC_SHA256_H
#define MC_SHA256_H
#include <stddef.h>
#include <stdint.h>
/* 仅用于授权码摘要，不用于CAN载荷或配置签名。
 * 输出固定64字节小写十六进制+结尾NUL，调用者必须提供65字节缓冲。 */
void mc_sha256_hex(const uint8_t *data,size_t length,char out[65]);
#endif
