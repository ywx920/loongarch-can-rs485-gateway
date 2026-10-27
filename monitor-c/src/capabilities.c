#include "monitor.h"
#include <stdio.h>
/* 本地能力清单：不代表板端或客户协议已验收。 */
void mc_capabilities_print(FILE *out) {
    fputs("{\"version\":\"" MC_VERSION "\",",out);
    fputs("\"schema\":1,\"protocol\":\"aux_monitor_dw_20261014\",\"language\":\"C11\",\"linkage\":\"static\",\"commands\":[{\"type\":\"01\",\"status\":\"implemented\",\"scope\":\"RS485 06/10; DO simulated only; ACK after RTU echo\"},{\"type\":\"02\",\"status\":\"implemented\",\"scope\":\"RS485; DI/AI simulated only; stale sample suppressed\"},{\"type\":\"03\",\"status\":\"implemented\",\"scope\":\"read-only RTU03 including CRC; 1..60 words\"},{\"type\":\"04\",\"status\":\"implemented\",\"scope\":\"local heartbeat; configurable owner; no remote aggregation\"},{\"type\":\"05/06\",\"status\":\"reserved\",\"scope\":\"no private reuse\"},{\"type\":\"07\",\"status\":\"partial\",\"implemented\":[2,3,4,5],\"pending\":[1],\"reason\":\"bitrate switching timing and rollback undefined\"},{\"type\":\"08\",\"status\":\"experimental\",\"requests\":[1,2,5,6,9],\"responses\":[3,4,7,8,10],\"default_enabled\":false,\"reason\":\"arbitration/scope/lease policy requires agreement\"},{\"type\":\"0F\",\"status\":\"pending_definition\",\"reason\":\"no payload specification\"}],\"physical_io_backend\":\"not implemented; hardware interface and register mapping required\",\"board_validation\":\"not performed\",\"query_write_passthrough\":false}",out);
    fputc('\n',out);
}
