# C 1.9.1-dev

当前源码是C11开发主版本，详见仓库根README。原目录说明混有旧版本行为，公开快照不沿用旧说明。

核心模块：protocol.c负责CAN ID/CRC；config.c负责JSON与持久化；engine.c负责协议路由与485事务；main_linux.c负责SocketCAN/串口事件循环；can_rate.c负责速率确认回退；can_guard.c负责单帧在途、超时与退避；status_report.c负责运行状态。

config中的文件都是模板，不代表真实接线。默认静态链接；tools/build_target.sh接收绝对交叉编译器路径。C++差分参考不在本次公开范围，MONITOR_CPP_REFERENCE保持为空。

公开快照不包含授权测试示例及历史测试目录，完整测试保留在本地开发工程。
