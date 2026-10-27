# LoongArch CAN ↔ RS485 Monitor

面向Linux/龙芯平台的C11双CAN至8路RS485网关，当前开发版本1.9.1-dev。

## 功能与边界

- 按箱地址、通道、从站路由：每板8路，支持同箱三板分配1～8、9～16、17～24。
- 01控制：Modbus06写1寄存器、10写2寄存器；有效从站应答后确认，默认4秒重复控制去重。
- 03查询：转发Modbus03请求，任意设备支持的16位起始地址，每次1～60个连续寄存器，分帧返回。
- 04心跳：每通道广播、帧间至少1ms。
- 07参数：支持通道、箱地址、周期及CAN速率设置；切速率需新速率确认，否则回退。
- 双CAN业务去重、各路发送完成等待、独立恢复、退避与运行状态。

当前客户网关模式02不启用；真实DO/DI/AI未接入；08不纳入业务；05/06 CAN类型预留；0F未实现。不是任意串口字节透传。远程07全局参数可能作用于同箱多板，详见docs。

这是开发版本，不代表完成整机现场验收。已有历史单路测试与正常重启记录，双路冗余、多板、真实从站和断电长稳仍需在最终配置上验证。

## Ubuntu 22.04 / WSL2构建

```sh
sudo apt-get update
sudo apt-get install build-essential cmake python3
cmake -S monitor-c -B build/native -DCMAKE_BUILD_TYPE=Debug -DMONITOR_BUILD_TESTS=ON
cmake --build build/native -j2
ctest --test-dir build/native --output-on-failure
```

原生程序用于主机软件检查，不能拿x86产物部署龙芯板。可选vcan测试在无环境时跳过。

## 龙芯静态交叉编译

```sh
sh monitor-c/tools/build_target.sh /absolute/path/to/loongarch64-linux-gnu-gcc
file monitor-c/build/board99/zxjb14a_monitor_c
readelf -l monitor-c/build/board99/zxjb14a_monitor_c
```

原目标环境使用GCC8.3-rc1.0、glibc2.28工具链，LoongArch64静态链接；不把工具链/sysroot放入此仓库。目标Linux4.19使用前需验证ABI兼容；不是任何GCC13工具链都能直接代替。

## 不含源码的部署包

release中三个tar.gz分别为通道1～8、9～16、17～24，箱地址默认十进制11，双CAN250K、485为9600/8N1、07启用。各包README包括安装步骤。每块板选一个包；其他采集箱改device_address，不改物理口号或串口路径。

板卡须已有硬件驱动及zxjb14a-hardware-prepare服务，本仓库不包含内核驱动。

```sh
cd /opt
tar -xzf c-1.9.1-dev-9-16.tar.gz
cd c-1.9.1-dev-9-16
sha256sum -c SHA256SUMS
vi monitor.site.json
sh install-outside.sh
systemctl enable --now zxjb14a-monitor-c
sync
```

runtime参数可能覆盖主JSON，换箱/分组前应停服务并备份核对。上传后核对校验值，关机使用poweroff再断电；sync不能替代存储健康检查。

## 公开内容

只包含当前C源码、测试、构建工具、配置模板、通用说明与通道部署包。不含客户原始协议文件、现场抓包/登录记录、账号密码、SSH密钥、工具链、旧Git历史或C++冻结参考。此项目未指定开源许可证，公开访问不等同于授予任意再分发许可。
