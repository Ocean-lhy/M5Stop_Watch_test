# M5STACK IO EXPANDER Driver

本组件主要为M5STACK搭载IO扩展芯片的硬件提供驱动支持

| 墨水屏型号        | 支持程度 | 支持mcu | 框架支持    | 备注   |
| ------------ | ---- | ----- | ------- | ---- |
| PI4IOE5V6408 | √    | esp32 | ESP-IDF | 尚不完善 |
|              | ×    |       |         |      |

## 添加到项目

1. 通过乐鑫的组件服务添加软件包（暂不支持）

```powershell 
 idf.py add-dependency m5_io_expander == 1.0.1
```


1. 通过M5STACK内部代码仓库添加软件包

```yaml 
  m5_io_expander:
    version: '*'
    git: https://{account}:{password}@gitlab.m5stack.com/embedded-common-components/m5_io_expander.git

```
