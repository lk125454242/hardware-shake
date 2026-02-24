# ESP32-C3-MINI-1

| 名称   | 序号        | 类型   | 功能 |
|--------|-------------|--------|------|
| GND    | 1, 2, 11, 14, 36-53 | P  | 接地 |
| 3V3    | 3           | P     | 供电 |
| NC     | 4, 7, 9, 10, 15, 17, 24, 25, 28, 29, 32-35 | — | 空管脚 |
| IO2    | 5           | I/O/T | GPIO2, ADC1_CH2, FSPIQ |
| IO3    | 6           | I/O/T | GPIO3, ADC1_CH3 |
| EN     | 8           | I     | 高电平：芯片使能；低电平：芯片关闭。注意不能让 EN 管脚浮空。 |
| IO0    | 12          | I/O/T | GPIO0, ADC1_CH0, XTAL_32K_P |
| IO1    | 13          | I/O/T | GPIO1, ADC1_CH1, XTAL_32K_N |
| IO10   | 16          | I/O/T | GPIO10, FSPICS0 |
| IO4    | 18          | I/O/T | GPIO4, ADC1_CH4, FSPIHD, MTMS |
| IO5    | 19          | I/O/T | GPIO5, ADC2_CH0, FSPIWP, MTDI |
| IO6    | 20          | I/O/T | GPIO6, FSPICLK, MTCK |
| IO7    | 21          | I/O/T | GPIO7, FSPID, MTDO |
| IO8    | 22          | I/O/T | GPIO8 |
| IO9    | 23          | I/O/T | GPIO9 |
| IO18   | 26          | I/O/T | GPIO18, USB_D- |
| IO19   | 27          | I/O/T | GPIO19, USB_D+ |
| RXD0   | 30          | I/O/T | GPIO20, U0RXD |
| TXD0   | 31          | I/O/T | GPIO21, U0TXD |

**类型说明：** P = 电源；I = 输入；O = 输出；T = 可设置为高阻。
