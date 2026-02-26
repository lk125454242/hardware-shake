ESP-IDF template app
====================

This is a template application to be used with [Espressif IoT Development Framework](https://github.com/espressif/esp-idf).

Please check [ESP-IDF docs](https://docs.espressif.com/projects/esp-idf/en/latest/get-started/index.html) for getting started instructions.

### 环境配置
```
idf.py set-target esp32c3
```
- **Flash 大小**：工程已用 `sdkconfig.defaults` 设为 4MB，无需在 menuconfig 里再设，可避免 “Detected size(4096k) larger than the size in the binary image header(2048k)” 告警。
- **I2C**：已使用新驱动 `driver/i2c_master.h`，不再出现 “please migrate to driver/i2c_master.h” 提示。

### 编译并写入
**管理员身份运行 esp power shell**
```
cd j:\ESP32\shake
idf.py build
idf.py -p COM4 flash monitor

idf.py -p COM4 monitor
```

### 若出现 `ninja: fatal: CreateProcess: Access is denied`

这是 Windows 下 ccache 或杀毒软件拦截子进程导致的，可任选其一：

1. **关闭 ccache（推荐）**  
   在运行 `idf.py` 前执行：
   ```powershell
   $env:IDF_CCACHE_ENABLE = "0"
   ```
   或通过 menuconfig：`idf.py menuconfig` → Component config → SDK tool configuration → 取消勾选 “Use ccache when compiling”。

2. 将工程目录（如 `J:\ESP32\shake`）和 ESP-IDF/Espressif 工具目录加入杀毒软件排除列表。

3. 以管理员身份打开 PowerShell/CMD 再执行 `idf.py -p COM4 flash monitor`。

*Code in this repository is in the Public Domain (or CC0 licensed, at your option.)
Unless required by applicable law or agreed to in writing, this
software is distributed on an "AS IS" BASIS, WITHOUT WARRANTIES OR
CONDITIONS OF ANY KIND, either express or implied.*


### 若出现 `build bootloading` 卡住

这是 Windows 下 ccache 或杀毒软件拦截子进程导致的

