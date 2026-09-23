# Arduino VSCode 扩展 - AI 助手指南

本项目使用 **Arduino VSCode Extension**，提供完整的 Arduino 开发支持。以下是你可以利用的功能和资源。

---

## 🔧 项目结构

本项目可能采用以下两种结构之一：

### 传统模式 (flat)
```
ProjectName/
├── ProjectName.ino       # 主程序文件
├── other_code.cpp        # 其他源文件（同级目录）
└── .vscode/
    ├── serial-data.log   # 串口数据日志
    └── build-output.log  # 编译输出日志
```

### 分组模式 (grouped)
```
ProjectName/
├── src/                  # 📁 源代码目录
│   ├── ProjectName.ino   # 主程序文件
│   ├── config/           # 配置相关代码
│   ├── drivers/          # 硬件驱动代码
│   └── utils/            # 工具函数
├── lib/                  # 项目本地库
├── build/                # 编译输出
├── .arduino/             # Arduino 配置
│   └── project.yaml      # 项目配置文件
└── .vscode/
    ├── serial-data.log   # 串口数据日志
    └── build-output.log  # 编译输出日志
```

> **提示**: 分组模式下，源代码可以放在 `src/` 的任意子目录中，编译时会自动收集所有 `.ino`、`.cpp`、`.c`、`.h` 文件。

---

## 📡 串口监视器数据访问

当用户询问关于 **串口数据、串口监视器、设备输出、调试信息** 等内容时，可以读取串口日志：

### 访问方式

```
read_file(".vscode/serial-data.log", 1, 500)
```

### 数据格式

- `>>>` 表示用户发送给设备的数据
- `<<<` 表示设备输出的数据
- 每行包含时间戳

### 何时使用

- 用户问"帮我看看串口输出"
- 用户问"分析设备发送的数据"
- 用户问"串口有什么错误"
- 用户问"检查 Arduino 的输出"
- 用户需要调试传感器数据
- 用户排查通信问题

### 注意事项

- 串口监视器打开后，数据会自动保存到此文件
- 数据每秒自动更新到文件
- 如果文件不存在，提示用户先打开串口监视器

---

## 🔨 编译输出日志访问

当用户询问关于 **编译错误、编译失败、上传失败、错误排查** 等内容时，优先检查编译日志：

### 访问方式

**优先读取最后几行来查看最新的错误信息：**
```
read_file(".vscode/build-output.log", -100, -1)  # 读取最后100行
```

**或读取完整日志：**
```
read_file(".vscode/build-output.log", 1, 500)
```

### 数据格式

- `[日期 时间] [INFO]` 表示普通信息（包括编译器命令）
- `[日期 时间] [WARNING]` 表示警告信息
- `[日期 时间] [ERROR]` 表示**真正的编译错误**（如语法错误、未定义引用等）
- `[日期 时间] [SUCCESS]` 表示成功信息

> **注意**: 日志系统会智能区分真正的错误和普通的编译器命令。编译器命令行（如 `xtensa-esp32s3-elf-g++`）会被标记为 INFO 而非 ERROR。

### 何时使用

- 用户问"检查编译错误"或"帮我看看编译失败的原因"
- 用户问"上传失败了怎么办"
- 用户问"错误排查"或"debug"
- 用户说"编译不过"或"编译报错"
- 用户需要分析编译警告

### 错误排查步骤

1. **优先读取日志最后 50-100 行**，查找 `[ERROR]` 标记的行
2. 根据错误信息定位问题：
   - `undefined reference` - 函数未定义或库未安装
   - `no matching function` - 函数参数类型不匹配
   - `expected ';'` 或 `expected '}'` - 语法错误
   - `'xxx' was not declared` - 变量/函数未声明
   - `avrdude: stk500` - 上传通信失败，检查端口和连接
3. 结合源代码分析具体问题并提供修复建议

### 注意事项

- 每次编译或上传时，日志文件会自动重置
- 文件包含完整的编译过程输出
- 如果文件不存在，提示用户先执行编译命令
- 如非用户特别说明，所有相对路径都在项目目录内

---

## ⚙️ 项目配置文件

项目配置存储在 `.arduino/project.yaml`：

```yaml
board:
  fqbn: "arduino:avr:uno"      # 目标板卡
  port: "COM3"                  # 串口
  options: {}                   # 板卡选项
build:
  outputPath: "build"           # 编译输出目录
```

---

## 🛠️ 可用命令

你可以提示用户使用以下命令（通过命令面板 `Ctrl+Shift+P`）：

### 编译和上传
| 命令 | 说明 |
|------|------|
| `Arduino: Compile` | 编译当前项目 |
| `Arduino: Upload` | 编译并上传到板卡 |
| `Arduino: Verify` | 仅验证代码（不上传） |

### 板卡和端口
| 命令 | 说明 |
|------|------|
| `Arduino: Select Board` | 选择目标板卡 |
| `Arduino: Select Port` | 选择串口 |
| `Arduino: Open Arduino Center` | 打开统一控制面板 |

### 串口监视器
| 命令 | 说明 |
|------|------|
| `Arduino: Open Serial Monitor` | 打开串口监视器 |
| `Arduino: Close Serial Monitor` | 关闭串口监视器 |
| `Arduino: Send to Serial` | 发送数据到串口 |

### 库管理
| 命令 | 说明 |
|------|------|
| `Arduino: Search Library` | 搜索库 |
| `Arduino: Install Library` | 安装库 |
| `Arduino: Add .ZIP Library...` | 从 ZIP 安装库 |

### 平台管理
| 命令 | 说明 |
|------|------|
| `Arduino: Install Platform` | 安装板卡平台 |
| `Arduino: Update Platform` | 更新平台 |

---

## 📚 常用 Arduino 函数参考

### 数字 I/O
```cpp
pinMode(pin, mode);           // mode: INPUT, OUTPUT, INPUT_PULLUP
digitalWrite(pin, value);     // value: HIGH, LOW
int digitalRead(pin);         // 返回 HIGH 或 LOW
```

### 模拟 I/O
```cpp
int analogRead(pin);                    // 返回 0-1023
analogWrite(pin, value);                // PWM 输出 0-255
analogReference(type);                  // DEFAULT, INTERNAL, EXTERNAL
```

### 时间函数
```cpp
unsigned long millis();       // 返回毫秒数
unsigned long micros();       // 返回微秒数
delay(ms);                    // 延时毫秒
delayMicroseconds(us);        // 延时微秒
```

### 串口通信
```cpp
Serial.begin(baudRate);       // 常用: 9600, 115200
Serial.print(data);           // 打印数据
Serial.println(data);         // 打印并换行
Serial.available();           // 检查可读字节数
Serial.read();                // 读取一个字节
Serial.write(data);           // 写入原始字节
```

### 中断
```cpp
attachInterrupt(digitalPinToInterrupt(pin), ISR, mode);
// mode: LOW, CHANGE, RISING, FALLING
detachInterrupt(digitalPinToInterrupt(pin));
```

---

## 🔌 常用传感器示例

### DHT 温湿度传感器
```cpp
#include <DHT.h>
#define DHTPIN 2
#define DHTTYPE DHT11
DHT dht(DHTPIN, DHTTYPE);

void setup() {
  dht.begin();
}

void loop() {
  float h = dht.readHumidity();
  float t = dht.readTemperature();
}
```

### 超声波测距 (HC-SR04)
```cpp
#define TRIG_PIN 9
#define ECHO_PIN 10

long duration;
int distance;

void setup() {
  pinMode(TRIG_PIN, OUTPUT);
  pinMode(ECHO_PIN, INPUT);
}

void loop() {
  digitalWrite(TRIG_PIN, LOW);
  delayMicroseconds(2);
  digitalWrite(TRIG_PIN, HIGH);
  delayMicroseconds(10);
  digitalWrite(TRIG_PIN, LOW);
  
  duration = pulseIn(ECHO_PIN, HIGH);
  distance = duration * 0.034 / 2;
}
```

---

## ⚠️ 常见问题排查

### 编译错误
1. 检查是否选择了正确的板卡 (`Arduino: Select Board`)
2. 检查是否安装了所需的库
3. 检查语法错误（缺少分号、括号不匹配等）

### 上传失败
1. 检查串口是否正确 (`Arduino: Select Port`)
2. 检查 USB 线是否连接
3. 某些板卡上传时需要按 RESET 按钮
4. 关闭串口监视器后重试

### 串口无输出
1. 检查 `Serial.begin()` 波特率是否匹配
2. 确认串口监视器已打开
3. 检查 `Serial.print()` 调用是否正确

---

## 💡 编码建议

1. **避免使用 `delay()`** - 在需要响应的程序中使用 `millis()` 进行非阻塞延时
2. **使用有意义的变量名** - 如 `temperatureC` 而非 `t`
3. **模块化代码** - 将功能拆分到 `src/` 的子目录中（分组模式）
4. **添加注释** - 解释硬件连接和关键逻辑
5. **使用 `const` 定义引脚** - 便于修改和理解

---

*此文件由 Arduino VSCode Extension 自动生成，用于辅助 AI 编程助手理解项目上下文。*
