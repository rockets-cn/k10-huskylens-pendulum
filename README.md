# K10 单摆实验

这个项目让 UNIHIKER K10 + HUSKYLENS 2 自己完成单摆实验：HUSKYLENS 2 用颜色识别追踪摆球中心，K10 通过 I2C 读取坐标，并在屏幕和网页上计算/显示单摆公式、频率、振幅和重力加速度 `g`。不需要 TF 卡，也不保存图片。

## 演示视频

<video src="./单摆实验_测重力加速度.mp4" controls width="100%"></video>

如果 GitHub 没有直接显示播放器，可以打开 [单摆实验_测重力加速度.mp4](./单摆实验_测重力加速度.mp4) 查看实验演示。

## 硬件方案

K10 只负责采样、计算和显示；HUSKYLENS 2 负责视觉定位。这样比只用 K10 内置摄像头更稳定，背景和光照对识别结果的影响更小。

```text
摆球 -> HUSKYLENS 2 Color Recognition -> I2C -> K10 -> f/T/A/theta/g -> 网页表格
```

连接方式：

```text
HUSKYLENS 2 Gravity 4Pin / Power Board -> K10 I2C/Gravity 口
VCC                                    -> VCC
GND                                    -> GND
SDA                                    -> SDA
SCL                                    -> SCL
```

在 HUSKYLENS 2 上把协议设置为 I2C，并进入 `Color Recognition`。用颜色明显的摆球或给摆球贴一块颜色标签，学习目标颜色，默认代码读取学习目标 `ID 1`。

## 1. 烧录前设置参数

打开 [src/main.cpp](./src/main.cpp)，先改顶部这些常量：

```cpp
constexpr int kPivotX = 160;                // 悬点在 HUSKYLENS 画面中的 x 坐标
constexpr int kPivotY = 24;                 // 悬点在 HUSKYLENS 画面中的 y 坐标
constexpr uint16_t kTargetId = 1;           // HUSKYLENS 学习到的颜色 ID
constexpr uint16_t kIgnoreFrameId = 2;      // 需要忽略的大框/背景颜色 ID
```

如果 HUSKYLENS 2 学到的颜色不是 `ID 1`，把 `kTargetId` 改成屏幕上显示的 ID。
如果画面里有固定大框或背景色块，可以单独学习成 `ID 2`，程序会忽略它，只用 `ID 1` 的摆球坐标计算。

摆长 `L` 不需要重新编译，在网页的“实验参数”里手动输入并保存。默认值是 `0.420 m`，保存后会写入 K10 的 NVS。

## 2. 上传固件

建议用这个 PlatformIO 路径，避免系统里旧版 `pio` 的 `intelhex` 问题：

```bash
/Users/rockets/.platformio/penv/bin/pio run -t upload
```

## 3. 在 K10 上实验

1. 让 HUSKYLENS 2 正对单摆摆动平面，并固定好镜头。
2. 在 HUSKYLENS 2 上选择 `Color Recognition`，学习摆球颜色。
3. 烧录后确认 K10 屏幕显示 `I2C:ok`。
4. 连接 K10 建立的 WiFi：`k10-pendulum`，密码 `12345678`。
5. 连接后系统通常会自动弹出配网页面；如果没有弹出，手动打开 `http://192.168.4.1/`。
6. 按 `A` 或网页里的“开始采样”开始实时采样。
7. 等摆动 5 到 10 个周期左右，按 `B` 或网页里的“停止并计算”停止并分析。

K10 会同时尝试连接局域网 WiFi。默认 SSID 是 `DFRobot-guest`，默认密码是 `dfrobot@2017`。配网成功后，网页会显示“局域网 IP”，同一局域网内可以用这个 IP 直接访问实验页面。

K10 会显示：

```text
2D ellipse fit
f, T, Amajor/Aminor, rmse, g, fps
```

其中 `f` 是频率，`T` 是周期，`Amajor/Aminor` 是二维椭圆拟合得到的长短半轴，`rmse` 是拟合误差，`g=(2πf)^2L`，`fps` 是实际有效采样帧率。

网页会显示：

- 实时状态：采样中/已计算、样本数、丢帧、最后坐标、目标颜色 ID、摆长和悬点。
- HUSKYLENS 诊断：当前返回的 block 数、匹配到的 ID、色块尺寸。点击开始后如果样本数不增长，先看这里是否有 `ID 1`。
- 计算结果表：二维椭圆拟合中心、长短半轴、拟合误差、周期、频率、角振幅、重力加速度和有效采样帧率。
- 最近样本表：最近 80 个 `(t, x, y)` 采样点。
- 实验参数：手动输入摆长 `L`。
- 配网：修改局域网 WiFi，并保存到 K10。
- `samples.csv`：下载本次采样的完整表格。
- `/ota`：上传新固件的 HTTP OTA 表单。

## 位置获取方式

每次采样时，K10 会：

1. K10 启动时把 HUSKYLENS 2 切到 `Color Recognition` 算法。
2. 通过 I2C 向 HUSKYLENS 2 请求 `ID 1` 的 color block。
3. 读取 block 的中心坐标 `(xCenter, yCenter)`。
4. 把坐标和当前时间戳记入采样数组。
5. 停止后同时拟合 `x(t)` 和 `y(t)` 的同频正弦/余弦模型，适配摆球在画面中呈椭圆运动的情况。

频率估算不再只看横向过零点。程序会在合理重力加速度对应的频率范围内搜索，让 `x = x0 + ax cos(ωt) + bx sin(ωt)` 和 `y = y0 + ay cos(ωt) + by sin(ωt)` 的二维残差最小。再由拟合出的椭圆长半轴估算角振幅。若采样时长不足、椭圆拟合误差过大、摆角过大，网页会把数据质量标为异常，而不是硬给出一个离谱的 `g`。

## 注意

- 摆角建议小于 `10°`，这样小角度公式更准。
- `g` 的准确度主要取决于摆长 `kPendulumLengthM` 和频率测量。
- 悬点坐标 `kPivotX/kPivotY` 主要影响角振幅 `theta`。
- 如果 K10 显示 `I2C:check`，检查 HUSKYLENS 2 是否设为 I2C、4Pin 线是否接好、HUSKYLENS 2 是否正常供电。
- 如果样本数不增长，通常是 HUSKYLENS 2 没有看到已学习的颜色；重新学习颜色、换更醒目的摆球颜色、调整镜头位置或避开背景同色物体。
- 第一次启用 OTA 分区后需要 USB 烧录一次。之后只要固件仍保留网页 `/ota`，可以在网页上继续无线更新。

## 可选：电脑端复核

[tools/analyze_pendulum.py](./tools/analyze_pendulum.py) 仍保留，可用于视频或图片序列复核结果。
