# K10 单摆实验

这个项目让 UNIHIKER K10 + HUSKYLENS 2 自己完成单摆实验：HUSKYLENS 2 用物体追踪识别摆球中心，K10 通过 I2C 读取坐标，并在屏幕上计算/显示单摆公式、频率、振幅和重力加速度 `g`。不需要 TF 卡，也不保存图片。

## 演示视频

<video src="./单摆实验_测重力加速度.mp4" controls width="100%"></video>

如果 GitHub 没有直接显示播放器，可以打开 [单摆实验_测重力加速度.mp4](./单摆实验_测重力加速度.mp4) 查看实验演示。

## 硬件方案

K10 只负责采样、计算和显示；HUSKYLENS 2 负责视觉定位。这样比只用 K10 内置摄像头更稳定，背景和光照对识别结果的影响更小。

```text
摆球 -> HUSKYLENS 2 Object Tracking -> I2C -> K10 -> f/T/A/theta/g
```

连接方式：

```text
HUSKYLENS 2 Gravity 4Pin / Power Board -> K10 I2C/Gravity 口
VCC                                    -> VCC
GND                                    -> GND
SDA                                    -> SDA
SCL                                    -> SCL
```

在 HUSKYLENS 2 上把协议设置为 I2C，并进入 `Object Tracking`。对准摆球后学习目标，默认代码读取学习目标 `ID 1`。

## 1. 烧录前设置参数

打开 [src/main.cpp](./src/main.cpp)，先改顶部这些常量：

```cpp
constexpr float kPendulumLengthM = 0.420f;  // 摆长：悬点到摆球中心，单位 m
constexpr int kPivotX = 160;                // 悬点在 HUSKYLENS 画面中的 x 坐标
constexpr int kPivotY = 24;                 // 悬点在 HUSKYLENS 画面中的 y 坐标
constexpr uint16_t kTargetId = 1;           // HUSKYLENS 学习到的摆球 ID
```

如果 HUSKYLENS 2 学到的摆球不是 `ID 1`，把 `kTargetId` 改成屏幕上显示的 ID。

## 2. 上传固件

建议用这个 PlatformIO 路径，避免系统里旧版 `pio` 的 `intelhex` 问题：

```bash
/Users/rockets/.platformio/penv/bin/pio run -t upload
```

## 3. 在 K10 上实验

1. 让 HUSKYLENS 2 正对单摆摆动平面，并固定好镜头。
2. 在 HUSKYLENS 2 上选择 `Object Tracking`，学习摆球目标。
3. 烧录后确认 K10 屏幕显示 `I2C:ok`。
4. 按 `A` 开始实时采样。
5. 等摆动 5 到 10 个周期左右，按 `B` 停止并在 K10 本机分析。

K10 会显示：

```text
x = x0 + A cos(2pi*f*t)
f, T, A, theta, g, fps
```

其中 `f` 是频率，`T` 是周期，`A` 是图像振幅，`theta` 是角振幅，`g=(2πf)^2L`，`fps` 是实际有效采样帧率。

## 位置获取方式

每次采样时，K10 会：

1. 通过 I2C 向 HUSKYLENS 2 请求 `ID 1` 的 block。
2. 读取 block 的中心坐标 `(xCenter, yCenter)`。
3. 把坐标和当前时间戳记入采样数组。
4. 停止后用横向坐标 `x(t)` 估算频率和周期。

## 注意

- 摆角建议小于 `10°`，这样小角度公式更准。
- `g` 的准确度主要取决于摆长 `kPendulumLengthM` 和频率测量。
- 悬点坐标 `kPivotX/kPivotY` 主要影响角振幅 `theta`。
- 如果 K10 显示 `I2C:check`，检查 HUSKYLENS 2 是否设为 I2C、4Pin 线是否接好、HUSKYLENS 2 是否正常供电。
- 如果样本数不增长，通常是 HUSKYLENS 2 没有看到已学习的目标；重新学习摆球或调整镜头位置。

## 可选：电脑端复核

[tools/analyze_pendulum.py](./tools/analyze_pendulum.py) 仍保留，可用于视频或图片序列复核结果。
