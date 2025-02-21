# NXP_self

### 好难啊
### 不会写
### 呜呜呜
### ＞︿＜


#### 介绍
NXP

# PID简析

## **PID代码整体结构分析**
### 1. **控制环组成**
- **角度环（PD控制）**：`angle_ring()`
- **角速度环（PI控制）**：`gyro_ring()`
- **速度环（PI控制）**：`speed_ring()`
- **转向控制**：
  - 内环（陀螺仪反馈）：`dir_in_ring()`
  - 外环（摄像头反馈）：`dir_out_ring()`

### 2. **控制流程**
```text
主循环（motor_control）：
├─ 每50次循环执行速度环（输出→角度环的输入）
├─ 每5次循环执行角度环（输出→角速度环的输入）
├─ 实时执行角速度环（输出→电机）
├─ 转向控制（dir_control）：
   ├─ 摄像头环（外环）每2次循环执行
   └─ 陀螺仪环（内环）实时修正
最终电机输出 = 角速度环输出 ± 转向控制输出



摄像头路径偏差 → dir_out_ring（外环PD） → 期望角速度
               ↓
陀螺仪角速度 → dir_in_ring（内环PI） → duty_dir → 电机差速
```
### 3. **原理**
- ![](https://mmbiz.qpic.cn/mmbiz_png/6qJsrrDdiaicrBbicu77lA9YJuoIjIA7yDaAgnpxznB1g15mtuTPJUNdJdt4lMn9UbNYSyUbBFZ1coNS6A8HefhJw/640?wx_fmt=png&from=appmsg&tp=webp&wxfrom=5&wx_lazy=1&wx_co=1)
- ![](https://mmbiz.qpic.cn/mmbiz_png/6qJsrrDdiaicrBbicu77lA9YJuoIjIA7yDaHAIoERJTEYLnGt7ickrhbxe5UsbKGfU7IyaDibwhGs925G5VCooR5l7g/640?wx_fmt=png&from=appmsg&tp=webp&wxfrom=5&wx_lazy=1&wx_co=1)
---
## **转向控制PID简析**
这段代码中的dir转向控制逻辑采用了串级PID结构，结合摄像头偏差和陀螺仪角速度实现差速转向。以下是核心逻辑的分步解析：

---

### **1. 控制架构**
采用**外环（位置环）+ 内环（速度环）**的双环结构：
- **外环**：`dir_out_ring`（PD控制器）  
  输入摄像头检测的路径偏差`turn_error`，输出期望的角速度`duty_camera`。
- **内环**：`dir_in_ring`（PI控制器）  
  输入外环输出的期望角速度，结合陀螺仪实际角速度，输出最终转向控制量`duty_gyro`。

---

### **2. 关键函数解析**
#### **dir_control 函数**
```c
void dir_control(float turn_error, float gyro) {
    static int32 flag_dir = 0;
    static int32 duty_camera = 0;
    static int32 duty_gyro = 0;

    flag_dir++;
    if (flag_dir >= 2) { // 每2次调用更新一次外环
        turn_error = get_turn_error(); // 获取摄像头路径偏差
        duty_camera = dir_out_ring(0, turn_error); // 外环计算期望角速度
        flag_dir = 0;
    }

    duty_gyro = dir_in_ring(duty_camera, gyro); // 内环计算实际控制量
    duty_dir = duty_gyro; // 输出转向控制量
}
```
- **外环更新频率**：每2次调用更新一次（降低计算负载，适应摄像头帧率）。
- **内环实时性**：每次调用都更新，快速响应陀螺仪数据。

---

#### **dir_out_ring（外环 PD 控制）**
```c
int32 dir_out_ring(int32 except, int32 input) {
    // PD控制：except=0（目标偏差为0）, input=turn_error
    dir_camera.error = input - except; // 计算路径偏差
    dir_camera.out = kp * error + kd * (error - last_error);
    // 输出期望角速度（duty_camera）
}
```
- **目标**：消除摄像头检测的路径偏差（`turn_error`）。
- **作用**：生成期望的车辆角速度，使车辆回归路径中心。

---

#### **dir_in_ring（内环 PI 控制）**
```c
int32 dir_in_ring(int32 except, int32 input) {
    // PI控制：except=duty_camera（期望角速度）, input=实际角速度gyro
    dir_gyro.error = except - input; // 角速度偏差
    dir_gyro.out = kp * error + ki * integral; // 积分抗静差
    // 输出最终转向控制量（duty_gyro）
}
```
- **目标**：通过调节电机差速，使车辆实际角速度匹配外环的期望值。

---

### **3. 转向执行逻辑**
在 `motor_control` 函数中，转向控制量 `duty_dir` 影响左右电机占空比：
```c
duty_l = duty_gyro - duty_dir; // 左电机减少占空比
duty_r = duty_gyro + duty_dir; // 右电机增加占空比
```
- **差速原理**：左右电机速度差产生转向力矩。
- **限幅保护**：`my_limit` 函数限制占空比在 [-3000, 3000]。

---

### **4. 数据流向**
```
摄像头路径偏差 → dir_out_ring（外环PD） → 期望角速度
               ↓
陀螺仪角速度 → dir_in_ring（内环PI） → duty_dir → 电机差速
```

---

### **5. 参数调整建议**
- **外环（dir_out_ring）**：
  - 增大 `kp`：加快路径跟踪响应，但可能引发振荡。
  - 增大 `kd`：抑制超调，增强稳定性。
- **内环（dir_in_ring）**：
  - 增大 `kp`：提高角速度跟踪速度。
  - 调整 `ki`：消除稳态误差，但过大会导致积分饱和。

---

### **总结**
转向控制通过摄像头获取路径偏差，经外环生成期望角速度，再通过内环结合陀螺仪数据调节电机差速，实现精准转向。双环结构兼顾了路径跟踪的准确性和动态响应的平稳性。


### **与之前伪代码的关键差异**
#### 1. **控制环顺序相反**
- **用户代码**：速度环 → 角度环 → 角速度环  
  ```c
  // motor_control中：
  duty_speed = speed_ring(...);          // 速度环输出
  duty_angle = angle_ring(duty_speed);   // 速度环输出作为角度环的期望输入
  duty_gyro = gyro_ring(duty_angle);     // 角度环输出作为角速度环的期望输入
  ```
- **伪代码**：角度环 → 角速度环 → 速度环（级联外→内）

**问题**：  
用户代码中速度环的输出直接作为角度环的期望角度输入，这可能导致逻辑矛盾（速度应受平衡控制影响，而非反之）。传统设计中，速度环应作为最内环，修正平衡控制。

#### 2. **执行频率分层**
- 速度环：低频执行（每50次循环）
- 角度环：中频执行（每5次循环）
- 角速度环：高频实时执行

**合理性**：  
高频执行角速度环有助于快速稳定姿态，但速度环低频可能导致速度响应滞后。需验证实际控制周期是否匹配动态需求。

#### 3. **转向控制集成**
- 独立转向环（摄像头+陀螺仪）与平衡控制分离，最终叠加到电机输出：
  ```c
  duty_l = duty_gyro - duty_dir;  // 左电机：平衡输出 - 转向修正
  duty_r = duty_gyro + duty_dir;  // 右电机：平衡输出 + 转向修正
  ```
**优势**：转向与平衡解耦，避免参数耦合问题。

#### 4. **PID实现细节**
- **角度环**：仅PD控制（无积分项）
- **角速度环**：PI控制（保留积分抗饱和）
- **速度环**：PI控制（积分限幅）
- **电机死区补偿**：
  ```c
  if (duty_gyro >= 0) duty_gyro += 100;  // 正向输出补偿
  else duty_gyro -= 100;                 // 负向输出补偿
  ```
**作用**：抵消电机启动静摩擦力，但固定值补偿可能不适用于所有场景。

#### -------------------------------------------------------

#### 软件架构
软件架构说明


#### 安装教程

1.  xxxx
2.  xxxx
3.  xxxx

#### 使用说明

1.  xxxx
2.  xxxx
3.  xxxx

#### 参与贡献

1.  Fork 本仓库
2.  新建 Feat_xxx 分支
3.  提交代码
4.  新建 Pull Request


#### 特技

1.  使用 Readme\_XXX.md 来支持不同的语言，例如 Readme\_en.md, Readme\_zh.md
2.  Gitee 官方博客 [blog.gitee.com](https://blog.gitee.com)
3.  你可以 [https://gitee.com/explore](https://gitee.com/explore) 这个地址来了解 Gitee 上的优秀开源项目
4.  [GVP](https://gitee.com/gvp) 全称是 Gitee 最有价值开源项目，是综合评定出的优秀开源项目
5.  Gitee 官方提供的使用手册 [https://gitee.com/help](https://gitee.com/help)
6.  Gitee 封面人物是一档用来展示 Gitee 会员风采的栏目 [https://gitee.com/gitee-stars/](https://gitee.com/gitee-stars/)
