# NXP_self

### 好难啊
### 不会写
### 呜呜呜
### ＞︿＜


#### 介绍
NXP

# PID简析

### **代码整体结构分析**
#### 1. **控制环组成**
- **角度环（PD控制）**：`angle_ring()`
- **角速度环（PI控制）**：`gyro_ring()`
- **速度环（PI控制）**：`speed_ring()`
- **转向控制**：
  - 内环（陀螺仪反馈）：`dir_in_ring()`
  - 外环（摄像头反馈）：`dir_out_ring()`

#### 2. **控制流程**
```text
主循环（motor_control）：
├─ 每50次循环执行速度环（输出→角度环的输入）
├─ 每5次循环执行角度环（输出→角速度环的输入）
├─ 实时执行角速度环（输出→电机）
├─ 转向控制（dir_control）：
   ├─ 摄像头环（外环）每2次循环执行
   └─ 陀螺仪环（内环）实时修正
最终电机输出 = 角速度环输出 ± 转向控制输出
```
![CSDN图标](https://mmbiz.qpic.cn/mmbiz_png/6qJsrrDdiaicrBbicu77lA9YJuoIjIA7yDaAgnpxznB1g15mtuTPJUNdJdt4lMn9UbNYSyUbBFZ1coNS6A8HefhJw/640?wx_fmt=png&from=appmsg&tp=webp&wxfrom=5&wx_lazy=1&wx_co=1 "CSDN图标")
![](https://mmbiz.qpic.cn/mmbiz_png/6qJsrrDdiaicrBbicu77lA9YJuoIjIA7yDaHAIoERJTEYLnGt7ickrhbxe5UsbKGfU7IyaDibwhGs925G5VCooR5l7g/640?wx_fmt=png&from=appmsg&tp=webp&wxfrom=5&wx_lazy=1&wx_co=1)
---

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
