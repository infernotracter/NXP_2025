from basic_data import *


class CCDHandler:
    def __init__(self, channel):
        """CCD数据处理类, channel: CCD通道"""
        self.data = [0] * 128
        self.last_mid_point = 0
        self.mid_point = 0
        self.left_point = 0
        self.right_point = 0
        self.channel = channel  # CCD通道
        self.lost_l = False
        self.lost_r = False

    def _update(self):
        """获取CCD数据"""
        self.data = ccd.get(self.channel)

    def _get_threshold(self):
        value_max = self.data[4]  # 从第5个元素开始考虑最大值
        value_min = self.data[4]  # 从第5个元素开始考虑最小值

        # 遍历5-122
        for i in range(5, 123):
            value_max = max(value_max, self.data[i])  # 限幅在最大传入数据和第5个元素值上
            value_min = min(value_min, self.data[i])  # 限幅在最小传入数据和第5个元素值上

        threshold = (value_max + value_min) / 2  # 计算阈值
        threshold = min(max(75, threshold), 255)  # 阈值限幅在75-256之间
        return threshold

    def get_mid_point(self, value, reasonrange, searchgap = 0, LeftEdge = 8, RightEdge = 122):
        """获取中点, value: 差比和公式的值, reasonrange: 合理范围, searchgap: 搜索间隔, LeftEdge: 左边界, RightEdge: 右边界"""
        self._update(self.channel)  # 更新数据
        for i in range(self.last_mid_point - 4 - searchgap, 1, -1):  # 用差比和公式判断是否找到边线
            if (abs(self.data[i+4]-self.data[i])*100/(self.data[i + 4]+self.data[i])) > value:
                self.left_point = i  # 左边点找到
                break
            elif i == 1:  # 如果找到1都没找到
                self.left_point = 0  # 强制令左边点为0
                break

        # 搜索右边点，以上次中点作为这次的起搜点
        for i in range(self.last_mid_point+4 + searchgap, 126):  # 注意这里应该是128，因为索引是从0开始的
            if (abs(self.data[i-4]-self.data[i])*100/(self.data[i-4]+self.data[i])) > value:  # 判断是否与右边点一致
                self.right_point = i  # 右边点找到
                break
            elif i == 126:  # 如果找到126都没找到
                self.right_point = 127  # 强制右左边点为127
                break

        if self.left_point < LeftEdge:
            self.lost_l = True
        if self.right_point > RightEdge:
            self.lost_r = True

        self.mid_point = int((self.left_point + self.right_point) / 2)  # 中点计算
        if abs(self.mid_point - self.last_mid_point) > reasonrange:  # 如果中点与上次中点差距过大
            self.mid_point = self.last_mid_point # 强制令中点为上次中点
        self.last_mid_point = self.mid_point  # 更新上次中点
        return self.mid_point  # 返回中点
    
# 实例
ccd_f = CCDHandler(0)  # 创建CCDHandler实例，通道为0
ccd_n = CCDHandler(1)  # 创建CCDHandler实例，通道为1


class GYRO_Z1:
    def __init__(self):
        self.data = 0
    def update(self):
        imu


# 常量定义（根据实际赛道调整）
CCD_LEFT_RING1_RANGE = (30, 42)       # 左圆环阶段1近端CCD左右边点范围
CCD_RIGHT_RING1_RANGE = (90, 108)
CCD_FAR_LEFT_LOSS = 7                 # 远端CCD左丢线阈值
CCD_FAR_RIGHT_RANGE = (88, 101)        # 远端CCD右边点范围
POINT_DIFF_THRESHOLD = 10             # 特征点差异阈值

# 赛道元素状态枚举
class RoadElement:
    NORMAL = 0
    circle_l1 = 1
    circle_l2 = 2
    circle_r1 = 3
    circle_r2 = 4
    zebra = 5

# 元素检测器类
class ElementDetector:
    def __init__(self):
        self.state = RoadElement.NORMAL
        self.ring_progress = 0  # 圆环进度
        self.zebra_count = 0    # 斑马线特征计数
        
    def detect(self, ccd1, ccd2, imu_data):
        """主检测函数"""
        # element = RoadElement.NORMAL
        
        # 优先检测斑马线
        if self._check_zebra(ccd2):
            element = RoadElement.zebra
        # 检测左圆环
        elif self._check_left_ring(ccd1, ccd2, imu_data):
            element = RoadElement.circle_l1 if self.state != RoadElement.circle_l2 else RoadElement.circle_l2
        # 检测右圆环
        elif self._check_right_ring(ccd1, ccd2, imu_data):
            element = RoadElement.circle_r1 if self.state != RoadElement.circle_r2 else RoadElement.circle_r2
            
        self._update_state(element, imu_data)
        return element
    
    def _check_left_ring(self, ccd1, ccd2, imu):
        """左圆环检测逻辑"""
        # 近端CCD特征检查
        near_valid = (CCD_LEFT_RING1_RANGE[0] <= ccd2.left <= CCD_LEFT_RING1_RANGE[1] and 
                     CCD_RIGHT_RING1_RANGE[0] <= ccd2.right <= CCD_RIGHT_RING1_RANGE[1])
        
        # 远端CCD特征检查
        far_valid = (ccd1.left < CCD_FAR_LEFT_LOSS and 
                    CCD_FAR_RIGHT_RANGE[0] <= ccd1.right <= CCD_FAR_RIGHT_RANGE[1])
        
        # 特征点一致性检查
        point_diff = abs(ccd1.right - ccd2.right)
        
        # 陀螺仪左转趋势验证
        gyro_z_valid = imu[5] > 2.0  # 假设z轴角速度正值代表左转
        
        return near_valid and far_valid and (point_diff <= POINT_DIFF_THRESHOLD) and gyro_z_valid
    
    def _check_right_ring(self, ccd1, ccd2, imu):
        """右圆环检测逻辑（对称实现）"""
        # 实现与左圆环对称的检测逻辑
        # ...
        
    def _check_zebra(self, ccd_data):
        """斑马线检测优化版"""
        transition = 0
        prev = ccd_data[0]
        
        # 动态阈值计算
        avg = sum(ccd_data[50:78]) / 28  # 中间区域平均值
        threshold = avg * 0.7
        
        # 统计有效跳变
        for i in range(1, 127):
            diff = abs(ccd_data[i] - prev)
            if diff > threshold:
                transition += 1
            prev = ccd_data[i]
            
        # 有效跳变特征判断
        if transition >= 6:  # 根据实际调整阈值
            self.zebra_count += 1
            if self.zebra_count >= 3:  # 连续检测提高鲁棒性
                return True
        else:
            self.zebra_count = 0
            
        return False
    
    def _update_state(self, element, imu):
        """状态机更新"""
        if element == RoadElement.circle_l1:
            if self.state != RoadElement.circle_l2:
                self.ring_progress = 0
                self.state = RoadElement.circle_l1
                
        elif element == RoadElement.circle_l2:
            self.ring_progress += abs(imu[5]) * 0.002  # 积分角速度计算进度
            if self.ring_progress >= 360:  # 完成一圈
                self.state = RoadElement.NORMAL
                
        # 其他状态更新...