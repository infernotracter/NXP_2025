from basic_data import *


class CCDHandler:
    def __init__(self, channel):
        """CCD数据处理类, channel: CCD通道"""
        self.data = [0] * 128
        self.last_mid = 0
        self.mid = 0
        self.left = 0
        self.right = 0
        self.channel = channel  # CCD通道

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

    def get_mid_point(self, tmpdata,  value, reasonrange, follow, searchgap = 0):
        """获取中点, value: 差比和公式的值, reasonrange: 合理范围(两次差值的范围),
        follow: if>0 补右边线，跟左边线, searchgap: 搜索间隔"""
        self.data = tmpdata
        count_up = 0
        count_down = 0
        for i in range(self.last_mid - 4 - searchgap, 1, -1):  # 用差比和公式判断是否找到边线
            if (abs(self.data[i+4]-self.data[i])*100/(self.data[i + 4]+self.data[i])) > value:
                self.left = i  # 左边点找到
                break
            elif i == 1:  # 如果找到1都没找到
                self.left = 0  # 强制令左边点为0
                break

        # 搜索右边点，以上次中点作为这次的起搜点
        for i in range(self.last_mid + 4 + searchgap, 126):  # 注意这里应该是128，因为索引是从0开始的
            if (abs(self.data[i-4]-self.data[i])*100/(self.data[i-4]+self.data[i])) > value:  # 判断是否与右边点一致
                self.right = i  # 右边点找到
                break
            elif i == 126:  # 如果找到126都没找到
                self.right = 127  # 强制右左边点为127
                break
        # if self.left < LeftEdge:
        #     self.lost_l = True
        # if self.right > RightEdge:
        #     self.lost_r = True
        self.mid = int((self.left + self.right) / 2)  # 中点计算

        if follow > 0 and self.right == 127:
            self.right = self.left + follow
        if follow < 0 and self.left == 0:
            self.left = self.right + follow        

        if abs(self.mid - self.last_mid) > reasonrange:  # 如果中点与上次中点差距过大
            self.mid = self.last_mid # 强制令中点为上次中点
        self.last_mid = self.mid  # 更新上次中点
        return self.mid  # 返回中点

# 常量定义（根据实际赛道调整）
ccd_near_l = (30, 42)       # 左圆环阶段1近端CCD左右边点范围
ccd_near_r = (90, 108)
ccd_near_lost = 10

ccd_far_right = (88, 101)        # 远端CCD右边点范围
ccd_far_left = (30, 42)         # 远端CCD左边点范围
ccd_far_lost = 7                 # 远端CCD左丢线阈值

ccd_near_lost = 10             # 特征点差异阈值

# 赛道元素状态枚举
class RoadElement:
    normal = 0
    circle_l1 = 1
    circle_l2 = 2
    circle_r1 = 3
    circle_r2 = 4
    zebra = 5

class ElementDetector:
    """赛道元素检测器"""
    def __init__(self):
        self.state = RoadElement.normal
        self.ring_progress = 0  # 圆环进度
        self.zebra_count = 0    # 斑马线特征计数
        self._ccd_far = 0
        self._ccd_near = 0
        self.imu_data = 0
        self.follow = 0
        
    def update(self, _ccd_far, _ccd_near, imu_data):
        """主检测函数"""
        self._ccd_far = _ccd_far
        self._ccd_near = _ccd_near
        self.imu_data = imu_data
        
        # if self._check_zebra(self._ccd_near):
        #     element = RoadElement.zebra
        if self._check_left_ring_1(self._ccd_far, self._ccd_near):
            element = RoadElement.circle_l1
        if self._check_right_ring_1(self._ccd_far, self._ccd_near):
            element = RoadElement.circle_r1
        if self._check_left_ring_2(self._ccd_far, self._ccd_near) and element == RoadElement.circle_l1:
            element = RoadElement.circle_l2
        if self._check_right_ring_2(self._ccd_far, self._ccd_near) and element == RoadElement.circle_r1:
            element = RoadElement.circle_r2
            
        self._update_state(element)
        return element
    
    def _check_left_ring_1(self):
        """左圆环检测逻辑"""
        # 近端CCD特征检查
        near_valid = (ccd_near_l[0] <= self._ccd_near.left <= ccd_near_l[1] and 
                     ccd_near_r[0] <= self._ccd_near.right <= ccd_near_r[1])
        
        # 远端CCD特征检查
        far_valid = (self._ccd_far.left < ccd_far_lost and 
                    ccd_far_right[0] <= self._ccd_far.right <= ccd_far_right[1])
        
        # 特征点一致性检查
        point_diff = abs(self._ccd_far.right - self._ccd_near.right)
        
        # 陀螺仪左转趋势验证
        gyro_z_valid = self.imu_data[5] > 2.0  # 假设z轴角速度正值代表左转
        
        return near_valid and far_valid and (point_diff <= ccd_near_lost) and gyro_z_valid
    
    def _check_right_ring_1(self):
        """右圆环检测逻辑"""
        # 近端CCD特征检查（左右镜像）
        near_valid = (ccd_near_r[0] <= self._ccd_near.left <= ccd_near_r[1] and 
                    ccd_near_l[0] <= self._ccd_near.right <= ccd_near_l[1])
        
        # 远端CCD特征检查（左右镜像）
        far_valid = (self._ccd_far.right > ccd_far_lost and 
                    ccd_far_left[0] <= self._ccd_far.left <= ccd_far_left[1])
        
        # 特征点一致性检查（比较左边缘）
        point_diff = abs(self._ccd_far.left - self._ccd_near.left)
        
        # 陀螺仪右转趋势验证
        gyro_z_valid = self.imu_data[5] < -2.0  # z轴角速度负值代表右转
        
        return near_valid and far_valid and (point_diff <= ccd_near_lost) and gyro_z_valid
            

    def _check_left_ring_2(self):
        """左圆环状态2检测：近端左丢线+特征点稳定"""
        # 近端CCD左丢线检查（left_point_2 <=10）
        near_left_lost = self._ccd_near.left <= ccd_near_lost
        
        # 近端右边界有效性检查（87 <= right_point_2 <=103）
        near_right_valid = ccd_near_r[0] <= self._ccd_near.right <= ccd_near_r[1]
        
        # 特征点稳定性检查（|right_point_1 - right_point_2| <=12）
        point_diff = abs(self._ccd_far.right - self._ccd_near.right)
        
        # 陀螺仪左转趋势验证（z轴角速度>1.5）
        gyro_z_valid = self.imu_data[5] > 1.5
        
        return near_left_lost and near_right_valid and (point_diff <= 12) and gyro_z_valid

    def _check_right_ring_2(self):
        """右圆环状态2检测：近端右丢线+特征点稳定"""
        # 近端CCD右丢线检查（right_point_2 >=115）
        near_right_lost = self._ccd_near.right >= ccd_far_lost
        
        # 近端左边界有效性检查（31 <= left_point_2 <=44）
        near_left_valid = ccd_near_l[0] <= self._ccd_near.left <= ccd_near_l[1]
        
        # 特征点稳定性检查（|left_point_1 - left_point_2| <=12）
        point_diff = abs(self._ccd_far.left - self._ccd_near.left)
        
        # 陀螺仪右转趋势验证（z轴角速度<-1.5）
        gyro_z_valid = self.imu_data[5] < -1.5
        
        return near_right_lost and near_left_valid and (point_diff <= 12) and gyro_z_valid
  
    def _check_zebra(self, _ccd_near):
        """斑马线检测"""
        # transition = 0
        # prev = _ccd_near[0]
        
        # # 动态阈值计算
        # avg = sum(_ccd_near[50:78]) / 28  # 中间区域平均值
        # threshold = avg * 0.7
        
        # # 统计有效跳变
        # for i in range(1, 127):
        #     diff = abs(_ccd_near[i] - prev)
        #     if diff > threshold:
        #         transition += 1
        #     prev = _ccd_near[i]
            
        # # 有效跳变特征判断
        # if transition >= 6:  # 根据实际调整阈值
        #     self.zebra_count += 1
        #     if self.zebra_count >= 3:  # 连续检测提高鲁棒性
        #         return True
        # else:
        #     self.zebra_count = 0
            
        # return False
        pass

    def _check_ring_right_3(self, imu_data):
        pass
    
    def _update_state(self, element, imu_data):
        """状态机更新"""
        if element == RoadElement.circle_l1:
            if self.state != RoadElement.circle_l2:
                self.ring_progress = 0
                self.state = RoadElement.circle_l1
                
        elif element == RoadElement.circle_l2:
            self.ring_progress += abs(imu_data[5]) * 0.002  # 积分角速度计算进度
            if self.ring_progress >= 360:  # 完成一圈
                self.state = RoadElement.normal
                
        # 其他状态更新...

class Distance:
    def __init__(self):
        self.data = 0
    def update(self, data):
        self.data += data
        return self.data
    def reset(self):
        self.data = 0
