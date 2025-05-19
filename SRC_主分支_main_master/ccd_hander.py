from basic_data import *


def check_tuple(data, count_up, count_down):
    """data: 数据， count_up: 白点， count_down: 黑点"""
    threshold = 110
    up_count = down_count = 0
    for num in data:
        if num > count_up:
            up_count += 1
        elif num < count_down:
            down_count += 1
    
    if up_count >= threshold:
        return 1  # 超过上限状态
    elif down_count >= threshold:
        return -1  # 低于下限状态
    return 0  # 正常状态

class CCDHandler:
    def __init__(self, channel):
        """CCD数据处理类, channel: CCD通道"""
        self.data = [0] * 128
        self.last_mid = 64
        self.mid = 64
        self.left = 0
        self.right = 127
        self.channel = channel  # CCD通道
        self.error=0
    def update(self):
        """更新ccd的data数据"""
        self.data = ccd.get(self.channel)
        return self.data
    def get_threshold(self):
        value_max = self.data[4]  # 从第5个元素开始考虑最大值
        value_min = self.data[4]  # 从第5个元素开始考虑最小值

        # 遍历5-122
        for i in range(5, 123):
            value_max = max(value_max, self.data[i])  # 限幅在最大传入数据和第5个元素值上
            value_min = min(value_min, self.data[i])  # 限幅在最小传入数据和第5个元素值上

        threshold = (value_max + value_min) // 2  # 计算阈值
        # threshold = min(max(75, threshold), 255)  # 阈值限幅在75-256之间
        return threshold

    def get_mid_point(self, value, reasonrange, follow = 0, searchgap = 0):
        """获取中点, value: 差比和公式的值, reasonrange: 合理范围(两次差值的范围),
        follow: if>0 补右边线，跟左边线, searchgap: 搜索间隔"""
        self.data = ccd.get(self.channel)
        # 判断中点值是否在赛道内，防止一直扫一边线
        #True: 这里的判断条件是为了防止中点值一直在赛道外
        # if self.last_mid < 30 or self.last_mid > 100:
        tmpthreshold = self.get_threshold()  # 计算阈值
        if self.data[self.last_mid] < tmpthreshold:
            self.midpoint_invalid(searchgap = searchgap, value = value)
            return self.mid
        # 主代码
        # 搜索边线
        self.search(searchgap, value)
        # if self.left == 0:
        #     self.left = self.right - self.ccd_near_lenth
        # if self.right == 127:
        #     self.right = self.left + self.ccd_near_lenth
        self.mid = (self.left + self.right) // 2

        if follow > 0:
            self.right = self.left + follow
        if follow < 0:
            self.left = self.right + follow

        if abs(self.mid - self.last_mid) > reasonrange:  # 如果中点与上次中点差距过大
            self.mid = self.last_mid # 强制令中点为上次中点
        self.last_mid = self.mid  # 更新上次中点
        return self.mid  # 返回中点
    def midpoint_invalid(self, searchgap, value):
        if self.last_mid > 64:
            for i in range(self.last_mid - searchgap, 1, -1):  # 用差比和公式判断是否找到边线
                if (abs(self.data[i+4]-self.data[i])*100/(self.data[i + 4]+self.data[i]+2)) > value:
                    self.right = i
                    break
                elif i == 1:  # 如果找到1都没找到
                    self.right = 0
                    break
            for i in range(self.right - searchgap, 1, -1):  # 用差比和公式判断是否找到边线
                if (abs(self.data[i+4]-self.data[i])*100/(self.data[i + 4]+self.data[i]+2)) > value:
                    self.left = i  # 左边点找到
                    break
                elif i == 1:  # 如果找到1都没找到
                    self.left = 0  # 强制令左边点为0
                    break
        elif self.last_mid < 64:
            for i in range(self.last_mid + searchgap, 126):
                if (abs(self.data[i-4]-self.data[i])*100/(self.data[i-4]+self.data[i]+1)) > value:
                    self.left = i  # 左边点找到
                    break
                elif i == 126:  # 如果找到126都没找到
                    self.left = 127  # 强制左边点为127
                    break
            for i in range(self.left + searchgap, 126):
                if (abs(self.data[i-4]-self.data[i])*100/(self.data[i-4]+self.data[i]+1)) > value:
                    self.right = i
                    break
                elif i == 126:  # 如果找到126都没找到
                    self.right = 127  # 强制右边点为127
                    break

    def search(self, searchgap, value):
        for i in range(self.last_mid - 4 - searchgap, 1, -1):  # 用差比和公式判断是否找到边线
            if (abs(self.data[i+4]-self.data[i])*100/(self.data[i + 4]+self.data[i]+2)) > value:
                self.left = i  # 左边点找到
                break
            elif i == 1:  # 如果找到1都没找到
                self.left = 0  # 强制令左边点为0
                break

        # 搜索右边点，以上次中点作为这次的起搜点
        for i in range(self.last_mid + 4 + searchgap, 126):  # 注意这里应该是128，因为索引是从0开始的
            if (abs(self.data[i-4]-self.data[i])*100/(self.data[i-4]+self.data[i]+1)) > value:  # 判断是否与右边点一致
                self.right = i  # 右边点找到
                break
            elif i == 126:  # 如果找到126都没找到
                self.right = 127  # 强制右左边点为127
                break
        
ccd_near = CCDHandler(0)
ccd_far=CCDHandler(1)


# 赛道元素状态枚举
class RoadElement:
    stop = -1
    normal = 0
    l1 = 1
    l2 = 2
    r1 = 3
    r2 = 4
    l3_not = 12
    r3_not = 13
    l3 = 5
    r3 = 6
    lin = 7
    lout = 8
    rin = 9
    rout = 10
    zebra = 11

class ElementDetector:
    """赛道元素检测器"""
    def __init__(self):
        self.state = RoadElement.normal
        self.ring_progress = 0  # 圆环进度
        self.zebra_count = 0    # 斑马线特征计数
        self._ccd_far = ccd_far
        self._ccd_near = ccd_near
        self.imu_data = [0] * 9
        self.enc_data = 0
        self.follow = 0
        self.far_or_near = True
        self.tmperror=0   #误差缓存
        self.outflag=0

        # 常量定义（根据实际赛道调整）
        self.ccd_near_l = [30, 42]       # 左圆环阶段1近端CCD左右边点范围
        self.ccd_near_r = [90, 108]
        self.ccd_near_l_lost = 10
        self.ccd_near_r_lost = 115

        self.ccd_far_right = [88, 101]        # 远端CCD右边点范围
        self.ccd_far_left = [30, 42]         # 远端CCD左边点范围
        self.ccd_far_l = [0, 0]              # 新增：远端CCD左边点调试用范围
        self.ccd_far_l_lost = 7                 # 远端CCD左丢线阈值
        self.ccd_far_r_lost = 120               # 远端CCD右丢线阈值

        self.POINT_diff_data = 12             # 特征点差异阈值

        self.GYRO_Z_ring3_data = 0.8
        self.DISTANCE_ring3_data = 0.1
        self.GYRO_Z_ring_in_data = 40
        self.DISTANCE_ring_out_data = 0.15

        self.ccd_near_length=50 #待测
        self.ccd_far_length=50 #待测

        self.DISTANCE_ring3_not_data = 10
        #-------------------我们的gyro圆环识别数据-------------------
        self.gyro_z_ring3=0.8  #待测
        self.gyro_z_ring4=1.0  #待测
        #-----------------------------------------------------------
    def debug(self):
        temp_ccd_near_data_l = 0
        temp_ccd_near_data_r = 0
        temp_ccd_far_data_l = 0
        temp_ccd_far_data_r = 0
        for _ in range(10):
            temp_ccd_near_data_l += ccd_near.update()
            temp_ccd_near_data_r += ccd_near.update()
            temp_ccd_far_data_l += ccd_far.update()
            temp_ccd_far_data_r += ccd_far.update()
        delta = 7
        self.ccd_near_l[0] = temp_ccd_near_data_l // 10 - delta
        self.ccd_near_l[1] = temp_ccd_near_data_l // 10 + delta
        self.ccd_near_r[0] = temp_ccd_near_data_r // 10 - delta
        self.ccd_near_r[1] = temp_ccd_near_data_r // 10 + delta
        self.ccd_near_length = abs(self.ccd_near_l[1] - self.ccd_near_l[0])
        self.ccd_far_l[0] = temp_ccd_far_data_l // 10 - delta
        self.ccd_far_l[1] = temp_ccd_far_data_l // 10 + delta
        self.ccd_far_right[0] = temp_ccd_far_data_r // 10 - delta
        self.ccd_far_right[1] = temp_ccd_far_data_r // 10 + delta
        self.ccd_far_length = abs(self.ccd_far_l[1] - self.ccd_far_l[0])

    def update(self):
        """主检测函数: , imu_data, enc_data """
        tempcheck = self.state
        # 判断全黑全白
        if check_tuple(self._ccd_near.data, 100, 20)==-1:
            self.state = RoadElement.stop # 跑出去了,别把车子撞坏了,歇歇吧
        # if self._check_zebra(self._ccd_near):
        #     element = RoadElement.zebra
        if self._left_1( ):
            self.state = RoadElement.l1
        if self._right_1( ):
            self.state = RoadElement.r1
        if self.state == RoadElement.l1:
            if self._left_2( ):
                self.state = RoadElement.l2
                self.follow = -self.ccd_near_length
        if self.state == RoadElement.r1:
            if self._right_2( ):
                self.state = RoadElement.r2
                self.follow = self.ccd_near_length

        # 防误判圆环
        if self.state == RoadElement.r2:
            if movementtype.mode == MOVEMENTTYPE.Mode_1:
                if self._right_3_not():
                    self.state = RoadElement.r3_not
                    self.follow = self.ccd_near_length
            if movementtype.mode == MOVEMENTTYPE.Mode_2:
                if self._right_3():
                    self.state = RoadElement.r3
                    self.follow = -self.ccd_near_length
        if self.state == RoadElement.l2:
            if movementtype.mode == MOVEMENTTYPE.Mode_1:
                if self._left_3_not():
                    self.state = RoadElement.l3_not
                    self.follow = self.ccd_near_length
            if movementtype.mode == MOVEMENTTYPE.Mode_2:
                if self._left_3():
                    self.state = RoadElement.l3
                    self.follow = -self.ccd_near_length

        # 圆环内部
        if self.state == RoadElement.r3:
            if self._right_in():
                self.state = RoadElement.rin
                self.follow = self.ccd_near_length
        if self.state == RoadElement.l3:
            if self._left_in():
                self.state = RoadElement.lin
                self.follow = -self.ccd_near_length

        # 出圆环
        if self.state == RoadElement.rin:
            if self._right_out():
                self.state = RoadElement.rout
        if self.state == RoadElement.lin:
            if self._left_out():
                self.state = RoadElement.lout
        
        # if tempcheck != self.state:
        #     beep.start('short')
        return self.state
    
    def _left_1(self):
        """左圆环检测逻辑"""
        # 近端CCD特征检查   近端左右边线都正常
        near_valid = (self.ccd_near_l[0] <=  self._ccd_near.left <= self.ccd_near_l[1] and 
                     self.ccd_near_r[0] <=  self._ccd_near.right <= self.ccd_near_r[1])
        
        # 远端CCD特征检查   远端左丢线，右边正常
        far_valid = (self._ccd_far.left < self.ccd_far_l_lost and 
                    self.ccd_far_right[0] <= self._ccd_far.right <= self.ccd_far_right[1])
        
        # 特征点一致性检查  检测远近端右边线是否为直线，防止误判左弯道
        point_diff = abs(self._ccd_far.right -  self._ccd_near.right)
        
        return near_valid and far_valid and (point_diff <= self.POINT_diff_data)
    
    def _right_1(self):
        """右圆环检测逻辑"""
        # 近端CCD特征检查（左右镜像）   近端左右边线都正常
        near_valid = (self.ccd_near_r[0] <=  self._ccd_near.left <= self.ccd_near_r[1] and 
                    self.ccd_near_l[0] <=  self._ccd_near.right <= self.ccd_near_l[1])
        
        # 远端CCD特征检查（左右镜像）   远端右丢线，左边正常
        far_valid = (self._ccd_far.right > self.ccd_far_r_lost and 
                    self.ccd_far_left[0] <= self._ccd_far.left <= self.ccd_far_left[1])
        
        # 特征点一致性检查（比较左边缘） 远近端左边线检测是否为直线，防止误判右弯道
        point_diff = abs(self._ccd_far.left - self._ccd_near.left)
        
        return near_valid and far_valid and (point_diff <= self.POINT_diff_data)
            

    def _left_2(self):
        """左圆环状态2检测：近端左丢线+特征点稳定"""
        # 近端CCD左丢线检查（left_point_2 <=10）
        near_left_lost =  self._ccd_near.left <= self.ccd_near_l_lost
        
        # 近端右边界有效性检查（87 <= right_point_2 <=103）
        near_right_valid = self.ccd_near_r[0] <=  self._ccd_near.right <= self.ccd_near_r[1]
        
        # 特征点稳定性检查（|right_point_1 - right_point_2| <=12）
        point_diff = abs(self._ccd_far.right -  self._ccd_near.right)

        return near_left_lost and near_right_valid and (point_diff <= self.POINT_diff_data)

    def _right_2(self):
        """右圆环状态2检测：近端右丢线+特征点稳定"""
        # 近端CCD右丢线检查（right_point_2 >=115）
        near_right_lost =  self._ccd_near.right >= self.ccd_near_r_lost
        
        # 近端左边界有效性检查（31 <= left_point_2 <=44）
        near_left_valid = self.ccd_near_l[0] <=  self._ccd_near.left <= self.ccd_near_l[1]
        
        # 特征点稳定性检查（|left_point_1 - left_point_2| <=12）
        point_diff = abs(self._ccd_far.left -  self._ccd_near.left)
        
        return near_right_lost and near_left_valid and (point_diff <= self.POINT_diff_data)
  
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

    def _right_3(self):
        gyro_z.start()
        distance.start()
        if -self.GYRO_Z_ring3_data < gyro_z.data < self.GYRO_Z_ring3_data:
            if distance.data > self.DISTANCE_ring3_data:
                gyro_z.reset()
                distance.reset()
                return True
        if gyro_z.data > self.GYRO_Z_ring3_data or gyro_z.data < -self.GYRO_Z_ring3_data:
            self.state = RoadElement.normal
            gyro_z.reset()
            distance.reset()        
    
    def _right_3_not(self):
        distance.start()
        # 近端CCD右丢线检查（right_point_2 >=115）
        near_right_lost =  self._ccd_near.right >= self.ccd_near_r_lost
        
        # 近端左边界有效性检查（31 <= left_point_2 <=44）
        near_left_valid = self.ccd_near_l[0] <=  self._ccd_near.left <= self.ccd_near_l[1]
        
        # 特征点稳定性检查（|left_point_1 - left_point_2| <=12）
        point_diff = abs(self._ccd_far.left -  self._ccd_near.left)
        
        if near_right_lost and near_left_valid and (point_diff <= self.POINT_diff_data) and \
            distance.data > self.DISTANCE_ring3_not_data:
            self.state = RoadElement.normal
            distance.reset()

    def _right_in(self):
        gyro_z.start()
        if gyro_z.data > self.GYRO_Z_ring_in_data or gyro_z.data < -self.GYRO_Z_ring_in_data:
            return True

    def _right_out(self):
        distance.start()
        if distance.data > self.DISTANCE_ring_out_data or distance.data < -self.DISTANCE_ring_out_data:
            self.state = RoadElement.normal
            distance.reset()

    def _left_3(self):
        gyro_z.start()
        distance.start()
        if -self.GYRO_Z_ring3_data < gyro_z.data < self.GYRO_Z_ring3_data:
            if distance.data < -self.DISTANCE_ring3_data:  # 距离方向取反
                gyro_z.reset()
                distance.reset()
                return True
        if gyro_z.data > self.GYRO_Z_ring3_data or gyro_z.data < -self.GYRO_Z_ring3_data:
            self.state = RoadElement.normal
            gyro_z.reset()
            distance.reset()

    def _left_3_not(self):
        distance.start()
        # 近端CCD左丢线检查（left_point_2 <= ...）
        near_left_lost =  self._ccd_near.left <= self.ccd_near_l_lost
        
        # 近端右边界有效性检查（右边界范围检查）
        near_right_valid = self.ccd_near_r[0] <=  self._ccd_near.right <= self.ccd_near_r[1]
        
        # 特征点稳定性检查（右远和右近的差值）
        point_diff = abs(self._ccd_far.right -  self._ccd_near.right)
        
        if near_left_lost and near_right_valid and (point_diff <= self.POINT_diff_data) and \
            distance.data > self.DISTANCE_ring3_not_data:
            self.state = RoadElement.normal
            distance.reset()

    def _left_in(self):
        gyro_z.start()
        # 陀螺仪极性取反（原右转检测正方向，左转检测负方向）
        if gyro_z.data < -self.GYRO_Z_ring_in_data or gyro_z.data > self.GYRO_Z_ring_in_data:
            return True

    def _left_out(self):
        distance.start()
        # 距离方向取反（原检测正方向，镜像后检测负方向）
        if distance.data < -self.DISTANCE_ring_out_data or distance.data > self.DISTANCE_ring_out_data:
            self.state = RoadElement.normal
            distance.reset()


#-----------------------------------我们自己的圆环识别-------------------------------------------------
# ---------------------------------------------------------------------------------------------------
        
# 之后改为update形式，不在函数里面做改动
    def _c_ring_left_1(self):  # 进入圆环，左边线丢失，右边线正常
        # 近端CCD左丢线检查（left_point_2 <=10）
        near_left_lost= self._ccd_near.left<= self.ccd_near_l[0]
        # 近端右边界有效性检查（87 <= right_point_2 <=103）
        near_right_valid= self.ccd_near_r[0] <=  self._ccd_near.right <= self.ccd_near_r[1]
        # 特征点稳定性检查（|right_point_1 - right_point_2| <=12）
        point_diff = abs(self._ccd_far.right -  self._ccd_near.right)
        gyro_z.start()
        if near_left_lost and near_right_valid and (point_diff <= self.POINT_diff_data):
            return True
        
    def _c_ring_right_1(self): # 进入圆环，右边线丢失，左边线正常
        # 近端CCD右丢线检查（right_point_2 >=115）
        near_right_lost= self._ccd_near.right>= self.ccd_near_r[1]
        # 近端左边界有效性检查（31 <= left_point_2 <=44）
        near_left_valid= self.ccd_near_l[0] <=  self._ccd_near.left <= self.ccd_near_l[1]
        # 特征点稳定性检查（|left_point_1 - left_point_2| <=12）
        point_diff = abs(self._ccd_far.left -  self._ccd_near.left)
        gyro_z.start()
        if near_right_lost and near_left_valid and (point_diff <= self.POINT_diff_data):
            return True
        
    def _c_ring_left_2(self):   #进入圆环，左边线丢失，用右边线补左边线，并开始积累误差
        # 近端CCD左丢线检查（left_point_2 <=10）
        near_left_lost= self._ccd_near.left<= self.ccd_near_l[0]
        gyro_z.update(imu.read()[5], 0.002)
        if near_left_lost :
            ccd_near.left=ccd_near.right-self.ccd_near_length
            stage_error.get_tmp()

    def _c_ring_right_2(self):
        near_right_lost= self._ccd_near.right>= self.ccd_near_r[1]
        gyro_z.update(imu.read()[5], 0.002)
        if near_right_lost :
            ccd_near.right=ccd_near.left+self.ccd_near_length
            stage_error.get_tmp()        

    def _c_ring_left_3(self):  #出环时右丢线，采用环内的平均error值完成转向，直到变为直路状态。
        near_right_lost= self._ccd_near.right>= self.ccd_near_r[1]
        gyro_z.update(imu.read()[5], 0.002)
        if near_right_lost and abs(gyro_z.data) > self.gyro_z_ring3:
            ccd_near.error=stage_error.tmperror
            stage_error.reset()
        
    def _c_ring_right_3(self):
        near_left_lost= self._ccd_near.left<= self.ccd_near_l[0]
        gyro_z.update(imu.read()[5], 0.002)
        if near_left_lost and abs(gyro_z.data) > self.gyro_z_ring3:
            ccd_near.error=stage_error.tmperror
            stage_error.reset()
        
    def _c_ring_left_4(self):    #出环变为正常状态，右边线补左边线
        gyro_z.update(imu.read()[5], 0.002)
        point_diff = abs(self._ccd_far.right -  self._ccd_near.right)
        if abs(gyro_z.data)>self.gyro_z_ring4 and point_diff:
            ccd_near.left=ccd_near.right-self.ccd_near_length
            self.outflag= RoadElement.lout

    def _c_ring_right_4(self):
        gyro_z.update(imu.read()[5], 0.002)
        point_diff = abs(self._ccd_far.left -  self._ccd_near.left)
        if abs(gyro_z.data)>self.gyro_z_ring4 and point_diff:
            ccd_near.right=ccd_near.left+self.ccd_near_length
            self.outflag=RoadElement.rout
        
    def _c_ring_out(self):
        gyro_z.reset()
        near_valid = (self.ccd_near_r[0] <=  self._ccd_near.left <= self.ccd_near_r[1] and 
                    self.ccd_near_l[0] <=  self._ccd_near.right <= self.ccd_near_l[1])
        if self.outflag and near_valid:
            self.outflag=0

    # 十字判断
    def _crossroad(self):
        near_ccd_lost= (ccd_near.left <= self.ccd_near_l[0] and ccd_near.right >= self.ccd_near_r[1])
        far_ccd_normal=(self.ccd_far_left[0]<=ccd_far.left<=self.ccd_far_left[1] and self.ccd_far_right[0]<=ccd_far.right<=self.ccd_far_right[1])
        return near_ccd_lost and far_ccd_normal
        
    # def _update_state(self, element, imu_data):
    #     """状态机更新"""
    #     if element == RoadElement.circle_l1:
    #         if self.state != RoadElement.circle_l2:
    #             self.ring_progress = 0
    #             self.state = RoadElement.circle_l1
                
    #     elif element == RoadElement.circle_l2:
    #         self.ring_progress += abs(imu_data[5]) * 0.002  # 积分角速度计算进度
    #         if self.ring_progress >= 360:  # 完成一圈
    #             self.state = RoadElement.normal
                
    #     # 其他状态更新...
elementdetector = ElementDetector()
class Distance:
    """行驶距离"""
    def __init__(self):
        self.start_flag = False
        self.data = 0
    def start(self):
        self.start_flag = True
    def update(self, tmpdata, delta_t):
        if self.start_flag:
            self.data += tmpdata / 1024 * 30 / 50 * 0.05 * 3.1415926 * delta_t
    def reset(self):
        self.data = 0
        self.start_flag = False
distance = Distance()

class Gyro_Z_Test:
    """陀螺仪Z轴积分"""
    def __init__(self):
        self.start_flag = False
        self.offset = [0.0] * 9
        self.data = 0.0
        self._getoffset()
    def _getoffset(self, num = 100):
        for _ in range(num):
            imu_data = imu.read()
            for i in range(9):
                self.offset[i] += imu_data[i]
        for i in range(9):
            self.offset[i] /= num
    def start(self):
        self.start_flag = True
    def update(self, tmpdata, delta_t, channel = 5):
        if self.start_flag:
            self.data += (tmpdata - self.offset[channel]) * delta_t
    def reset(self):
        self.data = 0
        self.start_flag = False
gyro_z = Gyro_Z_Test()

class Error_test:
    #求误差平均值
    def __init__(self):
        self.data=0.0
    def get_tmp(self):
        for _ in range(100):
            ccd_temp_data = ccd.get(0)
            tmp_mid = ccd_near.get_mid_point(value =31, reasonrange = 30, follow = 0, searchgap = 0)
            tmp_error=tmp_mid - 64
            self.data +=tmp_error
        self.tmperror=self.data/100
    def reset(self):
        self.data=0.0
stage_error=Error_test()