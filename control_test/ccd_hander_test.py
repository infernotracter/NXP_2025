from basic_data import *


def check_tuple(data, count_up, count_down):
    """统计超过count_up和低于count_down的元素数量，根据阈值返回状态"""
    OVER_THRESHOLD = 110
    over_count = sum(num > count_up for num in data)
    under_count = sum(num < count_down for num in data)
    
    if over_count >= OVER_THRESHOLD:
        return 1
    if under_count >= OVER_THRESHOLD:
        return -1
    return 0

class CCDHandler:
    def __init__(self, channel):
        self.data = [0] * 128
        self.last_mid = 64
        self.mid = 64
        self.left = 0
        self.right = 127
        self.channel = channel
        self.error = 0
        self.follow = 0

    def update(self):
        """更新CCD传感器数据"""
        self.data = ccd.get(self.channel)
        return self.data

    def get_threshold(self):
        """计算动态阈值"""
        relevant_data = self.data[4:123]  # 包含索引4到122
        return (max(relevant_data) + min(relevant_data)) // 2

    def get_mid_point(self, value, reasonrange, follow=0, searchgap=0):
        """获取赛道中线坐标及边界"""
        if self.follow != 0:
            self.follow = follow
        self.data = ccd.get(self.channel)  # 获取最新数据
        
        # 当上次中点无效时进行边界搜索
        if self.data[self.last_mid] < self.get_threshold():
            self._handle_invalid_midpoint(searchgap, value)
            self.mid = min(max(self.mid, 5), 122)
            return self.mid
        
        # 常规边界搜索
        self._search_boundaries(searchgap, value)
        
        # 计算中线并应用跟随偏移
        self.mid = (self.left + self.right) // 2
        self._apply_follow_offset()
        
        # 限制中线变化幅度
        self._limit_mid_change(reasonrange)
        
        # 确保中线在有效范围内
        self.mid = min(max(self.mid, 5), 122)
        return self.mid
    
    def read_mid_point(self, value, reasonrange, follow=0, searchgap=0):
        """获取赛道中线坐标及边界, get改成read"""  
        if self.follow != 0:
            self.follow = follow
        self.data=ccd.read(self.channel)
        # 当上次中点无效时进行边界搜索
        if self.data[self.last_mid] < self.get_threshold():
            self._handle_invalid_midpoint(searchgap, value)
            self.mid = min(max(self.mid, 5), 122)
            return self.mid
        
        # 常规边界搜索
        self._search_boundaries(searchgap, value)
        
        # 计算中线并应用跟随偏移
        self.mid = (self.left + self.right) // 2
        self._apply_follow_offset()
        
        # 限制中线变化幅度
        self._limit_mid_change(reasonrange)
        
        # 确保中线在有效范围内
        self.mid = min(max(self.mid, 5), 122)
        return self.mid

    def _handle_invalid_midpoint(self, search_gap, edge_ratio):
        """处理无效中线时的边界搜索"""
        if self.last_mid > 64:
            self.right = self._search_edge(self.last_mid - search_gap, 0, -1, 4, edge_ratio, 0)
            self.left = self._search_edge(self.right - search_gap, 0, -1, 4, edge_ratio, 0)
        else:
            self.left = self._search_edge(self.last_mid + search_gap, 126, 1, -4, edge_ratio, 127)
            self.right = self._search_edge(self.left + search_gap, 126, 1, -4, edge_ratio, 127)

    def _search_boundaries(self, search_gap, edge_ratio):
        """常规边界搜索逻辑"""
        self.left = self._search_edge(
            self.last_mid - 4 - search_gap, 0, -1, 4, edge_ratio, 0,
        )
        self.right = self._search_edge(
            self.last_mid + 4 + search_gap, 126, 1, -4, edge_ratio, 127,
        )

    def _search_edge(self, start, end, step, offset, ratio_thresh, default):
        """通用边界搜索函数, offset正负需要判断"""
        for i in range(start, end, step):
            # 边界检查
            if not (0 <= i + offset <= 127):
                continue
                
            # 计算差比和
            diff = abs(self.data[i + offset] - self.data[i])
            sum_ = self.data[i + offset] + self.data[i] + 1
            if sum_ == 0:
                continue
                
            if (diff * 100 / sum_) > ratio_thresh:
                return i
        return default

    def _apply_follow_offset(self):    #大于零跟左线，小于零跟右线
        """应用跟随偏移调整边界"""
        if self.follow > 0:
            self.right = self.left + self.follow
        elif self.follow < 0:
            self.left = self.right + self.follow

    def _limit_mid_change(self, max_change):
        """限制中线位置突变"""
        if abs(self.mid - self.last_mid) > max_change:
            self.mid = self.last_mid
        self.last_mid = self.mid
        
ccd_near = CCDHandler(1)
ccd_far=CCDHandler(0)

# 赛道元素状态枚举
class RoadElement:
    stop = -1
    normal = 0
    l1 = 1
    l2 = 2
    r1 = 3
    r2 = 4
    l3_not = 14
    r3_not = 15
    l3 = 5
    r3 = 6
    lin = 7
    lout = 8
    loutcoming = 88
    rin = 9
    rout = 10
    routcoming = 89
    zebrain = 11
    zebraout = 111
    ramp = 12
    barrier = 13

class CCD_Controller:
    """CCD控制器, 远近端, follow跟随偏移"""
    def __init__(self):
        self._ccd_far = ccd_far
        self._ccd_near = ccd_near
        self.error = 0
        self.last_error = 0
        self.far = False  # 是否使用远端CCD
        self.fix_error_value = 0  # 是否固定error值 出圆环时需要
        self.follow = 0
    def get_error(self):
        """获取CCD误差"""
        ccd_far.get_mid_point(value=31, reasonrange=128, follow=self.follow, searchgap=0)
        ccd_near.get_mid_point(value=31, reasonrange=128, follow=self.follow, searchgap=0)
        if self.fix_error_value != 0:
            return self.fix_error_value   #直接返回
        
        if self.far:
            self.error = ccd_far.mid - 64
        else:
            self.error = ccd_near.mid - 64
            
        return self.error
ccd_controller = CCD_Controller()

class ElementDetector:
    """赛道元素检测器"""
    def __init__(self):
        self.prev_state = RoadElement.normal
        self.state = RoadElement.normal
        self.ring_progress = 0  # 圆环进度
        self.zebra_count = 1    # 斑马线有几次是直接过
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
        self.ccd_near_l_lost = 7
        self.ccd_near_r_lost = 120

        self.ccd_far_r = [88, 101]        # 远端CCD右边点范围
        self.ccd_far_l = [30, 42]         # 远端CCD左边点范围
        self.ccd_far_l_lost = 7                 # 远端CCD左丢线阈值
        self.ccd_far_r_lost = 120               # 远端CCD右丢线阈值

        self.POINT_diff_data = 20             # 特征点差异阈值

        self.DISTANCE_ring_2_data = 140

        # l3
        self.GYRO_Z_ring3_data = 125
        self.DISTANCE_ring3_data = 700

        # lin
        self.GYRO_Z_ring_in_data = 1600
        self.DISTANCE_ring_in_data = 120

        self.DISTANCE_ring_out_data = 0.15
        self.DISTANCE_ring_out_out_data = 170
        self.ccd_near_length = 80
        self.ccd_far_length = 40
        self.DISTANCE_ring_outcoming_data = 300
        self.DISTANCE_ring3_not_data = 300
        self.DISTANCE_zebra_out_data = 80 # 斑马线
        self.ERROR_l_out_value = -20
        #-------------------我们的gyro圆环识别数据-------------------
        self.gyro_z_ring3=0.8  #待测
        self.gyro_z_ring4=1.0  #待测

        #-----------------------------------------------------------

        #------------------------避障需要的数据----------------------
        self.mid=ccd_near.mid
        self.left=ccd_near.left
        self.right=ccd_near.right
        self.last_lenth=40   #待测，估计值
        self.lenth=self.right-self.left
        #-----------------------------------------------------------

    def debug(self):
        """纠正CCD和陀螺仪数据"""
        temp_ccd_near_data_l = 0
        temp_ccd_near_data_r = 0
        temp_ccd_far_data_l = 0
        temp_ccd_far_data_r = 0
        for _ in range(10):
            ccd_near.update()
            ccd_far.update()
            temp_ccd_near_data_l += ccd_near.left
            temp_ccd_near_data_r += ccd_near.right
            temp_ccd_far_data_l += ccd_far.left
            temp_ccd_far_data_r += ccd_far.right
        delta = 8
        self.ccd_near_l[0] = temp_ccd_near_data_l // 10 - delta
        self.ccd_near_l[1] = temp_ccd_near_data_l // 10 + delta
        self.ccd_near_r[0] = temp_ccd_near_data_r // 10 - delta
        self.ccd_near_r[1] = temp_ccd_near_data_r // 10 + delta
        self.ccd_near_length = abs(self.ccd_near_l[1] - self.ccd_near_l[0])
        self.ccd_far_l[0] = temp_ccd_far_data_l // 10 - delta
        self.ccd_far_l[1] = temp_ccd_far_data_l // 10 + delta
        self.ccd_far_r[0] = temp_ccd_far_data_r // 10 - delta
        self.ccd_far_r[1] = temp_ccd_far_data_r // 10 + delta
        self.ccd_far_length = abs(self.ccd_far_l[1] - self.ccd_far_l[0])
        self.POINT_diff_data = temp_ccd_near_data_l // 10 + delta

    def update(self):
        """主检测函数: , imu_data, enc_data """
        tempcheck = self.state

#         if self.find_barrier() :
#             self.state = RoadElement.barrier
#             if self.find_barrier() == 1:
#                 self.follow = -self.ccd_near_length
#             elif self.find_barrier() == -1:
#                 self.follow = self.ccd_near_length
        # 判断全黑全白
        #if check_tuple(self._ccd_near.data, 90, 30)==-1:
            #self.state = RoadElement.stop # 跑出去了,别把车子撞坏了,歇歇吧
        # if self.state == RoadElement.stop:# 返回正常状态
        #     if self._check_normal() and movementtype.mode == MOVEMENTTYPE.default:
        #         self.state = RoadElement.normal

        # if self._check_zebra(self._ccd_near):
        #     self.state = RoadElement.zebrain
        #     movementtype.speed=0
        # if self.state == RoadElement.zebrain:
        #     if self._check_zebra_out():
        #         self.zebra_count -= 1
        #         self.state = RoadElement.normal
        #         if self.zebra_count < 0:
        #             self.state = RoadElement.stop # 完赛啦
        
        # if self._check_normal():
        #         self.state = RoadElement.normal

        if self.state == RoadElement.normal:
            if self._left_1( ):
                self.state = RoadElement.l1
        if self.state == RoadElement.normal:
            if self._right_1( ):
                self.state = RoadElement.r1
        if self.state == RoadElement.l1:
            if self._left_2( ):
                self.state = RoadElement.l2
        if self.state == RoadElement.r1:
            if self._right_2( ):
                self.state = RoadElement.r2

        # 防误判圆环 important
        if self.state == RoadElement.r2:
            if self._right_3():
                self.state = RoadElement.r3
        if self.state == RoadElement.l2:
            if self._left_3():
                self.state = RoadElement.l3

        # 进圆环 
        """分为进和不进"""
        if self.state == RoadElement.r3:
            if movementtype.mode == MOVEMENTTYPE.Mode_1:
                if self._right_in_not():
                    self.state = RoadElement.normal
            if movementtype.mode == MOVEMENTTYPE.Mode_2:
                if self._right_in():
                    self.state = RoadElement.rin

        if self.state == RoadElement.l3:
            if movementtype.mode == MOVEMENTTYPE.Mode_1:
                if self._left_in_not():
                    self.state = RoadElement.normal
            if movementtype.mode == MOVEMENTTYPE.Mode_2:
                if self._left_in():
                    self.state = RoadElement.lin

        # 出圆环
        if self.state == RoadElement.rin:
            if self._right_outcoming():
                self.state = RoadElement.routcoming
        if self.state == RoadElement.lin:
            if self._left_outcoming():
                self.state = RoadElement.loutcoming

        if self.state == RoadElement.routcoming:
            if self._right_out():
                self.state = RoadElement.rout
        if self.state == RoadElement.loutcoming:
            if self._left_out():
                self.state = RoadElement.lout

        if self.state == RoadElement.lout:
            if self._left_out_out():
                self.state = RoadElement.normal
        self._element_operations()  # 执行元素状态相关操作
        # if tempcheck != self.state:
        #     beep.start('short')
        return self.state
    
    def _element_operations(self):
        # 如果状态没有变化,直接返回,降低时间复杂度
        if self.prev_state == self.state:
            return
        # 状态变化
        # 防止上次的循迹状态（error, follow）影响当前状态
        ccd_controller.fix_error_value = 0
        ccd_controller.follow = 0
        ccd_controller.far = False
        element_gyro.clear()
        element_distance.clear()
        element_gyro.start()
        element_distance.start()
        if self.state == RoadElement.stop:  # 停止状态
            movementtype.speed = 0          
        if self.state == RoadElement.zebrain:
            ccd_controller.far = True
        if self.state == RoadElement.normal: # 正常状态
            ccd_controller.fix_error_value = 0
            ccd_controller.follow = 0
            # element_gyro.off()
            # element_distance.off()
        if self.state == RoadElement.l1:    #跟右边线
            ccd_controller.follow=-self.ccd_near_length
        if self.state == RoadElement.l2:
            ccd_controller.follow = -self.ccd_near_length

        if self.state == RoadElement.l3:
            ccd_controller.follow = -self.ccd_near_length
        if self.state == RoadElement.l3_not:
            ccd_controller.follow = -self.ccd_near_length

        if self.state == RoadElement.lin:
            ccd_controller.follow = self.ccd_near_length
            # self.tmperror=stage_error.get_tmp()

        if self.state == RoadElement.loutcoming:
            ccd_controller.follow = 0

        if self.state == RoadElement.lout:
            ccd_controller.fix_error_value = self.ERROR_l_out_value

        self.prev_state=self.state

    def _left_1(self):
        """左圆环检测逻辑"""
        if self._ccd_far.left < self.ccd_far_l_lost:
            if abs(self._ccd_far.right -  self._ccd_near.right) <= self.POINT_diff_data:
                return True
    
    def _right_1(self):
        """右圆环检测逻辑"""
        if self._ccd_far.right > self.ccd_far_r_lost:
            if abs(self._ccd_far.left -  self._ccd_near.left) <= self.POINT_diff_data:
                return True
            
    def _left_2(self):
        """左圆环状态2检测：近端左丢线+特征点稳定"""
        if self._ccd_near.left <= self.ccd_near_l_lost:
            if abs(self._ccd_far.left - self._ccd_near.left) <= self.POINT_diff_data:
                return True

    def _right_2(self):
        """右圆环状态2检测：近端右丢线+特征点稳定"""
        return self._ccd_near.right >= self.ccd_near_r_lost
  
    def _check_zebra(self, ccd_near):
        """斑马线检测逻辑"""
        crossings = 0         # 跳变次数统计
        threshold = 31
        min_crossings = 10    #最小的斑马线检测点次数，待测
        for i in range(30, 97):    #在靠近赛道中间的部分检测，待测
            diff = abs(ccd_near.data[i] - ccd_near.data[i + 3])*100
            sum = ccd_near.data[i] + ccd_near.data[i + 3] + 1
            ratio = abs(diff / sum)
            if ratio > threshold:  # 跳变阈值
                crossings += 1
        if crossings > min_crossings:
            return True
        return False
    
    def _check_zebra_out(self):
        if element_distance.data > self.DISTANCE_zebra_out_data:
            return True
        return False
    
    def _check_normal(self):
        if self._ccd_near.left > self.ccd_near_l_lost and self._ccd_near.right < self.ccd_near_r_lost:
            return True
        return False

    def _right_3(self):
        if element_distance.data > self.DISTANCE_ring3_data:
            if -self.GYRO_Z_ring3_data < element_gyro.data < self.GYRO_Z_ring3_data:
                self.state = RoadElement.rin
                return True
            else:
                self.state = RoadElement.normal
        if abs(element_gyro.data) > self.GYRO_Z_ring3_data:
            self.state = RoadElement.normal
        return False

    def _right_in_not(self):
        # 近端CCD右丢线检查（right_point_2 >=115）
        near_right_lost =  self._ccd_near.right >= self.ccd_near_r_lost
        
        # 近端左边界有效性检查（31 <= left_point_2 <=44）
        near_left_valid = self.ccd_near_l[0] <=  self._ccd_near.left <= self.ccd_near_l[1]
        
        # 特征点稳定性检查（|left_point_1 - left_point_2| <=12）
        point_diff = abs(self._ccd_far.left -  self._ccd_near.left)
        
        if near_right_lost and near_left_valid and (point_diff <= self.POINT_diff_data) and \
            element_distance.data > self.DISTANCE_ring3_not_data:
            self.state = RoadElement.normal

    def _right_in(self):
        if abs(element_gyro.data) > self.GYRO_Z_ring_in_data:
            return True

    def _left_3(self):
        if abs(element_distance.data) > self.DISTANCE_ring3_data:  # 距离方向取反
            if -self.GYRO_Z_ring3_data < element_gyro.data < self.GYRO_Z_ring3_data:
                return True
            else:
                self.state = RoadElement.normal
        if abs(element_gyro.data) > self.GYRO_Z_ring3_data:
            self.state = RoadElement.normal
        return False

    def _left_in_not(self):
        # 近端CCD左丢线检查（left_point_2 <= ...）
        near_left_lost =  self._ccd_near.left <= self.ccd_near_l_lost
        
        # 近端右边界有效性检查（右边界范围检查）
        near_right_valid = self.ccd_near_r[0] <=  self._ccd_near.right <= self.ccd_near_r[1]
        
        # 特征点稳定性检查（右远和右近的差值）
        point_diff = abs(self._ccd_far.right -  self._ccd_near.right)
        
        if near_left_lost and near_right_valid and (point_diff <= self.POINT_diff_data) and \
            element_distance.data > self.DISTANCE_ring3_not_data:
            return True

    def _left_in(self):
        # 陀螺仪极性取反（原右转检测正方向，左转检测负方向）
        if abs(element_gyro.data) > self.GYRO_Z_ring_in_data or abs(element_distance.data) > self.DISTANCE_ring_in_data:
            return True

    def _left_outcoming(self):
        # 超过一定距离并且全白
        if abs(element_distance.data) > self.DISTANCE_ring_outcoming_data:
            if check_tuple(self._ccd_near.data, 90, 30)==1 or check_tuple(self._ccd_far.data, 90, 30)==1:
                return True

    def _right_outcoming(self):
        # 超过一定距离并且全白
        if abs(element_distance.data) > self.DISTANCE_ring_outcoming_data:
            if check_tuple(self._ccd_near.data, 90, 30)==1 or check_tuple(self._ccd_far.data, 90, 30)==1:
                return True
            
    def _left_out_out(self):
        if abs(element_distance.data) > self.DISTANCE_ring_out_out_data or (self.ccd_near_l_lost < self._ccd_near.left and self._ccd_near.right < self.ccd_near_r_lost):
            return True


    def _right_out(self):
        if abs(element_distance.data) > self.DISTANCE_ring_out_data:
            return True
            
    def _left_out(self):
        if abs(element_distance.data) > self.DISTANCE_ring_out_data:
            return True


#-----------------------------------我们自己的圆环识别-------------------------------------------------
# ---------------------------------------------------------------------------------------------------
        
# # 之后改为update形式，不在函数里面做改动
#     def _c_ring_left_1(self):  # 进入圆环，左边线丢失，右边线正常
#         # 近端CCD左丢线检查（left_point_2 <=10）
#         near_left_lost= self._ccd_near.left<= self.ccd_near_l[0]
#         # 近端右边界有效性检查（87 <= right_point_2 <=103）
#         near_right_valid= self.ccd_near_r[0] <=  self._ccd_near.right <= self.ccd_near_r[1]
#         # 特征点稳定性检查（|right_point_1 - right_point_2| <=12）
#         point_diff = abs(self._ccd_far.right -  self._ccd_near.right)
#         gyro_z.start()
#         if near_left_lost and near_right_valid and (point_diff <= self.POINT_diff_data):
#             gyro_z.reset()
#             return True
        
#     def _c_ring_right_1(self): # 进入圆环，右边线丢失，左边线正常
#         # 近端CCD右丢线检查（right_point_2 >=115）
#         near_right_lost= self._ccd_near.right>= self.ccd_near_r[1]
#         # 近端左边界有效性检查（31 <= left_point_2 <=44）
#         near_left_valid= self.ccd_near_l[0] <=  self._ccd_near.left <= self.ccd_near_l[1]
#         # 特征点稳定性检查（|left_point_1 - left_point_2| <=12）
#         point_diff = abs(self._ccd_far.left -  self._ccd_near.left)
#         gyro_z.start()
#         if near_right_lost and near_left_valid and (point_diff <= self.POINT_diff_data):
#             gyro_z.reset()
#             return True
        
#     def _c_ring_left_2(self):   #进入圆环，左边线丢失，用右边线补左边线，并开始积累误差
#         # 近端CCD左丢线检查（left_point_2 <=10）
#         near_left_lost= self._ccd_near.left<= self.ccd_near_l[0]
#         if near_left_lost :
#             ccd_near.left=ccd_near.right-self.ccd_near_length
#             stage_error.get_tmp()

#     def _c_ring_right_2(self):
#         near_right_lost= self._ccd_near.right>= self.ccd_near_r[1]
#         if near_right_lost :
#             ccd_near.right=ccd_near.left+self.ccd_near_length
#             stage_error.get_tmp()        

#     def _c_ring_left_3(self):  #出环时右丢线，采用环内的平均error值完成转向，直到变为直路状态。
#         near_right_lost= self._ccd_near.right>= self.ccd_near_r[1]
#         gyro_z.start()
#         if near_right_lost and abs(gyro_z.data) > self.gyro_z_ring3:
#             ccd_near.error=stage_error.tmperror
#             gyro_z.reset()
#             stage_error.reset()
        
#     def _c_ring_right_3(self):
#         near_left_lost= self._ccd_near.left<= self.ccd_near_l[0]
#         gyro_z.start()
#         if near_left_lost and abs(gyro_z.data) > self.gyro_z_ring3:
#             ccd_near.error=stage_error.tmperror
#             gyro_z.reset()
#             stage_error.reset()
        
#     def _c_ring_left_4(self):    #出环变为正常状态，右边线补左边线
#         gyro_z.start()
#         point_diff = abs(self._ccd_far.right -  self._ccd_near.right)
#         if abs(gyro_z.data)>self.gyro_z_ring4 and point_diff:
#             ccd_near.left=ccd_near.right-self.ccd_near_length
#             gyro_z.reset()
#             self.outflag= RoadElement.lout

#     def _c_ring_right_4(self):
#         gyro_z.start()
#         point_diff = abs(self._ccd_far.left -  self._ccd_near.left)
#         if abs(gyro_z.data)>self.gyro_z_ring4 and point_diff:
#             ccd_near.right=ccd_near.left+self.ccd_near_length
#             gyro_z.reset()
#             self.outflag=RoadElement.rout
        
#     def _c_ring_out(self):
#         near_valid = (self.ccd_near_r[0] <=  self._ccd_near.left <= self.ccd_near_r[1] and 
#                     self.ccd_near_l[0] <=  self._ccd_near.right <= self.ccd_near_l[1])
#         if self.outflag and near_valid:
#             self.outflag=0

#     def find_barrier(self):
#         """障碍物检测"""
#         self.last_lenth=self.lenth
#         self.mid,self.left,self.right=ccd_near.get_mid_point(value =31, reasonrange = 128, follow = 0, searchgap = 0)
#         self.lenth=self.right-self.left
#         widthRate = abs(self.lenth - self.last_lenth) / self.last_lenth if self.last_lenth != 0 else 0
#         threshold = 0.3
# 
#         if widthRate > threshold:
#             if self.mid < 64:
#                 return 1   #障碍物在右边，小车贴左边线
#             else:
#                 return -1  #障碍物在左边，小车贴右边线
#         else:
#             return 0      #没检测到障碍物
            

    # 十字判断
    def _crossroad(self):
        near_ccd_lost= (ccd_near.left <= self.ccd_near_l[0] and ccd_near.right >= self.ccd_near_r[1])
        far_ccd_normal=(self.ccd_far_l[0]<=ccd_far.left<=self.ccd_far_l[1] and self.ccd_far_r[0]<=ccd_far.right<=self.ccd_far_r[1])
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
        self.start()
    def start(self):
        self.start_flag = True
    
    def clear(self):
        self.data=0

    def update(self, tmpdata, k):
        if self.start_flag:
            self.data += tmpdata * k
        if self.data > 9999999999:
            self.data = 0.0
    def off(self):
        self.data = 0
        self.start_flag = False
element_distance = Distance()
debugdistance = Distance()

class Gyro_Z_Test:
    """陀螺仪Z轴积分"""
    def __init__(self):
        self.start_flag = False
        self.offset = [0.0] * 9
        self.data = 0.0
        self.start()
        # self._getoffset()
    # def _getoffset(self, num = 50):
    #     for _ in range(num):
    #         imu_data = imu.read()
    #         for i in range(6):
    #             self.offset[i] += imu_data[i]
    #     for i in range(6):
    #         self.offset[i] /= num
    def clear(self):
        self.data=0

    def start(self):
        self.start_flag = True
    def update(self, tmpdata, k):
        if self.start_flag:
            self.data += (tmpdata + 142) * k
        if self.data > 999999999999:
            self.data = 0.0
    def off(self):
        self.data = 0
        self.start_flag = False
element_gyro = Gyro_Z_Test()
debuggyroz = Gyro_Z_Test()

class Error_test:
    #求误差平均值
    def __init__(self):
        self.data=0.0
    def get_tmp(self):
        for _ in range(10):
            tmp_mid = ccd_near.read_mid_point(value =31, reasonrange = 30, follow = 0, searchgap = 0)
            tmp_error=tmp_mid - 64
            self.data +=tmp_error
        self.tmperror=self.data/10
    def reset(self):
        self.data=0.0
stage_error=Error_test()



print("王小桃快跑，邮箱来了")

