from basic_data import *
import time
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
        if  follow != 0:
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
        if self.follow != 0:
            self._apply_follow_offset()
            self.mid = (self.left + self.right) // 2
            return self.mid
        
        # 限制中线变化幅度
        self._limit_mid_change(reasonrange)
        
        # 确保中线在有效范围内
        self.mid = min(max(self.mid, 5), 122)

        self.mid = (self.left + self.right) // 2
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
# 赛道元素状态枚举
class RoadElement:
    stop = -1
    normal = 0
    l1 = 1
    l2 = 2
    r1 = 5
    r2 = 4
    l3_not = 14
    r3_not = 15
    l3 = 3
    r3 = 6
    lin = 7
    lout = 8
    loutcoming = 88
    loutout = 888
    rin = 9
    rout = 10
    routcoming = 89
    routout = 999
    zebrain = 11
    zebraout = 111
    ramp = 12
    barrier = 13
    crossroad_1 = 16
    crossroad_2 = 17
    crossroad_all_in = 18
    crossroad_4 = 19
    crossroad_5 = 20


class CCD_Controller:
    """CCD控制器, 远近端, follow跟随偏移"""
    def __init__(self):
        self.error = 0
        self.last_error = 0
        self.far = False  # 是否使用远端CCD
        self.fix_error_value = 0  # 是否固定error值 出圆环时需要
        self.follow = 0
        self.value = 31
        self.ccd_near_length = 60
        self.ccd_far_length = 60
    def get_error(self):
        """获取CCD误差"""
        ccd_far.get_mid_point(value=self.value, reasonrange=128, follow=0, searchgap=0)
        ccd_near.get_mid_point(value=self.value, reasonrange=128, follow=0, searchgap=0)
        if self.fix_error_value != 0:
            return self.fix_error_value   #直接返回
        
        if self.far:
            self.error = ccd_far.mid - 64
        else:
            self.error = ccd_near.mid - 64
        
        if self.follow > 0:
            self.error = (ccd_near.left*2 + self.follow)//2 -64
            return self.error
            
        elif self.follow < 0:
            self.error = (ccd_near.right*2 +self.follow)//2 -64
            return self.error
            
        return self.error
ccd_controller = CCD_Controller()

class ElementDetector:
    """赛道元素检测器"""
    def __init__(self):
        self.crossing=0
        self.FLAG_crossroad = True
        self.prev_state = RoadElement.normal
        self.state = RoadElement.normal
        self.ring_progress = 0  # 圆环进度
        self.zebra_count = 1    # 斑马线有几次是直接过
        self.imu_data = [0] * 9
        self.enc_data = 0
        self.follow = 0
        self.far_or_near = True
        self.tmperror=0   #误差缓存
        self.outflag=0

        # 常量定义（根据实际赛道调整）
        self.ccd_near_l = [9, 60]       # 左圆环阶段1近端CCD左右边点范围
        self.ccd_near_r = [70, 120]
        self.ccd_near_l_lost = 7
        self.ccd_near_r_lost = 120

        self.ccd_far_r = [80, 120]        # 远端CCD右边点范围
        self.ccd_far_l = [20, 50]         # 远端CCD左边点范围
        self.ccd_far_l_lost = 7                 # 远端CCD左丢线阈值
        self.ccd_far_r_lost = 120               # 远端CCD右丢线阈值

        self.DISTANCE_crossroad_all_in_data = 220

        self.POINT_diff_data = 20            # 特征点差异阈值

        self.DISTANCE_ring_2_data = 20
        self.GYRO_Z_ring2_data = 100

        # l3
        self.GYRO_Z_ring3_data = 200
        self.DISTANCE_ring3_data = 120

        # lin
        self.GYRO_Z_ring_in_data = 1700
        self.DISTANCE_ring_in_data = 150

        self.DISTANCE_ring_out_data = 70
        self.DISTANCE_ring_out_out_data = 200
        self.ccd_near_length = 60
        self.ccd_far_length = 60
        self.DISTANCE_ring_outcoming_data = 160
        self.DISTANCE_ring3_not_data = 300
        self.DISTANCE_zebra_out_data = 100      #斑马线
        self.ERROR_l_out_value = -13
        #crossroad
        self.DISTANCE_crossroad_data = 80  #十字路口

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
        if abs(element_distance.data)>300:
            self.state = RoadElement.normal
#         if self.find_barrier() :
#             self.state = RoadElement.barrier
#             if self.find_barrier() == 1:
#                 self.follow = -self.ccd_near_length
#             elif self.find_barrier() == -1:
#                 self.follow = self.ccd_near_length
        # 判断全黑全白
        # #if check_tuple(ccd_near.data, 90, 30)==-1:
        #     #self.state = RoadElement.stop # 跑出去了,别把车子撞坏了,歇歇吧
        # if self.state == RoadElement.stop:# 返回正常状态
        #     if self._check_normal() and movementtype.mode == MOVEMENTTYPE.default:
        #         self.state = RoadElement.normal

        
        # if self._check_normal():
        #         self.state = RoadElement.normal

        
        if self._check_zebra():
            self.state = RoadElement.zebrain
            element_distance.clear()
            #exit(0)
            
        if self.state == RoadElement.normal:
            if self._left_1( ):
                self.state = RoadElement.l1
            
        # 四大十字
        if self.state == RoadElement.normal:
            if self._crossroad_coming():
                self.state = RoadElement.crossroad_1
        elif self.state == RoadElement.crossroad_1:
                if self._crossroad_out():
                    self.state = RoadElement.crossroad_2
        elif self.state == RoadElement.crossroad_2:
            if self._crossroad_all_in():
                    self.state = RoadElement.crossroad_all_in
        elif self.state == RoadElement.crossroad_all_in:
            if self._crossroad_coming():
                self.state = RoadElement.crossroad_4
        elif self.state == RoadElement.crossroad_4:
            if self._crossroad_out():
                self.state = RoadElement.normal

        elif self.state == RoadElement.zebrain:
            if self._check_zebra_out():
                self.zebra_count -= 1
                self.state = RoadElement.normal
                if self.zebra_count < 0:
                    motor_l.duty(0)
                    motor_r.duty(0)
                    quit()

        elif self.state == RoadElement.l1:
            if self._crossroad_coming():
                self.state = RoadElement.crossroad_1
            elif self._left_2( ):
                self.state = RoadElement.l2

        # 防误判圆环 important
 
        elif self.state == RoadElement.l2:
            if self._left_3():
                self.state = RoadElement.l3

        # 进圆环 


        elif self.state == RoadElement.l3:
            if movementtype.mode == MOVEMENTTYPE.Mode_1:
                if self._left_in_not():
                    self.state = RoadElement.normal
            elif movementtype.mode == MOVEMENTTYPE.Mode_2:
                if self._left_in():
                    self.state = RoadElement.lin

        # 出圆环

        elif self.state == RoadElement.lin:
            if self._left_outcoming():
                self.state = RoadElement.loutcoming

        elif self.state == RoadElement.loutcoming:
            if self._left_out():
                self.state = RoadElement.lout

        elif self.state == RoadElement.lout:
            if self._left_out_out():
                self.state = RoadElement.loutout
                
        # 出圆环
        elif self.state == RoadElement.loutout:
            self.state = RoadElement.normal

        # 在update方法中添加右圆环状态转换：
        if self.state == RoadElement.normal:
            if self._right_1():
                self.state = RoadElement.r1

        elif self.state == RoadElement.r1:
            if self._crossroad_coming():
                self.state = RoadElement.normal
            elif self._right_2():
                self.state = RoadElement.r2

        elif self.state == RoadElement.r2:
            if self._right_3():
                self.state = RoadElement.r3

        elif self.state == RoadElement.r3:
            if movementtype.mode == MOVEMENTTYPE.Mode_1:
                if self._right_in_not():
                    self.state = RoadElement.normal
            elif movementtype.mode == MOVEMENTTYPE.Mode_2:
                if self._right_in():
                    self.state = RoadElement.rin

        elif self.state == RoadElement.rin:
            if self._right_outcoming():
                self.state = RoadElement.routcoming

        elif self.state == RoadElement.routcoming:
            if self._right_out():
                self.state = RoadElement.rout

        elif self.state == RoadElement.rout:
            if self._right_out_out():
                self.state = RoadElement.routout 

        # 十字
        elif self.state == RoadElement.crossroad_1:
            if self._crossroad_out():
                if self.FLAG_crossroad:
                    self.state = RoadElement.crossroad_2
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
        # if self.state ==RoadElement.normal or self.state ==RoadElement.zebrain or self.state==RoadElement.zebraout:
        #     speed_controller.target_speed=speed_controller.tmp_speed
        # else:
        #     speed_controller.target_speed=speed_controller.tmp_speed * 0.4
        ccd_controller.fix_error_value = 0
        ccd_controller.follow = 0
        ccd_controller.far = False
        element_gyro.clear()
        element_distance.clear()
        element_gyro.start()
        if self.state == RoadElement.normal: # 正常状态
            ccd_controller.fix_error_value = 0
            ccd_controller.follow = 0
            ccd_controller.far = False

        elif self.state == RoadElement.stop:  # 停止状态
            speed_controller.target_speed = 10
        elif self.state == RoadElement.zebrain:
            ccd_controller.far = True
            # element_gyro.off()
            # element_distance.off()
        # elif self.state == RoadElement.l1:    #跟右边线
        #     ccd_controller.follow=-self.ccd_near_length
        elif self.state == RoadElement.l1:
            ccd_controller.follow = -self.ccd_near_length
        elif self.state == RoadElement.l2:
            ccd_controller.follow = -self.ccd_near_length

        elif self.state == RoadElement.l3:
            ccd_controller.follow = self.ccd_near_length
        elif self.state == RoadElement.l3_not:
            ccd_controller.follow = -self.ccd_near_length

        elif self.state == RoadElement.lin:
            ccd_controller.follow = 0
            # self.tmperror=stage_error.get_tmp()

        elif self.state == RoadElement.loutcoming:
            ccd_controller.fix_error_value = self.ERROR_l_out_value

        elif self.state == RoadElement.lout:
            ccd_controller.follow = -self.ccd_near_length

        elif self.state == RoadElement.crossroad_1:
            ccd_controller.far = True
        elif self.state == RoadElement.r1:
            ccd_controller.follow = self.ccd_near_length
            
        elif self.state == RoadElement.r2:
            ccd_controller.follow = self.ccd_near_length

        elif self.state == RoadElement.r3:
            ccd_controller.follow = -self.ccd_near_length

        elif self.state == RoadElement.rin:
            ccd_controller.follow = 0

        elif self.state == RoadElement.routcoming:
            ccd_controller.fix_error_value = -self.ERROR_l_out_value  # 注意符号取反

        elif self.state == RoadElement.rout:
            ccd_controller.follow = self.ccd_near_length

        elif self.state == RoadElement.routout:
            self.state = RoadElement.normal

        self.prev_state=self.state

    def _left_1(self):
        """左圆环检测逻辑"""
        # 近端CCD特征检查   近端左右边线都正常
        near_valid = (self.ccd_near_l[0] <=  ccd_near.left <= self.ccd_near_l[1] and 
                     self.ccd_near_r[0] <=  ccd_near.right <= self.ccd_near_r[1])
        
        # 远端CCD特征检查   远端左丢线，右边正常
        far_valid = (ccd_far.left < self.ccd_far_l_lost and 
                    self.ccd_far_r[0] <= ccd_far.right <= self.ccd_far_r[1])
        
        # 特征点一致性检查  检测远近端右边线是否为直线，防止误判左弯道
        point_diff = abs(ccd_near.right -  ccd_far.right)
        
#         print("{}   {}   {}".format(near_valid  ,far_valid ,point_diff))
        return near_valid and far_valid and point_diff < self.POINT_diff_data
        #return ccd_near.left < self.ccd_far_l_lost
    
            

    def _left_2(self):
        """左圆环状态2检测：近端左丢线+特征点稳定"""
        # 近端CCD左丢线检查（left_point_2 <=10）
        near_left_lost = ccd_near.left <= self.ccd_near_l_lost
        
        # 近端右边界有效性检查（87 <= right_point_2 <=103）
        near_right_valid = self.ccd_near_r[0] <=  ccd_near.right <= self.ccd_near_r[1]
        
        # 特征点稳定性检查（|right_point_1 - right_point_2| <=12）
        point_diff = abs(ccd_far.right -  ccd_near.right)
        
#       print("{}   {}   {}   ".format(near_left_lost , near_right_valid  ,point_diff))
        if abs(element_distance.data) > self.DISTANCE_ring_2_data or abs(element_gyro.data) > self.GYRO_Z_ring2_data:
            self.state = RoadElement.normal
        return near_left_lost and near_right_valid and point_diff < self.POINT_diff_data
        # return ccd_near.left < self.ccd_near_l_lost


  
    def _check_zebra(self):
        """斑马线检测逻辑"""
        crossings = 0         # 跳变次数统计
        threshold = 31
        min_crossings = 20    #最小的斑马线检测点次数，待测
        for i in range(30, 97):    #在靠近赛道中间的部分检测，待测
            diff = abs(ccd_near.data[i] - ccd_near.data[i + 3])*100
            sum_ = ccd_near.data[i] + ccd_near.data[i + 3] + 1
            ratio = abs(diff / sum_)
            if ratio > threshold:  # 跳变阈值
                crossings += 1
        self.crossing=crossings
        if crossings > min_crossings:
            return True
        return False
    
    def _check_zebra_out(self):
        if abs(element_distance.data) > self.DISTANCE_zebra_out_data:
            return True
        return False
    
    def _check_normal(self):
        if ccd_near.left > self.ccd_near_l_lost and ccd_near.right < self.ccd_near_r_lost:
            return True
        return False


    def _left_3(self):
        """left圆环状态3检测"""
        if abs(element_distance.data) > self.DISTANCE_ring3_data:  # 距离方向取反
            if -self.GYRO_Z_ring3_data < element_gyro.data < self.GYRO_Z_ring3_data:
                if self.ccd_near_l[0] < ccd_near.left < self.ccd_near_l[1]:
                    if abs(ccd_near.right - ccd_far.right) < self.POINT_diff_data:
                        return True
                    else:
                        self.state = RoadElement.normal
        if abs(element_gyro.data) > self.GYRO_Z_ring3_data:
            self.state = RoadElement.normal
        return False

    def _left_in_not(self):
        # 近端CCD左丢线检查（left_point_2 <= ...）
        near_left_lost =  ccd_near.left <= self.ccd_near_l_lost
        
        # 近端右边界有效性检查（右边界范围检查）
        near_right_valid = self.ccd_near_r[0] <=  ccd_near.right <= self.ccd_near_r[1]
        
        # 特征点稳定性检查（右远和右近的差值）
        point_diff = abs(ccd_far.right -  ccd_near.right)
        
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
            if (ccd_near.left<self.ccd_near_l_lost and ccd_near.right>self.ccd_near_r_lost)or (ccd_far.left <self.ccd_far_l_lost and ccd_far.right >self.ccd_far_r_lost):
                return True
        if abs(element_distance.data) > self.DISTANCE_ring3_data * 3.5:
            self.state = RoadElement.normal

            
    def _left_out_out(self):
        if abs(element_distance.data) > self.DISTANCE_ring_out_out_data:
            if ( ccd_near.right < self.ccd_near_r_lost):
                return True
        if abs(element_distance.data) > self.DISTANCE_ring3_data * 1.5:
            self.state = RoadElement.normal
            
    def _left_out(self):
        if abs(element_distance.data) > self.DISTANCE_ring_out_data:
            if ccd_near.right < self.ccd_near_r_lost:
                return True
    def _right_1(self):
        """右圆环检测逻辑（对称于_left_1）"""
        # 近端CCD特征检查（左右镜像）
        near_valid = (self.ccd_near_l[0] <= ccd_near.left <= self.ccd_near_l[1] and 
                    self.ccd_near_r[0] <= ccd_near.right <= self.ccd_near_r[1])
        
        # 远端CCD特征检查（左右镜像）
        far_valid = (ccd_far.right > self.ccd_far_r_lost and 
                    self.ccd_far_l[0] <= ccd_far.left <= self.ccd_far_l[1])
        
        # 特征点一致性检查（比较左边缘）
        point_diff = abs(ccd_near.left - ccd_far.left)
        
        return near_valid and far_valid and point_diff < self.POINT_diff_data

    def _right_2(self):
        """右圆环状态2检测（对称于_left_2）"""
        # 近端CCD右丢线检查
        near_right_lost = ccd_near.right >= self.ccd_near_r_lost
        
        # 近端左边界有效性检查
        near_left_valid = self.ccd_near_l[0] <= ccd_near.left <= self.ccd_near_l[1]
        
        # 特征点稳定性检查
        point_diff = abs(ccd_far.left - ccd_near.left)
        if abs(element_distance.data) > self.DISTANCE_ring_2_data or abs(element_gyro.data) > self.GYRO_Z_ring2_data:
            self.state = RoadElement.normal
        
        return near_right_lost and near_left_valid and point_diff < self.POINT_diff_data

    def _right_3(self):
        """右圆环状态3检测（对称于_left_3）"""
        if abs(element_distance.data) > self.DISTANCE_ring3_data:
            if -self.GYRO_Z_ring3_data < element_gyro.data < self.GYRO_Z_ring3_data:
                if self.ccd_near_r[0] < ccd_near.right < self.ccd_near_r[1]:
                    if abs(ccd_near.left - ccd_far.left) < self.POINT_diff_data:
                        return True
                    else:
                        self.state = RoadElement.normal
        if abs(element_gyro.data) > self.GYRO_Z_ring3_data:
            self.state = RoadElement.normal
        return False

    def _right_in_not(self):
        """右圆环防误判（对称于_left_in_not）"""
        near_right_lost = ccd_near.right >= self.ccd_near_r_lost
        near_left_valid = self.ccd_near_l[0] <= ccd_near.left <= self.ccd_near_l[1]
        point_diff = abs(ccd_far.left - ccd_near.left)
        
        if near_right_lost and near_left_valid and point_diff <= self.POINT_diff_data and \
           element_distance.data > self.DISTANCE_ring3_not_data:
            return True

    def _right_in(self):
        """进入右圆环（对称于_left_in）"""
        if abs(element_gyro.data) > self.GYRO_Z_ring_in_data or abs(element_distance.data) > self.DISTANCE_ring_in_data:
            return True

    def _right_outcoming(self):
        """准备出右圆环（对称于_left_outcoming）"""
        if abs(element_distance.data) > self.DISTANCE_ring_outcoming_data:
            if (ccd_near.left<self.ccd_near_l_lost and ccd_near.right>self.ccd_near_r_lost)or (ccd_far.left <self.ccd_far_l_lost and ccd_far.right >self.ccd_far_r_lost):
                return True
        if abs(element_distance.data) > self.DISTANCE_ring3_data * 3.5:
            self.state = RoadElement.normal

    def _right_out(self):
        """出右圆环（对称于_left_out）"""
        if abs(element_distance.data) > self.DISTANCE_ring_out_data:
            if ccd_near.left > self.ccd_near_l_lost:
                return True

    def _right_out_out(self):
        """完全出右圆环（对称于_left_out_out）"""
        if abs(element_distance.data) > self.DISTANCE_ring_out_out_data:
            if ccd_near.left > self.ccd_near_l_lost :
                return True
        if abs(element_distance.data) > self.DISTANCE_ring3_data * 1.5:
            self.state = RoadElement.normal
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
    def _crossroad_coming(self):
        near_valid = (ccd_far.left > self.ccd_far_l_lost) and (ccd_far.right < self.ccd_far_r_lost)
        far_ccd_lost=(ccd_far.left < self.ccd_far_l_lost) and (ccd_far.right > self.ccd_far_r_lost)
        return near_valid and far_ccd_lost
    def _crossroad_out(self):
        if abs(element_distance.data) > self.DISTANCE_crossroad_data:
            return True
    def _crossroad_all_in(self):
        if abs(element_distance.data) > self.DISTANCE_crossroad_all_in_data:
            return True
        if abs(element_distance.data) > self.DISTANCE_crossroad_data + 280:
            self.state = RoadElement.normal

        
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
        if self.data > 999999:
            self.data = 0.0
    def off(self):
        self.data = 0
        self.start_flag = False
element_distance = Distance()
alldistance = Distance()
speed_slow_distance = Distance()
class Speed_controller:
    def __init__(self):
        self.target_speed=-40         #turn_out_kp=-125.73     turn_in_kp=-5.18
        self.tmp_speed=self.target_speed
        self.fast_speed=-30
        self.slow_speed=-30
        self.slower_flag = False
        self.slow_distance_threshold = 150
    def slower(self, ccd_near, ccd_far):
        """检测是否需要进入慢速模式"""
        # 检测到需要减速的条件
        if abs(ccd_near.mid - ccd_far.mid) > 15:
            # 重置距离计数器并开始记录
            speed_slow_distance.clear()
            speed_slow_distance.start()
            self.slower_flag = True
        
        # 更新目标速度
        self.target_speed = self.slow_speed if self.slower_flag else self.tmp_speed
    
    def distance_connect(self):
        """检查是否达到慢速距离阈值，如果是则恢复正常速度"""
        if self.slower_flag and speed_slow_distance.data >= self.slow_distance_threshold:
            # 距离已达到阈值，恢复正常速度
            self.slower_flag = False
            # 停止并重置距离计数器
            speed_slow_distance.off()
            
#     def update(self):
#         speed_kd=scale_value(abs(ccd_near.mid-ccd_far.mid),0,20)
#         self.target_speed=min(int(self.target_speed *speed_kd),0)
#         if abs(alldistance.data) <2000:
#             self.target_speed=self.slow_speed
#         elif abs(alldistance.data) > 8000:
#             self.target_speed=self.slow_speed
#         else:
#             self.target_speed=self.fast_speed

speed_controller=Speed_controller()


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
        if self.data > 999999999:
            self.data = 0.0
    def off(self):
        self.data = 0
        self.start_flag = False
element_gyro = Gyro_Z_Test()
debuggyroz = Gyro_Z_Test()
print("王小桃快跑，邮箱来了")

