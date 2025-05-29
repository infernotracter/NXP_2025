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
        """获取赛道中线坐标及边界"""     #找到可能的问题所在，元素中已经没有对ccd_near类follow值的直接更改，
                                       #所以无法满足此条件判断
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



# 全局变量定义
prev_state = RoadElement.normal
state = RoadElement.normal
ring_progress = 0  # 圆环进度
zebra_count = 1    # 斑马线有几次是直接过
imu_data = [0] * 9
enc_data = 0
follow = 0
far_or_near = True
tmperror = 0   # 误差缓存
outflag = 0

# 常量定义（根据实际赛道调整）
ccd_near_l = [10, 50]       # 左圆环阶段1近端CCD左右边点范围
ccd_near_r = [70, 120]
ccd_near_l_lost = 7
ccd_near_r_lost = 120

ccd_far_r = [80, 120]        # 远端CCD右边点范围
ccd_far_l = [20, 50]         # 远端CCD左边点范围
ccd_far_l_lost = 7                 # 远端CCD左丢线阈值
ccd_far_r_lost = 120               # 远端CCD右丢线阈值

POINT_diff_data = 20            # 特征点差异阈值

DISTANCE_ring_2_data = 140

# l3
GYRO_Z_ring3_data = 700
DISTANCE_ring3_data = 110

# lin
GYRO_Z_ring_in_data = 1400
DISTANCE_ring_in_data = 80

DISTANCE_ring_out_data = 0.15
DISTANCE_ring_out_out_data = 170
ccd_near_length = 80
ccd_far_length = 60
DISTANCE_ring_outcoming_data = 300
DISTANCE_ring3_not_data = 300
DISTANCE_zebra_out_data = 80 # 斑马线
ERROR_l_out_value = -20

# 我们的gyro圆环识别数据
gyro_z_ring3 = 0.8  # 待测
gyro_z_ring4 = 1.0  # 待测

# 避障需要的数据
mid = ccd_near.mid
left = ccd_near.left
right = ccd_near.right
last_lenth = 40   # 待测，估计值
lenth = right - left

def debug():
    """纠正CCD和陀螺仪数据"""
    global ccd_near_l, ccd_near_r, ccd_far_l, ccd_far_r, ccd_near_length, ccd_far_length, POINT_diff_data
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
    ccd_near_l[0] = temp_ccd_near_data_l // 10 - delta
    ccd_near_l[1] = temp_ccd_near_data_l // 10 + delta
    ccd_near_r[0] = temp_ccd_near_data_r // 10 - delta
    ccd_near_r[1] = temp_ccd_near_data_r // 10 + delta
    ccd_near_length = abs(ccd_near_l[1] - ccd_near_l[0])
    ccd_far_l[0] = temp_ccd_far_data_l // 10 - delta
    ccd_far_l[1] = temp_ccd_far_data_l // 10 + delta
    ccd_far_r[0] = temp_ccd_far_data_r // 10 - delta
    ccd_far_r[1] = temp_ccd_far_data_r // 10 + delta
    ccd_far_length = abs(ccd_far_l[1] - ccd_far_l[0])
    POINT_diff_data = temp_ccd_near_data_l // 10 + delta

def elementdector():
    """主检测函数: , imu_data, enc_data """
    global state, prev_state, tmperror, outflag, zebra_count, follow
    tempcheck = state

    # 原有逻辑保持不变
    if state == RoadElement.normal:
        if _left_1():
            state = RoadElement.l1
    if state == RoadElement.normal:
        if _right_1():
            state = RoadElement.r1
    elif state == RoadElement.l1:
        if _left_2():
            state = RoadElement.l2
    elif state == RoadElement.r1:
        if _right_2():
            state = RoadElement.r2
    elif state == RoadElement.r2:
        if _right_3():
            state = RoadElement.r3
    elif state == RoadElement.l2:
        if _left_3():
            state = RoadElement.l3
    elif state == RoadElement.r3:
        if movementtype.mode == MOVEMENTTYPE.Mode_1:
            if _right_in_not():
                state = RoadElement.normal
        elif movementtype.mode == MOVEMENTTYPE.Mode_2:
            if _right_in():
                state = RoadElement.rin
    elif state == RoadElement.l3:
        if movementtype.mode == MOVEMENTTYPE.Mode_1:
            if _left_in_not():
                state = RoadElement.normal
        elif movementtype.mode == MOVEMENTTYPE.Mode_2:
            if _left_in():
                state = RoadElement.lin
    elif state == RoadElement.rin:
        if _right_outcoming():
            state = RoadElement.routcoming
    elif state == RoadElement.lin:
        if _left_outcoming():
            state = RoadElement.loutcoming
    elif state == RoadElement.routcoming:
        if _right_out():
            state = RoadElement.rout
    elif state == RoadElement.loutcoming:
        if _left_out():
            state = RoadElement.lout
    elif state == RoadElement.lout:
        if _left_out_out():
            state = RoadElement.normal

    _element_operations()  # 执行元素状态相关操作
    return state

def _element_operations():
    global prev_state, state, tmperror, follow, ccd_near_length, ERROR_l_out_value
    if prev_state == state:
        return
    ccd_controller.fix_error_value = 0
    ccd_controller.follow = 0
    ccd_controller.far = False
    element_gyro.clear()
    element_distance.clear()
    element_gyro.start()
    element_distance.start()
    
    if state == RoadElement.stop:
        movementtype.speed = 0
    elif state == RoadElement.zebrain:
        ccd_controller.far = True
    elif state == RoadElement.normal:
        ccd_controller.fix_error_value = 0
        ccd_controller.follow = 0
    elif state == RoadElement.l3:
        ccd_controller.follow = -ccd_near_length
    elif state == RoadElement.l3_not:
        ccd_controller.follow = -ccd_near_length
    elif state == RoadElement.lin:
        ccd_controller.follow = ccd_near_length
        tmperror = stage_error.get_tmp()
    elif state == RoadElement.loutcoming:
        ccd_controller.follow = 0
    elif state == RoadElement.lout:
        ccd_controller.fix_error_value = ERROR_l_out_value
    
    prev_state = state

def _left_1():
    near_valid = (ccd_near_l[0] <= ccd_near.left <= ccd_near_l[1] and 
                 ccd_near_r[0] <= ccd_near.right <= ccd_near_r[1])
    far_valid = (ccd_near.left < ccd_far_l_lost and 
                ccd_far_r[0] <= ccd_near.right <= ccd_far_r[1])
    point_diff = abs(ccd_near.right - ccd_near.right)
    return near_valid and far_valid and point_diff < POINT_diff_data

def _right_1():
    near_valid = (ccd_near_r[0] <= ccd_near.left <= ccd_near_r[1] and 
                ccd_near_l[0] <= ccd_near.right <= ccd_near_l[1])
    far_valid = (ccd_near.right > ccd_far_r_lost and 
                ccd_far_l[0] <= ccd_near.left <= ccd_far_l[1])
    point_diff = abs(ccd_near.left - ccd_near.left)
    return near_valid and far_valid and point_diff < POINT_diff_data

def _left_2():
    near_left_lost = ccd_near.left <= ccd_near_l_lost
    near_right_valid = ccd_near_r[0] <= ccd_near.right <= ccd_near_r[1]
    point_diff = abs(ccd_far.right - ccd_near.right)
    return near_left_lost and near_right_valid and point_diff < POINT_diff_data

def _right_2():
    near_right_lost = ccd_near.right >= ccd_near_r_lost
    near_left_valid = ccd_near_l[0] <= ccd_near.left <= ccd_near_l[1]
    point_diff = abs(ccd_far.left - ccd_near.left)
    return near_right_lost and near_left_valid and point_diff < POINT_diff_data

def _check_zebra(ccd_near):
    crossings = 0
    threshold = 31
    min_crossings = 10
    for i in range(30, 97):
        diff = abs(ccd_near.data[i] - ccd_near.data[i + 3]) * 100
        sum_val = ccd_near.data[i] + ccd_near.data[i + 3] + 1
        ratio = abs(diff / sum_val)
        if ratio > threshold:
            crossings += 1
    return crossings > min_crossings

def _check_zebra_out():
    return element_distance.data > DISTANCE_zebra_out_data

def _check_normal():
    return ccd_near.left > ccd_near_l_lost and ccd_near.right < ccd_near_r_lost

def _right_3():
    global state
    if element_distance.data > DISTANCE_ring3_data:
        if -GYRO_Z_ring3_data < element_gyro.data < GYRO_Z_ring3_data:
            state = RoadElement.rin
            return True
        else:
            state = RoadElement.normal
    if abs(element_gyro.data) > GYRO_Z_ring3_data:
        state = RoadElement.normal
    return False

def _right_in_not():
    global state
    near_right_lost = ccd_near.right >= ccd_near_r_lost
    near_left_valid = ccd_near_l[0] <= ccd_near.left <= ccd_near_l[1]
    point_diff = abs(ccd_far.left - ccd_near.left)
    if near_right_lost and near_left_valid and point_diff <= POINT_diff_data and element_distance.data > DISTANCE_ring3_not_data:
        state = RoadElement.normal
        return True
    return False

def _right_in():
    return abs(element_gyro.data) > GYRO_Z_ring_in_data

def _left_3():
    global state
    if abs(element_distance.data) > DISTANCE_ring3_data:
        if -GYRO_Z_ring3_data < element_gyro.data < GYRO_Z_ring3_data:
            if ccd_near_l[0] < ccd_near.left < ccd_near_l[1]:
                if abs(ccd_near.left - ccd_near.right) < POINT_diff_data:
                    return True
                else:
                    state = RoadElement.normal
    if abs(element_gyro.data) > GYRO_Z_ring3_data:
        state = RoadElement.normal
    return False

def _left_in_not():
    global state
    near_left_lost = ccd_near.left <= ccd_near_l_lost
    near_right_valid = ccd_near_r[0] <= ccd_near.right <= ccd_near_r[1]
    point_diff = abs(ccd_far.right - ccd_near.right)
    if near_left_lost and near_right_valid and point_diff <= POINT_diff_data and element_distance.data > DISTANCE_ring3_not_data:
        state = RoadElement.normal
        return True
    return False

def _left_in():
    return abs(element_gyro.data) > GYRO_Z_ring_in_data or abs(element_distance.data) > DISTANCE_ring_in_data

def _left_outcoming():
    if abs(element_distance.data) > DISTANCE_ring_outcoming_data:
        if check_tuple(ccd_near.data, 90, 30) == 1 or check_tuple(ccd_far.data, 90, 30) == 1:
            return True
    return False

def _right_outcoming():
    if abs(element_distance.data) > DISTANCE_ring_outcoming_data:
        if check_tuple(ccd_near.data, 90, 30) == 1 or check_tuple(ccd_far.data, 90, 30) == 1:
            return True
    return False

def _left_out_out():
    return abs(element_distance.data) > DISTANCE_ring_out_out_data or (ccd_near_l_lost < ccd_near.left and ccd_near.right < ccd_near_r_lost)

def _right_out():
    return abs(element_distance.data) > DISTANCE_ring_out_data

def _left_out():
    return abs(element_distance.data) > DISTANCE_ring_out_data

def _crossroad():
    near_ccd_lost = (ccd_near.left <= ccd_near_l[0] and ccd_near.right >= ccd_near_r[1])
    far_ccd_normal = (ccd_far_l[0] <= ccd_far.left <= ccd_far_l[1] and ccd_far_r[0] <= ccd_far.right <= ccd_far_r[1])
    return near_ccd_lost and far_ccd_normal

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

