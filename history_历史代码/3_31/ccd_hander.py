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

class road_type:
    """道路类型"""
    circle_l1 = 'circle_l1'  # 左环
    circle_l2 = 'circle_l2'  # 左环
    circle_r1 = 'circle_r1'  # 右环
    circle_r2 = 'circle_r2'  # 右环
    lost = 'lost'  # 丢失
    cross = 'cross'  # 十字路口


def search_element():
    """搜索元素"""
    if ccd_f.lost_l and ccd_f.lost_r:
        return road_type.lost
    elif ccd_f.lost_l and not ccd_f.lost_r:


