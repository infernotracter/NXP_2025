from PIL import Image, ImageGrab
import numpy as np

# 1. 从剪贴板获取图片
image = ImageGrab.grabclipboard()
if image is None:
    raise ValueError("剪贴板中无图片，请先复制图片")
elif isinstance(image, list):
    # 如果是文件路径列表，则加载第一个图片文件
    image = Image.open(image[0])

# 2. 转换为LAB颜色空间
lab_image = image.convert('LAB')
lab_array = np.array(lab_image)

# 3. 移除明显错误值并计算极值
mask = np.all((lab_array >= 0) & (lab_array <= 255), axis=2)
valid_pixels = lab_array[mask]

if valid_pixels.size == 0:
    raise ValueError("未找到有效像素值")

l_min = np.min(valid_pixels[:, 0])
l_max = np.max(valid_pixels[:, 0])

a_min = np.min(valid_pixels[:, 1])
a_max = np.max(valid_pixels[:, 1])

b_min = np.min(valid_pixels[:, 2])
b_max = np.max(valid_pixels[:, 2])

# 4. 调整极值范围（最大加5，最小减5）
num = 5
adj_l_min = (int(l_min) - num)
adj_l_max = (int(l_max) + num)

adj_a_min = (int(a_min) - num)
adj_a_max = (int(a_max) + num)

adj_b_min = (int(b_min) - num)
adj_b_max = (int(b_max) + num)

# 5. 以要求格式输出
result_format = f"({adj_l_min},{adj_l_max},{adj_a_min},{adj_a_max},{adj_b_min},{adj_b_max})"
print("调整后的LAB极值结果:")
print(result_format)