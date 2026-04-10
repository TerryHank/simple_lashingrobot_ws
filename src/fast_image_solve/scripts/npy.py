import numpy as np

# 加载npy文件
data = np.load('/home/hyq/icc/lashingrobots/pose_matrix.npy')

# 打印数据类型、形状和内容
print("----type----")
print(type(data))
print("----shape----")
print(data.shape)
print("----data----")
print(data)