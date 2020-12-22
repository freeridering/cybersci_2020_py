import numpy as np
import scipy.integrate
import math
import parameter
import scipy.stats
import random
import pickle

pickle_file = open('result.pkl', 'rb')
data = pickle.load(pickle_file)
p = data['p']  # 环境参数
uavs = data['uavs']  # 初始化无人机群
targets = data['targets']  # 初始化目标
pickle_file.close()
