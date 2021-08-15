#! /usr/bin/env python
#coding=utf-8

import torch
import numpy as np
import json
import config
import matplotlib.pyplot as plt
#python -m visdom.server
#from visdom import Visdom

with open('/home/eric/catkin_ws/src/hrl_project/base_action/src/SAC_model/20210813-09-53_stage1/reward.json') as f:
    data = json.load(f)
"""
#print(len(data))
viz = Visdom(env='ation_result')

for i in range(len(data)):
    x = i
    y = data[i]
    viz.line(X = np.array([x]), Y = np.array([y]), win = 'window', update = 'append')
"""
plt.plot(range(len(data)), data, color='royalblue', label='Soft-Actor-Critic')
plt.xlabel('Episodes') # 設定x軸標題
#plt.xticks(range(len(data)), rotation='vertical') # 設定x軸label以及垂直顯示
plt.title('Base action') # 設定圖表標題
plt.show()
