import numpy as np
import matplotlib.pyplot as plt

x_ticks = [20000,40000,60000,80000,100000,120000,140000,160000,180000,200000]
y_ticks = np.linspace(-1.0,1.0,11)

dqn = [-0.356, -0.181, 0.024, 0.132, 0.192, 0.189, 0.238, 0.202, 0.247,0.223]



dqn_k = [0.142, 0.183, 0.197, 0.234, 0.263, 0.245, 0.283, 0.266, 0.317, 0.322]


ppo = [0.156, 0.121, 0.264, 0.362, 0.313, 0.389, 0.416, 0.402, 0.395, 0.412]

ppo_k = [0.312, 0.273, 0.297, 0.354, 0.345, 0.373,  0.433, 0.413, 0.427, 0.446]

plt.plot(x_ticks, dqn, color="blue", linewidth=2.5, linestyle="-",  label='DQN')
plt.plot(x_ticks, ppo, color="red", linewidth=2.5, linestyle="-",  label='PPO')

plt.xlim(0,200000)
#plt.ylim(0.0,1.0)
plt.ylim(-0.5,1.0)
plt.xlabel('Steps')
# plt.ylabel('Known Ratio')
# plt.title('Known Ratio of 200 Steps')
plt.ylabel('Reward')
plt.title('Average Reward of 200 Steps')
 
ax=plt.gca()  #gca:get current axis得到当前轴
#设置图片的右边框和上边框为不显示
ax.spines['right'].set_color('none')
ax.spines['top'].set_color('none')
ax.grid(True, linestyle='-.')
 
#挪动x，y轴的位置，也就是图片下边框和左边框的位置
ax.spines['bottom'].set_position(('data',0))  #data表示通过值来设置x轴的位置，将x轴绑定在y=0的位置
ax.spines['left'].set_position(('axes',0))  #axes表示以百分比的形式设置轴的位置，即将y轴绑定在x轴50%的位置，也就是x轴的中点


# print(x_ticks,y_ticks)
# plt.xticks(x_ticks)
# plt.yticks(y_ticks)
plt.legend()

plt.show()

