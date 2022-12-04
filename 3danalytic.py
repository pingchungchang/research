import matplotlib.pyplot as plt
import numpy as np

inp  = open('/home/pcc/coords.txt','r')
inp = inp.read().split(',')
for i in range(len(inp)):
    inp[i] = inp[i].split(' ')
fig = plt.figure()
ax = fig.add_subplit(projection='3d')
for i in inp:
    ax.scatter(i[0],i[1],i[2])
plt.show()
