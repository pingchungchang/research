import matplotlib.pyplot as plt
import numpy as np
from mpl_toolkits.mplot3d import Axes3D
from matplotlib import cm
import math
lim = 5
inp  = open('/home/pcc/coords.txt','r')
inp = inp.read().split(',')
inp.pop()
big = 0
for i in range(len(inp)):
    inp[i] = inp[i].split(' ')
    for j in range(len(inp[i])):
        inp[i][j] = float(inp[i][j])
    # big = max(big,inp[i][2])
fig = plt.figure()
ax = Axes3D(fig)
print(type(ax))
# exit()
inp = np.array(inp)
print(inp)
cc = len(inp[:,0])
for i in range(len(inp)):
    inp[i][2] /= math.sqrt(inp[i][0]**2+inp[i][1]**2)
X = inp[:,0].reshape((1,cc)).T
Y = inp[:,1].reshape((1,cc)).T
Z = inp[:,2].reshape((1,cc)).T
for i in range(len(inp)):
    big = max(big,inp[i][2])
# surf = ax.plot_surface(X,Y,Z,cmap=cm.coolwarm,linewidth=0,antialiased=False)
# fig.colorbar(surf,shrink=7,aspect=5,pad=1)
for i in inp:
    # print(float(i[2])/big)
    ccc = (min(float(i[2]/lim),1),0,0)
    # ccc = (min(float(i[2])/10.0,1),i[2]/big/2,0)
    # ccc = (float(float(i[2])/10.0),0)
    ax.scatter(float(i[0]),float(i[1]),float(i[2]),c=ccc)
plt.show()
