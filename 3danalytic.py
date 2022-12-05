import matplotlib.pyplot as plt
import numpy as np
from mpl_toolkits.mplot3d import Axes3D
from matplotlib import cm

inp  = open('/home/pcc/coords.txt','r')
inp = inp.read().split(',')
inp.pop()
for i in range(len(inp)):
    inp[i] = inp[i].split(' ')
    for j in range(len(inp[i])):
        inp[i][j] = float(inp[i][j])
fig = plt.figure()
ax = Axes3D(fig)
print(type(ax))
# exit()
inp = np.array(inp)
print(inp)
cc = len(inp[:,0])
X = inp[:,0].reshape((1,cc)).T
Y = inp[:,1].reshape((1,cc)).T
Z = inp[:,2].reshape((1,cc)).T

# surf = ax.plot_surface(X,Y,Z,cmap=cm.coolwarm,linewidth=0,antialiased=False)
# fig.colorbar(surf,shrink=7,aspect=5,pad=1)
for i in inp:
    ax.scatter(float(i[0]),float(i[1]),float(i[2]),c='red')
plt.show()
