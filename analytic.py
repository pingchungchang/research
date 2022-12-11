import matplotlib.pyplot as plt
import math
barfile = open('/home/pcc/barpos.txt','r')
movfile = open('/home/pcc/movpos.txt','r')

bars = barfile.read().split(' ')
mov = movfile.read().split(' ')
out = open('/home/pcc/coords.txt','a')
bars.pop()
mov.pop()
for i in range(len(bars)):
    bars[i] = bars[i].split(',')
    for j in range(len(bars[i])):
        bars[i][j] = float(bars[i][j])
for i in range(len(mov)):
    mov[i] = mov[i].split(',')
    for j in range(len(mov[i])):
        mov[i][j] = float(mov[i][j])
dictionary = {}
dict2 = {}
for i in bars:
    dictionary[i[2]] = [i[0],i[1]]
for i in mov:
    if type(dictionary.get(i[2],-1.0)) != float:
        tmp = dictionary.get(i[2])
        dict2[i[2]] = abs(tmp[1]-i[1])
        # dictionary[i[2]] = abs(tmp[0]-i[0])
        out.write(str(i[0])+str(' ')+str(i[1])+str(' ')+str(math.sqrt((i[0]-tmp[0])**2+(tmp[1]-i[1])**2))+',')
        dictionary[i[2]] = math.sqrt(abs(tmp[0]-i[0])**2+abs(tmp[1]-i[1])**2)
x = []
y = []
y2 = []
for key,val in dictionary.items():
    if type(val) == float:
        if val>10:
            continue
        x.append(key)
        y.append(val)
for key,val in dict2.items():
    if type(val) == float:
        if val>10:
            continue
        y2.append(val)
# print(x)
# print(y)
plt.scatter(x,y,c = "cyan")
# plt.scatter(x,y2,c = 'red')
plt.show()

