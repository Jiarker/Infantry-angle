import matplotlib.pyplot as plt
import numpy as np

result = []
time = []
angle = []
time_after = []
angle_after = []
label = []
num = 0
w = 0

first_line = True

with open("./txt/29.txt", "r") as txt:
    for line in txt.readlines():
        curLine = line.strip().split(" ")
        floatLine = map(float,curLine) 
        floatLine = list(floatLine)

        if first_line:
            w = floatLine[0]
            result.append(floatLine[1])
            result.append(floatLine[2])
            result.append(floatLine[3])
            result.append(floatLine[4])
            first_line = False

        else:
            label = floatLine[2]
            if label == 0:
                time.append(floatLine[0])
                angle.append(floatLine[1])
            
            else:
                time_after.append(floatLine[0])
                angle_after.append(floatLine[1])                
            

x = np.linspace(time[0], time_after[-1], 256, endpoint=True)
line = result[0]*np.sin(x*w) + result[1]*np.cos(x*w) + result[2]*x + result[3]
plt.plot(x,line)

plt.scatter(time, angle, s=5, c='red')
plt.scatter(time_after, angle_after, s=5, c='green')
plt.show()

