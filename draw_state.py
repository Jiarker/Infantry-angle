import matplotlib.pyplot as plt

predict_angle = []
predict_time = []

now_angle = []
now_time = []


with open("./src/Algorithm/configure/Detector/Fitting/buff_state/42.txt", "r") as speed_txt:
    for line in speed_txt.readlines():
        curLine = line.strip().split(" ")
        floatLine = map(float,curLine) 
        floatLine = list(floatLine)

        predict_angle.append(floatLine[0])
        predict_time.append(floatLine[1])

        now_angle.append(floatLine[2])
        now_time.append(floatLine[3])


plt.scatter(predict_time, predict_angle, s=5, c='blue')
plt.scatter(now_time, now_angle, s=5, c='red')

plt.show()