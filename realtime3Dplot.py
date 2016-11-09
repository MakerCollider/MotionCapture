import serial
import numpy as np
import matplotlib as mpl
import mpl_toolkits.mplot3d.axes3d as p3 
import matplotlib.pyplot as plt
import matplotlib.animation as animation
import _thread
import datetime
import re
mylock = _thread.allocate_lock()

lineData = np.empty((3,60000))
count = 0

try:
    ser = serial.Serial('COM10',115200)
except Exception as e:
    print('open serial failed.error:',e)
    exit(1)
print('A Serial Echo Is Runnig....')
def receiveData():
    while True:
        global mylock,ser,count
        data = ser.readline()
        dataStr = str(data)
        dataStr = re.match(r'b\'(?:-?\d+\.\d+,){2}-?\d+\.\d+\\r\\n',dataStr)
        if dataStr:
            dataStr = dataStr.group()
        else:
            continue
        dataStr = dataStr.strip('b\'\\r\\n')
        dataStr = dataStr.split(',')
    
        receive = [float(item) for item in dataStr]
        if count<60000:
            count +=1
                
        print(receive)
        mylock.acquire()
        if count<=60000:
            lineData[0,count-1]=receive[0]
            lineData[1,count-1]=receive[1]
            lineData[2,count-1]=receive[2]
        else:
            for index in range(59999):
                lineData[:,index]=lineData[:,index+1]
            lineData[0,59999]=receive[0]
            lineData[1,59999]=receive[1]
            lineData[2,59999]=receive[2]
        mylock.release()
_thread.start_new_thread(receiveData,())

fig = plt.figure()
ax = p3.Axes3D(fig)
lastTime = datetime.datetime.now()

def update_line(num,line):
    global mylock,lastTime,count
    mylock.acquire()
    if count<1:
        line.set_data(lineData[0:2,:1])
        line.set_3d_properties(lineData[2,:1])
    else:
        line.set_data(lineData[0:2,:count-1])
        line.set_3d_properties(lineData[2,:count-1])
    #print((datetime.datetime.now()-lastTime).microseconds)
    #lastTime = datetime.datetime.now()
    mylock.release()
    return line
line = ax.plot(lineData[0,:1],lineData[1,:1],lineData[2,:1])
ax.set_xlim3d([-5.0,5.0])
ax.set_xlabel('X')
ax.set_ylim3d([-5.0,5.0])
ax.set_ylabel('Y')
ax.set_zlim3d([-5.0,5.0])
ax.set_zlabel('Z')
line_ani = animation.FuncAnimation(fig,update_line,interval = 50,blit = False,fargs = (line))
plt.show()


