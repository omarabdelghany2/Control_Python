
from Control import *

def Get_sensor_measurment() :
    out=int(input("Enter your sensor input : "))
    return out




PID_Test=Control_Var(0,0,0,0)

PID_Test.Kp=7
PID_Test.Ki=0
PID_Test.Kd=0
PID_Test.tau=0.01
PID_Test.limMin=-100.0
PID_Test.limMax= 100.0

PID_Test.limMaxInt=100
PID_Test.limMinInt=-100
PID_Test.T=0.01
setpoint=0

while(1) :
    measurment =Get_sensor_measurment()
    out=PID_Test.PIDController_Update(setpoint,measurment)
    print(out)








