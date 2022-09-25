










class Control_Var :
    def __init__(self,integrator,preverror,differentiator,prevmeasurement):
        self.integrator =integrator
        self.preverror =preverror
        self.differentiator =differentiator
        self.prevmeasurment=prevmeasurement
    
    Kp=0
    Ki=0
    Kd=0
    tau=0
    limMin=0,
    limMax=0
    limMinInt=0
    limMaxInt=0
    T=0 
    out=0

    def PIDController_Update(self,setpoint,actual):
        self.error=setpoint-actual
        self.proportional =self.Kp * self.error
        self.differentiator=-(2*self.Kd*(actual-self.prevmeasurment)+(2*self.tau-self.differentiator))/(2*self.tau+self.T)
        self.integrator=self.integrator+0.5*self.Ki*self.T*(self.error+self.preverror)
        if(self.integrator>self.limMaxInt):
            self.integrator=self.limMaxInt
        elif (self.integrator<self.limMinInt):
            self.integrator=self.limMinInt
        self.out = self.proportional+self.integrator+self.differentiator

        if(self.out>self.limMax):
            self.out=self.limMax
        elif(self.out<self.limMin):
            self.out=self.limMin
        self.prevmeasurment=actual
    
        return self.out
    

class ROV_MOTION :
    ROV_ROATATING=False
    ROV_PITCHING=False
    ROV_ROLLING=False
class PID_STATES : 

    YAW_PID_STATE=False
    PITCH_PID_STATE=False
    ROLL_PID_STATE =False
class SETPOINTS  :
    YAW_SETPOINT=0
    PITCH_SETPOINT=0
    ROLL_SETPOINT=0

MOTION_STATES=ROV_MOTION()
ON_OFF_STATES=PID_STATES()
ANGLE_SETPOINTS=SETPOINTS()

IMU_READING = [0,0,0]



# PID=Control_Var(5,5,5,5)
# PID.PIDController_Update(20,10)
# print(PID.error)
	
YAW_AXIS=0
PITCH_AXIS=2
ROLL_AXIS=1

def Set_setpoint(axis) :
    if(axis[YAW_AXIS]!=0) :
        MOTION_STATES.ROV_ROATATING=True
    else :
        MOTION_STATES.ROV_ROATATING=False
    if(axis[PITCH_AXIS]!=0) :
        MOTION_STATES.ROV_PITCHING=True
    else :
        MOTION_STATES.ROV_PITCHING=False

    if(axis[ROLL_AXIS]!=0) :
        MOTION_STATES.ROV_ROLLING=True
    else :
        MOTION_STATES.ROV_ROLLING=False

    if(MOTION_STATES.ROV_ROATATING==True) :
        PID_STATES.YAW_PID_STATE=False
        ANGLE_SETPOINTS.YAW_SETPOINT=IMU_READING[0]
    else :
        PID_STATES.YAW_PID_STATE=True


    if(MOTION_STATES.ROV_PITCHING==True) :
        PID_STATES.PITCH_PID_STATE=False
        ANGLE_SETPOINTS.PITCH_SETPOINT=IMU_READING[1]
    else :
        PID_STATES.PITCH_PID_STATE=True    

    if(MOTION_STATES.ROV_ROLLING==True) :
        PID_STATES.ROLL_PID_STATE=False
        ANGLE_SETPOINTS.ROLL_SETPOINT=IMU_READING[2]
    else :
        PID_STATES.ROLL_PID_STATE=True  
          
  