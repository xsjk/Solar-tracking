from sun_tracking import *

lightmeters = Lightmeters(33,35,34,32)
voltmeters = Voltmeters(36,39)

arm = Arm(18,19)
boost = Boost(17,16)

yaw_PID = PID(0.01,0.001,0.005,[-88,88])
pitch_PID = PID(0.01,0.001,0.005,[-70,70])

Vout_PID = PID(2,0.1,0,[0,100])
Vsolar_PID = PID(2,0.1,0,[0,100])

armctrl = ArmCtrl(lightmeters,arm,{'yaw':yaw_PID,'pitch':pitch_PID})
volctrl = VolCtrl(voltmeters,boost,{'Vout':Vout_PID,'Vsolar':Vsolar_PID})
iotctrl = IOTCtrl(armctrl,volctrl)

arm.reset()
connectWifi("SSID","PASSWORD")
iotctrl.init("CLIENT_ID","SERVER",0,"username","password",60)

led = LED(2)

while True:
  led(time()%2)
  iotctrl.mainloop()
  armctrl.mainloop()
  volctrl.mainloop()
  



