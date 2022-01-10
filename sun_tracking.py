from machine import ADC,Pin,PWM,TouchPad,Timer
from time import sleep,time
import network
from umqtt.simple import MQTTClient

publish_TOPIC = '$sys/480460/esp32_sun/dp/post/json'
subscribe_TOPIC ='$sys/480460/esp32_sun/cmd/request/+'

def med(l):
  return sorted(l)[int(len(l)/2)]

def clip(v,bound):
  return max(min(v,bound[1]),bound[0])

def connectWifi(ssid,passwd):
  global wlan
  wlan=network.WLAN(network.STA_IF)         #create a wlan object
  wlan.active(True)                         #Activate the network interface
  wlan.disconnect()                         #Disconnect the last connected WiFi
  wlan.connect(ssid,passwd)                 #connect wifi
  while(wlan.ifconfig()[0]=='0.0.0.0'):
    sleep(0.1)

class PID:
  
  P = I = D = O = 0
  Imax = float('inf')
  Imin = -Imax
  
  def __init__(self,Kp,Ki,Kd,bound):
    self.Kp = Kp
    self.Ki = Ki
    self.Kd = Kd
    self.bound(*bound)
  
  def bound(self,min_,max_):
    if self.Ki==0:
      self.Imax = float('inf')
      self.Imin = -self.Imax
    else:
      self.Imin = min_/self.Ki
      self.Imax = max_/self.Ki
    
  def update(self,P):
    self.D = self.P - P
    self.I = clip(self.I+self.P,[self.Imin,self.Imax])
    self.P = P
    self.O = self.P*self.Kp + self.I*self.Ki + self.D*self.Kd
    return self.O
  
  def __repr__(self):
    return 'PID(Kp={},Ki={},Kd={})'.format(self.Kp,self.Ki,self.Kd)

class Servo:
  
  def __init__(self,pin,PWMrange=[22,118],angle=0):
    self.PWM = PWM(Pin(pin),freq=50)
    self.PWMrange = PWMrange
    self.set(angle)
    
  def set(self,angle):
    self.angle = clip(angle,[-90,90])
    #self.PWM.duty(int(self.PWMrange[0]+(self.PWMrange[1]-self.PWMrange[0])*(angle+90)/180))
    self.PWM.duty_u16(int((self.PWMrange[0]+(self.PWMrange[1]-self.PWMrange[0])*(self.angle+90)/180)*64))

class Arm:
  
  yaw = pitch = 0
  
  def __init__(self,yaw,pitch):
    self.servo_yaw = Servo(yaw,[22,123])
    self.servo_pitch = Servo(pitch,[22,118])
    
  def set_yaw(self,yaw):
    self.yaw = yaw
    self.servo_yaw.set(yaw)
    
  def set_pitch(self,pitch):
    self.pitch = pitch
    self.servo_pitch.set(pitch)
    
  def reset(self):
    self.set(0,0)
    
  def remain(self):
    self.set(self.yaw,self.pitch)
  
  def set(self,yaw,pitch):
    LIMITED  = (abs(self.yaw)-90)*(abs(yaw)-90)<0
    if LIMITED:
      pitch *= -1
      if yaw>90:
        yaw -= 180
      elif yaw<-90:
        yaw += 180
      
    self.set_pitch(pitch)
    self.set_yaw(yaw)
    
    return LIMITED

class ADCs(dict):
  def __init__(self,**Pins):
    super().__init__((k,ADC(Pin(v))) for k,v in Pins.items())
    for v in self.values():
      v.atten(ADC.ATTN_11DB)
  def read(self):
    return {k:med([v.read() for i in range(5)]) for k,v in self.items()}
    
class Lightmeters(ADCs):
  def __init__(self,up,down,left,right):
    super().__init__(up=up,down=down,left=left,right=right)    
    
  def get_diff(self):
    data = self.read()
    diff = {}
    if data['up']>50:
      data['up'] = 1.3218*data['up']+60.3
    diff['pitch'] = (data['up']-data['down'])*(1-max(data['down'],data['up'])/4095)
    diff['yaw'] = (data['right']-data['left'])*(1-max(data['left'],data['right'])/4095)
    return diff
  
  def get_sum(self):
    return sum(self.read().values())
    
class Voltmeters(ADCs):
  def __init__(self,Vout,Vsolar):
    super().__init__(Vout=Vout,Vsolar=Vsolar)
  
  def get_Vout(self):
    raw = self['Vout'].read()
    if raw<10:
      return raw/10*0.4
    else:
      return (raw-10)/4095*3.3*670/200+0.4
  
  def get_Vsolar(self):
    raw = self['Vsolar'].read()
    if raw<10:
      return raw/10*0.45
    else:
      return (raw-10)/4095*3.3*670/200+0.45
  

class PIDCtrl:
  def __init__(self,inputs,outputs,pids):
    self.inputs = inputs
    self.outputs = outputs
    self.pids = pids
    
  def get_error(self):
    error = None
    return error
  
  def get_out(self,error):
    out = None
    return out
    
  def set_out(self,out):
    pass
    
class ArmCtrl(PIDCtrl):
  
  timer = Timer(3)
  
  LIMITED = False
  MODE = 'AUTO'
  out = {'yaw':0,'pitch':0}
    
  def __init__(self,inputs,outputs,pids):
    super().__init__(inputs,outputs,pids)
    pass
  
  def get_error(self):
    errors = self.inputs.get_diff()
    errors['yaw'] *= abs(self.outputs.pitch)/(self.outputs.pitch+1e-5)
    return errors
  
  def get_out(self,err):
    out = {}
    out['yaw'] = self.pids['yaw'].update(err['yaw'])
    out['pitch'] = self.pids['pitch'].update(err['pitch'])
    return out
    
  def LIMITED_callback(self,_):
    self.LIMITED = False
  
  def set_out(self,out):
    if self.outputs.set(**out):
      self.out['yaw'] = self.outputs.yaw
      self.out['pitch'] = self.outputs.pitch
      print('reset')
      self.LIMITED = True
      self.timer.init(mode=Timer.ONE_SHOT, period=700,callback=self.LIMITED_callback)
      self.pids['yaw'].I *= -0.9
      self.pids['yaw'].D *= -1
      self.pids['pitch'].I *= -1
      self.pids['pitch'].D *= -1

  def reset(self):
    self.out['yaw'] = 0
    self.out['pitch'] = 0
    self.set_out(self.out)

  def mainloop(self):
    if self.MODE=='AUTO':
      if self.inputs.get_sum()<100:
        self.reset()
        self.pids['yaw'].I = 0
        self.pids['yaw'].D = 0
        self.pids['pitch'].I = 0
        self.pids['pitch'].D = 0
        
      elif not self.LIMITED:
        self.err = self.get_error()
        self.out = self.get_out(self.err)
        self.set_out(self.out)
      else:
        self.outputs.remain()
    elif self.MODE=='MANUAL':
      self.outputs.set(**self.out)
  
  def set_MODE(self,m):
    self.MODE = m

class VolCtrl(PIDCtrl):
  MODE = 'Stable'
  target_U = 1
  K = 0.8
  timer = Timer(3)
  GET_OPEN_U = False
  
  def __init__(self,inputs,outputs,pids):
    super().__init__(inputs,outputs,pids)
    pids['Vout'].I = pids['Vout'].Imax
    pass
  
  def set_target_U(self,v):
    if v<0:
      print('target_U too small')
    else:
      self.target_U = v
      print('set_target_U',v)
  
  def get_error(self):
    if self.MODE=='Stable':
      return self.inputs.get_Vout()-self.target_U
    elif self.MODE=='MPPT':
      return self.inputs.get_Vsolar()-self.target_U
      
  def set_MODE(self,m):
    if m=='MPPT':
      self.timer.init(mode=Timer.PERIODIC, period=5000,callback=self.MPPT_callback)
      self.set_target_U(self.get_open_voltage()*self.K)
    elif m=='Stable':
      self.set_target_U(self.inputs.get_Vout())
      self.timer.deinit()
    self.MODE = m
  
  def get_out(self,err):
    if self.MODE=='Stable':
      return self.pids['Vout'].update(err)
    elif self.MODE=='MPPT':
      return self.pids['Vsolar'].update(err)
    
  def get_open_voltage(self):
    duty_ = self.outputs.pwm.duty()
    print('duty:',duty_)
    self.outputs.off()
    self.GET_OPEN_U = True
    sleep(0.1)
    self.GET_OPEN_U = False
    open_voltage = self.inputs.get_Vsolar()
    self.outputs.on()
    self.outputs.pwm.duty(duty_)
    return open_voltage
    
  def MPPT_callback(self,t):
    open_U = self.get_open_voltage()#including sleep!
    print('open_U',open_U)
    self.set_target_U(open_U*self.K)
    
  def set_out(self,out):
    self.outputs.set(out)
    return out<0
  
  def mainloop(self):
    self.err = self.get_error()
    self.out = self.get_out(self.err)
    if self.set_out(self.out):
      #print('target_U too large')
      self.set_target_U(max(self.target_U-0.001,0))
      
   



class Boost:
  def __init__(self,pwm,switch):
    self.pwm = PWM(Pin(pwm),freq=25000,duty=1023)
    self.switch = Pin(switch,Pin.OUT)
    self.on()
  
  def set(self,duty):
    self.pwm.duty_u16(int(clip(duty,[0,100])/100*1023*64))
    
  def on(self):
    self.switch.value(1)
    
  def off(self):
    self.switch.value(0)
    self.pwm.duty(0)

class IOTCtrl(MQTTClient):
  timer = Timer(5)

  def __init__(self,armctrl,volctrl):
    self.armctrl = armctrl
    self.volctrl = volctrl

  def init(self,client_id,server,port,user,password,keepalive):
    super().__init__(client_id, server,port,user,password,keepalive)
    self.set_callback(self.callback)
    self.connect()
    self.subscribe(subscribe_TOPIC)
    self.timer.init(mode=Timer.PERIODIC, period=500, callback=self.upload)
  
  def stop(self):
    self.timer.deinit()
    self.volctrl.timer.deinit()
  
  def upload(self,timer):
    data = self.data()#{'yaw':self.armctrl.outputs.yaw,'pitch':self.armctrl.outputs.pitch,'MPPT':self.volctrl.inputs.get_Vsolar(),'output_voltage':self.volctrl.inputs.get_Vout()}
    if self.volctrl.GET_OPEN_U:
      data_ = {"id":456,"dp":{k:[{"v":v}] for k,v in data.items() if k in ['yaw','pitch']}}
    else:
      data_ = {"id":456,"dp":{k:[{"v":v}] for k,v in data.items()}}
      data_['dp']["UI"] = [{ "v": [int(1000*data['MPPT']),int(1000*data['output_voltage'])]}]
    try:
      self.publish(topic=publish_TOPIC, msg=str(data_), retain=False, qos=0)
    except Exception as e:
      print("publish 失败",e)
      self.timer.deinit()

  def mainloop(self):
    self.check_msg()

  def callback(self, topic, msg):
    sleep(0.01)
    print('接收',msg)
    if msg == b'Stable':
      self.volctrl.set_MODE('Stable')
    elif msg == b'MPPT':
      self.volctrl.set_MODE('MPPT')
    elif msg == b'auto_on':
      self.armctrl.set_MODE('AUTO')
    elif msg == b'auto_off':
      self.armctrl.set_MODE('MANUAL')
    elif msg == b'reset':
      self.armctrl.reset()
      sleep(0.5)
    elif msg == b'+':
      self.volctrl.set_target_U(self.volctrl.target_U+0.1)
    elif msg == b'-':
      self.volctrl.set_target_U(self.volctrl.target_U-0.1)
    elif msg.startswith('yaw'):
      self.armctrl.out['yaw'] = -int(msg[3:])
    elif msg.startswith('pitch'):
      self.armctrl.out['pitch'] = clip(int(msg[5:]),[-80,80])
    elif msg.startswith('python '):
      try:
        exec(msg[7:])
      except Exception as e:
        print(e)
      else:
        print('exec successful')

  def data(self):
    return {'yaw':self.armctrl.outputs.yaw,'pitch':self.armctrl.outputs.pitch,'MPPT':self.volctrl.inputs.get_Vsolar(),'output_voltage':self.volctrl.inputs.get_Vout()}
    

class LED(Pin):
  def __init__(self,pin):
    super().__init__(pin,Pin.OUT)

  def switch(self):
    self.value(not self.value())

  def flash(self):
    pass





