#!/usr/bin/env python

import sys
import rospy
from sensortag.srv import *
from operateSensor import *
from std_msgs.msg import Float32
import time
import subprocess, signal
import os
from sensortag.msg import *

#map between uuid and handles
#2902 - 0x0022 - notifications

#temperature
#aa01 - 0x0021 - data
#aa02 - 0x0024 - config
#aa03 - 0x0026 - period

#movement
#aa81 - 0x39 - data
#aa82 - 0x3c - configuration
#aa83 - 0x3e - period

#humidity
#aa21 - 0x0029 data
#aa22 - 0x002c configuration
#aa23 - 0x002e period


tag = None
cbs = None
bluetooth_adr = None
restarting = False
last_tem = -1
last_hum = -1
last_time = -1

fre = 1

def temperature():
  if(not restarting):
      handle = 0x21
      global tag
      global cbs
 
      try:
        tag.con.sendline('char-read-hnd 0x%02x' % handle)
        tag.con.expect('descriptor: .*? \r')  
        after = tag.con.after
        #print after
        hxstr = after.split()[1:]
        arr = [long(float.fromhex(n)) for n in hxstr[:]] 
        #print arr
        output = cbs.temperature(arr)
        #print output
        if (output == None):
          output = -1
          #print output
        return output        
      except pexpect.TIMEOUT:
        print "exception"
        restart()
        return -1   


def humidity():
  if(not restarting):
      handle = 0x29
      global tag
      global cbs

      try:
        tag.con.sendline('char-read-hnd 0x%02x' % handle)
        tag.con.expect('descriptor: .*? \r')
           
        after = tag.con.after
        #print after
        hxstr = after.split()[1:]
        arr = [long(float.fromhex(n)) for n in hxstr[:]] 
        #print arr
        output = cbs.humidity(arr)
        #print output
        if (output == None):
          output = -1
          #print output
        
        return output
      except pexpect.TIMEOUT:
        print "exception"
        restart()
        return -1 


def handle_reading(req):
  if(not restarting):
    global tag
    global cbs
    global fre
    global last_hum
    global last_tem
    global last_time
    print "Returning [%s]"%(req.sensorType)

    #check it 10 times faster then it is the rate
    #we dont know when the request came
    #rate = rospy.Rate(float(fre)*10.0)
    #curr_tem = last_tem
    #curr_hum = last_hum
    #curr_time = time.clock()
    #while(last_time < curr_time):
    #  if((curr_tem != last_tem) and (req.sensorType == "temperature")):
    #    break; #we have new measurement
    #  if((curr_hum != last_hum) and (req.sensorType == "humidity")):
    #    break; #we have new measurement
      
    #  rate.sleep()

    if req.sensorType == "temperature":
      
      output = temperature()

    elif req.sensorType == "humidity":
      output = humidity()

    else:
      print "Unsupported request!"
      output = -1

     
    return ReadSensorResponse(output)

def sensortag_measure():
    global restarting
    global fre
    global last_time
    global last_tem
    global last_hum

    pub_t = rospy.Publisher('/sensor_tag/temperature', MeasurementStamped, queue_size=10)
    pub_h = rospy.Publisher('/sensor_tag/humidity', MeasurementStamped, queue_size=10)
    msg_t = MeasurementStamped()
    msg_h = MeasurementStamped()

    rospy.init_node('sensortag_read')
    s = rospy.Service('sensortag_read', ReadSensor, handle_reading)
    print "Ready to read sensortag."

    rate = rospy.Rate(fre) # 0.00166666 once in 10minutes
    while not rospy.is_shutdown():
      if( not (restarting)):
        last_time = time.clock()
        msg_t.measurement.data = temperature()
        msg_h.measurement.data = humidity()

        msg_t.header = std_msgs.msg.Header()
        msg_t.header.stamp = rospy.Time.now()
        msg_h.header = std_msgs.msg.Header()
        msg_h.header.stamp = rospy.Time.now()
        

        last_tem = msg_t.measurement.data
        last_hum = msg_h.measurement.data

        pub_t.publish(msg_t)
        pub_h.publish(msg_h)
      rate.sleep()

    rospy.spin()

def restart():
   global tag
   global cbs
   global bluetooth_adr
   global restarting

   if(restarting): #do not try to restart again
     return

   restarting = True
  
   
   if tag is not None:
     print "delete tag"
     del tag
     global tag 
     tag = None

   if cbs is not None:
     print "delete cbs"
     del cbs
     global cbs 
     cbs = None

 #kill gattool
  
   p = subprocess.Popen(['ps', '-A'], stdout=subprocess.PIPE)
   out, err = p.communicate()

   for line in out.splitlines():
     if 'gatttool' in line:
       print "killing gattool"
       pid = int(line.split(None, 1)[0])
       os.kill(pid, signal.SIGKILL)

   time.sleep(2)

   
   tag = SensorTag(bluetooth_adr)
   cbs = SensorCallbacks(bluetooth_adr)
 
   print "restart finish"

   #enable temperature readings
   tag.char_write_cmd(0x24,01)
   #enable humidity readings
   tag.char_write_cmd(0x2c,01)

   restarting = False

if __name__ == "__main__":

    if len(sys.argv) < 2:
        print "put bluetooth address of the sensor device as the parameter"
        sys.exit(2)

    global tag
    global cbs
    global bluetooth_adr
    #TODO figure out how to set the address of the device as a parameter
    ad = sys.argv[1]
    tag = SensorTag(ad)
    cbs = SensorCallbacks(ad)
    bluetooth_adr =ad
    
    print "initialised"

    #enable temperature readings
    tag.char_write_cmd(0x24,01)
    #enable humidity readings
    tag.char_write_cmd(0x2c,01)

    sensortag_measure()

    global tag
    global cbs

    if tag is not None:
     print "delete tag"
     del tag

    if cbs is not None:
     print "delete cbs"
     del cbs
