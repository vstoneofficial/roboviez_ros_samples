#!/usr/bin/env python
import serial
import time
import struct
import rospy
from std_msgs.msg import Bool
from std_msgs.msg import Int16
from std_msgs.msg import Float64
from std_msgs.msg import Float64MultiArray
from std_msgs.msg import UInt8MultiArray
from roboviez_ros_msgs.msg import IMU
from roboviez_ros_msgs.msg import Gamepad
from roboviez_ros_msgs.msg import MotionMap
from roboviez_ros_msgs.msg import Walking
from roboviez_ros_msgs.srv import MemRead
from roboviez_ros_msgs.srv import MemWrite

con=serial.Serial('/dev/ttyAMA0', 115200)
servo_num=26

pow_update=False
power=False
mot_update=False
mot_num=0
walk_update=False
walk_enable=False
walk_v=0
walk_h=0
walk_z=0
memwrite_update=False
memwrite_addr=0
memwrite_data=[]
memread_update=False
memread_addr=0
memread_length=0
memread_data=[]


def callback_power(msg):

    global power
    global pow_update

    rospy.loginfo("recv power msg.")
    power=msg.data
    pow_update=True

def callback_uartmotion(msg):

    global mot_update
    global mot_num
    mot_num = msg.data
    mot_update=True



def callback_walking(msg):

    global walk_update
    global walk_enable
    global walk_v
    global walk_h
    global walk_z

    walk_enable=msg.walking
    walk_v=msg.v_axis
    walk_h=msg.h_axis
    walk_z=msg.z_axis
    walk_update=True




def handle_memmap_read(req):

    global memread_update
    global memread_addr
    global memread_length
    global memread_data

    memread_addr=req.Address
    memread_length=req.Length
    memread_data=[0,1,2]
    memread_update=True

    while memread_update:
        time.sleep(0.012)

    return [memread_data]


def handle_memmap_write(req):

    global memwrite_update
    global memwrite_addr
    global memwrite_data

    datalen = len(req.Buf.data)
    memwrite_data=[0]*datalen
    for num in range(datalen):
        memwrite_data[num] = ord(req.Buf.data[num])

    memwrite_addr=req.Address
    memwrite_update=True
    return []



def ishex(c):
    if '0' <= c <='9':
        return True
    elif 'a' <= c <='f':
        return True
    elif 'A' <= c <='F':
        return True

    return False


def tosword(msb,lsb):
    var = (msb<<8)|lsb
    if var>32768:
        var -= 65536
    return var

def serial_write(address,data):
    wbuff = "w {0:x}".format(address)
    for item in data:
        wbuff += " {0:02x}".format(item)
    wbuff += '\r'

    con.write(wbuff.encode())
    time.sleep(0.012)
    rbuff=con.read(con.inWaiting())



def serial_read(address,length):


    data=[0]*length
    try:
        wbuff = "r {0:x} {1:x}\r".format(address,length)

        con.write(wbuff.encode())
        time.sleep(0.012)
        rbuff=con.read(con.inWaiting())

        readlines=rbuff.splitlines()
        readlines.pop(0)

        for line in readlines:

            if line[0] == '#':
                params=line[1:].split()
                offset=int(params[0], 16) - address

                for num in range(1, len(params)):
 
                    index = offset+num-1
                    if 0 <= index < length:
                        param = params[num]
                        if len(param) == 2 and ishex(param[0]) and ishex(param[1]):
                            data[index] = int(param,16)
                        else:
                            break

    except:
        err_msg="catch exception from serial_read"
        #rospy.loginfo(err_msg)

    return data


def main():
    global pow_update
    global power
    global mot_update
    global mot_num
    global walk_update
    global walk_enable
    global walk_v
    global walk_h
    global walk_z
    global memwrite_update
    global memwrite_addr
    global memwrite_data
    global memread_update
    global memread_addr
    global memread_length
    global memread_data

    rospy.init_node('roboviez_ros_controller')
    rospy.loginfo("Start roboviez_ros_controller.")

    imu_pub = rospy.Publisher('/roboviez_ros_controller/imu', IMU, queue_size=500)
    gamepad_pub = rospy.Publisher('/roboviez_ros_controller/gamepad', Gamepad, queue_size=500)
    motionmap_pub = rospy.Publisher('/roboviez_ros_controller/motionmap', MotionMap, queue_size=500)
    voltage_pub = rospy.Publisher('/roboviez_ros_controller/voltage', Float64, queue_size=500)
    positions_pub = rospy.Publisher('/roboviez_ros_controller/positions', Float64MultiArray, queue_size=500)

    rospy.Subscriber('/roboviez_ros_controller/poweron', Bool, callback_power)
    rospy.Subscriber('/roboviez_ros_controller/uartmotion', Int16, callback_uartmotion)
    rospy.Subscriber('/roboviez_ros_controller/walking', Walking, callback_walking)

    rospy.Service('/roboviez_ros_controller/memmap_read', MemRead, handle_memmap_read)
    rospy.Service('/roboviez_ros_controller/memmap_write', MemWrite, handle_memmap_write)

    time.sleep(0.012)


    serial_write(0x09c2,[0,0,0,0,0,0]);


    imu = IMU()
    gamepad = Gamepad()
    motionMap = MotionMap()
    voltage = Float64()
    positions = [0]*servo_num


    rospy.loginfo("Establish connection..")
    r = rospy.Rate(10)
    while not rospy.is_shutdown():

        imu_data = serial_read(0x0ef2,14)
        imu.acc.x= tosword(imu_data[1], imu_data[0]) /16384.0;
        imu.acc.y= tosword(imu_data[3], imu_data[2]) /16384.0;
        imu.acc.z= tosword(imu_data[5], imu_data[4]) /16384.0;
        imu.temperature= tosword(imu_data[7], imu_data[6]) /16.0 + 25.0;
        imu.gyro.x = tosword(imu_data[9], imu_data[8]) /16.384;
        imu.gyro.y = tosword(imu_data[11], imu_data[10]) /16.384;
        imu.gyro.z = tosword(imu_data[13], imu_data[12]) /16.384;
        imu_pub.publish(imu)

        pad_data = serial_read(0x0ee4,12)
        gamepad.button=pad_data[0]|(pad_data[1]<<8)
        gamepad.stick_rx=tosword(pad_data[5],pad_data[4])
        gamepad.stick_ry=tosword(pad_data[7],pad_data[6])
        gamepad.stick_lx=tosword(pad_data[9],pad_data[8])
        gamepad.stick_ly=tosword(pad_data[11],pad_data[10])
        gamepad_pub.publish(gamepad)


        motnum_data = serial_read(0x0ff8,4)
        motionMap.motion_num=motnum_data[0]|(motnum_data[1]<<8)
        motionMap.map_num=motnum_data[2]|(motnum_data[3]<<8)
        motionmap_pub.publish(motionMap);


        voltage_data = serial_read(0x0090,2)
        voltage.data=tosword(voltage_data[1],voltage_data[0]) / 4096.0 *16.62
        voltage_pub.publish(voltage);


        position_data=[]
        position_data.extend(serial_read(0x0d80,12))
        position_data.extend(serial_read(0x0d80+12,12))
        position_data.extend(serial_read(0x0d80+12*2,12))
        position_data.extend(serial_read(0x0d80+12*3,16))

        for num in range(servo_num):
            pos = tosword(position_data[1+num*2],position_data[0+num*2]) /1800.0 * 3.1415926
            if pos<=3.1415926*2: 
                positions[num] = pos 

        pubpositions = Float64MultiArray(data=positions)
        positions_pub.publish(pubpositions)

        if pow_update == True:
            if power == True:
                rospy.loginfo("power on.")
                serial_write(0x0048,[01,00])
            else:
                rospy.loginfo("power off.")
                serial_write(0x0048,[00,00])
            pow_update=False

        if mot_update == True:
            lsb = mot_num&0xff
            msb=(mot_num>>8) & 0xff
            serial_write(0x09c0,[lsb,msb])
            rospy.loginfo("motion_num:{0}.".format(mot_num))
            mot_update = False

        if walk_update == True:
            if walk_enable == True :
                rospy.loginfo("walking start")
                serial_write(0x09c0,[01,00])
            else:
                rospy.loginfo("walking end")
                serial_write(0x09c0,[00,00])


            rospy.loginfo("axis input v: {0:f} , h:{1:f} ,turn:{1:f}".format(walk_v,walk_h,walk_z))

            bin=[0]*6;
            if walk_v<0 :
                bin[0]=int(255+walk_v*127)
                bin[1]=255
            else :
                bin[0]=int(walk_v*127)
                bin[1]=0
            if walk_h<0 :
                bin[2]=int(255+walk_h*127)
                bin[3]=255
            else :
                bin[2]=int(walk_h*127)
                bin[3]=0
            if walk_z<0 :
                bin[4]=int(255+walk_z*127)
                bin[5]=255
            else :
                bin[4]=int(walk_z*127)
                bin[5]=0

            serial_write(0x09c2,bin);

            walk_update = False

        if memwrite_update == True:
            rospy.loginfo("mem write request: addr={0:x}, length={1:x}".format(memwrite_addr, len(memwrite_data) ));
            serial_write(memwrite_addr,memwrite_data)

            memwrite_update = False


        if memread_update == True:
            rospy.loginfo("mem write request: addr={0:x}, length={1:x}".format(memread_addr, memread_length ));
            memread_data=serial_read(memread_addr, memread_length )

            memread_update = False

        r.sleep()

    rospy.loginfo("Exit roboviez_ros_controller.")

    con.close()

if __name__ =='__main__':
    main()
