
"""
MIT LICENSE:
https://mit-license.org/
Copyright © 2020 TLA
Permission is hereby granted, free of charge, to any person obtaining a copy of
this software and associated documentation files (the “Software”), to deal in
the Software without restriction, including without limitation the rights to
use, copy, modify, merge, publish, distribute, sublicense, and/or sell copies of
the Software, and to permit persons to whom the Software is furnished to do so,
subject to the following conditions:
The above copyright notice and this permission notice shall be included in all
copies or substantial portions of the Software.
"""

import serial
import serial.tools.list_ports
from serial import tools
import pyvesc
import sys
import time
import struct
from pyvesc import VESCMessage
import math
import matplotlib.pyplot as plt
import random
from pykalman import KalmanFilter
import multiprocessing as mp
import select

_ZERO_POSITION = -65
_COUNTS_PER_ROTATION = 16384
_COUNTS_TO_DEGREES = 360/_COUNTS_PER_ROTATION

ports = serial.tools.list_ports.comports(include_links=False)

for p in ports:
    if 'ChibiOS' in str(p.product):
        vesc_port = p.device

ser = serial.Serial(vesc_port)  # open serial port

ser.reset_output_buffer()
ser.reset_input_buffer()


# ser.close()
#
# sys.exit(0)

# while True:
#
#     # make a SetDutyCycle message
# my_msg = pyvesc.SetDutyCycle(1e5)
# print(my_msg.duty_cycle) # prints value of my_msg.duty_cycle
# my_packet = pyvesc.encode(my_msg)
# # my_packet (type: bytes) can now be sent over your UART connection
#
# # buff is bytes filled from your UART connection
# my_msg, consumed = pyvesc.decode(buff)
# buff = buff[consumed:]  # remove consumed bytes from the buffer
# if my_msg:
#   print(my_msg.duty_cycle)    # prints value of my_msg.duty_cycle


def unpack(msg_bytes):
    msg_id = struct.unpack_from(VESCMessage._endian_fmt + VESCMessage._id_fmt, msg_bytes, 0)
    #print(VESCMessage._msg_registry)
    msg_type = VESCMessage.msg_type(*msg_id)
    #VESCMessage.msg_type(*msg_id)
    data = None
    data = list(struct.unpack_from(VESCMessage._endian_fmt + msg_type._fmt_fields, msg_bytes, 1))
    for k, field in enumerate(data):
        try:
            data[k] = data[k]/msg_type._field_scalars[k]
        except (TypeError, IndexError) as e:
            pass
    return data[0]



class EncoderMessage(metaclass=pyvesc.VESCMessage):
    id = 38
    fields = [
        ('encoder_value', 'i')
    ]



def set_rpm(rpm):
    msg = pyvesc.SetRPM(rpm)
    msg.id=39
    ser.write(pyvesc.encode(msg))
    while not ser.in_waiting > 9:
        time.sleep(0.01)
    buffer = ser.read(10)
    response = pyvesc.decode(buffer)
    if response[0]:
        return(response[0].encoder_value)

def set_brake_current(current):
    msg = pyvesc.SetCurrentBrake(current)
    ser.write(pyvesc.encode(msg))

def set_current(current):
    msg = pyvesc.SetCurrent(current)
    msg.id=39
    ser.write(pyvesc.encode(msg))
    while not ser.in_waiting > 9:
        pass
        time.sleep(0.00001)
    buffer = ser.read(10)
    response = pyvesc.decode(buffer)
    if response[0]:
        return(response[0].encoder_value)

#
# speed = 0
#
# while True:
#     set_rpm(speed)
#     time.sleep(0.1)
#     speed += 100

def main(pipe = None):

    plotdata = []
    _BAD_DIFF_THRESHOLD = 5000000
    MAX_CURRENT_AMPS = 15
    _MILLIAMPS_PER_AMP = 1000
    MAX_CURRENT_MILLIAMPS = MAX_CURRENT_AMPS * _MILLIAMPS_PER_AMP
    current_milliamps = 0
    start_pos = set_current(current_milliamps)
    print(start_pos)
    # sys.exit()
    zero_pos = (start_pos- _ZERO_POSITION)%_COUNTS_PER_ROTATION
    if zero_pos > _COUNTS_PER_ROTATION/2.0:
        zero_pos -= _COUNTS_PER_ROTATION

    home_position = start_pos + zero_pos

    # print(start_pos)
    # # print(zero_pos)
    # sys.exit()
    setpoint = zero_pos * -1
    wait_time_s = 50
    start_time = time.time()
    last_error = 0
    cumulative_error = 0
    # rpm_changeover = 500000
    samplenum = 0
    position_list = []
    error_list = []
    diff_list = []
    loop_time = time.time()
    pipe_count = 0
    motor_enabled = True
    sys.stdin = open(0)
    try:
        while True:
            #current_milliamps = 000
            # if abs(current) < rpm_changeover:
            i,o,e = select.select([sys.stdin],[],[],0.0001)
            for s in i:
                if s == sys.stdin:
                    input = sys.stdin.readline()
                    if input:
                        print("Got Input: {}".format(input))
                        motor_enabled = motor_enabled==False
            _GAIN_FACTOR = 1.0
            if not motor_enabled:
                _GAIN_FACTOR = 0.0
            if abs(current_milliamps) > 20:
                updated_pos = set_current(int(current_milliamps*_GAIN_FACTOR)) - start_pos
            else:
                updated_pos = set_current(0) - start_pos
            #print((updated_pos + zero_pos) * _COUNTS_TO_DEGREES)
            # else:
            #     updated_pos = set_current(int(math.copysign(8000,current))) - start_pos


            # updated_pos = set_current(3000)
            #position_list.append(updated_pos)
            duration = time.time() - start_time
            if duration > wait_time_s:
                start_time = time.time()
                duration = 0
                # setpoint += _COUNTS_PER_ROTATION * random.choice([1,-1]) * 0.5
                # cumulative_error = 0
            #setpoint = math.sin(duration/wait_time_s * 2 * math.pi) * 400000 # + 200000
            #setpoint = 0
            error = updated_pos - setpoint
            # error_list.append(error)
            # if abs(error) < 1000:
            #     cumulative_error = 0
            # cumulative_error += error
            # cumulative_error *= 0.3
            _NUM_ERR_AVG = 10
            # recent_errors = error_list[-_NUM_ERR_AVG:]
            # recent_error_diffs = 0
            # for index in range(len(recent_errors)-1):
            #     recent_error_diffs += (recent_errors[index] - recent_errors[index+1])
            # recent_error_diffs /= -(_NUM_ERR_AVG-1)
            loop_duration = time.time() - loop_time
            loop_time = time.time()


            diff = (error - last_error) / loop_duration
            if abs(diff) < _BAD_DIFF_THRESHOLD:
                diff_list.append(diff)
                diff_list = diff_list[-_NUM_ERR_AVG:]
            diff_average = sum(diff_list)/_NUM_ERR_AVG
            #print("Recent: {}, Error: {}".format(diff_average, diff))
            last_error = error
            # if abs(error) < 2000:
            #     p_gain = -0.425
            #     d_gain = -2
            # else:
            p_gain = -25.0
            d_gain = -3.5
            # if
            if abs(diff_average) < 1000:
                diff_average = 0
            d_adjustment = diff_average * d_gain
            #print(d_adjustment)
            # DMAX = 5000
            # if d_adjustment > DMAX:
            #     d_adjustment = DMAX
            # if d_adjustment < -DMAX:
            #     d_adjustment = -DMAX

            if abs(error) < 50 or abs(diff_average) < 1000:
                MAX_CURRENT_CHANGE = 200
            else:
                MAX_CURRENT_CHANGE = 1000



            new_current = error * p_gain + d_adjustment #+ cumulative_error * -0.0


            # Set maximum rate of change to current control.
            current_diff = new_current - current_milliamps
            if current_diff > MAX_CURRENT_CHANGE:
                current_milliamps+=MAX_CURRENT_CHANGE
            elif current_diff < -MAX_CURRENT_CHANGE:
                current_milliamps-=MAX_CURRENT_CHANGE
            else:
                current_milliamps = new_current

            current_milliamps = int(current_milliamps)

            if current_milliamps > MAX_CURRENT_MILLIAMPS:
                current_milliamps = MAX_CURRENT_MILLIAMPS
            if current_milliamps < -MAX_CURRENT_MILLIAMPS:
                current_milliamps = -MAX_CURRENT_MILLIAMPS

            position_degrees = ((updated_pos + zero_pos))


            FACTOR = 2.0

            pipe_count+=1
            if pipe_count%10==0:
                pipe_count=0
                if pipe != None:
                    pipe.send(int(position_degrees * FACTOR))

            # if abs((updated_pos + zero_pos) * _COUNTS_TO_DEGREES) > 450:
            #     current_milliamps = int(current_milliamps * 1.5)
            #plotdata.append(updated_pos)
            #if duration > 1:
            #plotdata.append([duration,diff])
            plotdata.append(diff)

            #print("position: {}, loop rate: {}".format(updated_pos * _COUNTS_TO_DEGREES, 1/(loop_duration)))
            #print("{},{},{}".format(samplenum, updated_pos, setpoint))
            samplenum+=1
            time.sleep(0.001)
            ser.reset_input_buffer()
    except KeyboardInterrupt:
        set_brake_current(2000)

    kf = KalmanFilter(initial_state_mean=0, n_dim_obs=2)

    # params = [[0.99762599]
    #  [1.65527125]
    #  [1.8524171 ]]

    # kf_vals =  kf.em(plotdata).smooth([[2,0], [2,1], [2,2]])[0]
    # print(kf_vals)
    # plotdata, _ = kf.filter(plotdata)
    plt.plot(plotdata)
    plt.ylabel('some numbers')
    #plt.show()

if __name__=="__main__":
    main()
