
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

import asyncio
from evdev import UInput, categorize, ecodes, AbsInfo, resolve_ecodes, event_factory, list_devices, InputDevice
import sys
import time
import multiprocessing as mp
import atexit
import select

cap = {
   ecodes.EV_FF:  [
    ecodes.FF_AUTOCENTER,
    ecodes.FF_CONSTANT,
    ecodes.FF_PERIODIC,
    ecodes.FF_SINE,
    ecodes.FF_TRIANGLE,
    ecodes.FF_SQUARE,
    ecodes.FF_RAMP,
    ecodes.FF_SPRING,
    ecodes.FF_FRICTION,
    ecodes.FF_DAMPER,
    ecodes.FF_RUMBLE,
    ecodes.FF_INERTIA,
    ecodes.FF_GAIN,
    ecodes.FF_CUSTOM
   ],
   ecodes.EV_KEY: [ecodes.KEY_A, ecodes.KEY_B],
   ecodes.EV_ABS: [(ecodes.ABS_X, AbsInfo(value=0, min=-32768, max=32768, fuzz=0, flat=0, resolution=0)),
                    (ecodes.ABS_Y, AbsInfo(value=0, min=-32768, max=32768, fuzz=0, flat=0, resolution=0)),
                    (ecodes.ABS_Z, AbsInfo(value=0, min=-32768, max=32768, fuzz=0, flat=0, resolution=0))],
   #ecodes.EV_REL: [ecodes.REL_X, ecodes.REL_Y]
}


JOY_MATCH = "Arduino LLC Arduino Leonardo"



def process_event(event, device, effects, gain):
    print(categorize(event))

    # print(resolve_ecodes(cap, [event.code]))
    # print(device)

    if event.type == ecodes.EV_SYN:
        print("============== SYN ==============")

    if event.type == ecodes.EV_FF:
        if event.code == ecodes.FF_GAIN:
            print("SETTING GAIN TO {}".format(event.value))
            gain = event.value
            #pass
        else:
            try:
                print("code {}, type: {}, value: {}".format(ecodes.FF[effects[event.code]], ecodes.EV[event.type], event.value))
            except:
                print("Exception")
                print("code {}, type: {}, value: {}".format(event.code, event.type, event.value))
    else:
        print("code {}, type: {}, value: {}".format(event.code, event.type, event.value))

    # Wait for an EV_UINPUT event that will signal us that an
    # effect upload/erase operation is in progress.
    if event.type == ecodes.EV_UINPUT:

        if event.code == ecodes.UI_FF_UPLOAD:
            #print(type(event.type))
            #print("GGGGGGGGGGG: {}".format(event.type))
            upload = device.begin_upload(event.value)
            upload.retval = 0
            effects.append(upload.effect.type)

            print(f'[upload] effect_id: {upload.effect.id}, type: {upload.effect.type}')
            device.end_upload(upload)

        elif event.code == ecodes.UI_FF_ERASE and False:
            erase = device.begin_erase(event.value)
            print(f'[erase] effect_id {erase.effect_id}')

            erase.retval = 0
            device.end_erase(erase)
    return effects


async def print_events(device):
    effects = []
    gain = 1.0
    axis = 0
    async for event in device.async_read_loop():
        process_event(event, device, effects, gain)


def main(pipe=None):

    joy = None

    for fn in list_devices():
        d = InputDevice(fn)
        #print(d.name)
        if d.name == JOY_MATCH:
           joy = d

    if joy:
        atexit.register(joy.ungrab)  # Don't forget to ungrab the joystick on exit!
        joy.grab()
    else:
        print("PEDALS NOT FOUND.")

    print("Create device")
    ui = UInput(cap, name='Steering Wheel', version=0x2)
    # print(ui.capabilities(verbose=True).keys())
    # for val in dir(ui):
    #     print(val)

    print(ui.device)
    # print(ui.capabilities()[21])

    # print(ecodes.FF)

    effects = []
    gain = 1.0
    axis = None
    pedal1 = 0
    pedal2 = 0
    pedal1min = 32768
    pedal2min = 32768
    last_axis = None
    while True:
        while True:
            event = ui.read_one()
            print(dir(ui))
            if event == None:
                break
            effects = process_event(event, ui, effects, gain)

        while True and joy:
            event = joy.read_one()
            if event == None:
                break
            else:
                if event.code==ecodes.ABS_X and event.type==3:
                    pedal1 = event.value
                    if pedal1 < pedal1min:
                        pedal1min = pedal1

                if event.code==ecodes.ABS_Y and event.type==3:
                    pedal2 = event.value
                    if pedal2 < pedal2min:
                        pedal2min = pedal2
                #    print(categorize(event))
                    #print("code {}, type: {}, value: {}".format(event.code, event.type, event.value))


        #print("outside loop")
        time.sleep(0.01)
        if pipe != None:
            while pipe.poll():
                axis = int(0 - pipe.recv())
                # if last_axis == None:
                    # last_axis = axis
                    # ui.write(ecodes.EV_ABS, ecodes.ABS_X, axis)
                    # ui.write(ecodes.EV_ABS, ecodes.ABS_Y, axis)
                    # ui.syn()
                #print("GOT AXIS {}".format(axis))
                # axis_diff = last_axis - axis
                p1range = 32768 - pedal1min
                p2range = 32768 - pedal2min

                try:
                    p1val = (pedal1 - pedal1min)/p1range * 2 * 32768 - 32768
                    p2val = (pedal2 - pedal2min)/p2range * 2 * 32768 - 32768
                except ZeroDivisionError:
                    p1val = 0
                    p2val = 0


                #print("{}, {}, {}".format(p1range, pedal1, pedal1min))
                ui.write(ecodes.EV_ABS, ecodes.ABS_X, axis)
                ui.write(ecodes.EV_ABS, ecodes.ABS_Y, int(p1val))
                ui.write(ecodes.EV_ABS, ecodes.ABS_Z, int(p2val))
                ui.syn()
                # last_axis = axis



    # asyncio.ensure_future(print_events(ui))
    # loop = asyncio.get_event_loop()
    # loop.run_forever()
    #
    # axis+=1
    # device.write(ecodes.EV_ABS, ecodes.ABS_X, axis)
    # device.syn()

if __name__=="__main__":
    main()

