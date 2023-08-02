# module imgprocess.py

import sensor, image, time, math, pyb, lcd

uart = pyb.UART(3,115200) 

sensor.reset()
sensor.set_pixformat(sensor.RGB565)
sensor.set_framesize(sensor.QVGA)
sensor.set_auto_gain(False) # must be turned off for color tracking
sensor.set_auto_whitebal(False) # must be turned off for color tracking
sensor.skip_frames(time = 2000)

roi = [0,0,320,200]
thresholds=[(79, 58, 29, 4, 20, 77),(82, 18, 33, 16, 27, 76)]

while True:
    img = sensor.snapshot()
    img.gamma_corr(gamma = 1.0, contrast = 1.0, brightness = 0.2)
    img.laplacian(2, sharpen=True)

    cxy=[]
    blobs = img.find_blobs(thresholds,0,roi,pixels_threshold=30, merge=True)
    if blobs:
        blobs.sort(key=lambda b: -b.pixels()*b.cy()*b.cy())
        img.draw_rectangle(blobs[0].rect())
        img.draw_cross(blobs[0].cx(), blobs[0].cy())
        cxy.append(blobs[0].cx())
        cxy.append(blobs[0].cy())
        cxy.append(blobs[0].pixels()*blobs[0].cy()*blobs[0].cy())

        #uart.write(repr(cxy)+'\n')
        print(repr(cxy))
        time.sleep(0.1)
