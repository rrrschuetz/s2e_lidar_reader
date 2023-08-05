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
thresholds_amber=[(79, 58, 29, 4, 20, 77),(82, 18, 33, 16, 27, 76)]
thresholds_red=[(23, 63, 7, 56, 2, 49)]


while True:
    img = sensor.snapshot()
    img.gamma_corr(gamma = 1.0, contrast = 1.0, brightness = 0.2)
    img.laplacian(2, sharpen=True)

    cxy=[]
    a_pix = 0
    r_pix = 0

    blobs = img.find_blobs(thresholds_amber,0,roi,pixels_threshold=30, merge=True)
    if blobs:
        blobs.sort(key=lambda b: -b.pixels())
        img.draw_rectangle(blobs[0].rect())
        img.draw_cross(blobs[0].cx(), blobs[0].cy())
        (a_x1,y,a_x2,h) = blobs[0].rect()
        a_pix=blobs[0].pixels()

    blobs = img.find_blobs(thresholds_red,0,roi,pixels_threshold=30, merge=True)
    if blobs:
        blobs.sort(key=lambda b: -b.pixels())
        img.draw_rectangle(blobs[0].rect())
        img.draw_cross(blobs[0].cx(), blobs[0].cy())
        (r_x1,r,r_x2,h) = blobs[0].rect()
        r_pix=blobs[0].pixels()

    if a_pix != 0 and a_pix >= r_pix:
        print("AMBER",a_x1,a_x2)
    elif r_pix != 0 and r_pix >= a_pix:
        print("RED",r_x1,r_x2)
    else:
        print("NONE")

    time.sleep(0.1)
