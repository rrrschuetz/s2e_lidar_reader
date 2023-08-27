# module main.py for s2e_slidar_reader

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
thresholds_white = [(100, 73, -15, 4, -15, 4)]
thresholds_blue = [(31, 92, -19, 6, -64, -17)]

while True:
    img = sensor.snapshot()
    img.gamma_corr(gamma = 1.0, contrast = 1.0, brightness = 0.2)
    img.laplacian(2, sharpen=True)

    a_pix = 0
    r_pix = 0

    blobs = img.find_blobs(thresholds_blue,0,roi,pixels_threshold=30, merge=True)
    if blobs:
        blobs.sort(key=lambda b: -b.pixels())
        img.draw_rectangle(blobs[0].rect())
        img.draw_cross(blobs[0].cx(), blobs[0].cy())
        (_,a_y,_,a_h) = blobs[0].rect()
        a_pix=blobs[0].pixels()

    blobs = img.find_blobs(thresholds_amber,0,roi,pixels_threshold=30, merge=True)
    if blobs:
        blobs.sort(key=lambda b: -b.pixels())
        img.draw_rectangle(blobs[0].rect())
        img.draw_cross(blobs[0].cx(), blobs[0].cy())
        (_,r_y,_,r_h) = blobs[0].rect()
        r_pix=blobs[0].pixels()

    if a_pix != 0 and a_pix >= r_pix:
        data_str = "{},{},{}".format(1.0, a_y, a_y+a_h)
    elif r_pix != 0 and r_pix >= a_pix:
        data_str = "{},{},{}".format(2.0, r_y, r_y+r_h)
    else:
        data_str = "{},{},{}".format(0.0, 0, 0)

    uart.write(data_str)
    print(data_str)

    time.sleep(0.05)
