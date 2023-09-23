import sensor, image, time, math, pyb, lcd

uart = pyb.UART(3,115200)

sensor.reset()
sensor.set_pixformat(sensor.RGB565)
sensor.set_framesize(sensor.QVGA)
sensor.set_auto_gain(False) # must be turned off for color tracking
sensor.set_auto_whitebal(False) # must be turned off for color tracking
sensor.skip_frames(time = 2000)

yellow = (76, 86, 10, -15, 67, 35)
red = (58, 36, 58, 33, 54, -1)
green = (37, 52, -39, -27, 37, 17)
blue = (31, 92, -19, 6, -64, -17)
white = (82, 100, -22, 2, 43, 5)
black = (18, 30, -30, -5, 38, -10)

thresholds=[yellow, red, green, blue]
roi = [0,0,320,200]

while True:
    img = sensor.snapshot()
    img.gamma_corr(gamma = 1.0, contrast = 1.0, brightness = 0.2)
    img.laplacian(2, sharpen=True)

    blob_entries = []
    blobs = img.find_blobs(thresholds,0,roi,pixels_threshold=200, merge=True)
    for blob in blobs:
        img.draw_rectangle(blob.rect())
        img.draw_cross(blob.cx(), blob.cy())
        (b_x,_,b_w,_) = blob.rect()
        blob_entries.append("{},{},{}".format(blob.code(), b_x, b_x+b_w))

    bloblist = ','.join(blob_entries)
    uart.write(bloblist)
    print(bloblist)
    #time.sleep(0.05)
