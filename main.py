import sensor, image, time, math, pyb, lcd

uart = pyb.UART(3,115200)

sensor.reset()
sensor.set_pixformat(sensor.RGB565)
sensor.set_framesize(sensor.QVGA)
sensor.set_auto_gain(True) # must be turned off for color tracking
sensor.set_auto_whitebal(True) # must be turned off for color tracking
sensor.skip_frames(time = 2000)

#yellow = (76, 86, 10, -15, 67, 35)
yellow = (95,100,-24,2,12,46)
#red = (58, 36, 58, 33, 54, -1)
red = (44,100,35,77,-9,31)
#green = (37, 52, -39, -27, 37, 17)
green = (52,100,-63,-26,-128,59)
blue = (31, 92, -19, 6, -64, -17)
white = (82, 100, -22, 2, 43, 5)
black = (18, 30, -30, -5, 38, -10)
silver = (100, 255, 0, 64, 0, 64)

thresholds=[yellow, red, green, blue]
roi = [0,0,320,200]

counter = 0
while True:
    time.sleep(0.05)
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
    if bloblist:
        filename = "image."+str(counter)+".jpg"
        #img.save(filename)
        counter += 1
        if counter > 999: counter = 0
        uart.write(bloblist)
        print(bloblist)
        continue

    gray_img = img.to_grayscale()
    gray_img.binary([silver])
    lines = gray_img.find_lines(threshold=1000)
    for l in lines:
        if abs(l.theta()) < 10:
            img.draw_line(l.line()),color=255)
            uart.write("TARGET")
            print("TARGET")
            continue
    
