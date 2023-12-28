import sensor, image, time, math, pyb, lcd, os

usb = pyb.USB_VCP()
red_led = pyb.LED(1)
green_led = pyb.LED(2)
blue_led = pyb.LED(3)

red_led.on()
green_led.off()
blue_led.off()

sensor.reset()
sensor.set_pixformat(sensor.RGB565)
sensor.set_framesize(sensor.QVGA)
sensor.set_auto_gain(True) # must be turned off for color tracking
sensor.set_auto_whitebal(True) # must be turned off for color tracking
sensor.skip_frames(time = 2000)

#yellow = (95,100,-24,2,12,46)
#red = (44,100,35,77,-9,31)
#green = (100, 26, -128, -29, 9, 80)
#blue = (31, 92, -19, 6, -64, -17)
#white = (82, 100, -22, 2, 43, 5)
#black = (18, 30, -30, -5, 38, -10)
#silver = (100, 255, 0, 64, 0, 64)

#green = (67, 51, -66, -29, 18, 56)
#green = (69, 38, -66, -20, 16, 55)
green = (54, 43, -32, -8, -8, 31)
red = (40, 74, 23, 84, 13, 62)
thresholds=[green, red]

roi = [0,0,320,100]

while True:

    #time.sleep(0.05)
    img = sensor.snapshot()
    img.lens_corr(strength=2.6, zoom=1.0)
    img.gamma_corr(gamma = 1.0, contrast = 1.0, brightness = 0.2)
    img.laplacian(2, sharpen=True)

    blob_entries = []
    blobs = img.find_blobs(thresholds,0,roi,pixels_threshold=640, merge=True)
    for blob in blobs:
        img.draw_rectangle(blob.rect(),color=(0,0,255),thickness=5)
        img.draw_cross(blob.cx(), blob.cy())
        (b_x,b_y,b_w,b_h) = blob.rect()
        blob_entries.append("{},{},{}".format(blob.code(), b_y, b_y+b_h))

    bloblist = ','.join(blob_entries)
    if bloblist:
        jpg = img.compress(quality=85)  # Compress image into JPEG format
        header = "STR,{},JPG,{}\n".format(len(bloblist), len(jpg))
        usb.write(header)
        usb.write(bloblist)
        usb.write(jpg)
