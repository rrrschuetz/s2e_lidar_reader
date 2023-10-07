import sensor, image, time, math, pyb, lcd, os

save_dir = "/sd/saved_images/"

#uart = pyb.UART(3,115200)
usb = pyb.USB_VCP()

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
#green = (52,100,-63,-26,-128,59)
green = (100, 26, -128, -29, 9, 80)
blue = (31, 92, -19, 6, -64, -17)
white = (82, 100, -22, 2, 43, 5)
black = (18, 30, -30, -5, 38, -10)
silver = (100, 255, 0, 64, 0, 64)

thresholds=[green, blue]
roi = [0,0,320,200]

min_degree = 70
max_degree = 110

def save_image_to_sd(img, counter):
    try:
        filename = save_dir + "image_{}.jpg".format(counter)
        img.save(filename)
        #print("Image saved to", filename)
    except Exception as e:
        print("Failed to save image:", e)

#counter = 0
while True:
    time.sleep(0.05)
    img = sensor.snapshot()
    img.lens_corr(strength=2.6, zoom=1.0)
    img.gamma_corr(gamma = 1.0, contrast = 1.0, brightness = 0.2)
    img.laplacian(2, sharpen=True)

    blob_entries = []
    blobs = img.find_blobs(thresholds,0,roi,pixels_threshold=200, merge=True)
    for blob in blobs:
        img.draw_rectangle(blob.rect(),color=(0,0,255),thickness=5)
        img.draw_cross(blob.cx(), blob.cy())
        (b_x,_,b_w,_) = blob.rect()
        blob_entries.append("{},{},{}".format(blob.code(), b_x, b_x+b_w))

    bloblist = ','.join(blob_entries)
    if bloblist:
        #save_image_to_sd(img, counter)
        #counter += 1
        #if counter > 99999: counter = 0
        #uart.write(bloblist)
        #print(bloblist)
        jpg = img.compress(quality=85)  # Compress image into JPEG format
        header = "STR,{},JPG,{}\nn".format(len(bloblist), len(jpg))
        usb.write(header)
        usb.write(bloblist)
        usb.write(jpg)
        continue

    num_lines = 0
    for l in img.find_lines(roi,threshold=2500, theta_margin=25, rho_margin=25):
        if (min_degree <= l.theta()) and (l.theta() <= max_degree):
            num_lines += 1
            img.draw_line(l.line(), color=(0, 0, 255),thickness=5)

    if num_lines > 0:
        #save_image_to_sd(img, counter)
        #counter += 1
        #if counter > 99999: counter = 0
        #uart.write("TARGET")
        #print("TARGET")
        text = "TARGET"
        jpg = img.compress(quality=85)  # Compress image into JPEG format
        header = "STR,{},JPG,{}".format(len(text), len(jpg))
        usb.write(header)
        usb.write(text)
        usb.write(jpg)
        continue

