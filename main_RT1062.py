import sensor, image
import machine

# Get the unique machine ID
unique_id = machine.unique_id()
unique_id_hex = ''.join(['{:02x}'.format(byte) for byte in unique_id])

sensor.reset()
sensor.set_pixformat(sensor.RGB565)
sensor.set_framesize(sensor.QVGA)
sensor.skip_frames(time = 2000)

sensor.set_auto_gain(False, gain_db = 20) # must be turned off for color tracking
sensor.skip_frames(time = 2000)

sensor.set_auto_whitebal(False) # must be turned off for color tracking
sensor.skip_frames(time = 2000)

sensor.set_saturation(3)

green1 =  (30, 100, -64, -8, -32, 32)  # generic green
green2 = (0, 100, -128, -6, 0, 127)
#green = (38,60,-9,-128,-128, 127)

red1 =  (30, 100, 15, 127, 15, 127)    # generic red
red2 = (0, 100, 44, 127, 127, -20)
#red = (0,49,127,6,-2,20)

magenta = (0, 100, 32, 127, 127, -94)

thresholds=[green1, green1, red1, red1, magenta]
roi = [0,0,320,140]

while True:
    try:
        img = sensor.snapshot()
        img.lens_corr(strength=2.6, zoom=1.0)
        #img.gamma_corr(gamma = 2.0)

        blob_entries = []
        blobs = img.find_blobs(thresholds,0,roi,pixels_threshold=160, merge=False)
        for blob in blobs:
            (b_x,b_y,b_w,b_h) = blob.rect()
            b_c = blob.code()
            if (b_c == 16 and b_h/b_w < 1) or (b_c in [1,2,4,8] and b_h/b_w > 1):
                #img.draw_rectangle(blob.rect(),color=(0,0,255),thickness=3)
                #img.draw_cross(blob.cx(), blob.cy())
                blob_entries.append("{},{},{}".format(b_c, b_x, b_x+b_w))

        bloblist = ','.join(blob_entries)
        if bloblist:
            print("{},".format(unique_id_hex)+bloblist+"\n")

    except Exception as e:
        pass
