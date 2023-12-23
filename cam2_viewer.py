import os
import time
from PIL import Image, ImageTk
import tkinter as tk

def find_latest_file(directory,pattern):
    jpg_files = [os.path.join(directory, f) for f in os.listdir(directory) if f.lower().endswith('.jpg') and pattern in f]
    if not jpg_files:
        return None 
    return max(jpg_files,key=os.path.getmtime)

def show_image(path, label):
    image = Image.open(path)
    image = image.transpose(Image.FLIP_TOP_BOTTOM) 
    image = image.transpose(Image.FLIP_LEFT_RIGHT)
    image.thumbnail((400, 300)) # Resize to fit half the window, adjust as needed
    photo = ImageTk.PhotoImage(image)

    label.config(image=photo)
    label.image = photo # Keep a reference

def main(directory, poll_interval=1):
    root = tk.Tk()
    root.geometry('800x600') # Adjust window size as needed

    frame = tk.Frame(root)
    frame.pack(expand=True, fill=tk.BOTH)

    label1 = tk.Label(frame)
    label1.pack(side=tk.RIGHT, expand=True, fill=tk.BOTH)

    label2 = tk.Label(frame)
    label2.pack(side=tk.LEFT, expand=True, fill=tk.BOTH)

    last_shown = []

    def update():
        latest1 = find_latest_file(directory,'image_1_')
        latest2 = find_latest_file(directory,'image_2_')
        show_image(latest1,label1)
        show_image(latest2,label2)
        root.after(poll_interval * 10, update)

    update()
    root.mainloop()

if __name__ == "__main__":
    directory = '/mnt/rpi/test/saved_images' # Change this to your NFS-mounted folder
    main(directory)
