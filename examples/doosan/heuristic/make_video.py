import cv2
import os
from tqdm import tqdm
import natsort

image_folder = "benchmark2/bench2_scene/random"
video_name = "benchmark2/bench2_random.mp4"

folders = os.listdir(image_folder)
folders = natsort.natsorted(folders)
images = [img for img in folders if img.endswith(".png")]

frame = cv2.imread(os.path.join(image_folder, images[0]))
height, width, layers = frame.shape

video = cv2.VideoWriter(
    video_name, cv2.VideoWriter_fourcc(*"DIVX"), 60, (width, height)
)
for image in tqdm(images):
    video.write(cv2.imread(os.path.join(image_folder, image)))

cv2.destroyAllWindows()
video.release()
