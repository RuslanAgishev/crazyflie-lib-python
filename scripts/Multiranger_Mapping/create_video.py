import cv2
import os

image_folder = 'images1'
video_name = 'video1.avi'

images = [img for img in os.listdir(image_folder) if img.endswith(".png")]
frame = cv2.imread(os.path.join(image_folder, images[0]))
height, width, layers = frame.shape

video = cv2.VideoWriter(video_name, fourcc=cv2.VideoWriter_fourcc(*'XVID'), fps=30, frameSize=(width,height))

for i in range(1,len(images)-1):
    print(i)
    image_name = str(i)+'.png'
    i += 1
    video.write(cv2.imread(os.path.join(image_folder, image_name)))

cv2.destroyAllWindows()
video.release()