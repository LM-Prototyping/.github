import cv2
import os

PREFIX = "real"
EVAL_DIR_NAME = "compass_sensor"

VIDEO_DIR = "../recording/" + PREFIX + "/edited/" + EVAL_DIR_NAME + ".mp4"
IMAGE_DIR = "../recording/" + PREFIX + "/" + EVAL_DIR_NAME + "/"

try:
    os.mkdir(IMAGE_DIR)
except:
    pass

video_cap = cv2.VideoCapture(VIDEO_DIR)

print("Frame Rate: ", video_cap.get(cv2.CAP_PROP_FPS))

frame_rate = int(video_cap.get(cv2.CAP_PROP_FPS))
milliseconds_per_frame = int((1.0 / frame_rate) * 1000.0)

success = True

current_frame_time = 0

while success:
    success, frame = video_cap.read()

    if not success:
        break

    cv2.imwrite(IMAGE_DIR + str(current_frame_time) + ".png", frame)

    current_frame_time += milliseconds_per_frame

    # print(frame, success)

# cv2.imshow("frame", frame)

# cv2.waitKey(0)