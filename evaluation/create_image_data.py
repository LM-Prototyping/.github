import cv2
import numpy as np
import os
import re

from multiprocessing import Process, Queue


"""
    Create Image Data

    maps over all images from a eval run, extracts coordinates of red and green markers
    on robot and saves everything in csv file
"""

PREFIX = "simulation"
EVAL_DIR_NAME = "Eval_TouchSensor_touch_sensor_04_28__15_28_21"

IMAGE_DIR = "../recording/" + PREFIX + "/" + EVAL_DIR_NAME
CROPPED_IMAGE_PATH = "cropped_recordings/" + PREFIX + "/" + EVAL_DIR_NAME

PROCESSING = 16

try:
    os.mkdir(CROPPED_IMAGE_PATH)
except:
    pass

images = np.array([(os.path.join(IMAGE_DIR, img), img) for img in os.listdir(IMAGE_DIR)])

upper_bound_red = np.array([50, 50, 255])
lower_bound_red = np.array([20, 20, 100])

upper_bound_green = np.array([100, 255, 100])
lower_bound_green = np.array([10, 100, 10])

green_mean_array = []
red_mean_array = []
robot_mean_array = []

images_splitted = np.array_split(images, PROCESSING)
value_queue = Queue(maxsize=len(images) + 1)

image_name_matcher = re.compile("\d+")

images_data = []


def find_coordinates_of_value_in_mask(mask, mask_size, value):
    coordinates = []
    x_size, _ = mask_size

    mask_flat = mask.flatten()

    for v, index in zip(mask_flat, range(len(mask_flat))):
        if v == value:
            y_coord = int(index / x_size)
            x_coord = index % x_size 

            coordinates.append((x_coord, y_coord))

    if len(coordinates) <= 0:
        return None

    return coordinates


def get_mean_of_coordinates(coordinates):
    _sum = np.array([0, 0])

    for pixel in coordinates:
        _sum += np.array(pixel)

    return np.array([int(x / len(coordinates)) for x in _sum])


def process(images, queue, process_index):
    counter = 0

    for path, img_name in images:

        img = cv2.imread(path)

        index = image_name_matcher.search(img_name).group()

        cropped_image = img[44:-43, 43:-44]

        # cv2.imwrite(CROPPED_IMAGE_PATH + "/" + img_name, cropped_image)

        # For mask each pixel is a value between 0 and 255
        mask_red = cv2.inRange(cropped_image, lower_bound_red, upper_bound_red)
        mask_green = cv2.inRange(cropped_image, lower_bound_green, upper_bound_green)

        # Get Green and Red Mean Coordinate
        red_pixel_coordinates = find_coordinates_of_value_in_mask(mask_red, mask_red.shape, 255)
        green_pixel_coordinates = find_coordinates_of_value_in_mask(mask_green, mask_green.shape, 255)
        
        if not red_pixel_coordinates or not green_pixel_coordinates:
            continue
        
        red_mean = get_mean_of_coordinates(red_pixel_coordinates)

        green_mean = get_mean_of_coordinates(green_pixel_coordinates)

        robot_middle = [int(i / 2) for i in red_mean + green_mean]

        queue.put((red_mean, green_mean, robot_middle, index))

        counter += 1

        print(f"{process_index}:", counter, "/", len(images))

    # queue.close()
    # queue.join_thread()


processes = [None] * PROCESSING

for i in range(PROCESSING):
    processes[i] = Process(target=process, args=(images_splitted[i], value_queue, i))
    processes[i].start()

for i in range(PROCESSING):
    processes[i].join()

for i in range(len(images)):
    try:
        red, green, robot, index = value_queue.get(timeout=1)

        green_mean_array.append(green)
        red_mean_array.append(red)
        robot_mean_array.append(robot)

        images_data.append((index, red, green, robot))
    except:
        pass

new_img = cv2.imread("simulation_empty.png")

for coord in robot_mean_array:
    cv2.circle(new_img, tuple(coord), 4, (0, 255, 0), -1)


# Sort images data by index
images_data.sort(key=lambda t: t[0])

# Save Image Data in CSV
with open(CROPPED_IMAGE_PATH + "/data.csv", "w") as file:
    file.write("timestep, red, green, robot_middle\n")
    for index, r, g, robot in images_data:
        file.write(f"{index}, {r}, {g}, {np.array(robot)}\n")

# Save Image
cv2.imwrite(CROPPED_IMAGE_PATH + "/path.png", new_img)

cv2.imshow("ImageWithCircle", new_img)

cv2.waitKey(0)
cv2.destroyAllWindows()