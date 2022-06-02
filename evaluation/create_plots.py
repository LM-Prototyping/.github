from array import array
import csv
import math
import numpy as np
import cv2
import os

from mpl_toolkits import mplot3d
import matplotlib.pyplot as plt


SETUP_NAME = "set_pow_right"

setup = {
    "compass_sensor": ["compass_sensor", "Eval_CompassSensor_compass_sensor_04_28__19_28_48"],
    "distance_sensor": ["distance_sensor", "Eval_DistanceSensor_distance_sensor_05_13__14_44_39"],
    "light_sensor": ["light_sensor", "Eval_LightSensor_light_sensor_04_28__15_17_22"],
    "touch_sensor": ["touch_sensor", "Eval_TouchSensor_touch_sensor_04_28__15_28_21"],

    "set_pos_left": ["set_pos_left", "Eval_NoSensors_set_position_turn_left_04_20__22_08_46"],
    "set_pos_right": ["set_pos_right", "Eval_NoSensors_set_position_turn_right_04_21__00_02_05"],
    
    "set_pow_left": ["set_pow_left", "Eval_NoSensors_set_power_turn_left_04_21__20_13_50"],
    "set_pow_right": ["set_pow_right", "Eval_NoSensors_set_power_turn_right_04_21__20_18_25"],

    "set_vel_left": ["set_vel_left", "Eval_NoSensors_set_velocity_turn_left_04_21__19_49_28"],
    "set_vel_right": ["set_vel_right", "Eval_NoSensors_set_velocity_turn_right_04_21__19_56_08"],
}

current_setup = setup[SETUP_NAME]


REAL_EVAL_DIR_NAME = current_setup[0]
REAL_PATH = "cropped_recordings/real/" + REAL_EVAL_DIR_NAME

SIM_EVAL_DIR_NAME = current_setup[1]
SIM_PATH = "cropped_recordings/simulation/" + SIM_EVAL_DIR_NAME

RESULT_DIR = "results/" + REAL_EVAL_DIR_NAME + "/"

try:
    os.mkdir(RESULT_DIR)
except:
    pass


array_from_string = lambda s: np.fromstring(s.replace("[", "").replace("]", ""), sep=" ", dtype=int)

def read_csv_file(file):
    next(file)
    reader = csv.reader(file, delimiter=",")
    rows = []

    for row in reader:
        rows.append(row)
        # print(", ".join(row))

    rows.sort(key=lambda t: int(t[0]))

    return rows


def create_timestep_dict(timesteps):
    timestep_dict = {}
    last_time = None

    for timestep in timesteps:
        time, red, green, robot_middle = timestep
        
        timestep_dict[int(time)] = {
            "red": array_from_string(red),
            "green": array_from_string(green),
            "robot_middle": array_from_string(robot_middle)
        }

        if not last_time == None:
            timestep_dict[last_time]["next_step"] = int(time)

        last_time = int(time)
    
    return timestep_dict


def sub_vector(a, b):
    return [a[0] - b[0], a[1] - b[1]]

def norm(a):
    return math.sqrt(a[0] ** 2 + a[1] ** 2)

def scalar_product(a, b):
    return a[0] * b[0] + a[1] * b[1]

def is_left(a, b, c):
    return ((b[0] - a[0]) * (c[1] - a[1]) - (b[1] - a[1]) * (c[0] - a[0])) > 0

def mult_vector(factor, v):
    return [factor * v[0], factor * v[1]]


def calc_real_offset(real_zero_time, sim_zero_time):
    offset = sub_vector(sim_zero_time["robot_middle"], real_zero_time["robot_middle"])
    
    print(offset)

    new_r = real_zero_time["red"] + offset

    real_r_norm = sub_vector(new_r, sim_zero_time["robot_middle"])
    sim_r_norm = sub_vector(sim_zero_time["red"], sim_zero_time["robot_middle"])
    sim_g_norm = sub_vector(sim_zero_time["green"], sim_zero_time["robot_middle"])

    is_point_left = is_left(sim_r_norm, sim_g_norm, real_r_norm)
    phi = math.acos(
        scalar_product(real_r_norm, sim_r_norm) / 
        (norm(real_r_norm) * norm(sim_r_norm))
    ) * (1 if is_point_left else -1)

    rotation = np.matrix([
        [math.cos(phi), - math.sin(phi)],
        [math.sin(phi), math.cos(phi)]
    ])

    dist_sim_r = norm(sim_r_norm)
    dist_real_r = norm(real_r_norm)

    scale = dist_sim_r / dist_real_r

    return offset, rotation, sim_zero_time["robot_middle"], scale


def real_point_to_sim(real_point, offset, rot_origin, scale):
    point = real_point + offset

    point_new_origin = sub_vector(point, rot_origin)

    return [int(i) for i in [i + j for i, j in zip(mult_vector(scale, point_new_origin), rot_origin)]]


def draw_points_real(img, real_timesteps):
    start_time = 0

    timestep = real_timesteps[start_time]

    y_val = []
    x_val = []

    while timestep:
        y_val.append(timestep["robot_middle"][0])
        x_val.append(timestep["robot_middle"][1])

        cv2.circle(img, tuple(timestep["robot_middle"]), 4, (255, 0, 0), -1)

        if not timestep.get("next_step") or not real_timestep_dict.get(timestep.get("next_step")):
            break

        timestep = real_timesteps[timestep["next_step"]]

    # cv2.line(img, tuple(real_timesteps[0]["red"] + offset), tuple(real_timesteps[0]["green"] + offset), (100, 0, 0), 4)

    return x_val, y_val 


def normalise_real_points(real_timesteps, offset, rot_origin, scale):
    for key in real_timesteps.keys():
        old_point = real_timesteps[key]["robot_middle"]

        real_timesteps[key]["robot_middle"] = real_point_to_sim(old_point, offset, rot_origin, scale)

    return real_timesteps


def draw_points_sim(img, sim_timesteps):
    start_time = 0

    timestep = sim_timesteps[start_time]

    y_val = []
    x_val = []

    while timestep:
        y_val.append(timestep["robot_middle"][0])
        x_val.append(timestep["robot_middle"][1])

        cv2.circle(img, tuple(timestep["robot_middle"]), 4, (0, 0, 255), -1)

        if not timestep.get("next_step"):
            break

        timestep = sim_timesteps[timestep["next_step"]]

    cv2.line(img, tuple(sim_timesteps[0]["red"]), tuple(sim_timesteps[0]["green"]), (0, 0, 100), 4)

    return x_val, y_val

def pixelInMeter(pixels):
    return ((math.tan(1 / 2) * 4) / 1080) * pixels

def create_real_pos_for_timesteps(sim_timesteps, real_timesteps):
    SIM_TIMESTEP = 32

    timesteps = [i for i in sim_timesteps.keys()]

    new_real_timestep_dict = {}

    timesteps.sort(key=lambda t: t)

    curr_real_timestep_obj = real_timesteps[0]
    current_real_timestep = 0

    for timestep in timesteps:
        if not curr_real_timestep_obj.get("next_step"):
            break

        if timestep >= curr_real_timestep_obj["next_step"]:
            current_real_timestep = curr_real_timestep_obj["next_step"]
            curr_real_timestep_obj = real_timesteps[curr_real_timestep_obj["next_step"]]

        if current_real_timestep == timestep:
            new_real_timestep_dict[timestep] = { 
                "robot_middle": curr_real_timestep_obj["robot_middle"], 
                "next_step": timestep + SIM_TIMESTEP 
            }
            continue

        if curr_real_timestep_obj.get("next_step") and timestep > current_real_timestep and timestep < curr_real_timestep_obj["next_step"]:
            next_real_timestep = real_timesteps[curr_real_timestep_obj["next_step"]]
            
            diff = [i - j for i, j in zip(next_real_timestep["robot_middle"], curr_real_timestep_obj["robot_middle"])]

            x = (timestep - current_real_timestep)

            
            new_pos = [int(i) for i in (np.array(curr_real_timestep_obj["robot_middle"]) + np.array([(x / (curr_real_timestep_obj["next_step"] - current_real_timestep)) * i for i in diff]))]
            
            # print(timestep, current_real_timestep, x, diff, new_pos, curr_real_timestep_obj["robot_middle"], next_real_timestep["robot_middle"])
            
            new_real_timestep_dict[timestep] = { "robot_middle": list(new_pos), "next_step": timestep + SIM_TIMESTEP }
            # curr_real_timestep_obj["robot_middle"] = list(new_pos)

    return new_real_timestep_dict


def create_plots(sim_timestep, real_timestep):
    current_time = 0

    sim_step = sim_timestep[current_time]
    real_step = real_timestep[current_time]

    y_val = []
    x_val = []

    max = (0, 0)

    while (
        sim_step.get("next_step") 
        and sim_timestep.get(sim_step.get("next_step"))
        and real_step.get("next_step") 
        and real_timestep.get(real_step.get("next_step"))
    ):
        difference = sim_step["robot_middle"] - real_step["robot_middle"]

        # print(sim_step["robot_middle"], real_step["robot_middle"], difference)

        difference_in_meter = pixelInMeter(norm(difference))

        y_val.append(difference_in_meter)
        x_val.append(current_time / 1000)

        if difference_in_meter > max[0]:
            max = difference_in_meter, current_time / 1000

        sim_step = sim_timestep[sim_step.get("next_step")]
        real_step = real_timestep[real_step.get("next_step")]
        
        current_time = sim_step.get("next_step")

        pass

    text = "max difference with {:.3f}m at time={:.3f}s".format(max[0], max[1])

    bbox_props = dict(boxstyle="square,pad=0.3", fc="w", ec="k", lw=0.72)
    kw = dict(xycoords='data',textcoords="axes fraction",
              bbox=bbox_props, ha="right", va="top")

    plt.axhline(y=max[0], color="r", linestyle="-")

    plt.xlabel("Time in [s]")
    plt.ylabel("Distance in [m]")
    # plt.yticks([0, max[0]], labels=["0", "max"])
    
    plt.annotate(text, xy=(max[1], max[0]), xytext=(0.94, 0.96), **kw)
    plt.plot(x_val, y_val, label="Error between robot path in simulation and reality")
    plt.legend(loc="lower right")
    plt.savefig(RESULT_DIR + "difference_plot.png")
    plt.show()


def create_3d_plot(sim_timesteps, real_timesteps):
    def _extract_axes(timesteps):
        z_data = list(timesteps.keys())
        z_data.sort(key=lambda t: t)

        x_data = []
        y_data = []

        for time in z_data:
            point = timesteps[time]["robot_middle"]

            x_data.append(point[0])
            y_data.append(point[1])

        return x_data, y_data, z_data

    def _draw_plots(ax, x_data, y_data, z_data, label):        
        ax.plot3D(x_data, y_data, z_data, "gray")
        ax.scatter3D(x_data, y_data, z_data, cmap="Greens", label=label)
 

    ax = plt.axes(projection="3d")

    ## SIM
    x_data, y_data, z_data = _extract_axes(sim_timesteps)
    _draw_plots(ax, x_data, y_data, z_data, "Robot path in simulation")

    ## REAL
    x_data, y_data, z_data = _extract_axes(real_timesteps)
    _draw_plots(ax, x_data, y_data, z_data, "Robot path in reality")

    ax.legend()
    ax.set_zlabel("Time in [ms]")
    ax.set_xlabel("X-coordinate in pixel space")
    ax.set_ylabel("Y-coordinate in pixel space")

    plt.savefig(RESULT_DIR + "3d_path_plot.png")
    plt.show()


real_file = open(REAL_PATH + "/data.csv")
sim_file = open(SIM_PATH + "/data.csv")

real_timestep_dict = create_timestep_dict(read_csv_file(real_file))
sim_timestep_dict = create_timestep_dict(read_csv_file(sim_file))

offset, rotation, rot_origin, scale = calc_real_offset(real_timestep_dict[0], sim_timestep_dict[0])
real_timestep_dict = normalise_real_points(real_timestep_dict, offset, rot_origin, scale)

real_timestep_dict = create_real_pos_for_timesteps(sim_timestep_dict, real_timestep_dict)

print(len(real_timestep_dict.keys()), len(sim_timestep_dict.keys()))

new_img = cv2.imread("simulation_empty.png")

x_real, y_real = draw_points_real(new_img, real_timestep_dict)
x_sim, y_sim = draw_points_sim(new_img, sim_timestep_dict)

cv2.imshow("ImageWithCircle", new_img)
cv2.waitKey(0)

cv2.imwrite(RESULT_DIR + "path.png", new_img)

plt.scatter(y_sim, x_sim, label="Robot path in simulation")
plt.scatter(y_real, x_real, label="Robot path in reality")

plt.xlabel("X-Coordinate in pixel space")
plt.ylabel("Y-Coordinate in pixel space")

plt.legend()

plt.gca().set_aspect("equal", adjustable="box")

plt.savefig(RESULT_DIR + "2d_path.png")
plt.show()


# create_plot(sim_timestep_dict, real_timestep_dict, offset, rotation, rot_origin, scale)


# print(real_timestep_dict.keys(), sim_timestep_dict.keys())
# print(real_timestep_dict)

create_plots(sim_timestep_dict, real_timestep_dict)

create_3d_plot(sim_timestep_dict, real_timestep_dict)

# read_csv_file(real_file)

# print(create_timestep_dict(read_csv_file(sim_file)))

cv2.imwrite(RESULT_DIR + "path.png", new_img)