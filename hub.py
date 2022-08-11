from autodriver import AutoDriver
import setup_path
import airsim
from datetime import datetime
import cv2
import time
import numpy as np
import matplotlib.pyplot as plt
import signal
import sys
import os
import os.path
from array import *
import pyaudio
import sounddevice as sd
import scipy.io.wavfile as wav
import pathlib
import math
# Line below ensures that Ctrl+C is working with import from scipy
os.environ['FOR_DISABLE_CONSOLE_CTRL_HANDLER'] = '1'
from scipy.interpolate import interp1d

# New imports for 3. year
import serial
import random
from threading import Thread
from decompress import *

np.seterr(divide='ignore', invalid='ignore')


def mainMenu():
    print("+-------------------------------+--------------------------------------------------+")
    print("| Airsim: Burnacol | Authors: Anej Krajnc, Gasper Sircelj, Ziga Pecar, Ziga Pusnik |")
    print("+-------------------------------+--------------------------------------------------+")
    print("| Functions:                    | Description:                                     |")
    print("| 1. Output                     | Output Functions                                 |")
    print("| 2. Input                      | Input Functions                                  |")
    print("| 3. Test                       | Testing Scenaries                                |")
    print("| 4. Quit                       | Leave Environment                                |")
    print("+-------------------------------+--------------------------------------------------+")


def outputMenu():
    print("+-------------------------------+--------------------------------------------------+")
    print("| Functions:                    | Description:                                     |")
    print("| 1. All                        | Output all data -> File                          |")
    print("| 2. Speed & Gear & RPM         | Output speed, gear, RPM -> File                  |")
    print("| 3. Position & Rotation        | Output XYZ + WXYZ -> File                        |")
    print("| 4. Distance Sensor            | Output distance (meters) -> File                 |")
    print("| 5. Barometer Sensor           | Output timestamp, altitude, pressure -> File     |")
    print("| 6. Capture Camera Images      | PNG,PFM -> File                                  |")
    print("| 7. Back                       | Back to Main Menu                                |")
    print("+-------------------------------+--------------------------------------------------+")


def inputMenu():
    print("+-------------------------------+--------------------------------------------------+")
    print("| Functions:                    | Description:                                     |")
    print("| 1. Microphone                 | Input from microphone                            |")
    print("| 2. Audio                      | Input from sound file                            |")
    print("| 3. USB                        | Input from USB                                   |")
    print("| 4. WiFi - Network             | Input from WiFi                                  |")
    print("| 5. Back                       | Back to Main Menu                                |")
    print("+-------------------------------+--------------------------------------------------+")


def inputMenu_Microphone():
    print("+-------------------------------+--------------------------------------------------+")
    print("| Functions:                    | Description:                                     |")
    print("| 1. Microphone - Record        | Record from microphone                           |")
    print("| 2. Microphone - PPM Decoder   | Real-Time PPM decode                             |")
    print("| 3. Back                       | Back to Input Menu                               |")
    print("+-------------------------------+--------------------------------------------------+")


def testMenu():
    print("+-------------------------------+--------------------------------------------------+")
    print("| Scenaries:                    | Description:                                     |")
    print("| 1. Driving - Lines            | Driving between road lines                       |")
    print("| 2. Driving - Target           | Driving & following Target                       |")
    print("| 3. Driving w/ Plug&Play       | Driving algorithm and controller                 |")
    print("| 4. Back                       | Back to Main Menu                                |")
    print("+-------------------------------+--------------------------------------------------+")

# To catch CTRL+C


def signal_handler(sig, frame):
    global stop
    global listen_USB
    global stop_alg_hum
    global stop_driving_target
    changed = 0
    try:
        listen_USB
        stop_alg_hum
        stop_driving_target
        if stop_driving_target is not None:
            stop_driving_target = True
            changed += 1
        if stop_alg_hum is not None:
            stop_alg_hum = True
            changed += 1
        if listen_USB is not None and changed == 0:
            listen_USB = False
            changed += 1
            global USB
            USB.join(1.0)
    except NameError:
        stop_alg_hum = True
        stop_driving_target = True
        pass
    if changed == 0:
        stop = True


# Try parse value -> int
def intTryParse(value):
    try:
        return int(value), True
    except ValueError:
        return value, False

# Connect with AirSim simulator -> Simulator MUST be running!


def connectToAirSim():
    client = airsim.CarClient()
    client.confirmConnection()
    return client

# Output all data -> Output_Data.txt


def outputAll():
    global stop
    stop = False
    signal.signal(signal.SIGINT, signal_handler)
    f = open("./Output_Data.txt", "w")
    start = time.time()
    while (cv2.waitKey(1) & 0xFF) == 0xFF:
        if stop:
            break
        data_car = client.getDistanceSensorData()
        car_state = client.getCarState()
        pos = car_state.kinematics_estimated.position
        orientation = car_state.kinematics_estimated.orientation
        barometer_data = client.getBarometerData()
        seconds = (time.time() - start)  # * 1000 -> to ms
        print("Time\tSpeed:\tGear:\tRPM:\tPosition: (X, Y, Z)\t\tOrientation: (W, X, Y, Z)\t\tDistance:\tTimestamp:\tAltitude:\tPressure:")
        print("%.2f\t%d\t%d\t%d\t(%.2f, %.2f, %.2f)\t\t(%.2f, %.2f, %.2f, %.2f)\t\t%.2f\t%d\t%d\t%d" % (seconds, car_state.speed, car_state.gear, car_state.rpm, pos.x_val, pos.y_val,
              pos.z_val, orientation.w_val, orientation.x_val, orientation.y_val, orientation.z_val, data_car.distance, barometer_data.time_stamp, barometer_data.altitude, barometer_data.pressure))
        f.write("Time:\tSpeed:\tGear:\tRPM:\tPosition: (X, Y, Z)\t\tOrientation: (W, X, Y, Z)\t\tDistance:\tTimestamp:\tAltitude:\tPressure:\n")
        f.write("%.2f\t%d\t%d\t%d\t(%.2f, %.2f, %.2f)\t\t(%.2f, %.2f, %.2f, %.2f)\t\t%.2f\t%d\t%d\t%d\n" % (seconds, car_state.speed, car_state.gear, car_state.rpm, pos.x_val, pos.y_val,
                pos.z_val, orientation.w_val, orientation.x_val, orientation.y_val, orientation.z_val, data_car.distance, barometer_data.time_stamp, barometer_data.altitude, barometer_data.pressure))
        time.sleep(0.1)
    f.close()

# Output speed, gear, RPM -> Speed_Gear_RPM.txt


def outputSpeed_Gear_RPM():
    global stop
    stop = False
    signal.signal(signal.SIGINT, signal_handler)
    f = open("./Speed_Gear_RPM.txt", "w")
    start = time.time()
    while (cv2.waitKey(1) & 0xFF) == 0xFF:
        if stop:
            break
        car_state = client.getCarState()
        seconds = (time.time() - start)  # * 1000 -> to ms
        print("Time: %.3f\tSpeed: %d\tGear: %d\tRPM: %d" %
              (seconds, car_state.speed, car_state.gear, car_state.rpm))
        f.write("Time: %.3f\tSpeed: %d\tGear: %d\tRPM: %d\n" %
                (seconds, car_state.speed, car_state.gear, car_state.rpm))
        time.sleep(0.1)
    f.close()

# Output position XYZ + orientation WXYZ -> Position_XYZ_Orientation_WXYZ.txt


def outputPosition():
    global stop
    stop = False
    signal.signal(signal.SIGINT, signal_handler)
    f = open("./Position_XYZ_Orientation_WXYZ.txt", "w")
    start = time.time()
    while (cv2.waitKey(1) & 0xFF) == 0xFF:
        if stop:
            break
        car_state = client.getCarState()
        pos = car_state.kinematics_estimated.position
        orientation = car_state.kinematics_estimated.orientation
        seconds = (time.time() - start)  # * 1000 -> to ms
        
        print("Time: %.3f\tPosition: X: %.2f Y: %.2f Z: %.2f\tOrientation: W: %.2f X: %.2f Y: %.2f Z: %.2f" % (
            seconds, pos.x_val, pos.y_val, pos.z_val, orientation.w_val, orientation.x_val, orientation.y_val, orientation.z_val))
        f.write("Time: %.3f\tPosition: X: %.2f Y: %.2f Z: %.2f\tOrientation: W: %.2f X: %.2f Y: %.2f Z: %.2f\n" % (
            seconds, pos.x_val, pos.y_val, pos.z_val, orientation.w_val, orientation.x_val, orientation.y_val, orientation.z_val))
        time.sleep(0.1)
    f.close()

# Output DistanceSensor data (distance) -> DistanceSensorData.txt


def outputDistanceSensor():
    global stop
    stop = False
    signal.signal(signal.SIGINT, signal_handler)
    f = open("./DistanceSensorData.txt", "w")
    start = time.time()
    while (cv2.waitKey(1) & 0xFF) == 0xFF:
        if stop:
            break
        data_car = client.getDistanceSensorData()
        seconds = (time.time() - start)  # * 1000 -> to ms
        print("Time: %.2f\tDistance: %.2f" % (seconds, data_car.distance))
        f.write("Time: %.2f\tDistance: %.2f\n" % (seconds, data_car.distance))
        time.sleep(0.1)
    f.close()

# Output BarometerSensor data (timestamp, altitude (above sea level), pressure) -> BarometerSensorData.txt


def outputBarometerSensor():
    global stop
    stop = False
    signal.signal(signal.SIGINT, signal_handler)
    f = open("./BarometerSensorData.txt", "w")
    while (cv2.waitKey(1) & 0xFF) == 0xFF:
        if stop:
            break
        barometer_data = client.getBarometerData()
        print("Timestamp: %d\tAltitude: %d\tPressure: %d" % (
            barometer_data.time_stamp, barometer_data.altitude, barometer_data.pressure))
        f.write("Timestamp: %d\tAltitude: %d\tPressure: %d\n" % (
            barometer_data.time_stamp, barometer_data.altitude, barometer_data.pressure))
        time.sleep(0.1)
    f.close()

# Output Camera Data (Images -> AirSimImages/*.png|*.pfm)


def CaptureImage():
    car_state = client.getCarState()
    path = pathlib.Path(__file__).parent.absolute()
    directory = os.path.join(path, "AirSimImages")
    print("Saving images to %s" % directory)
    try:
        os.makedirs(directory)
    except OSError:
        if not os.path.isdir(directory):
            raise
    responses = client.simGetImages([airsim.ImageRequest("0", airsim.ImageType.DepthVis),
                                     airsim.ImageRequest(
                                         "0", airsim.ImageType.DepthPerspective, True),
                                     airsim.ImageRequest(
                                         "0", airsim.ImageType.Scene),
                                     airsim.ImageRequest("0", airsim.ImageType.Scene, False, False)])  # uncompressed
    print('Retrieved images: %d' % len(responses))
    for response_i, response in enumerate(responses):
        dat = os.path.join(directory, f"{response.image_type}_{response_i}")
        if response.pixels_as_float:
            print("%s Type %d, size %d" %
                  (dat, response.image_type, len(response.image_data_float)))
            airsim.write_pfm(os.path.normpath(dat + '.pfm'),
                             airsim.get_pfm_array(response))
        elif response.compress:
            print("%s Type %d, size %d" %
                  (dat, response.image_type, len(response.image_data_uint8)))
            airsim.write_file(os.path.normpath(dat + '.png'),
                              response.image_data_uint8)
        else:
            print("%s Type %d, size %d" %
                  (dat, response.image_type, len(response.image_data_uint8)))
            Numpyarray = np.fromstring(
                response.image_data_uint8, dtype=np.uint8)
            Numpyarray = Numpyarray.reshape(response.height, response.width, 3)
            cv2.imwrite(os.path.normpath(dat + '.png'), Numpyarray)

# Callback function for recording and showing microphone


def callback_Record(indata, frames, time, status):
    if status:
        print(status)
    normalise = np.linalg.norm(indata) * 10
    print("|" * int(normalise))
    global output
    output = np.append(output, indata.copy())

# Input of microphone and writing to rec_mic.wav/.txt -> sounddevice needed


def recordAudio(num):
    global output
    output = np.array([], ndmin=2)
    Fvz = 44100
    sd.default.samplerate = Fvz
    with sd.InputStream(Fvz, channels=1, callback=callback_Record):
        print("Recording started...")
        sd.sleep(num * 1000)
    print("Recording stopped...")
    print("Writing to Audio file... -> rec_mic.wav")
    wav.write('./rec_mic.wav', Fvz, output)
    print("Writing completed!")
    print("Writing to text file... -> rec_mic.txt")
    with open('./rec_mic.txt', 'w') as f:
        f.write("Fvz: %s Shape: %s\n[" % (Fvz, output.shape))
        counter = 0
        for item in output:
            if counter == 7:
                f.write("\n")
                counter == 0
            f.write("%s   " % item)
            counter += 1
        f.write("]\n")
    print("Writing completed!")

# Function for decoding PPM signal


def callback_DecodePPM():
    threshhold = 0.2
    D = 44100 * 1.5 // 1000
    L = 44100 * 1 // 1000
    R = 44100 * 2 // 1000
    DLR = np.array([L, D, R])
    pulse_len = 20 * L
    pulse_max = 8 * R
    sync = pulse_len - pulse_max
    #print("D - ", D, "\nL - ", L, "\nR - ", R, "\nsync - ", sync)
    global query
    data = query.copy() * 40
    # data = data / np.max(np.abs(data))
    idx_under_thresh = data < threshhold
    data[idx_under_thresh] = -0.5
    # np.interp(data, (np.min(data), np.max(data)), (-1, +1))
    query = np.array([], ndmin=2)
    #print(np.min(data), np.max(data))
    #print(np.min(data), np.max(data))
    idx_zero_crossings = np.where(np.diff(np.signbit(data)))[0]
    # print(idx_zero_crossings)
    if idx_zero_crossings.shape[0] != 0:
        idx_sync = np.where(np.diff(idx_zero_crossings) > sync)[0]
        # print(idx_sync)
        if idx_sync.shape[0] == 0:
            return
        idx_start = idx_zero_crossings[idx_sync[0] + 1] + 1
        if idx_sync.shape[0] > 1:
            idx_end = idx_zero_crossings[idx_sync[1] + 1]
            idx_zero_crossings = idx_zero_crossings[idx_sync[0] +
                                                    1: idx_sync[1] + 1]
        else:
            idx_end = data.shape[0]
            idx_zero_crossings = np.append(
                idx_zero_crossings[idx_sync[0] + 1:], data.shape[0] - 1)
        # print(idx_zero_crossings)
        #data = data[idx_start: idx_end]
        global PPM_canals
        canals_start = np.delete(idx_zero_crossings, np.arange(
            1, idx_zero_crossings.shape[0], 2))
        canal_values = np.diff(canals_start)
        if canal_values.shape[0] < 8:
            return
        for i in range(canal_values.shape[0]):
            diff = np.abs(DLR - canal_values[i])
            canal_values[i] = DLR[np.where(diff == diff.min())][0]
        car_controls = airsim.CarControls()
        for i in range(len(PPM_canals)):
            if PPM_canals[i] == "throttle":
                if canal_values[i] == 44:
                    car_controls.throttle = 0
                elif canal_values[i] == 66:
                    car_controls.throttle = 0.5
                elif canal_values[i] == 88:
                    car_controls.throttle = 1
            elif PPM_canals[i] == "steering":
                if canal_values[i] == 44:
                    car_controls.steering = -1
                elif canal_values[i] == 66:
                    car_controls.steering = 0
                elif canal_values[i] == 88:
                    car_controls.steering = 1
            elif PPM_canals[i] == "control":
                if canal_values[i] == 44:
                    client.enableApiControl(False)
                elif canal_values[i] == 66:
                    client.enableApiControl(True)
                elif canal_values[i] == 88:
                    client.enableApiControl(True)
        print("Throttle: %d\tSteering: %d\tControl: %s" % (
            car_controls.throttle, car_controls.steering, client.isApiControlEnabled()))
        if client.isApiControlEnabled():
            client.setCarControls(car_controls)

# Callback function for capturing PPM signal to array


def callback_PPM(indata, frames, time, status):
    if status:
        print(status)
    len = indata.shape[0]
    global query
    query = np.append(query, indata.copy())
    if query.shape[0] >= len * 2:
        callback_DecodePPM()

# Input of microphone and decoding of PPM signal -> sounddevice needed


def decodePPM():
    global query
    query = np.array([], ndmin=2)
    global stop
    stop = False
    Fvz = 44100
    sd.default.samplerate = Fvz
    with sd.InputStream(Fvz, channels=1, callback=callback_PPM):
        print("Decoding started...")
        signal.signal(signal.SIGINT, signal_handler)
        while (cv2.waitKey(1) & 0xFF) == 0xFF:
            if stop:
                break
    client.enableApiControl(False)
    print("Decoding stopped...")

# Input of Audio file and playing it -> sounddevice needed


def inputAudio(filename):
    print("Reading: ", filename)
    Fvz, y = wav.read(filename)
    print("Fvz: %d Shape: %s" % (Fvz, y.shape))
    print("Playing...")
    sd.play(y, Fvz)
    sd.wait()
    print("Playing finished...")

# Input of USB -> pyserial needed


def inputUSB():
    signal.signal(signal.SIGINT, signal_handler)
    global listen_USB
    listen_USB = True
    global USB
    USB = Thread(target=Plug_N_Play, args=(), daemon=True)
    USB.start()

# Thread listening and reading from USB (Plug & Play)


def Plug_N_Play():
    SERIAL_PORT = 'COM5'
    SERIAL_RATE = 115200
    ser = []
    print("Start listening port:", SERIAL_PORT)
    global listen_USB
    global human_control
    global roll
    global pitch
    while listen_USB:
        try:
            # connect to USB
            ser = serial.Serial(SERIAL_PORT, SERIAL_RATE)
            print("Connected USB!")
            print()
            print("Start reading port...")
            stop_USB = False
            while (cv2.waitKey(1) & 0xFF) == 0xFF:
                if stop_USB:
                    break
                try:
                    # flush old data
                    ser.flushInput()
                    # read 25 bytes in which our data is compressed
                    numpy_data = ser.read(25)
                    # send data to decompress and get ASCII array
                    (data, length) = decompress(numpy_data, len(numpy_data))
                    data = data.astype(int)
                    # convert data to string and split by separator ';'
                    line = ''.join(chr(ch) for ch in data)
                    line = line.split(";")
                    # use data for the program
                    human_control = bool(int(line[1]) == 1)
                    new_roll = float(line[2])
                    new_pitch = float(line[3])
                    if abs(roll - new_roll) < 50:
                        roll = new_roll
                    if abs(pitch - new_pitch) < 10:
                        pitch = new_pitch
                except (serial.SerialException, IndexError) as e:
                    stop_USB = True
                    human_control = False
                # finally:
                #     print(human_control, roll, pitch)
            print()
            print("Disconnected USB!")
        except serial.SerialException:
            pass
    print()
    print("Closing port:", SERIAL_PORT)
    human_control = False
    roll = 0.0
    pitch = 0.0
    try:
        # clean up serial port
        ser.close()
    except Exception:
        pass

# Driving with algorithm and option for Plug&Play
# Testing for API control - driving straight, reverse and braking


def driving_Alg_Hum():
    print("Starting driving w/ Plug&Play...")
    global human_control
    global roll
    global pitch
    global stop_alg_hum
    stop_alg_hum = False
    car_controls = airsim.CarControls()
    signal.signal(signal.SIGINT, signal_handler)
    client.enableApiControl(True)
    delta_time = 2
    while (cv2.waitKey(1) & 0xFF) == 0xFF:
        if stop_alg_hum:
            break
        car_state = client.getCarState()
        # use handbrake as human_control
        car_controls.handbrake = human_control;
        print("Human: %s\tSteering... %.2f\tThrottle... %.2f Brake... %.2f Speed... %d Gear... %d" % (
            car_controls.handbrake, car_controls.steering, car_controls.throttle, car_controls.brake, car_state.speed, car_state.gear))
        if human_control:
            steering = change_Interval(roll, -180.0, 180.0, -1.0, 1.0, 10.0)
            throttle = change_Interval(-pitch, -90.0, 90.0, -1.0, 1.0, 5.0)
        else:
            if delta_time > 1:
                steering = random.uniform(-1.0, 1.0)
                throttle = random.uniform(-1.0, 1.0)
                delta_time = 0
            else:
                delta_time += 0.1
        if throttle < 0:
            car_controls.is_manual_gear = True
            car_controls.manual_gear = -1
            car_controls.brake = -1
        else:
            car_controls.manual_gear = 0
            car_controls.is_manual_gear = False
            car_controls.brake = 0
        car_controls.steering = steering
        car_controls.throttle = throttle
        client.setCarControls(car_controls)
        time.sleep(0.1)
    car_state = client.getCarState()
    speed = car_state.speed
    car_controls.steering = 0
    if car_controls.throttle < 0:
        car_controls.throttle = 1
        car_controls.brake = 0
    else:
        car_controls.throttle = 0
        car_controls.brake = -1
    client.setCarControls(car_controls)
    while car_state.speed > 0:
        if speed < car_state.speed:
            car_controls.throttle = 1 if car_controls.throttle == 0 else car_controls.throttle == 0
            car_controls.brake = 0 if car_controls.brake == -1 else car_controls.brake == -1
            client.setCarControls(car_controls)
            speed = car_state.speed
        car_state = client.getCarState()
        time.sleep(0.1)
    car_controls.brake = 0
    print("Stopped driving w/ Plug&Play...")
    client.enableApiControl(False)

    # client.enableApiControl(True)
    # print("API Control enabled: %s" % client.isApiControlEnabled())
    # car_controls = airsim.CarControls()
    # client.reset()
    # # get state of the car
    # car_state = client.getCarState()
    # print("Speed %d, Gear %d" % (car_state.speed, car_state.gear))
    # #Throt_Steer = [[1, -1], [0, -1], [-1, -1]]
    # S = [50, 75, 100]
    # T = []
    # for speed in S:
    #     # go forward
    #     car_controls.throttle = 1
    #     car_controls.steering = 0
    #     client.setCarControls(car_controls)
    #     print("Go Forward throttle = 0.5 steering = 0")
    #     while car_state.speed < speed:
    #         car_state = client.getCarState()
    #         print("Speed %d, Gear %d" % (car_state.speed, car_state.gear))
    #         time.sleep(0.1)
    #     # apply brakes
    #     car_controls.throttle = 0
    #     car_controls.brake = -1
    #     client.setCarControls(car_controls)
    #     print("Apply brakes with throttle = %d breaking = %d" % (car_controls.throttle,  car_controls.brake))
    #     elapsed_time = 0
    #     while car_state.speed > 0:
    #         car_state = client.getCarState()
    #         print("Speed %d, Gear %d, Time Elapsed  %.2f" % (car_state.speed, car_state.gear, elapsed_time))
    #         elapsed_time += 0.1
    #         time.sleep(0.1)
    #     car_controls.brake = 0
    #     client.reset()
    #     T.append(elapsed_time)
    # print("\n")
    # print(T)
    # Go forward + steer right
    #car_controls.throttle = 0.5
    #car_controls.steering = 1
    # client.setCarControls(car_controls)
    #print("Go Forward, steer right throttle = 0.5 steering = 1")
    # time.sleep(3) # let car drive a bit

    # go reverse
    #car_controls.throttle = -0.5
    #car_controls.is_manual_gear = True
    #car_controls.manual_gear = -1
    #car_controls.steering = 0
    #car_controls.brake = car_controls.throttle
    # client.setCarControls(car_controls)
    # print("Go reverse, steer right throttle = -0.5 manual_gear = -1 steering =
    # 0")
    # time.sleep(10) # let car drive a bit
    # car_controls.is_manual_gear = False # change back gear to auto
    #car_controls.manual_gear = 0
    # client.enableApiControl(False)


def change_Interval(value, in_min, in_max, out_min, out_max, ignore):
    if value > -ignore and value < ignore:
        value = 0
    return (out_max - out_min) * ((value - in_min) / (in_max - in_min)) + out_min


def capture_Image():
    # Request image from center camera
    responses = client.simGetImages(
        [airsim.ImageRequest(0, airsim.ImageType.Scene, False, False)])
    response = responses[0]
    # Get numpy array - 1D
    img_1D = np.frombuffer(response.image_data_uint8, dtype=np.uint8)
    # Get numpy array - 3D - RGB
    img_RGB = img_1D.reshape(response.height, response.width, 3)
    # Flip image vertically to correct it
    img_RGB = np.flipud(img_RGB)
    return img_RGB


def show_Image(title, img):
    cv2.imshow(title, img)
    cv2.waitKey()
    cv2.destroyAllWindows()


def detect_skin(slika):
    slika_cpy = slika.copy()
    R = slika_cpy[..., 0]
    G = slika_cpy[..., 1]
    B = slika_cpy[..., 2]
    r = R / (R + G + B)
    g = G / (R + G + B)
    b = B / (R + G + B)
    skin_detect_1 = (r / g) > 1.185
    skin_detect_2 = (r * b) / (r + b + g)**2 > 0.107
    skin_detect_3 = (r * g) / (r + b + g)**2 > 0.112
    return skin_detect_1 & skin_detect_2 & skin_detect_3


def apply_mask(img, mask):
    filter = np.zeros(img.shape[:2], dtype=np.uint8)
    filter[mask] = 255
    return cv2.bitwise_and(img, img, mask=filter)

# get average line


def get_length_threshold(lines):
    lengths = []
    for line in lines:
        for x1, y1, x2, y2 in line:
            lengths.append(((x2-x1) ** 2 + (y2-y1) ** 2) ** 0.5)

    # set threshold to top 80% longest lines
    return np.quantile(lengths, 0)


def average_line(x_list, y_list, counter):
    if counter > 0:
        polyfit = np.polyfit(y_list, x_list, deg=1)
        poly = np.poly1d(polyfit)
        return poly
    return None


def draw_line(image, list):
    BOTTOM_Y = image.shape[0]
    TOP_Y = image.shape[0] * 3 // 5
    LANE_COLOR = (0, 255, 0)
    x_start = int(list(BOTTOM_Y))
    x_end = int(list(TOP_Y))
    # cv2.line(image, (x_start, BOTTOM_Y), (x_end, TOP_Y), LANE_COLOR, 5)
    return x_start, x_end


def pipeline(image_original):
    image = np.copy(image_original)
    iH = image.shape[0]
    iW = image.shape[1]
    region = [
        (0, iH),
        (0, iH/1.5),
        (iW, iH/1.5),
        (iW, iH)
    ]

    gray = cv2.cvtColor(image, cv2.COLOR_RGB2GRAY)
    edges = cv2.Canny(gray, 150, 255)
    mask = np.zeros(image.shape[:2], dtype=np.uint8)
    pts = np.array([region], np.int32)
    cv2.fillPoly(mask, [pts], 255)
    edges = cv2.bitwise_and(edges, mask)

    lines = cv2.HoughLinesP(
        edges,
        rho=1.0,
        theta=np.pi / 180,
        threshold=20,
        minLineLength=30,
        maxLineGap=10
    )
    if lines is None:
        return image, None, None
    left_counter = right_counter = 0
    left_x, left_y, right_x, right_y = [], [], [], []
    length_threshold = get_length_threshold(lines)
    for line in lines:
        for x1, y1, x2, y2 in line:
            # for every line

            if x1 == x2:
                continue  # to avoid division by 0

            # in code, y is positive down
            slope = (y1 - y2) / (x2 - x1)
            length = ((x2-x1) ** 2 + (y2-y1) ** 2) ** 0.5

            # ensure only long lines are considered
            if length < length_threshold:
                continue

            # these coords belong to right line
            if slope < 0:
                right_counter += 1
                right_x += [x1, x2]
                right_y += [y1, y2]
            # left line
            else:
                left_counter += 1
                left_x += [x1, x2]
                left_y += [y1, y2]

    # calculate linear fit
    line_left = average_line(left_x, left_y, left_counter)
    line_right = average_line(right_x, right_y, right_counter)
    return image, line_left, line_right


def driving_Lines():
    global stop
    stop = False
    signal.signal(signal.SIGINT, signal_handler)
    client.enableApiControl(True)
    car_controls = airsim.CarControls()
    print("Driving between lines... STARTING")
    while (cv2.waitKey(1) & 0xFF) == 0xFF:
        if stop:
            break
        img = capture_Image()
        iH = img.shape[0]
        iW = img.shape[1]
        img, line_left, line_right = pipeline(img)
        car_state = client.getCarState()

        if line_left is None and line_right is None:
            print("No lanes detected! Exiting...")
            break

        elif line_left is None and line_right is not None:  # turn strong left
            print("Turning left... ", car_state.speed)
            # draw_line(img, line_right)
            car_controls.brake = 0
            if car_state.speed <= 0:
                car_controls.throttle = 0.2
            elif car_state.speed > 20:
                print("breaking...")
                car_controls.brake = -0.15

            car_controls.steering = -0.3
        elif line_left is not None and line_right is None:  # turn strong right
            print("Turning right...", car_state.speed)
            # draw_line(img, line_left)
            car_controls.brake = 0
            if car_state.speed <= 0:
                car_controls.throttle = 0.2
            elif car_state.speed > 20:
                print("Breaking...")
                car_controls.brake = -0.15

            car_controls.steering = 0.3
        else:  # keep driving in the middle
            print("Keeping straight...", car_state.speed)
            # draw_line(img, line_left)
            # draw_line(img, line_right)
            car_controls.brake = 0
            MIN_Y = iH * 2 // 3
            MAX_Y = iH
            left_start, left_end = draw_line(img, line_left)
            right_start, right_end = draw_line(img, line_right)
            center_lane_x_start = (right_start + left_start) // 2
            center_lane_x_end = (right_end + left_end) // 2
            x_car = iW // 2
            car_vector = np.array([x_car, MAX_Y]) - np.array([x_car, MIN_Y])
            road_vector = np.array(
                [center_lane_x_end, MAX_Y]) - np.array([center_lane_x_start, MIN_Y])
            car_vector_norm = car_vector / np.max(np.abs(car_vector))
            road_vector_norm = road_vector / np.max(np.abs(road_vector))
            # print(car_vector_norm, road_vector_norm)
            cross_product = np.cross(road_vector_norm, car_vector_norm)
            angle = np.arcsin(cross_product)
            angle2norm = interp1d([-np.pi/2, np.pi/2], [-0.3, 0.3])
            steering = -float(angle2norm(angle))

            # print((angle, steering))
            if car_controls.steering > 0.3 or car_controls.steering < -0.3:
                car_controls.throttle = 0.1
            else:
                car_controls.throttle = 0.3

            if car_state.speed > 20:
                car_controls.steering = steering * 0.8
            else:
                car_controls.steering = steering
            # LANE_COLOR = (0, 255, 0)
            # cv2.line(img, (center_lane_x_start, MAX_Y), (center_lane_x_end, MIN_Y), LANE_COLOR, 3)
            # CAR_CENTER_COLOR = (255, 0, 0)
            # cv2.line(img, (x_car, MAX_Y), (x_car, MIN_Y), CAR_CENTER_COLOR, 3)
            # show_Image("Line", img)
        client.setCarControls(car_controls)
        # show_Image("current state", img)
        time.sleep(0.1)
    car_state = client.getCarState()
    car_controls.throttle = 0
    car_controls.steering = 0
    car_controls.brake = -1
    client.setCarControls(car_controls)
    while car_state.speed > 0:
        car_state = client.getCarState()
        time.sleep(0.1)
    car_controls.brake = 0
    client.enableApiControl(False)
    print("Driving between lines... STOPPED")


def px_to_m(P):
    return (P * 0.026458333) / 100


def optical_Flow_Update(prev, prev_gray, frame, mask, color, lk_params):
    # Converts each frame to grayscale - we previously only converted the first frame to grayscale
    global init
    iH = frame.shape[0]
    iW = frame.shape[1]
    mask_frame = detect_skin(frame)
    frame_skin = apply_mask(frame, mask_frame)
    frame = cv2.cvtColor(frame, cv2.COLOR_RGB2BGR)
    frame_skin = cv2.cvtColor(frame_skin, cv2.COLOR_RGB2BGR)
    gray = cv2.cvtColor(frame_skin, cv2.COLOR_BGR2GRAY)
    # Calculates sparse optical flow by Lucas-Kanade method
    # https://docs.opencv2.org/3.0-beta/modules/video/doc/motion_analysis_and_object_tracking.html#calcopticalflowpyrlk
    next, status, error = cv2.calcOpticalFlowPyrLK(
        prev_gray, gray, prev, None, **lk_params)
    if next is None:
        return prev, prev_gray, mask, False
    # Selects good feature points for previous position
    good_old = prev[status == 1]
    # Selects good feature points for next position
    good_new = next[status == 1]
    # Draws the optical flow tracks
    old_sum = np.array([0, 0], dtype=np.float16)
    new_sum = np.array([0, 0], dtype=np.float16)
    orig_min_x = min_x = frame.shape[1]
    orig_max_x = max_x = 0
    counter = 0
    for i, (new, old) in enumerate(zip(good_new, good_old)):
        # Returns a contiguous flattened array as (x, y) coordinates for new point
        a, b = new.ravel()
        if a < min_x:
            min_x = a
        elif a > max_x:
            max_x = a
        # Returns a contiguous flattened array as (x, y) coordinates for old point
        c, d = old.ravel()
        if init:
            if c < orig_min_x:
                orig_min_x = c
            elif c > orig_max_x:
                orig_max_x = c
        old_sum += np.array([c, d])
        new_sum += np.array([a, b])
        counter += 1

    old_sum /= counter
    new_sum /= counter
    if math.isnan(new_sum[0]) or math.isnan(new_sum[1]):
        return prev, prev_gray, mask, False

    global F, W, ori_D
    # F = (P x D) / W
    # F - Focal length
    # P - target width in image (px)
    # D - distance camera -> target (m)
    # W - target width in real life (m)
    if init:
        P = np.abs(orig_max_x - orig_min_x)
        D = ori_D
        W = 0.5
        F = (P * D) / W
        init = False

    # DISTANCE CAR -> TARGET
    # Dˇ = (W x F) / Pˇ
    # D - new distance camera -> target (m)
    # W - target width in real life (m)
    # F - Focal length
    # P - new target width in image (px)
    new_P = np.abs(max_x - min_x)
    global new_D, target_dir
    new_D = (W * F) / new_P

    # DIRECTION TARGET
    target_vector = new_sum - old_sum
    target_vector_norm = target_vector / np.max(np.abs(target_vector))
    angel_dir = np.arctan(target_vector[1] / target_vector[0])
    if math.isnan(angel_dir) or np.abs(np.pi/2 - angel_dir) < np.pi/16:
        target_dir = 0
    elif angel_dir < 0:
        target_dir = -1
    else:
        target_dir = 1
    x_car = iW // 2
    car_vector = np.array([x_car, iH]) - np.array([x_car, 0])
    car_vector_norm = car_vector / np.max(np.abs(car_vector))
    # print(car_vector_norm, road_vector_norm)
    cross_product = np.cross(car_vector_norm, target_vector)
    angle = np.arcsin(cross_product)
    angle2norm = interp1d([-np.pi/2, np.pi/2], [-1, 1])
    global position
    if np.abs(new_sum[0] - iW/2) < 10:
        position = 0
    elif new_sum[0] < iW / 2:
        position = -1
    else:
        position = 1
    global steering
    steering = -float(angle2norm(angle))

    # Draws line between new and old position with green color and 2 thickness
    mask = cv2.line(mask, (int(new_sum[0]), int(
        new_sum[1])), (int(old_sum[0]), int(old_sum[1])), color, 2)
    # Draws filled circle (thickness of -1) at new position with green color and radius of 3
    frame = cv2.circle(frame, (int(new_sum[0]), int(new_sum[1])), 3, color, -1)
    # Overlays the optical flow tracks on the original frame
    output = cv2.add(frame, mask)
    # Updates previous frame
    prev_gray = gray.copy()
    # Updates previous good feature points
    prev = good_new.reshape(-1, 1, 2)
    # Opens a new window and displays the output frame
    cv2.imshow("sparse optical flow", output)
    return prev, prev_gray, mask, True


def optical_Flow_Init(first_frame):
    # Parameters for Shi-Tomasi corner detection
    global init
    init = True
    color = (0, 255, 0)
    feature_params = dict(maxCorners=300, qualityLevel=0.2,
                          minDistance=2, blockSize=7)
    # Parameters for Lucas-Kanade optical flow
    lk_params = dict(winSize=(15, 15), maxLevel=2, criteria=(
        cv2.TERM_CRITERIA_EPS | cv2.TERM_CRITERIA_COUNT, 10, 0.03))
    # The video feed is read in as a VideoCapture object
    # cap = cv2.VideoCapture("optical_flow_target.mp4")
    # ret = a boolean return value from getting the frame, first_frame = the first frame in the entire video sequence
    # ret, first_frame = cap.read()
    # Converts frame to grayscale because we only need the luminance channel for detecting edges - less computationally expensive
    mask_frame = detect_skin(first_frame)
    first_frame = apply_mask(first_frame, mask_frame)
    first_frame = cv2.cvtColor(first_frame, cv2.COLOR_RGB2BGR)
    prev_gray = cv2.cvtColor(first_frame, cv2.COLOR_BGR2GRAY)
    # Finds the strongest corners in the first frame by Shi-Tomasi method - we will track the optical flow for these corners
    # https://docs.opencv2.org/3.0-beta/modules/imgproc/doc/feature_detection.html#goodfeaturestotrack
    prev = cv2.goodFeaturesToTrack(prev_gray, mask=None, **feature_params)
    # Creates an image filled with zero intensities with the same dimensions as the frame - for later drawing purposes
    mask = np.zeros_like(first_frame)
    return lk_params, prev, prev_gray, mask, color


def car_controls_update():
    global new_D, steering, position, maintain_D, target_dir, autoDriverObj
    car_controls = airsim.CarControls()
    car_state = client.getCarState()
    car_controls.gear_immediate = True
    car_controls.brake = 0

    new_time = time.time()
    autoDriverObj.updateMeasurement(new_time, new_D)
    currentObjectThrust = autoDriverObj.thrust
    # comply with controll restraints
    if currentObjectThrust < 0:
        currentObjectThrust = 0
    if currentObjectThrust > 1:
        currentObjectThrust = 1

    car_controls.throttle = currentObjectThrust
    # if new_D > maintain_D:
    #     car_controls.throttle = 0.4
    # elif new_D < maintain_D:
    #     if car_state.speed <= 0:
    #         car_controls.brake = 0
    #     else:
    #         car_controls.brake = -0.6
    #     car_controls.throttle = 0
    # else:
    #     car_controls.throttle = 0.2
    if new_D < maintain_D - 1:
        if car_state.speed <= 0:
            car_controls.brake = 0
        else:
            car_controls.brake = -0.5
        car_controls.throttle = 0
    elif new_D < maintain_D:
        car_controls.throttle = 0.0
        car_controls.brake = -0.2

    # print(new_D, currentObjectThrust, car_controls.brake)
    # car_controls.steering = position;

    if math.isnan(steering):
        if position == -1:
            steering = -0.5
        elif position == 1:
            steering = 0.5
        else:
            steering = 0
    else:
        if position == -1 and steering > 0:
            steering = -steering
        elif position == 1 and steering < 0:
            steering = -steering
        else:
            steering = steering
    car_controls.steering = steering

    if target_dir == 0:
        dir = "Straight"
    elif target_dir == -1:
        dir = "Left"
    else:
        dir = "Right"

    print("Distance... %.2f  Target direction... %s\tSteering... %.2f  Throttle... %.2f" % (
        new_D, dir, car_controls.steering, car_controls.throttle))
    client.setCarControls(car_controls)


def driving_Target():
    start_frame = capture_Image()
    lk_params, prev, prev_gray, mask, color = optical_Flow_Init(start_frame)
    print("Following Target... STARTING")
    global ori_D, maintain_D
    ori_D, isInt = intTryParse(input("Enter a starting distance: "))
    maintain_D, isInt = intTryParse(input("Enter a maintaining distance: "))
    g = 9.81
    objectMass = 0.56
    objectMaxThrust = 71.94
    objectNeutralBreakForce = (objectMass * g) * 0.15
    desiredDistance = maintain_D
    global autoDriverObj
    autoDriverObj = AutoDriver(objectMass,
                               objectMaxThrust,
                               objectNeutralBreakForce,
                               desiredDistance)
    client.enableApiControl(True)
    start = time.time()
    global stop_driving_target
    stop_driving_target = False
    signal.signal(signal.SIGINT, signal_handler)
    while (True):
        next_frame = capture_Image()
        prev, prev_gray, mask, isDetected = optical_Flow_Update(
            prev, prev_gray, next_frame, mask, color, lk_params)
        car_controls_update()
        if stop_driving_target:
            break
        # isDetected = False
        if not isDetected:
            print("Target is not detected... Stopping")
            break
    cv2.destroyAllWindows()
    client.enableApiControl(False)
    print("Following Target... STOPPED")


if __name__ == '__main__':
    # clear = lambda: os.system('clear') -> Linux
    def clear(): return os.system('cls')  # -> Windows
    client = connectToAirSim()
    run_main = True
    global PPM_canals
    global human_control
    global roll
    global pitch
    human_control = False
    roll = pitch = 0.0
    while run_main:
        clear()
        mainMenu()
        try:
            num = input("Enter a number: ")
        except (RuntimeError, TypeError, NameError, ValueError, KeyboardInterrupt):
            pass
        if num == '1':
            run_output = True
            clear()
            while run_output:
                outputMenu()
                num = input("Enter a number: ")
                if num == '1':
                    outputAll()
                elif num == '2':
                    outputSpeed_Gear_RPM()
                elif num == '3':
                    outputPosition()
                elif num == '4':
                    outputDistanceSensor()
                elif num == '5':
                    outputBarometerSensor()
                elif num == '6':
                    CaptureImage()
                elif num == '7':
                    run_output = False
                    clear()
                else:
                    print("Invalid input! Enter number from 1 to 5!")
        elif num == '2':
            run_input = True
            clear()
            while run_input:
                inputMenu()
                try:
                    num = input("Enter a number: ")
                except (RuntimeError, TypeError, NameError, ValueError, KeyboardInterrupt):
                    pass
                if num == '1':
                    run_input_mic = True
                    clear()
                    while run_input_mic:
                        inputMenu_Microphone()
                        try:
                            num = input("Enter a number: ")
                        except (RuntimeError, TypeError, NameError, ValueError, KeyboardInterrupt):
                            pass
                        if num == '1':
                            try:
                                num, isInt = intTryParse(
                                    input("Recording duration (seconds): "))
                                if isInt:
                                    recordAudio(num)
                                else:
                                    print("Input must be a number!")
                            except (RuntimeError, TypeError, NameError, ValueError, KeyboardInterrupt):
                                pass
                        elif num == '2':
                            PPM_canals = ["throttle", "",
                                          "steering", "", "", "control"]
                            decodePPM()
                        elif num == '3':
                            run_input_mic = False
                            clear()
                        else:
                            print("Invalid input! Enter number from 1 to 3!")
                elif num == '2':
                    try:
                        filename = (input("Enter path to Audio file: "))
                        if os.path.exists(filename):
                            inputAudio(filename)
                        else:
                            print("Invalid input! Filename doesn't exists!")
                    except (RuntimeError, TypeError, NameError, ValueError, KeyboardInterrupt):
                        pass
                        clear()
                elif num == '3':
                    inputUSB()
                elif num == '4':
                    print("Wifi - Network needs to be implemented!")
                elif num == '5':
                    run_input = False
                    clear()
                else:
                    print("Invalid input! Enter number from 1 to 5!")
        elif num == '3':
            run_test = True
            clear()
            while run_test:
                testMenu()
                try:
                    num = input("Enter a number: ")
                except (RuntimeError, TypeError, NameError, ValueError, KeyboardInterrupt):
                    pass
                if num == '1':
                    driving_Lines()
                elif num == '2':
                    driving_Target()
                elif num == '3':
                    driving_Alg_Hum()
                elif num == '4':
                    run_test = False
                    clear()
                else:
                    print("Invalid input! Enter number from 1 to 4!")
        elif num == '4':
            run_main = False
        else:
            print("Invalid input! Enter number from 1 to 4!")
