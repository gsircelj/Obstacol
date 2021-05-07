#!/usr/bin/env python
# coding: utf-8

# In[ ]:


import airsim
import cv2
import numpy as np
import os
import tempfile
import time

# connect to the AirSim simulator
client = airsim.CarClient()
client.confirmConnection()
car_controls = airsim.CarControls()

# naredi nov direktorij za shranjevanje slik
tmp_dir = os.path.join("", "airsim_car")
print ("Saving images to %s" % tmp_dir)
try:
    os.makedirs(tmp_dir)
except OSError:
    if not os.path.isdir(tmp_dir):
        raise

def getSpeed():
    car_state = client.getCarState()
    return car_state.speed

def getGear():
    car_state = client.getCarState()
    return car_state.gear

def getImages(idx):
    # get camera images from the car
    responses = client.simGetImages([
        airsim.ImageRequest("0", airsim.ImageType.DepthVis),  #depth visualization image
        airsim.ImageRequest("1", airsim.ImageType.Scene)]) #scene vision image in png format
    #scene vision image in uncompressed RGB array
    print('Retrieved images: %d' % len(responses))

    for response_idx, response in enumerate(responses):
        filename = os.path.join(tmp_dir, f"{idx}_{response.image_type}_{response_idx}")
	
        if response.pixels_as_float:
            print("Type %d, size %d" % (response.image_type, len(response.image_data_float)))
            airsim.write_pfm(os.path.normpath(filename + '.pfm'), airsim.get_pfm_array(response))
        elif response.compress: #png format
            print("Type %d, size %d" % (response.image_type, len(response.image_data_uint8)))
            airsim.write_file(os.path.normpath(filename + '.png'), response.image_data_uint8)
        else: #uncompressed array
            print("Type %d, size %d" % (response.image_type, len(response.image_data_uint8)))
            img1d = np.fromstring(response.image_data_uint8, dtype=np.uint8) # get numpy array
            img_rgb = img1d.reshape(response.height, response.width, 3) # reshape array to 3 channel image array H X W X 3
            cv2.imwrite(os.path.normpath(filename + '.png'), img_rgb) # write to png


def reset():
    client.reset()
    print("Reset")

def brake():
    car_controls.brake = 1
    client.setCarControls(car_controls)
    print("Apply brakes")
    time.sleep(3)   # let car drive a bit
    car_controls.brake = 0 #remove brake