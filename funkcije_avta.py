#!/usr/bin/env python
# coding: utf-8

# In[ ]:


import airsim
import cv2
import numpy as np
import os
import tempfile
import time
import PSpincalc as sp

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

#ustvari se mi quaterion objekt, ki pa ga nato s klicem funkcije to string spremenim v string, ter returnam ta string       
def getOrientation():
    car_state = client.getCarState()
    return toStringOrientation(car_state.kinematics_estimated.orientation)
    
#spremeni moj quaterion??? v string kjer razčlenim podatke ter jih vrnem v string
def toStringOrientation(temp):
    #moj quaterion sprenim v numpy array, ki ga nato s pomocjo funkcije pretvorim v eulerjeve enote
    quaternion = np.array([temp.w_val, temp.x_val, temp.y_val, temp.z_val])
    yo = sp.Q2EA(quaternion, EulerOrder="zyx", ignoreAllChk=True)[0]
    #spodaj si nastavim string, kjer predstavim eulerjeve enote
    return "Orientacija avta:\n" + "x: " + str(yo[0]) + "\ny: " + str(yo[1]) + "\nz: " + str(yo[2])
    
#returna polozaj v obliki stringa drugace je prej klicom funkcije spodnje to Vector3r oblika
def getPosition():
    car_state = client.getCarState()
    return toStringPosition(car_state.kinematics_estimated.position)

#Vector3r mi pretvori v string
def toStringPosition(temp):
    return "Polozaj avta:\n" "x: " + str(temp.x_val) + "\ny: " + str(temp.y_val) + "\nz: " + str(temp.z_val)

#dobimo object collision info, ki ga nato s pomocje funkcije tostring damo v string xD pa nato returnamo ta formatiran string
def getCollision():
    collision_info = client.simGetCollisionInfo()
    return toStringCollision(collision_info)
    
    
#tukaj glede na razlice parametre collision infa tvorimo razlicne string npr ce smo se ze zaleteli ali se ne
def toStringCollision(temp):
    strRezultat = "Ali smo ze imeli karambol: "
    if temp.has_collided == 0:
        strRezultat = strRezultat + "ne :)"
    elif temp.has_collided ==  1:
        strRezultat = strRezultat + "da :(" + "\nLokacija karambola:" + "\nx: " + str(temp.impact_point.x_val) + "\ny: " + str(temp.impact_point.y_val) + "\nz: " + str(temp.impact_point.z_val)
        strRezultat = strRezultat + "\nZaleteli smo se v " + str(temp.object_name) + " s id-jem '" + str(temp.object_id) + "'\nKako dalec smo prodrli v objekt: " + str(abs(temp.penetration_depth))
        strRezultat = strRezultat + "\nCas karambola: " + str(temp.time_stamp)
    return strRezultat

def reset():
    client.reset()
    print("Reset")

def brake():
    car_controls.brake = 1
    client.setCarControls(car_controls)
    print("Apply brakes")
    time.sleep(3)   # let car drive a bit
    car_controls.brake = 0 #remove brake
