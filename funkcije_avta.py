#!/usr/bin/env python
# coding: utf-8

# In[ ]:


import airsim

# connect to the AirSim simulator
client = airsim.CarClient()
client.confirmConnection()
client.enableApiControl(True)
car_controls = airsim.CarControls()

def getSpeed():
    car_state = client.getCarState()
    return car_state.speed

def getGear():
    car_state = client.getCarState()
    return car_state.gear

