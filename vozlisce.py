#!/usr/bin/env python
# coding: utf-8

# In[7]:


import time
import funkcije_avta
import keyboard

f = open("izpis_podatkov.txt", "w")

# shranjevanje trenutne iteracije
idx = 0
pogoj = True
while pogoj:
    car_speed = funkcije_avta.getSpeed()
    car_gear = funkcije_avta.getGear()

    f.write("hitrost: %d, prestava: %d\n" % (car_speed, car_gear))
    print("hitrost: %d, prestava: %d" % (car_speed, car_gear))

    # trenutno se shranita slika scene in globinska slika
    funkcije_avta.getImages(idx)
    
    # tako dobimo manj podatkov in s tem preglednejši izgled podatkov
    # v prihajajočih nadgradnjah to izbriši
    time.sleep(1)

    idx += 1

    print("\n")


    if keyboard.is_pressed("f"):
        funkcije_avta.reset()

    if keyboard.is_pressed("space"):
        funkcije_avta.brake()

    if keyboard.is_pressed("p"):
        pogoj = False

f.close()




