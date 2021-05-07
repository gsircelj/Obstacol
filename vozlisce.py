#!/usr/bin/env python
# coding: utf-8

# In[7]:


import time
import funkcije_avta

f = open("izpis_podatkov.txt", "w")

# shranjevanje trenutne iteracije
idx = 0

while True:
    car_speed = funkcije_avta.getSpeed()
    car_gear = funkcije_avta.getGear()
    car_position = funkcije_avta.getPosition()
    car_orientation = funkcije_avta.getOrientation()
    car_collision = funkcije_avta.getCollision()

    f.write("hitrost: %d, prestava: %d\n" % (car_speed, car_gear))
    f.write(car_position)
    f.write(car_orientation)
    f.write(car_collision)
    
   
    print("hitrost: %d, prestava: %d" % (car_speed, car_gear))
    print(car_orientation)
    print(car_position)
    
    
    print(car_collision)
     
 
    # trenutno se shranita slika scene in globinska slika
    funkcije_avta.getImages(idx)
    
    # tako dobimo manj podatkov in s tem preglednejši izgled podatkov
    # v prihajajočih nadgradnjah to izbriši
    time.sleep(1)

    idx += 1

    print("\n")
f.close()




