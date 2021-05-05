#!/usr/bin/env python
# coding: utf-8

# In[7]:


import time
import funkcije_avta

f = open("izpis_podatkov.txt", "w")

while True:
    car_speed = funkcije_avta.getSpeed()
    car_gear = funkcije_avta.getGear()
    f.write("hitrost: %d, prestava: %d\n" % (car_speed, car_gear))
    
    # tako dobimo manj podatkov in s tem preglednejši izgled podatkov
    # v prihajajočih nadgradnjah to izbriši
    time.sleep(0.5)

f.close()


# In[ ]:




