#!/usr/bin/env python3

# README:
# All the stuff commented out around the while loop is used to determine the burntime for bestFlight
# If you need to get this value then uncomment and fix all the spacing (bcs python)
# This simulation does not account for the parachute because who cares, however it might make your graphs look wierd
# if ay > 200 at any point then you probably fucked up the simulation

import math
import csv
import requests
import matplotlib.pyplot as plt

xgraph = []
ygraph = []

probing = False     # change to true if you want to probe for the right burn time, remember to comment out the file writing

ymid = 0.0          # alt at middle of interval
vymid = 0.0         # velocity middle of interval
aymid = 0.0         # acceleration at middle of flight
ay = 0.0            # ft/s^2, y acceleration #change this # why change this?
y = 0.0             # ft, altitude
vy = 0.0            # ft/s, velocity
t = 0.0             # s, time
dt = 0.00001        # s, make this value a really low number, the lower the more accurate
g = 32.17405        # ft/s^2, make more accurate
apogee = 4700.0     # ft

#tabSA = 0.00694444  # surface area ft^2 (equal to 1 in^2)
#tabSA = 0
#tabSA = 0.0138889
tabSA = 0
tabdragC = 1.2      # UPDATE if you get a better const
p = 0.0765          # Air density lbm/ft3, check if units are right, UPDATE if you have any idea what value to use for this

# I fucked up the math here, used diameter where radius should be, not sure if that diameter value was correct anyway so will leave for now
rockSA = math.pi * ( 0.31291667 ** 2 ) # ft^2, from pi*r^2 IDK if I fucked up the math I'm CS not a calculator
rockdragC = 0.295   # UPDATE if you get a better const

avgThrust = 328.8955   # lbf, T H R U S T (not time) 328.8955
avgThrust *= g      # CHECK MATH freedom units are hard
#burnT = 3.34       # s, CDR burn time was listed at 4.47 but we need a perfect flight
#burnT = 3.625681538852102 # s, burn time for best flight as determined by CODING and ALGORITHIMS
burnT = 3.4512
mass = 55.18        # lbs, of rocket

burnInc = 0.5       # when probing for burn time
prevHigher = False

avgThrust /= mass   # we want acceleration by thrust

T0 = 4              # ground temp, degrees Celcius TODO: CHANGE LAUNCH DAY
Md = 0.0289         # const, kg/mol
Mv = 0.0180         # const, kg/mol
percHumidity = 0.5  # decimal percentage of humidity TODO: CHANGE LAUNCH DAY



max = 0.0           # highest the simulated rocket went
maxv = 0.0

count = 0

extension = 0   # when

#file = open("bestFlight.txt","w+")     # comment out when probing for burntime
#file.write( "altitude,velocity,time\n" )

velocities = []
rockC = []
dragIndex = 0

def ReadDrags():        # make sure pulling in force of drag
    with open('finalDrag.csv') as csvfile:
        data = csv.reader(csvfile, delimiter=',')
        for row in data:
            velocities.append(float(row[0]))
            rockC.append(float(row[1]))
    pass

def calcRockDrag(vy):
    #checkval = min(enumerate(velocities), key=lambda x: abs(x[1]-3.35))
    #print(rockC[checkval[0]])
    #return rockC[checkval[0]]
    global dragIndex
    while abs(velocities[dragIndex] - vy) > abs(velocities[dragIndex + 1] - vy):
        dragIndex += 1
    return rockC[dragIndex]

def calcDensity(height):
    global T0
    global percHumidity
    global Md
    global Mv
    Th = T0 - 0.00356 * height
    Ptotal = 101.29 * pow( (( Th + 273.1 ) / 288.08 ), 5.256 )
    Pdry = Ptotal * ( 1 - percHumidity )
    Pvap = Ptotal * ( percHumidity )
    P = ( Pdry * Md + Pvap * Mv ) / ( 8.314 * Th )          # kg/m^3
    return P * 0.062428


ReadDrags()
#calcRockDrag()


while math.fabs( apogee - max ) > 0.01:       # for finding the correct burn time within 0.01 ft
    ymid = 0.0
    vymid = 0.0
    aymid = 0.0
    ay = 0.0
    y = 0.0
    vy = 0.0
    t = 0.0
    dt = 0.00001
    avgThrust = 328.8955 * g / mass
    max = 0
    prevHigher = False
    drag = 0
    extension = 0
    dragIndex = 0
    while y >= 0: # or vy > 0:
        # kills the engine at the end of burn time
        if t >= burnT:
            avgThrust = 0
            extension = 1
        # acceleration by drag
        #drag = (1/mass) * 0.5 * (tabSA * tabdragC * extension + rockSA * rockdragC) * p * (vymid ** 2 )
        drag = (1/mass) * ( calcRockDrag(vy) + 0.5 * (tabSA * tabdragC * extension) * calcDensity(y) * (vymid ** 2 ) )
        # makes drag point the opposite direction of velocity
        if vy < 0:
            drag *= -1
        vymid = vy + ay*0.5*dt                      # vy at middle of interval
        aymid = ( -1 * g ) - drag + avgThrust  # ay at middle of interval
        y += vymid * dt                             # new altitude
        vy += aymid * dt                            # new velocity
        t += dt                                     # new time
        xgraph.append(t)
        ygraph.append(y)
        if y > max:
            max = y
        if vy > maxv:
            maxv = vy
        # prints to file every 100 ms (any more would just be unnecesary)
        if count > 10000:
            #file.write(str(y) + "," + str(vy) + "," + str(t) + "\n" )        # comment this shit out when probing for initial burn time
            count = 0
        #print("altitude", y)                   # comment this shit out when probing for initial burn time
        #print("velocity", vy)                  # comment this shit out when probing for initial burn time
        count += 1
    print("max: ", max)
    print("maxv: ", maxv)
    print("burnT ", burnT)
    # increments or decrements the burn time increment
    # if the previous loop burn time was too low and now is too high will make increment smaller
    # eventually this will get really close to burn time required for apogee
    if probing:
        if max > apogee:
            if not prevHigher:
                burnInc /= 1.5
            burnT -= burnInc
            prevHigher = True
        else:
            burnT += burnInc
    else:
        break

plt.plot(xgraph, ygraph)
plt.xlabel('time')
plt.ylabel('ft')
plt.title('Rocket Man')
#plt.show()
