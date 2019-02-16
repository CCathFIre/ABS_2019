# README:
# All the stuff commented out around the while loop is used to determine the burntime for bestFlight
# If you need to get this value then uncomment and fix all the spacing (bcs python)
# This simulation does not account for the parachute because who cares, however it might make your graphs look wierd
# if ay > 200 at any point then you probably fucked up the simulation

import math

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

tabSA = 0.00694444  # surface area ft^2 (equal to 1 in^2)
tabdragC = 1.2      # UPDATE if you get a better const
p = 0.0765          # Air density lbm/ft3, check if units are right, UPDATE if you have any idea what value to use for this

rockSA = math.pi * ( 0.6258333 ** 2 ) # ft^2, from pi*r^2 IDK if I fucked up the math I'm CS not a calculator
rockdragC = 0.295   # UPDATE if you get a better const

avgThrust = 250.9   # lbf, T H R U S T (not time)
avgThrust *= g      # CHECK MATH freedom units are hard
#burnT = 4.47       # s, CDR burn time was listed at 4.47 but we need a perfect flight
burnT = 6.786024780273437 # s, burn time for best flight as determined by CODING and ALGORITHIMS
mass = 52.5         # lbs, of rocket

burnInc = 0.5       # when probing for burn time
prevHigher = False

avgThrust /= mass   # we want acceleration by thrust
print("avg thrust ", avgThrust)

max = 0.0           # highest the simulated rocket went
maxv = 0.0

count = 0

file = open("bestFlight.txt","w+")     # comment out when probing for burntime
file.write( "altitude,velocity,time\n" )

#check = 0
#while math.fabs( apogee - max ) > 0.01:       # for finding the correct burn time within 0.01 ft
#    ymid = 0.0
#    vymid = 0.0
#    aymid = 0.0
#    ay = 0.0
#    y = 0.0
#    vy = 0.0
#    t = 0.0
#    dt = 0.00001
#    avgThrust = 250.9 * g / mass
#    max = 0
#    prevHigher = False
#    drag = 0
while y >= 0: # or vy > 0:
    # kills the engine at the end of burn time
    if t >= burnT:
        avgThrust = 0
    # acceleration by drag
    drag = (1/mass) * 0.5 * (tabSA * tabdragC + rockSA * rockdragC) * p * (vymid ** 2 )
    # makes drag point the opposite direction of velocity
    if vy < 0:
        drag *= -1
    vymid = vy + ay*0.5*dt                      # vy at middle of interval
    aymid = ( -1 * g ) - drag + avgThrust  # ay at middle of interval
    y += vymid * dt                             # new altitude
    vy += aymid * dt                            # new velocity
    t += dt                                     # new time
    if y > max:
        max = y
    if vy > maxv:
        maxv = vy
    # prints to file every 100 ms (any more would just be unnecesary)
    if count > 10000:
        file.write(str(y) + "," + str(vy) + "," + str(t) + "\n" )        # comment this shit out when probing for initial burn time
        count = 0
    print("altitude", y)                   # comment this shit out when probing for initial burn time
    print("velocity", vy)                  # comment this shit out when probing for initial burn time
    count += 1
print("max: ", max)
print("maxv: ", maxv)
print("burnT ", burnT)
    # increments or decrements the burn time increment
    # if the previous loop burn time was too low and now is too high will make increment smaller
    # eventually this will get really close to burn time required for apogee
    #if max > apogee:
    #    if not prevHigher:
    #        burnInc /= 2.0
    #    burnT -= burnInc
    #    prevHigher = True
    #else:
    #    burnT += burnInc
    #check += 1
