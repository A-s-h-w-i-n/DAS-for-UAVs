#********************************************GROUND STATION CODE FOR**************************#
#*********************************************DATA ACQUISITION SYSTEM****************************#
#********************************************PAYLOAD DROP MECHANISM***************************#
#********************************************AUTOMATIC PAYLOAD DROP****************************#
#**************************************************ANTENNA TRACKER**********************************#


import serial       #Import the required modules
import struct
import cv2
import time
from math import radians,sin,cos,asin,sqrt,fabs, atan2, atan, fabs    #Import all required math functions

cap = cv2.imread('logo.jpg')        #Load image         
arduino = serial.Serial('COM19', 57600,timeout=0.1)     #Define serial ports
arduino1 = serial.Serial('COM15', 57600, timeout=0.1)
cv2.imshow('frame',cap)

time.sleep(3)   #wait for 3 seconds to receive arduino data

fh=open("height5.log","w")      #Open altitude log file

alt,flag,d,vel,lat,lon,flag2=0,0,0,0,0,0,0
hav = d + 100
tilt_angle = 0
pan_angle = 0

def haversine(lat1, lon1, lat2, lon2):      #Calculate distance between 2 gps coordinates
    lat1, lon1, lat2, lon2 = map(radians, [lat1, lon1, lat2, lon2])
    dlon = lon2 - lon1 
    dlat = lat2 - lat1 
    a = sin(dlat/2)**2 + cos(lat1) * cos(lat2) * sin(dlon/2)**2
    c = 2*atan2(sqrt(a), sqrt(1-a))
    return 6371000 * c

def distance(vel,alt):      #Calculate distance from drop zone
    t = sqrt(2*fabs(alt)/9.81)
    return vel*t*5/18

def pan_tilt(glat, glon, flat, flon, height):       #Tilt and Pan angles of antenna tracker
    lat_diff = flat-glat
    lon_diff = flon-glon
    X = cos(flat) * sin(lon_diff) 
    Y = cos(glat) * sin(flat) - sin(glat) * cos(flat) * cos(lon_diff)

    pan_angle = atan2(X,Y)
    pan_angle = pan_angle/0.0174533
    pan_angle = (int)(pan_angle+360)%360

    dist = haversine(glat, glon, flat, flon)

    tilt_angle = atan(fabs(height/dist))
    tilt_angle = int(tilt_angle/0.0174533)
    if pan_angle<=180:
        pan_angle = 180 - pan_angle
        tilt_angle = 180 - tilt_angle
    else:
        pan_angle = 360 - pan_angle

    arduino1.write(str(int(pan_angle))+'n')
    arduino1.write(str(int(tilt_angle)))

x_time = time.time()
t_end = time.time() 

while True:
    try:
        command = arduino.read(1) #Read the first byte
        if command == "h":
            flag = 1
            alt = arduino.readline()
            if time.time() - t_end >= 0.5:      #Write into log file every 0.5 second
                fh.write(alt)
                t_end = time.time()
            alt = float(alt)                       #Altitude
            alt_feet = alt
            print  alt
            alt = alt/3.28084 
            lat = arduino.readline()      #Latitude      
            lat = float(lat)
            #print  lat
            lon = arduino.readline()     #Longitude
            lon = float(lon)
            #print  lon
            vel = arduino.readline()     #Velocity
            vel = float(vel)
            #print  vel
        
        elif (command == "t" and flag2 != 2):
            print "DROPPED AT: " + str(alt_feet)
            flag2 = 2
            fh.write("\nDROPPED AT: " + str(alt_feet) + "\n")
            
        else:    
            arduino.flushInput();       #Flush input buffer to remove garbage

        # flag == 1 is checked to ensure that data is being received
        
        if flag == 1:      #Calculate horizontal range of payload at any given time and velocity
            hav = haversine(12.970335, 79.155372, lat, lon)
            d = distance(vel,alt)
            #print hav
            #print d
            print
            
        if time.time() - x_time >= 0.7 and flag == 1:        #Send pan and tilt angles to antenna tracker every 0.7 sec
            pan_tilt(12.975904, 79.160224, lat, lon, alt)
            x_time = time.time()

        t = cv2.waitKey(30) & 0xFF

        if (t == ord('d') or t == ord('D') or flag2 == 1) :                #Check for drop command or automatic payload drop
            flag2 = 1
            arduino.write("d")  
        elif (t == ord('r') or t == ord('R')):                 #Check for reset command
            arduino.write("r")
        elif (t == ord('o') or t == ord('O')):
            arduino.write("o")
        elif (t == ord('c') or t == ('C')):
            arduino.write("c")
        elif (t == ord('q') or t == ord('Q')):               #Check command for ending program
            break
        
    except:
        print "hello"
        pass
    
cv2.destroyAllWindows()
