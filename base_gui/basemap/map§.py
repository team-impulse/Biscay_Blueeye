from mpl_toolkits.basemap import Basemap
import matplotlib.pyplot as plt
import matplotlib.pylab as pylab
import numpy as np
import string
import matplotlib.cm as cm
import time
from drawnow import *
from pylab import *
import random
#NB
#ORIGINAL GRAPHING CODE AND CONCEPT (of using scatter graph) FROM https://stevendkay.wordpress.com/2010/02/24/plotting-points-on-an-openstreetmap-export/
const_pos_sharefile = 'location.csv'
xarray = []
yarray = []
x=-9.385770 #longitudes
y=39.108877 #latitudes
plt.ion()
def makeFigs():
    m = Basemap(llcrnrlon=-9.382918,llcrnrlat=39.119533,urcrnrlon= -9.376521,urcrnrlat= 39.127501,  resolution='l',projection='merc')
    x1,y1=m(x,y)
    xarray.append(x1)
    yarray.append(y1)
    m.drawcoastlines()
    im = plt.imread("map.png")
    m.imshow(im, origin='upper')
    print xarray
    print yarray
    print "======"
    m.scatter(xarray,yarray,c='b',marker=".",alpha=1.0)
while(1):
    #X AND Y ARE THE RECEIVED DATA FROM THE GPS SENSOR
    #for now they have just been set to random numbers between the limits
    pos = open(const_pos_sharefile,'r')
    last_line = pos.readline()
    while(last_line!=""):
        y = float(last_line.split(",")[0])
        x = float(last_line.split(",")[1])
        last_line = pos.readline()
    drawnow(makeFigs)
    time.sleep(0.1)

    #SYSTEM TO START DUMPING OLD DATA - might be completely useless?
    threshold = 1500 #maximum pieces of data
    if len(xarray) > threshold:
        xarray.pop(0)
        yarray.pop(0)
