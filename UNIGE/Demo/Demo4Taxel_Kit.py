# -*- coding: utf-8 -*-
"""
Created on Wed May 24 14:17:15 2023

@author: tls
"""

import sys
import os
from pathlib import Path

cwdpath = Path(os.getcwd())
ParentPath = cwdpath.parent.absolute()
LibraryPath = str(ParentPath)+"\\Library"
sys.path.append(LibraryPath)
RessourcesPath = str(ParentPath)+"\\Ressources"
sys.path.append(RessourcesPath)

########################################################################
port = 10 #enter the correct port number
pathData = str(ParentPath)+"\\Data\\Calib.csv"
ack=False  #True: Falt free instantiation (to do if you just plugged MelexIO) / False: Fast instantiation
sfi=True  #True: Stray field rejection / False: Sensitive to stray field
########################################################################

import time
import matplotlib.pyplot as plt
import numpy as np
from matplotlib.patches import Circle,Arrow
import matplotlib.font_manager as font_manager
import matplotlib
# Add every font at the specified location

for font in font_manager.findSystemFonts(RessourcesPath):
    font_manager.fontManager.addfont(font)

from screeninfo import get_monitors
for i,m in enumerate(get_monitors()): #get size of the screen
    if i==0:
        w = m.width
        h = m.height
    
# Set font family globally
try :
    plt.rcParams['font.family'] = 'Lato'
except :
    print("Lato font not installed !")
plt.rcParams["grid.color"] = "#b2c4cb"
plt.rcParams["text.color"] = "#ffffff"
plt.rcParams["axes.labelcolor"] = "white"
plt.rcParams["legend.labelcolor"] = "white"
plt.rcParams['toolbar'] = 'None'

import TactaxisForce

flag = True
data = []
mapping = np.array([[-1,1],[1,-1],[1,1],[-1,-1]])



def live_update_demo(blit = False):
    global Sensor,Ftare,w,h
    fig = plt.figure(facecolor="#00354B")
    figManager = plt.get_current_fig_manager()
    figManager.set_window_title('Tactaxis Demo')
    figManager.resize(w*0.8,h*0.9)  #to change for resolution issue
    figManager.window.wm_geometry("+0+0") #set the offset
    figManager.show()
    fig.canvas.flush_events()
    #time.sleep(1)


    #cosmetics
    cmap = matplotlib.colors.LinearSegmentedColormap.from_list("", ["#EEA320","#DB4140"])
    #circle = plt.Circle((0, 0), 1,edgecolor = "#b2c4cb",ls = "--",lw = 5,fill=False)
    #SquareMid = Circle((0, 0), 0.05,color = "#b2c4cb")
    #arrowX = Arrow(0,0,1,0,width = 0.35,color="#b2c4cb",alpha=0.6)
    #arrowY = Arrow(0,0,0,1,width = 0.35,color="#b2c4cb",alpha=0.6)
    
    logo = plt.imread(RessourcesPath+"\\MelexisLogo.png")
    TactaxisConcept = plt.imread(RessourcesPath+"\\TactaxisDiagram.png")
    
    
    #create subplots
    plt.subplots_adjust(left=0.05, bottom=0.05, right=0.95, top=0.95, wspace=0.2, hspace=0.1)

    axTitle = fig.add_subplot(4,4, 1)
    axTaxtlab = fig.add_subplot(4,4,5)
    axConcept = fig.add_subplot(2,3,4)
    axGrid =  fig.add_subplot(3,4, (2,12))
    

    #Fill subplots

    axTitle.axis("off")
    axTitle.imshow(logo,aspect="equal")

    axConcept.axis("off")
    axConcept.imshow(TactaxisConcept,aspect="equal")

    axTaxtlab.axis("off")
    txtTactaxis = axTaxtlab.text(0.3,0.35,"Tactaxis"+u"\u2122",fontsize=40,fontweight = "bold",color = "#FFFFFF")

    axGrid.set_aspect('equal', 'box')
    axGrid.set_facecolor("#00354B")
    #axLeft.add_patch(circle)
    axGrid.spines['bottom'].set_color('#b2c4cb')
    axGrid.spines['top'].set_color('#b2c4cb')
    axGrid.spines['left'].set_color('#b2c4cb')
    axGrid.spines['right'].set_color('#b2c4cb')
    axGrid.tick_params(axis='x',which='both',bottom=False,top=False,labelbottom=False,direction="in",color = "#FFFFFF")
    axGrid.tick_params(axis='y',which='both',left=False,right=False,labelleft=False,direction="in",color = "#FFFFFF")
    axGrid.set_ylim([-2,2])
    axGrid.set_xlim([-2,2])
    axGrid.grid()
    
    

    fig.canvas.draw()

    # cache the background
    axLbackground = fig.canvas.copy_from_bbox(axGrid.bbox)

    plt.show(block=False)

    while True:
        F = Sensor.getMultForceCust(dual=False)
        F -=Ftare
        Lpoint = []
        for k in range(4):
            size = np.max([0.25+0.04*F[k][2],0.03])#*(0.2*(k+1))  #size of the dot
            if np.abs(F[k][0])<0.01 and np.abs(F[k][1])<0.01:  #deadzone
                point = Circle((mapping[k][0],mapping[k][1]),size,color=cmap(-F[k][2]/6))
            else :
                point = Circle((mapping[k][0]+F[k][1],mapping[k][1]+F[k][0]),size,color=cmap(-F[k][2]/6))
            Lpoint.append(point)
            axGrid.add_artist(point)
                
        fig.canvas.restore_region(axLbackground)
        # redraw just the points
        for k,point in enumerate(Lpoint) :
            axGrid.draw_artist(point)
        # fill in the axes rectangle
        fig.canvas.blit(axGrid.bbox)
        fig.canvas.flush_events()


if __name__ == '__main__':
    print("Enabling connection to the board")
    with TactaxisForce.TactaxisForce(str(port),pathDataset=pathData,activeDUT=[0,1,2,4,5,8,9,10],sfi=sfi,ack=ack) as Sensor:
        Ftare = np.zeros((4,3))
        Ftareav = np.zeros((10,4,3))
        for k in range(10):
            Fnew = Sensor.getMultForceCust(dual=False)
            Ftareav[k]=Fnew
        Ftare = np.mean(Ftareav,axis=0)
        print("Connection established")
        
        live_update_demo(True)
