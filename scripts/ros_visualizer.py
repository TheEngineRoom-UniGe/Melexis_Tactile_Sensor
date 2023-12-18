#!/usr/bin/env python
import sys
import os
from pathlib import Path
import rospy
from melexis_v2.msg import MelexisForce

cwdpath = Path(os.path.dirname(os.path.realpath(__file__)))
# print("Current working directory: {0}".format(cwdpath))
ParentPath = cwdpath.parent.absolute()
LibraryPath = str(ParentPath)+"/UNIGE/Library"
# print("Library path: {0}".format(LibraryPath))
sys.path.append(LibraryPath)
RessourcesPath = str(ParentPath)+"/UNIGE/Ressources"
sys.path.append(RessourcesPath)
sys.path.append(cwdpath)
 
########################################################################
port = '/dev/ttyACM0' #enter the correct port number
pathData = str(ParentPath)+"/UNIGE/Data/Calib.csv"
ack=True  #True: Falt free instantiation (to do if you just plugged MelexIO) / False: Fast instantiation
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
try :
    for i,m in enumerate(get_monitors()): #get size of the screen
        if i==0:
            w = m.width
            h = m.height
except :
    w = 1900
    h = 1080
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
plt.rcParams['backend'] = 'TkAgg'

import TactaxisForce

flag = True
data = []

def live_update_demo():
    global Sensor,Ftare,w,h
    
    fig = plt.figure(facecolor="#00354B")
    plt.subplots_adjust(left=0.05, bottom=0.05, right=0.95, top=0.95, wspace=0.2, hspace=0.1)
    figManager = plt.get_current_fig_manager()
    figManager.set_window_title('Tactaxis Demo')
    figManager.resize(w*0.8,h*0.9)  #to change for resolution issue
    try :
        figManager.window.wm_geometry("+0+0") #set the offset
    except :
        print("Visualization bug, offset of the window might have to be adjusted by hand")
    figManager.show()
    fig.canvas.flush_events()

    #variables
    X=[]
    Y=[]
    Y1=[]
    Y2=[]


    #cosmetics
    cmap = matplotlib.colors.LinearSegmentedColormap.from_list("", ["#EEA320","#DB4140"])
    circle = plt.Circle((0, 0), 1,edgecolor = "#b2c4cb",ls = "--",lw = 5,fill=False)
    SquareMid = Circle((0, 0), 0.05,color = "#b2c4cb")
    arrowX = Arrow(0,0,1,0,width = 0.35,color="#b2c4cb",alpha=0.6)
    arrowY = Arrow(0,0,0,1,width = 0.35,color="#b2c4cb",alpha=0.6)
    
    logo = plt.imread(RessourcesPath+"/MelexisLogo.png")
    TactaxisConcept = plt.imread(RessourcesPath+"/TactaxisDiagram.png")
    
    #create subplots
    
    axTitle = fig.add_subplot(3,5, (1,2))
    axConcept = fig.add_subplot(3,4,4)
    axTaxtlab = fig.add_subplot(3,4,3)
    ax1 = fig.add_subplot(3,4, (7,8))
    ax2 = fig.add_subplot(3,4, (11,12),sharex=ax1)
    ax3 = fig.add_subplot(3,4, (5,10))

    #Fill subplots

    axTitle.axis("off")
    axTitle.imshow(logo,aspect="equal")

    axConcept.axis("off")
    axConcept.imshow(TactaxisConcept,aspect="equal")

    axTaxtlab.axis("off")
    txtTactaxis = axTaxtlab.text(0.3,0.35,"Tactaxis"+u"\u2122",fontsize=65,fontweight = "bold",color = "#FFFFFF")

    ax1.set_facecolor("#00354B")
    ax1.tick_params(labelsize=12,color= "#ffffff",labelcolor="#ffffff")
    # ax1.set_xticks([])
    ax1.set_ylim([-6.5, 0.3])
    ax1.grid(color= "#3e4242",linewidth=4)
    ax1.set_ylabel("Fnormal [N]",fontsize=15,fontweight = "bold",color = "#FFFFFF")
    ax1.spines['bottom'].set_color('#b2c4cb')
    ax1.spines['top'].set_color('#b2c4cb')
    ax1.spines['left'].set_color('#b2c4cb')
    ax1.spines['right'].set_color('#b2c4cb')
    ax1.yaxis.label.set_color('#ffffff')
    line1, = ax1.plot([],lw=3,ls="dotted",marker="o",markersize=10,color = "#65BBA9",animated = False)

    ax2.set_facecolor("#00354B")
    ax2.tick_params(labelsize=12,color= "#ffffff",labelcolor="#ffffff")

    ax2.set_xlabel("Time",fontsize=15,fontweight = "bold",color = "#FFFFFF")
    ax2.set_ylabel("Fshear [N]",fontsize=15,fontweight = "bold",color = "#FFFFFF")
    ax2.tick_params(labelsize=12,color= "#b2c4cb")
    ax2.spines['bottom'].set_color('#b2c4cb')
    ax2.spines['top'].set_color('#b2c4cb')
    ax2.spines['left'].set_color('#b2c4cb')
    ax2.spines['right'].set_color('#b2c4cb')
    # ax2.set_xticks([])
    ax2.set_ylim([-1.3, 1.3])
    ax2.grid(color= "#3e4242",linewidth=4)
    ax2.yaxis.label.set_color('#ffffff')
    line2, = ax2.plot([],lw=3,ls="dotted",marker="o",markersize=10,color="#DB4140",animated = False,label="Fx")
    line3, = ax2.plot([],lw=3,ls="dotted",marker="o",markersize=10,color="#EEA320",animated = False,label="Fy")
    ax2.legend(framealpha=0)

    ax3.set_aspect('equal', 'box')
    ax3.set_facecolor("#00354B")
    ax3.add_patch(circle)
    ax3.spines['bottom'].set_color('#b2c4cb')
    ax3.spines['top'].set_color('#b2c4cb')
    ax3.spines['left'].set_color('#b2c4cb')
    ax3.spines['right'].set_color('#b2c4cb')
    ax3.add_patch(SquareMid)
    ax3.tick_params(axis='x',which='both',bottom=False,top=False,labelbottom=False,direction="in",color = "#FFFFFF")
    ax3.tick_params(axis='y',which='both',left=False,right=False,labelleft=False,direction="in",color = "#FFFFFF")
    ax3.set_ylim([-1,1])
    ax3.set_xlim([-1,1])
    ax3.grid()
    txtFx = ax3.text(0.9,0.1,"Fx",fontsize=20,fontweight = "bold",color = "#b2c4cb")
    txtFy = ax3.text(0.1,0.85,"Fy",fontsize=20,fontweight = "bold",color = "#b2c4cb")
    arrowXX = ax3.add_artist(arrowX)
    arrowYY = ax3.add_artist(arrowY)

    fig.canvas.draw()

    # cache the background
    axbackground  = fig.canvas.copy_from_bbox(ax1.bbox)
    ax2background = fig.canvas.copy_from_bbox(ax2.bbox)
    ax3background = fig.canvas.copy_from_bbox(ax3.bbox)


    t_start = time.time()
    i=0

    
    pub = rospy.Publisher('/melexis_force', MelexisForce, queue_size=10)


    while True:
        tlim=100
        X.append(time.time()-t_start)
        B,F = Sensor.getForce(dual=True)
        F -=Ftare
        Y.append(F[2])
        Y1.append(F[0])
        Y2.append(-F[1])
        size = np.max([0.25+0.04*F[2],0.03])  #size of the dot
        if np.abs(F[0])<0.01 and np.abs(F[1])<0.01:  #deadzone
            arrow1 = Circle((0,0),size,color=cmap(-F[2]/6))
        else :
            arrow1 = Circle((-F[0],F[1]),size,color=cmap(-F[2]/6))
        line1.set_xdata(X)
        line1.set_ydata(Y)
        line2.set_xdata(X)
        line2.set_ydata(Y1)
        line3.set_xdata(X)
        line3.set_ydata(Y2)
        arrowz =ax3.add_artist(arrow1)
        if i>2 and i<=tlim:
            ax1.set_xlim([0,np.max([2,X[-1]])])
        if i>tlim:
            ax1.set_xlim([X[-tlim],X[-1]])

        fig.canvas.restore_region(axbackground)
        fig.canvas.restore_region(ax2background)
        fig.canvas.restore_region(ax3background)
        # redraw just the points
        ax1.draw_artist(line1)
        ax2.draw_artist(line2)
        ax2.draw_artist(line3)
        ax3.draw_artist(arrow1)
        # fill in the axes rectangle
        fig.canvas.blit(ax1.bbox)
        fig.canvas.blit(ax2.bbox)
        fig.canvas.blit(ax3.bbox)
        #flush
        fig.canvas.flush_events()
        i+=1
        if i%1000==0: #Delete points after 1000 iteration, only keep 200 for memory issue
            Y = Y[-200:]
            X = X[-200:]
            Y1 = Y1[-200:]
            Y2 = Y2[-200:]

        msg = MelexisForce()
        msg.header.stamp = rospy.Time.now()
        msg.Fz = F[2]
        msg.Fx = F[0]
        msg.Fy = F[1]
        msg.Bx00 = B[0]
        msg.Bz00 = B[1]
        msg.Bx01 = B[2]
        msg.Bz01 = B[3]
        msg.Bx10 = B[4]
        msg.Bz10 = B[5]
        msg.Bx11 = B[6]
        msg.Bz11 = B[7]
        msg.Temperature = B[8]

        pub.publish(msg)


if __name__ == '__main__':
    rospy.init_node('melexis_ros_visualizer', anonymous=True)
    node_name = rospy.get_name()
    port = rospy.get_param('~port')

    print(f'[{node_name}] ==> ', 'Trying to open port:', port)
    print(f'[{node_name}] ==> ', "Enabling connection to the board")
    
    with TactaxisForce.TactaxisForce(str(port),pathDataset=pathData,sfi=sfi,ack=ack) as Sensor:
        Ftare = np.zeros(3)
        Ftareav = np.zeros((10,3))
        for k in range(10):
            Bnew,Fnew = Sensor.getForce(dual=True)
            Ftareav[k]=Fnew
        Ftare = np.mean(Ftareav,axis=0)
        print(f'[{node_name}] ==> ', "Connection established")
        live_update_demo()
