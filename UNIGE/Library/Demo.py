import sys
import os
from pathlib import Path
import time
import matplotlib.pyplot as plt
import numpy as np
from matplotlib.patches import Circle, PathPatch,Arrow
import matplotlib.artist as art
import matplotlib.cm as cm
cwdpath = Path(os.getcwd())
ParentPath = cwdpath.parent.absolute()
LibraryPath = str(ParentPath)+"/Library"
sys.path.append(LibraryPath)  

import TactaxisForce

########################################################################
port = '/dev/ttyACM1' #enter the correct port number
pathData = str(ParentPath)+"/UNIGE/Data/Calib.csv"
########################################################################

flag = True
data = []

def live_update_demo(blit = False):
    global Sensor,Ftare
    fig = plt.figure(figsize=(15,7),facecolor='whitesmoke')
    ax1 = fig.add_subplot(2,16, (12,16))
    ax2 = fig.add_subplot(2,16, (28,32),sharex=ax1)
    ax3 = fig.add_subplot(2,16, (1,26))
    X=[]
    Y=[]
    Y1=[]
    Y2=[]
    line1, = ax1.plot([],lw=3,ls="--",marker="o",markersize=10,color = "black",animated = False)

    #ax1.set_xlabel("Time [s]",fontsize=15,fontweight = "bold")
    ax1.set_ylabel("Fnormal [N]",fontsize=15,fontweight = "bold")
    ax1.set_title("Fast Animation",fontsize=25,fontweight = "bold",pad=25)
    #ax1.set_xlim([0,10])
    ax1.tick_params(labelsize=12)
    ax1.set_xticks([])
    #.set_yticks(np.arange(-0.5,15,0.5))
    ax1.set_ylim([-8, 0.3])
    ax1.grid()
    
    line2, = ax2.plot([],lw=3,ls="--",marker="o",markersize=10,color = "blue",animated = False,label="Fx")
    line3, = ax2.plot([],lw=3,ls="--",marker="o",markersize=10,color = "red",animated = False,label="Fy")
    ax2.set_xlabel("Time [s]",fontsize=15,fontweight = "bold")
    ax2.set_ylabel("Fshear [N]",fontsize=15,fontweight = "bold")

    ax2.tick_params(labelsize=12)
    ax2.set_xticks([])
    #.set_yticks(np.arange(-0.5,15,0.5))
    ax2.set_ylim([-1.3, 1.3])
    ax2.grid()
    
    ax3.set_xlabel("Fy Normalized",fontsize=12)
    ax3.set_ylabel("Fx Normalized",fontsize=12)
    ax3.tick_params(axis='x',which='both',bottom=False,top=False,labelbottom=False,direction="in")
    ax3.tick_params(axis='y',which='both',left=False,right=False,labelleft=False,direction="in")
    ax3.set_title("Angle of shear forces",fontsize=17,fontweight = "bold") #pad = 25
    ax3.set_xlim([-1.1,1.1])
    ax3.set_ylim([-1.1,1.1])
    ax3.grid()
    scalmap = cm.ScalarMappable(cmap=plt.cm.YlOrRd)
    scalmap.set_clim(vmin=0,vmax=1.2)
    cbar = fig.colorbar(scalmap, ax=ax3,label = "Norm of shear forces")
    arrow1 = Arrow(0,0,0,0,width = 0.1,color=plt.cm.YlOrRd(1/1.2))
    #arrax, = ax3.Arrow(0,0,1,0,width = 0.1,color=plt.cm.YlOrRd(1/1.5))
    arrowz = ax3.add_artist(arrow1)
    circle = plt.Circle((0, 0), 1,edgecolor = "Black",ls = "--",lw = 3,fill=False)
    ax3.add_patch(circle)
    fig.canvas.draw()   # note that the first draw comes before setting data 
    

    if blit:
        # cache the background
        axbackground  = fig.canvas.copy_from_bbox(ax1.bbox)
        ax2background = fig.canvas.copy_from_bbox(ax2.bbox)
        ax3background = fig.canvas.copy_from_bbox(ax3.bbox)

    plt.show(block=False)


    t_start = time.time()
    i=0
    while True:
        tlim=100
        X.append(time.time()-t_start)
        B,F = Sensor.getForce(dual=True)
        F -=Ftare
        Fr = np.sqrt(np.power(F[0],2)+np.power(F[1],2))
        Y.append(F[2])
        Y1.append(F[0])
        Y2.append(-F[1])
        arrowz.remove()
        #ax3.clear() 
        if Fr<0.18:#0.18
            #arrow1 = Circle((0,0),0.05+F[2]/10,color="Yellow")
            arrow1 = Circle((0,0),0.1,color="Yellow")
        else :
            arrow1 = Arrow(0,0,F[0]/(Fr),-F[1]/(Fr),width = 0.5,color=plt.cm.YlOrRd(Fr/1.2))

        
        
        line1.set_xdata(X)
        line1.set_ydata(Y)
        line2.set_xdata(X)
        line2.set_ydata(Y1)
        line3.set_xdata(X)
        line3.set_ydata(Y2)
        arrowz =ax3.add_artist(arrow1)
        if i>2 and i<=tlim:
            ax1.set_xlim([0,np.max([3,X[-1]])])
        if i>tlim:
            ax1.set_xlim([X[-tlim],X[-1]])

        if blit:
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

        else:
            # redraw everything
            fig.canvas.draw()

        fig.canvas.flush_events()
        i+=1


if __name__ == '__main__':
    print("Enabling connection to the board")
    with TactaxisForce.TactaxisForce(str(port),pathDataset=pathData) as Sensor:
        Ftare = np.zeros(3)
        Ftareav = np.zeros((10,3))
        for k in range(10):
            Bnew,Fnew = Sensor.getForce(dual=True)
            Ftareav[k]=Fnew
        Ftare = np.mean(Ftareav,axis=0)
        print("Connection established")
        live_update_demo(True)