# -*- coding: utf-8 -*-
"""
Created on Wed Apr 13 15:31:00 2022

@author: tls
"""

import Melexio_Base
import numpy as np
import time
import os

class Tactaxis(Melexio_Base.MelexIO): 
    """
    Class that inherits from Melexio_Base class, high-level
    Sets the board to work with the sensor module
    Has methods to measure the magnetic signal of the chip (Running pattern, parsing the data and transfroming it to mT)
    """
    def __init__(self,portName,activeDUT=[8,9]):
        super(Tactaxis,self).__init__(portName)
        frequencyTck    = 40
        voltageAPPDac   = 4000
        voltageETMDac   = 4000
        voltageRSTDac   = 0
        delayTon        = 0
        delayToff       = 0
        BytesToReadout  = 6
        S2M_MODE        = 0x03  #24bits
        
        self.activeDUT = np.sort(activeDUT)
        self.rstRegValueExpected = [0x8008,0x9008,0x8009,0x9009,0x8000]        #When receiving these values from the rst register (when running pattern) the output is correct.
        self.mapChipPort = [[0,8],[1,9],[2,10],[3,11],[4,12],[5,13],[6,14],[7,15],[8,9]]          #Position of the two dies in the readout buffer (With the big melexIO shield)
        self.S2M_Mode = S2M_MODE                                               #2 is 2 bytes, 3 is 3 bytes, 4 is 4 bytes   
        self.BytesToReadout = BytesToReadout                                   #self.mapChipPort[-1] is the device connected to the signle modules
        self.setTckFrequency(frequencyTck)
        self.setVoltageEnterTM(voltageETMDac)
        self.setVoltageReset(voltageRSTDac)
        self.setVoltageApp(voltageAPPDac)
        self.setDelaySupplyOn(delayTon)
        self.setDelaySupplyOff(delayToff)
        self.setNumberTDOBytes(BytesToReadout)
        self.setS2MMode(S2M_MODE)
        self.setActiveDUT()
        print("Init of the board done")
    
    def ConvertBytesArray(self,Arr):
        """Returns an array corresponding to the 16 bits words contained in successive bytes in Arr
        Here the protocol to gather the 16 bits is not intuitive because of earlier issues with the serial communication
        Refer to the documentation of the Hardware communication to understand
        """
        
        if self.S2M_Mode==3:
            """     0b[0:7]  = Arr[x%3==0][0:7]
                    0b[7]    = Arr[x%3==2][7]
                    0b[8:16] = Arr[x%3==1][0:7]
                    0b[16]   = Arr[x%3==2][6]
                    Device number = Arr[x%3==1][0:3]
            """
            n = len(Arr)
            Out = np.zeros(n//3)
            lsb=0
            msb=0
            for i,byte in enumerate(Arr):
                res = 0
                if i%3==0: #first byte is lsb
                    lsb = int(byte)%128 #Take the 7 LSB
                elif i%3==1 :      #second byte is msb
                    msb = int(byte)%128
                elif i%3==2:
                    a = int(byte)>>6
                    msb+=128*(a%2==1)
                    a = a>>1
                    lsb+=128*(a%2==1) 
                    #recreating the result
                    res = (msb<<8)+lsb
                    Out[i//3]=int(res)
            return Out
        
        elif self.S2M_Mode==4:
            """     0b[0:7]  = Arr[x%4==0][0:7]
                    0b[7]    = Arr[x%4==2][0]
                    0b[8:16] = Arr[x%4==1][0:7]
                    0b[16]   = Arr[x%4==3][0]
            """
            n = len(Arr)
            Out = np.zeros(n//4) ###Change here
            lsb=0
            msb=0
            for i,byte in enumerate(Arr):
                res = 0
                if i%4==0: #first byte is lsb
                    lsb = int(byte)%128 #Take the 7 LSB
                elif i%4==1 :      #second byte is msb
                    msb = int(byte)%128
                elif i%4==2:
                    lsb+=128*(int(byte)%2==1)
                elif i%4==3:
                    msb+=128*(int(byte)%2==1)  
                    #recreating the result
                    res = (msb<<8)+lsb
                    Out[i//4]=int(res)
            return Out
        
        elif self.S2M_Mode==2:
            n = len(Arr)
            Out = np.zeros(n//2) ###Change here
            lsb=0
            msb=0
            for i,byte in enumerate(Arr):
                res = 0
                if i%2==0: #first byte is lsb
                    lsb = int(byte) #Take the 7 LSB
                elif i%2==1 :      #second byte is msb
                    msb = int(byte)
                    res = (msb<<8)+lsb
                    Out[i//2]=int(res)
            return Out
        
    def runConversion(self):
        """Main method to get the data from the chips, returns the magnetic signals for both patterns, the temperature and the rstRegister
        It operates in 3 steps.
        1) Getting the raw data from runPattern1 and runPattern2 commands
        2) Parsing the data
        3) Checking if the values received are correct by looking at the ResetReg values
    
        Returns 8 buffers of length 16 (number of dies tacking in charge by the board)

        - buffer_i_m1 (corresponding to Bx of the first disk - mode 1)
        - buffer_j_m1 (corresponding to Bz of the first disk - mode 1)
        - buffer_i_m2 (corresponding to Bx of the second disk - mode 2)
        - buffer_j_m2 (corresponding to Bx of the second disk - mode 2)
        - int_temp_m1 (corresponding to the temperature of the first disk - mode 1)
        - int_temp_m2 (corresponding to the temperature of the second disk - mode 2)
        - DEVICESTA_m1 (corresponding to error flags in the first disk - mode 1)
        - DEVICESTA_m2 (corresponding to error flags in the second disk - mode 2)
        DEVICESTA is 1 by default, 0 if there is an error
        """
        errorStatus = self.ErrorMessages_e['ERR_NOERR']
        nActiveDUT = len(self.activeDUT)
        
        buffer_i_m1 = np.zeros(nActiveDUT,dtype=int)
        buffer_i_m2 = np.zeros(nActiveDUT,dtype=int)
        buffer_j_m1 = np.zeros(nActiveDUT,dtype=int)
        buffer_j_m2 = np.zeros(nActiveDUT,dtype=int)
        int_temp    = np.zeros(nActiveDUT,dtype=int)
        ResetReg    = np.zeros(nActiveDUT,dtype=int)
        DEVICESTA   = np.ones(nActiveDUT,dtype=int)

        #Step 1: raw values
        errorStatus,dataMode,Arr,t_res = self.runPattern1(self.BytesToReadout,nActiveDUT*self.S2M_Mode*self.BytesToReadout)
        if (errorStatus != self.ErrorMessages_e['ERR_NOERR']):
            print(errorStatus)
            return (errorStatus)
       
        #Step2: Parsing the data
        ReconectBytes = self.ConvertBytesArray(Arr)
        # Parse the various buffers / devices
        for i in range(nActiveDUT): #device
            buffer_i_m1[i]=ReconectBytes[self.BytesToReadout*i+2]
            buffer_j_m1[i]=ReconectBytes[self.BytesToReadout*i+3]
            buffer_i_m2[i]=ReconectBytes[self.BytesToReadout*i+4]
            buffer_j_m2[i]=ReconectBytes[self.BytesToReadout*i+5]
            int_temp[i]   =ReconectBytes[self.BytesToReadout*i+1]
            ResetReg[i]   =ReconectBytes[self.BytesToReadout*i+0]
            
        
            #Step 3: Checking for error
            #Check error : Correct is 0x8008 or 0x9009 or 0x9008, else is wrong
            if ResetReg[i] not in self.rstRegValueExpected :
                DEVICESTA[i] = 0
                
        return buffer_i_m1,buffer_j_m1,buffer_i_m2,buffer_j_m2,int_temp,DEVICESTA
        
        
    def getValues(self,NoErr = True,repMax=10):
        """Runs conversion and returns the chips values but with additionnal features.
        
        Parameters:
            - ListDevice, a list of numbers corresponding to the dies we are willing to read (other values won't be read)
            - NoErr, if True, the runConversion will be repeated untill there is no error flag in the values, else it is runned once
            - rep Max, maximum number of times runConversion is repeated before triggering an alert message and return the last values
        """
        buffer_i_m1,buffer_j_m1,buffer_i_m2,buffer_j_m2,int_temp,DEVICESTA = self.runConversion()
        if NoErr:
            rep=1
            while len(np.where(DEVICESTA<1)[0]) and rep<repMax:
                buffer_i_m1,buffer_j_m1,buffer_i_m2,buffer_j_m2,int_temp,DEVICESTA= self.runConversion()
                rep+=1
            if rep ==repMax and not(len(np.where(DEVICESTA<1)[0])):
                print("Too many errors, couldn't find a perfect readout")
        return np.array([buffer_i_m1,buffer_j_m1,buffer_i_m2,buffer_j_m2,int_temp])
    
    def signedLSB(self,lsb):
        """Method that corrects the sign of a signed 16 bits number"""
        out = lsb
        for k in range(len(lsb)):
            if lsb[k] > 32767:
                out[k] -=65536
        return out
    
    def getSensisitivityFromTemp(self,Tlin):
        """Method that corrects the temperature drift of the sensitivity of the HE"""
        Td = (Tlin-865)/8
        return np.poly1d([-5.33e-8,1.4e-5,-0.00463,1])(Td)
            
    def getValuesmT(self,NoErr = True,repMax=10):
        """Same as getValues but returns the corresponding values of the magnetic signal in mT"""
        buffer_i_m1,buffer_j_m1,buffer_i_m2,buffer_j_m2,int_temp= self.getValues(NoErr = NoErr,repMax=repMax)
        S_HE = self.getSensisitivityFromTemp(int_temp)  #Change of the sensitivity of the HE due to temperature modification
        Bx_0 = np.round(self.signedLSB(buffer_i_m1)/(214.6*S_HE),3)
        Bz_0 = np.round(self.signedLSB(buffer_j_m1)/(216.25*S_HE),3)
        Bx_1 = np.round(self.signedLSB(buffer_i_m2)/(214.6*S_HE),3)
        Bz_1 = np.round(self.signedLSB(buffer_j_m2)/(216.25*S_HE),3)
        int_temp = (int_temp-865)/8+35
        return [Bx_0,Bz_0,Bx_1,Bz_1,int_temp]
    
    def getValuesmTAv(self,av=1,NoErr = True,repMax=10):
        """Same as getValuesmT but repeats it "av" times and takes its average"""
        Vals = []
        for k in range(av):
            Vals.append(self.getValuesmT(NoErr = NoErr,repMax=repMax))
        Vals = np.mean(Vals,axis=0)
        return Vals
    
    def getValuesSingleChip(self,av=1,NoErr = True,repMax=10):
        """Merges the two dies information into one device (chip) based on the mapping hardcoded in self.mapChipPort
        Need to have only two chip selected"""
        if len(self.activeDUT)!=2:
            print("Not the right number of DUTs, readout probably wrong")
        ResRaw = self.getValuesmTAv(av=av,NoErr = NoErr,repMax=repMax)
        out = np.zeros(10)
        #Some signs needs to be adjusted
        out[0] =    ResRaw[2][1] #Bx00
        out[1] =    ResRaw[3][1] #Bz00
        out[2] = -1*ResRaw[0][1] #Bx01
        out[3] = -1*ResRaw[1][1] #Bz01
        out[4] = -1*ResRaw[2][0] #Bx10
        out[5] =    ResRaw[3][0] #Bz10
        out[6] =    ResRaw[0][0] #Bx11
        out[7] = -1*ResRaw[1][0] #Bz11
        out[8] =    ResRaw[4][1] #Temp Die 1
        out[9] =    ResRaw[4][0] #Temp Die 2
        return out
    
    def getValuesMultiChip(self,av=1,NoErr = True,repMax=10):
        """Merges the two dies information into one device (chip) based on the mapping hardcoded in self.mapChipPort
        Need to have only two chip selected"""
        """
        Only taking the 5 good maps
        """
        if len(self.activeDUT)%2!=0:
            print("Not the right number of DUTs, readout probably wrong")
        ResRaw = self.getValuesmTAv(av=av,NoErr = NoErr,repMax=repMax)
        nchips = len(self.activeDUT)//2
        out = np.zeros((nchips,10))
        #Some signs needs to be adjusted
        for n in range(nchips):
            out[n][0] =    ResRaw[2][n+nchips] #Bx00
            out[n][1] =    ResRaw[3][n+nchips] #Bz00
            out[n][2] = -1*ResRaw[0][n+nchips] #Bx01
            out[n][3] = -1*ResRaw[1][n+nchips] #Bz01
            out[n][4] = -1*ResRaw[2][n] #Bx10
            out[n][5] =    ResRaw[3][n] #Bz10
            out[n][6] =    ResRaw[0][n] #Bx11
            out[n][7] = -1*ResRaw[1][n] #Bz11
            out[n][8] =    ResRaw[4][n+nchips] #Temp Die 1
            out[n][9] =    ResRaw[4][n] #Temp Die 2
        return out

if __name__ == '__main__':                                                                                                                            
    with Tactaxis('COM56',activeDUT=[8,9]) as Sensor:
        cwd = os.getcwd()
        pathToPattern =cwd+'\\TactAxis_Seq_App.txt'
        Pat = Sensor.formatPattern(pathToPattern)
        dataReadout = Sensor.loadPattern(Pat,ack=False)  #set ack to True if there are some connectivity issue (might take more time)
        Sensor.flushInput()
        Sensor.flushOutput()
        Values = Sensor.getValuesSingleChip(av=1,NoErr = True,repMax=10)
        print(Values)