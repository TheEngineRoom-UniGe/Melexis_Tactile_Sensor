# -*- coding: utf-8 -*-
"""
Created on Wed Mar 30 11:40:53 2022

@author: tls
"""

import USB_VCP
import numpy as np
import time
import os

class MelexIO(USB_VCP.USB_VCP): 
    """
    Class that inherits from USB_VCP class, mid-level
    Encapsulates the commands into explicit ones
    """
    
    def __init__(self,portName,activeDUT = [8,9]):
        super(MelexIO,self).__init__(portName)
        self.activeDUT = activeDUT
        self.setActiveDUT()
        
    def Get_IDN(self):
        """Method to get inforamtion about the ID of the MelexIO board"""
        self.sendCMD("CMD_IDN")
        Err,str_idn,dataOut,timeoutRet= self.readDataTimeOut()
        if Err == self.ErrorMessages_e["ERR_NOERR"]:    
            self.infoMessage(str_idn, "data IDN")
        else :
            print("Error found:")
            self.ErrorMessage_Handler(ErrorMsg = Err)
        return str_idn
    

    def Get_REVID(self):
        self.sendCMD("CMD_GET_REVID")
        Err,str_idn,dataOut,timeoutRet= self.readDataTimeOut()
        if Err == self.ErrorMessages_e["ERR_NOERR"]:    
            self.infoMessage(str_idn, "data Rev ID")
        else :
            print("Error found:")
            self.ErrorMessage_Handler(ErrorMsg = Err)
        return str_idn
    

    def Get_DEVID(self):
        self.sendCMD("CMD_GET_DEVID");
        Err,str_idn,dataOut,timeoutRet=self.readDataTimeOut()
        if Err == self.ErrorMessages_e["ERR_NOERR"]:    
            self.infoMessage(str_idn, "data Dev ID")
        else :
            print("Error found:")
            self.ErrorMessage_Handler(ErrorMsg = Err)
        return str_idn


    def Get_CPUID(self):
        """Method to get inforamtion about the ID of the CPU"""
        self.sendCMD("CMD_GET_MCU_UIDw")
        Err,str_idn,dataOut,timeoutRet=self.readDataTimeOut()
        out = ""
        for  i,d in enumerate(dataOut):
            out+=str(hex(d))
        if Err == self.ErrorMessages_e["ERR_NOERR"]:    
            self.infoMessage(out, "Device ID")
        else :
            print("Error found:")
            self.ErrorMessage_Handler(ErrorMsg = Err)
        return out
    

    def Get_FWID(self):
        self.sendCMD("CMD_GET_FWVER")
        Err,str_idn,dataOut,timeoutRet = self.readDataTimeOut()
        if Err == self.ErrorMessages_e["ERR_NOERR"]:    
            self.infoMessage(str_idn, "data Firmware ID")
        else :
            print("Error found:")
            self.ErrorMessage_Handler(ErrorMsg = Err)
        return str_idn
    
    def Get_HWID(self):
        self.sendCMD("CMD_GET_HWVER")
        Err,str_idn,dataOut,timeoutRet = self.readDataTimeOut()
        if Err == self.ErrorMessages_e["ERR_NOERR"]:    
            self.infoMessage(str_idn, "data Hardware ID")
        else :
            print("Error found:")
            self.ErrorMessage_Handler(ErrorMsg = Err)
        return str_idn
    
    def  setSilentMode(self):
        self.silentMode = True

    def  resetSilentMode(self):
        self.silentMode = False

    def  setTimeout(self,newTimeOut):
        self.timeout = newTimeOut
        
    #For the next commands, ack means that the method is waiting for the answer from the controller and checks that it is correct

    def loadConstVector(self,ack=True):
        writeOrder = self.sendCMD("CMD_LOAD_CONST_PATT")
        if ack :
            Err,str_idn,dataOut,timeoutRet = self.readDataTimeOut()
            if Err != self.ErrorMessages_e["ERR_NOERR"]:    
                print("Error found:")
                self.ErrorMessage_Handler(ErrorMsg = Err)
            return str_idn
        return writeOrder
    
    def loadConstVector23(self,ack=True):
        writeOrder = self.sendCMD("CMD_LOAD_VECTOR_23")
        if ack :
            Err,str_idn,dataOut,timeoutRet = self.readDataTimeOut()
            if Err != self.ErrorMessages_e["ERR_NOERR"]:    
                print("Error found:")
                self.ErrorMessage_Handler(ErrorMsg = Err)
            return str_idn
        return writeOrder
    
    def  runVector(self,ack=True):
        writeOrder = self.sendCMD("CMD_RUN_MUPET_PATT")
        if ack :
            Err,str_idn,dataOut,timeoutRet = self.readDataTimeOut()
            if Err != self.ErrorMessages_e["ERR_NOERR"]:    
                print("Error found:")
                self.ErrorMessage_Handler(ErrorMsg = Err)
            return str_idn
        return writeOrder

    def  stopVector(self,ack=True):
        writeOrder = self.sendCMD("CMD_STOP_MUPET_PATT")
        if ack :
            Err,str_idn,dataOut,timeoutRet = self.readDataTimeOut()
            if Err != self.ErrorMessages_e["ERR_NOERR"]:    
                print("Error found:")
                self.ErrorMessage_Handler(ErrorMsg = Err)
            return str_idn
        return writeOrder

    def  enableSupply(self,Supply,ack=True):
        writeOrder = self.sendParameter("CMD_ENABLE_SUPPLY", Supply)
        if ack :
            Err,str_idn,dataOut,timeoutRet = self.readDataTimeOut()
            if Err != self.ErrorMessages_e["ERR_NOERR"]:    
                print("Error found:")
                self.ErrorMessage_Handler(ErrorMsg = Err)
            return str_idn
        return writeOrder

    def  disableSupply(self,Supply,ack=True):
        writeOrder = self.sendParameter("CMD_DISBLE_SUPPLY", Supply)
        if ack :
            Err,str_idn,dataOut,timeoutRet = self.readDataTimeOut()
            if Err != self.ErrorMessages_e["ERR_NOERR"]:    
                print("Error found:")
                self.ErrorMessage_Handler(ErrorMsg = Err)
            return str_idn
        return writeOrder

    def  setDelaySupplyOff(self,Delay,ack=True):
        writeOrder = self.sendParameter("CMD_SET_MUPET_DELSOF", Delay)
        if ack :
            Err,str_idn,dataOut,timeoutRet = self.readDataTimeOut()
            if Err != self.ErrorMessages_e["ERR_NOERR"]:    
                print("Error found:")
                self.ErrorMessage_Handler(ErrorMsg = Err)
            return str_idn
        return writeOrder

    def  setDelaySupplyOn(self,Delay,ack=True):
        writeOrder = self.sendParameter("CMD_SET_MUPET_DELSON", Delay)
        if ack :
            Err,str_idn,dataOut,timeoutRet = self.readDataTimeOut()
            if Err != self.ErrorMessages_e["ERR_NOERR"]:    
                print("Error found:")
                self.ErrorMessage_Handler(ErrorMsg = Err)
            return str_idn
        return writeOrder

    def  setTckFrequency(self,Freq,ack=True):
        writeOrder = self.sendParameter("CMD_SET_MUPET_TCKF", Freq)
        if ack :
            Err,str_idn,dataOut,timeoutRet = self.readDataTimeOut()
            if Err != self.ErrorMessages_e["ERR_NOERR"]:    
                print("Error found:")
                self.ErrorMessage_Handler(ErrorMsg = Err)
            return str_idn
        return writeOrder

    def  setVoltageReset(self,Voltage,ack=True):
        writeOrder = self.sendParameter("CMD_SET_MUPET_VRST", Voltage)
        if ack :
            Err,str_idn,dataOut,timeoutRet = self.readDataTimeOut()
            if Err != self.ErrorMessages_e["ERR_NOERR"]:    
                print("Error found:")
                self.ErrorMessage_Handler(ErrorMsg = Err)
            return str_idn
        return writeOrder

    def  setVoltageApp(self,Voltage,ack=True):
        writeOrder = self.sendParameter("CMD_SET_MUPET_VNOM", Voltage)
        if ack :
            Err,str_idn,dataOut,timeoutRet = self.readDataTimeOut()
            if Err != self.ErrorMessages_e["ERR_NOERR"]:    
                print("Error found:")
                self.ErrorMessage_Handler(ErrorMsg = Err)
            return str_idn
        return writeOrder

    def  setVoltageEnterTM(self,Voltage,ack=True):
        writeOrder = self.sendParameter("CMD_SET_MUPET_VETM", Voltage)
        if ack :
            Err,str_idn,dataOut,timeoutRet = self.readDataTimeOut()
            if Err != self.ErrorMessages_e["ERR_NOERR"]:    
                print("Error found:")
                self.ErrorMessage_Handler(ErrorMsg = Err)
            return str_idn
        return writeOrder

    def  setDeviceEnable(self,Devices,ack=True):
        writeOrder = self.sendParameter("CMD_SET_MUPET_DUTEN", Devices)
        if ack :
            Err,str_idn,dataOut,timeoutRet = self.readDataTimeOut()
            if Err != self.ErrorMessages_e["ERR_NOERR"]:    
                print("Error found:")
                self.ErrorMessage_Handler(ErrorMsg = Err)
            return str_idn
        return writeOrder

    def  setNumberTDOBytes(self,TDOBytes,ack=True):
        writeOrder = self.sendParameter("CMD_SET_MUPET_TDOB", TDOBytes)
        if ack :
            Err,str_idn,dataOut,timeoutRet = self.readDataTimeOut()
            if Err != self.ErrorMessages_e["ERR_NOERR"]:    
                print("Error found:")
                self.ErrorMessage_Handler(ErrorMsg = Err)
            return str_idn
        return writeOrder

    def  setACQType(self,ACQType,ack=True):
        writeOrder = self.sendParameter("CMD_SET_MUPET_ACQTY", ACQType)
        if ack :
            Err,str_idn,dataOut,timeoutRet = self.readDataTimeOut()
            if Err != self.ErrorMessages_e["ERR_NOERR"]:    
                print("Error found:")
                self.ErrorMessage_Handler(ErrorMsg = Err)
            return str_idn
        return writeOrder
    
    def  resetDevice(self,ack=True):
        writeOrder = self.sendCMD("CMD_RESET_DEVICE")
        if ack :
            Err,str_idn,dataOut,timeoutRet = self.readDataTimeOut()
            if Err != self.ErrorMessages_e["ERR_NOERR"]:    
                print("Error found:")
                self.ErrorMessage_Handler(ErrorMsg = Err)
            return str_idn
        return writeOrder

    def  setExeLed(self,Led,ack=True):
        writeOrder = self.sendParameter("CMD_CTRL_LED_EXEC", Led)
        if ack :
            Err,str_idn,dataOut,timeoutRet = self.readDataTimeOut()
            if Err != self.ErrorMessages_e["ERR_NOERR"]:    
                print("Error found:")
                self.ErrorMessage_Handler(ErrorMsg = Err)
            return str_idn
        return writeOrder

    def  setDrdLed(self,Led,ack=True):
        writeOrder = self.sendParameter("CMD_CTRL_LED_DRD", Led)
        if ack :
            Err,str_idn,dataOut,timeoutRet = self.readDataTimeOut()
            if Err != self.ErrorMessages_e["ERR_NOERR"]:    
                print("Error found:")
                self.ErrorMessage_Handler(ErrorMsg = Err)
            return str_idn
        return writeOrder

    def  setErrorState( self,Led,ack=True):
        writeOrder = self.sendParameter("CMD_RESET_ERR_STATE", Led)
        if ack :
            Err,str_idn,dataOut,timeoutRet = self.readDataTimeOut()
            if Err != self.ErrorMessages_e["ERR_NOERR"]:    
                print("Error found:")
                self.ErrorMessage_Handler(ErrorMsg = Err)
            return str_idn
        return writeOrder

    def  setMode(self,Mode,ack=True):
        writeOrder = self.sendParameter("CMD_SET_MODE",Mode)
        if ack :
            Err,str_idn,dataOut,timeoutRet = self.readDataTimeOut()
            if Err != self.ErrorMessages_e["ERR_NOERR"]:    
                print("Error found:")
                self.ErrorMessage_Handler(ErrorMsg = Err)
            return str_idn
        return writeOrder

    def  setExtMupetDac(self,Data,ack=True):
        writeOrder = self.sendParameter("CMD_SET_DAC_MUPET", Data)
        if ack :
            Err,str_idn,dataOut,timeoutRet = self.readDataTimeOut()
            if Err != self.ErrorMessages_e["ERR_NOERR"]:    
                print("Error found:")
                self.ErrorMessage_Handler(ErrorMsg = Err)
            return str_idn
        return writeOrder
    
    def setS2MMode(self,Data,ack=True):
        writeOrder = self.sendParameter("CMD_SET_S2M_MODE", Data)
        if ack :
            Err,str_idn,dataOut,timeoutRet = self.readDataTimeOut()
            if Err != self.ErrorMessages_e["ERR_NOERR"]:    
                print("Error found:")
                self.ErrorMessage_Handler(ErrorMsg = Err)
            return str_idn
        return writeOrder

    def  runPattern1(self,tdoNumber,sizeExpected=256):
        """
        Run pattern stored in buffer 1 and readout the value
        tdoNumber is the size of the output for one active device
        szeExpected is the expected size of the response from the device
        """
        Err_Status = self.sendParameter("CMD_RUN_PATT_1", tdoNumber)
        if (Err_Status != self.ErrorMessages_e['ERR_NOERR']):
            print("Error found:")
            self.ErrorMessage_Handler(Err_Status)
            return Err_Status,0,0,0
        Err_Status,dataOut,ArrBytes,timeoutRet = self.readDataTimeOut(nofBytes=sizeExpected)
        return Err_Status,dataOut,ArrBytes,timeoutRet
    
    def  runPattern2(self,tdoNumber,sizeExpected=256):
        """
        Run pattern stored in buffer 2 and readout the value
        tdoNumber is the size of the output for one active device
        szeExpected is the expected size of the response from the device
        """
        Err_Status = self.sendParameter("CMD_RUN_PATT_2", tdoNumber)
        if (Err_Status != self.ErrorMessages_e['ERR_NOERR']):
            print("Error found:")
            self.ErrorMessage_Handler(Err_Status)
            return Err_Status,0,0,0
        Err_Status,dataOut,ArrBytes,timeoutRet = self.readDataTimeOut(nofBytes=sizeExpected)
        return Err_Status,dataOut,ArrBytes,timeoutRet
    
    
    def setActiveDUT(self,ack=True):
        data16Bit = 0
        for dutNumb in self.activeDUT:
            data16Bit+=1*np.power(2,dutNumb)
        Arr = self.createArrayFromCmd("CMD_SET_ACTIVE_DUT",data=data16Bit)    #Creates the array of bytes
        dataOut = self.formatSendFromArray(Arr)         #Formats the array
        writeOrder = (self.write(dataOut))
        if ack :
            Err,str_idn,dataOut,timeoutRet = self.readDataTimeOut()
            if Err != self.ErrorMessages_e["ERR_NOERR"]:    
                print("Error found:")
                self.ErrorMessage_Handler(ErrorMsg = Err)
            return str_idn
        return writeOrder

    def changeActiveDUT(self,newDUTs,ack=True):
        List = np.sort(newDUTs)
        self.activeDUT = List
        return self.setActiveDUT(ack=ack)
    


if __name__ == '__main__':
    frequencyTck    = 40
    voltageAPPDac   = 4000
    voltageETMDac   = 4000
    voltageRSTDac   = 0
    delayTon        = 0
    delayToff       = 0
    BytesToReadout  = 6
    S2M_MODE        = 0x03
    with MelexIO('COM7') as Board: 
        time.sleep(1)
        print(Board.Get_IDN())
        #Init of the board
        dataReadout = Board.setTckFrequency(frequencyTck)
        dataReadout = Board.setVoltageEnterTM(voltageETMDac)
        dataReadout = Board.setVoltageReset(voltageRSTDac)
        dataReadout = Board.setVoltageApp(voltageAPPDac)
        dataReadout = Board.setDelaySupplyOn(delayTon)
        dataReadout = Board.setDelaySupplyOff(delayToff)
        dataReadout = Board.setNumberTDOBytes(BytesToReadout)
        dataReadout = Board.setS2MMode(S2M_MODE)
        dataReadout = Board.setActiveDUT()
        cwd = os.getcwd()
        pathToPattern =cwd+'\\TactAxis_Seq_App.txt'
        Pat = Board.formatPattern(pathToPattern)
        dataReadout = Board.loadPattern(Pat,ack=False)
        Board.flushInput()
        Board.flushOutput()
        time.sleep(1)
        out = Board.runPattern1(BytesToReadout,2*S2M_MODE*BytesToReadout)
        print(out[2])
        print(Board.Get_IDN())
        Board.close()
