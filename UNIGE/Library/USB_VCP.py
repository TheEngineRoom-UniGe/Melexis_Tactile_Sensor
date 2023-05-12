# -*- coding: utf-8 -*-
"""
Created on Tue Mar 29 08:33:02 2022

@author: tls
"""

import serial
import numpy as np
import time
import sys

                
class USB_VCP(serial.Serial):
    """
    Class that inherits from Serial class (pyserial), low-level
    Main methods are sendCMD that sends a specific command to the microcontroler and readDataTimeOut that reads the data received.
    """
    def __init__(self,portName):
        #init of the super class, if it doesn't work, stop the script.
        try:    
            super(USB_VCP,self).__init__(port=portName)
        except serial.serialutil.SerialException as ex:
            print("Error while connecting to the serial port:")
            print(ex.args)
            print("If access denied, try reseting the console. Otherwise, make sure the port is correct")
            print("Terminating the script")
            sys.exit()
        
        #parameters of the serial communication
        self.is_open = True
        self.baudrate = 115200
        self.BytesSize = 4
        self.parity = serial.PARITY_NONE
        self.stopbits = serial.STOPBITS_ONE
        self.timeout = 1
        self.write_timeout = 1
        self.dsrdtr = True
        self.silentMode = False
        self.bufferRX_size = 64
        
        
        self.FRAME_COMPLETED    = 0xAA
        self.ADDRESS_CMD        = 0x00
        self.ADDRESS_DAT        = 0x01
        self.ADDRESS_EOF        = 0x07
        
        #Command dictionnary to communicate with the MCU
        self.CommandsSet_e = {
            "CMD_IDN"                     : 0x00, # Get ID of the MelexIO`
            "CMD_GET_REVID"               : 0x01,
            "CMD_GET_DEVID"               : 0x02,
            "CMD_GET_MCU_UIDw"            : 0x03, # Get CPU ID
            "CMD_GET_FWVER"               : 0x04, # Firmware Version
            "CMD_ENABLE_SUPPLY"           : 0x05,
            "CMD_DISBLE_SUPPLY"           : 0x06,
            "CMD_CTRL_LED_EXEC"           : 0x07, # Control the LED EXEC
            "CMD_CTRL_LED_DRD"            : 0x08, # Control the LED DRD
            "CMD_RESET_ERR_STATE"         : 0x09, # command to reset error state
            "CMD_DBG_COMMNAD1"            : 0x0A,
            "CMD_SET_MODE"                : 0x0B,
            "CMD_GET_HWVER"               : 0x0C, # Get HW Version
            "CMD_SET_CONECTFLAG"          : 0x0D,
            "CMD_RESET_DEVICE"            : 0x0F,

            #MUPET related commands
            "CMD_SET_MUPET_CONFIG"        : 0x10,
            "CMD_LOAD_MUPET_PATT"         : 0x11, # Load single int to the pattern
            "CMD_RUN_MUPET_PATT"          : 0x12,
            "CMD_LOAD_CONST_PATT"         : 0x13, # Use hardcoded pattern 1
            "CMD_RUN_PATT_1"              : 0x14, # Run pattern stored in buffer 1
            "CMD_RUN_PATT_2"              : 0x15, # Run pattern stored in buffer 2
            "CMD_LOAD_VECTOR_23"          : 0x16, # Use hardcoded pattern 2
            "CMD_RES_5"                   : 0x17,
            "CMD_RES_6"                   : 0x18,
            "CMD_SET_S2M_MODE"            : 0x19, # Change the active devices (filter of the 16 possible dies connected)
            "CMD_RES_8"                   : 0x1A,
            "CMD_RES_9"                   : 0x1B,
            "CMD_RES_10"                  : 0x1C,
            "CMD_RES_11"                  : 0x1D,
            "CMD_RES_12"                  : 0x1E,
            "CMD_STOP_MUPET_PATT"         : 0x1F,
            
            #Command space to readout data
            "CMD_GET_DUT_DATA"            : 0x20,
            "CMD_GET_DUT_ICC"             : 0x21,
            "CMD_GET_DUT_VCC"             : 0x22,
            "CMD_GET_HV_VCC"              : 0x23,
            "CMD_RES_13"                  : 0x24,
            "CMD_RES_14"                  : 0x25,
            "CMD_RES_15"                  : 0x26,
            "CMD_RES_16"                  : 0x27,
            "CMD_RES_17"                  : 0x28,
            "CMD_RES_18"                  : 0x29,
            "CMD_RES_19"                  : 0x2A,
            "CMD_RES_20"                  : 0x2B,
            "CMD_RES_21"                  : 0x2C,
            "CMD_RES_22"                  : 0x2D,
            "CMD_RES_23"                  : 0x2E,
            "CMD_RES_24"                  : 0x2F,
            
            #Command space to control MUPET parameters
            "CMD_SET_ACTIVE_DUT"          : 0x30,   # New command
            "CMD_RES_26"                  : 0x31,
            "CMD_SET_DAC_MUPET"           : 0x32,   # command to set MUPET level voltage
            "CMD_SET_DAC_EXT"             : 0x33,   # command to set external DAC voltage
            "CMD_RES_27"                  : 0x34,
            "CMD_SET_MUPET_DELSON"        : 0x35,   # command to set delay between enabling supply an first communication clock edge
            "CMD_SET_MUPET_DELSOF"        : 0x36,   # command to set delay between last communication clock edge and turning off supply
            "CMD_SET_MUPET_TCKF"          : 0x37,   # command to set frequency for the test mode communication
            "CMD_SET_MUPET_VRST"          : 0x38,   # command to set Voltage level for reset device
            "CMD_SET_MUPET_VNOM"          : 0x39,   # command to set Voltage level for application mode
            "CMD_SET_MUPET_VETM"          : 0x3A,   # command to set Voltage level for entering in test mode
            "CMD_SET_MUPET_DUTEN"         : 0x3B,   # Command to enable devices on the line
            "CMD_SET_MUPET_TDOB"          : 0x3C,   # Command to define number of expected bytes
            "CMD_SET_MUPET_ACQTY"         : 0x3D,   # Command to set acquisition type
            "CMD_RES_28"                  : 0x3E,
            "CMD_RES_29"                  : 0x3F,

            "CMD_RESERVED"                : 0xFF	# Reserved command
            }
        

        
        #Error dictionnary, signal the MCU can send
        self.ErrorMessages_e ={
            "ERR_NOERR"           :   0x00,
            "ERR_PORT_OPEN"       :   0x01,
            "ERR_PORT_CLOSE"      :   0x02,
            "ERR_DATA_NA"         :   0x03,
            "ERR_FILE_FORMAT"     :   0x04,
            "ERR_FILE_NF"         :   0x05,
            "ERR_FILE_BREAK"      :   0x06,
            "ERR_ACK_TIMEOUT"     :   0x07,
            "ERR_HW_COMM"         :   0x08
            }
        
      
        #error managment
        self.MSG_ERROR_CANTCONHW  = "Can't connect to the hardware";
        self.MSG_ERROR_PORTCLOSE  = "Port is closed. Please intiate class with parameter <PORT_NAME> and call Open() functions";
        self.MSG_ERROR_DATANA     = "Data is not avaliable";
        self.MSG_ERROR_FILENF     = "File not found";
        self.MSG_ERROR_FILEFORM   = "File has wrong format";
        self.MSG_ERROR_FILEBRK    = "File is broken. There is no EOF identificator";
        self.MSG_ERROR_TIMEOUT    = "Hardware reply timeout";
        self.MSG_ERROR_HWERROR    = "Unknown hardware error occured";
        self.MSG_ERROR_UNKNCMD    = "Hardware Error. Unknown command was sent";
        self.MSG_ERROR_UNKNRSP    = "Hardware Error. Unknown respond was received";
        self.MSG_ERROR_HWOVERFLOW = "Hardware Error. Internal memory is overflow";
        self.MSG_ERROR_UNKNOWNER  = "Unknown error";
        self.MSG_NOERROR          = "";

        
        self.CPT_ERROR_CONISSUE = "Connection issue";
        self.CPT_ERROR = "ERROR";
        self.CPT_HWERROR = "Hardware ERROR";
        self.CPT_ERROR_FILE = "File error";
        self.CPT_TIMEOUT = "Timeout";
        self.CPT_INFO = "Information";
        self.ErrorDict= {
                self.ErrorMessages_e['ERR_NOERR']       :       self.MSG_NOERROR+self.CPT_ERROR ,         
                self.ErrorMessages_e['ERR_DATA_NA']     :       self.MSG_ERROR_DATANA+self.CPT_ERROR  ,     
                self.ErrorMessages_e['ERR_PORT_CLOSE']  :       self.MSG_ERROR_PORTCLOSE+self.CPT_ERROR,    
                self.ErrorMessages_e['ERR_PORT_OPEN']   :       self.MSG_ERROR_CANTCONHW+self.CPT_ERROR ,  
                self.ErrorMessages_e['ERR_FILE_BREAK']  :       self.MSG_ERROR_FILEBRK+self.CPT_ERROR,     
                self.ErrorMessages_e['ERR_FILE_FORMAT'] :       self.MSG_ERROR_FILEFORM+self.CPT_ERROR ,  
                self.ErrorMessages_e['ERR_FILE_NF']     :       self.MSG_ERROR_FILENF+self.CPT_ERROR ,     
                self.ErrorMessages_e['ERR_ACK_TIMEOUT'] :       self.MSG_ERROR_TIMEOUT+self.CPT_HWERROR,
                self.ErrorMessages_e['ERR_HW_COMM']     :       self.MSG_ERROR_HWERROR +  self.CPT_HWERROR                         
            }
        
        
    def ErrorMessage_Handler(self,ErrorMsg):
        print(self.ErrorDict.get(ErrorMsg, self.MSG_ERROR_UNKNOWNER+self.CPT_ERROR)) 
        return ErrorMsg

    def infoMessage(self,info,label):
        if info!="":
            if not self.silentMode:
                print(label+": "+str(info))
    
    def sendCMD(self,cmd,data=None):
        """Sends a command to the USB port
        cmd is a command name from the CommandSet_e dictionnary
        data is a 16bits number that is used to communicate parameter with the command
        """
        Arr = self.createArrayFromCmd(cmd,data=data)    #Creates the array of bytes
        dataOut = self.formatSendFromArray(Arr)         #Formats the array
        return(self.write(dataOut))
    
    def sendParameter(self,cmd,data):
        """Derivates from sendCMD to explicitly ask for a parameter"""
        self.sendCMD(cmd,data=data)
        return self.ErrorMessages_e['ERR_NOERR']
    
    def createArrayFromCmd(self,cmd,data=None):
        """Creates an array of 8 bytes (still kept as integers) that will be used to send it to the MelexIO
        cmd is a command name from the CommandSet_e dictionnary
        data is a 16bits number that is used to communicate parameter with the command
        """
        Arr = np.zeros(8,dtype=int)
        Arr[self.ADDRESS_EOF] = self.FRAME_COMPLETED        #Last byte is a 0xAA
        Arr[self.ADDRESS_CMD] = self.CommandsSet_e[cmd]     #First byte is the command number
        if data !=None :                                    #need to fill the Array with the two slots for the data
            MSB = data//256                                 #separate the data into two bytes
            LSB = data%256
            Arr[self.ADDRESS_DAT] = LSB
            Arr[self.ADDRESS_DAT+1] = MSB
        return Arr
    
    def formatSendFromArray(self,Arr):
        """Formats the array of 64 bytes to be in the shape of a single byte string \\xXX\\xYY\\xZZ..."""
        stringOut = ""
        for i in Arr :
            number = str(hex(i))
            if len(number)==3 :             # i<16, missing a 0 after "0x"
                number = "0x0"+number[-1]
            number = number[2:]             #Getting only last two hex digits to send to the COM port
            stringOut+=number               #Concatenate the string
        dataOut = bytes.fromhex(stringOut)
        return dataOut
      
    def readDataTimeOut(self,timeout=1,nofBytes=None):
        """
        Method that reads the serial port and returns it
        
        Parameters:
            -timeout is the maximum time it waits to return something. If time exceeds this argument, and nothing was detected on the port, there is a timeout message
            -nofBytes is the number of bytes we expect to see. The program will keep looking untill it founds the enough number of bytes in the buffer. If None, there is no constraints. 
        
        Out:
            -Error status
            -Raw string read at the input buffer
            -Array of bytes (made from the raw strung input) that is more pleasant to use
            -Time taken to read
            
        """
        self.timeout = timeout #updates the timeout parameter of the Serial class
        Err_Status = self.ErrorMessages_e["ERR_NOERR"]
        t_init = time.time()
        while(True):
            t_current = time.time()
            if(self.in_waiting > 0):    #If the reading buffer is not empty
                a = self.in_waiting
                if not(nofBytes!=None and nofBytes>a):                  #Reading the buffer
                    serialString = self.read(size = a)
                    ArrBytes = []
                    for k in serialString:
                        ArrBytes.append(k)
                    return Err_Status,serialString,ArrBytes,t_current-t_init
            if t_current-t_init >timeout:
                print("Timeout")
                Err_Status = self.ErrorMessages_e["ERR_ACK_TIMEOUT"]
                return Err_Status,0,0,0
            time.sleep(0.000001)
    
    def formatPattern(self,pathToPattern):
        file = open(pathToPattern)
        for i,k in enumerate(file):
            if i==0: #check for the first line only
                Out = k.split(",")
                Out = np.array(Out,dtype=int)
        file.close()
        return Out
    
    def loadPattern(self,pattern,patternBuffer=1,ack=True):
        """
        Pattern is an array of elements in {0,1,2,3,4,5} resulting of the pattern computed to communicate in MUPET with the DUT
        This functions loads this pattern in the Melexio (should be called at the init)
        """
        
        n = len(pattern)
        for k in range(n):
            MSB = k//256                                 #separate the data into two bytes
            LSB = k%256
            Arr = np.zeros(8,dtype=int)
            Arr[self.ADDRESS_EOF] = self.FRAME_COMPLETED        #Last byte is a 0xAA
            Arr[self.ADDRESS_CMD] = self.CommandsSet_e["CMD_LOAD_MUPET_PATT"]     #First byte is the command number
                            
            Arr[1] = patternBuffer
            Arr[3] = LSB
            Arr[4] = MSB
            Arr[5] = pattern[k]
            dataOut = self.formatSendFromArray(Arr)         #Formats the array
            writeOrder = (self.write(dataOut)) #do something in case is not correct
            #print("send")
            if ack :
                Err,str_idn,dataOut,timeoutRet = self.readDataTimeOut()
                if Err != self.ErrorMessages_e["ERR_NOERR"]:    
                    print("Error found:")
                    self.ErrorMessage_Handler(ErrorMsg = Err)
                
if __name__ == '__main__':
    with USB_VCP('COM7') as Conn: 
        time.sleep(1)
        Conn.sendCMD("CMD_IDN")
        B = Conn.readDataTimeOut()
        Conn.sendCMD("CMD_IDN")
        B = Conn.readDataTimeOut()
        print(B[1])
        Conn.close()

