# Customer_Tactaxis
This is the software library to communicate with the MelexIO board in the context of Tactaxis sensor experiments.
Refer to the **Tactaxis user manual** for a more detailled explanation on the evaluation kit.
## Preliminary steps
### From the hardware side
In order to work with the sensor, you have to plug the PCB that has the sensor to the control board called MelexIO. To do so, you have flat cables (that can be bent) that directly connect them together (with a plastic white connector on each side).
The control board has to be connected to your computer with the USB-C cable. It is both powering the board and used for communication. The communication port related to this connector needs to be determined. On windows you can open the device managed tab and look at “Ports (COM & LPT)”. On linux you can use the command “dmesg | grep tty”
### From the software side
The library used to control the device needs to be downloaded. It is made of 5 python files that are self sufficient.
- USB_VCP.py: low-level class to deal with the communication between the computer and the board (inherited from serial.Serial).
- MelexIO_Base.py: mid-level class with generic functions to communicate with the board (inherited from USB_VCP.py).
- Tactaxis.py: High-level class (inherited from MelexIO_Base.py).
- forceComputation.py: Deals with the force inference and the training of a machine learning model.
- TactaxisForce.py: Class that is the final interface with the user.

In addition to these files, a calibration dataset is provided. It is a CSV file that contains both the magnetic data and its related force measured by an external force sensor.
Before running the first scripts, check that all libraries are installed correctly. You can run the script called **runMeFirst.py** that will check and install all libraries that can be used in the rest of the scripts. To make sure that the installation is successful, you can run the code **test_Libraries.py** in the Test_Files folder and check that there is no error.
A test file named test_communication.py in the Test_Files folder can be used to check if the connection with the sensor is established and that there is no known bug. It will also check for the external libraries dependencies and install them through pip in case it is required.


