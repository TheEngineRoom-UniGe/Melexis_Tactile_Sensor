# -*- coding: utf-8 -*-
"""
Created on Tue May 10 12:00:13 2022

@author: tls
"""

import sys
import subprocess

# implement pip as a subprocess:
print("Install pySerial")
subprocess.check_call([sys.executable, '-m', 'pip', 'install', 'pyserial'])
print("done")

print("Install path")
subprocess.check_call([sys.executable, '-m', 'pip', 'install', 'path'])
print("done")

print("Install Numpy")
subprocess.check_call([sys.executable, '-m', 'pip', 'install', 'numpy'])
print("done")

print("Install Pandas")
subprocess.check_call([sys.executable, '-m', 'pip', 'install', 'pandas'])
print("done")

print("Install scikit-learn")
subprocess.check_call([sys.executable, '-m', 'pip', 'install', 'scikit-learn'])
print("done")

print("All librairies successfully installed")
