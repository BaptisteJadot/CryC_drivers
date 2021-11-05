# -*- coding: utf-8 -*-
"""
Created on Fri Nov  5 14:34:41 2021

@author: manip.batm
"""
import pyvisa
import time

AWG_VISA = "TCPIP::192.168.1.12::4000::SOCKET" 
rm = pyvisa.ResourceManager()
awg = rm.open_resource(AWG_VISA, 
            write_termination="\n",
            read_termination="\n")

print(awg.write("OUTPUT1:STATE ON"))
time.sleep(2)
print(awg.write("OUTPUT1:STATE OFF"))

print(awg.query("*IDN?"))

awg.close()