# -*- coding: utf-8 -*-
"""
Created on Thu Aug 12 09:44:21 2021

@author: manip.batm
"""

import numpy as np
from nifpga import Session, FpgaViState
from nifpga.status import RpcConnectionErrorError, FpgaBusyInteractiveError
from FPGA_utils import build_order,uint16_to_float,float_to_uint16
import time
        
bitfile_path = """C:/Users/manip.batm/Documents/FPGA_Batch_2_0_5_CryoC009/Labview_2019/FPGA Bitfiles/FPGA_CIR7_only_SPI.lvbitx"""
ip_address = "192.168.1.21"
FPGA = Session(bitfile=bitfile_path,resource='rio://'+ip_address+'/RIO0',no_run=True,reset_if_last_session_on_exit=False)
print(FPGA.fpga_vi_state)
# FPGA.run()
# # print(FPGA.fpga_vi_state)

# incoming orders FIFO
orders_fifo = FPGA.fifos['Incoming orders']
orders_fifo.configure(1050) # configure depth
orders_fifo.start()

# SPI data fifo
SPI_data = FPGA.fifos['SPI data']
SPI_data.configure(512) # configure depth
SPI_data.start()

orders_fifo.write([build_order([(2,8),(4,8),(0,8*2),(1000,32)])]) # set ticks
orders_fifo.write([build_order([(2,8),(1,8),(0,8*4),(0x70,8),(0x81,8)])]) # put one suiveur OFF
orders_fifo.write([build_order([(2,8),(1,8),(0,8*4),(0x71,8),(0x83,8)])]) # put one suiveur OFF
orders_fifo.write([build_order([(2,8),(1,8),(0,8*4),(0x72,8),(0x07,8)])]) # put one suiveur OFF
orders_fifo.write([build_order([(2,8),(1,8),(0,8*4),(0x73,8),(0x8F,8)])]) # put one suiveur OFF
orders_fifo.write([build_order([(2,8),(2,8),(0,8*3),(0x70,8),(1,16)])]) # write


orders_fifo.write([build_order([(2,8),(3,8),(0,8*3),(0,8),(4,16)])]) # read SPI
SPI_content = SPI_data.read(4,timeout_ms=2000)
print([(d>>8,d%256) for d in SPI_content.data])