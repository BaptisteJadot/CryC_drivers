# -*- coding: utf-8 -*-
"""
Created on Mon Aug 16 13:15:56 2021

@author: manip.batm
"""

#############
## SPI INIT
#############
SPI_content = {}
# registers
for i in range(63):
    SPI_content[i] = i
    
# counter
SPI_content[0x40] = 0x00
SPI_content[0x41] = 0x01

# SRAM
SPI_content[0x50] = 0x00
SPI_content[0x51] = 0x00
SPI_content[0x52] = 0x07

# suiveurs
SPI_content[0x70] = 0 | 0x80
SPI_content[0x71] = 1 | 0x80
SPI_content[0x72] = 2 | 0x80
SPI_content[0x73] = 3 | 0x80

# BIST clear
SPI_content[0x80] = 0x00

# mode
# mode = {'no_SPI':0,'counter':1<<4,'register':1<<4+1,'SRAM':1<<4+2}['counter']
SPI_content[0xFE] = 1<<7 | 1<<6 | 1<<5 | 0<<4 | 0<<3

# reset
SPI_content[0xFF] = 0x01

#############
## DC DACS
#############
DAC_dict = {}

DAC_dict['VE0N'] = {'panel':0,'channel':1,'lower_limit':0.,'upper_limit':1.8}
DAC_dict['VE1N'] = {'panel':0,'channel':3,'lower_limit':0.,'upper_limit':1.8}
DAC_dict['MODE_ROT'] = {'panel':0,'channel':4,'lower_limit':0.,'upper_limit':1.8}
DAC_dict['SEL_MAT'] = {'panel':0,'channel':5,'lower_limit':0.,'upper_limit':1.8}
DAC_dict['OSC_VCO'] = {'panel':0,'channel':6,'lower_limit':0.,'upper_limit':1.8}

DAC_dict['VBGPA'] = {'panel':3,'channel':0,'lower_limit':-3.,'upper_limit':0.}
DAC_dict['VBGNA'] = {'panel':3,'channel':1,'lower_limit':0.,'upper_limit':3.}
DAC_dict['VBGNC'] = {'panel':3,'channel':2,'lower_limit':0.,'upper_limit':3.}
DAC_dict['VBGPC'] = {'panel':3,'channel':3,'lower_limit':-3.,'upper_limit':0.}
DAC_dict['VBGNM'] = {'panel':3,'channel':4,'lower_limit':0.,'upper_limit':3.}
DAC_dict['VBGPM'] = {'panel':3,'channel':5,'lower_limit':-3.,'upper_limit':0.}

DAC_dict['CMD_R0'] = {'panel':2,'channel':0,'lower_limit':0.,'upper_limit':1.8}
DAC_dict['CMD_R1'] = {'panel':2,'channel':1,'lower_limit':0.,'upper_limit':1.8}
DAC_dict['CMD_R2'] = {'panel':2,'channel':2,'lower_limit':0.,'upper_limit':1.8}
DAC_dict['CMD_R3'] = {'panel':2,'channel':3,'lower_limit':0.,'upper_limit':1.8}
DAC_dict['CMD_R4'] = {'panel':2,'channel':4,'lower_limit':0.,'upper_limit':1.8}
DAC_dict['CMD_R5'] = {'panel':2,'channel':5,'lower_limit':0.,'upper_limit':1.8}
DAC_dict['CMD_R6'] = {'panel':2,'channel':6,'lower_limit':0.,'upper_limit':1.8}

