# -*- coding: utf-8 -*-
"""
Created on Mon Aug 16 13:15:56 2021

@author: manip.batm
"""

#############
## DC DACS
#############
DAC_dict = {}

# DAC_dict['VE0N'] = {'panel':0, 'channel':1, 'lower_limit':0., 'upper_limit':1.8}
# DAC_dict['VE1N'] = {'panel':0, 'channel':3, 'lower_limit':0., 'upper_limit':1.8}
DAC_dict['MODE_ROT'] = {'panel':0, 'channel':4, 'lower_limit':0., 'upper_limit':1.8}
DAC_dict['SEL_MAT'] = {'panel':0, 'channel':5, 'lower_limit':0., 'upper_limit':1.8}
DAC_dict['OSC_VCO'] = {'panel':0, 'channel':6, 'lower_limit':0., 'upper_limit':1.8}
DAC_dict['RESETN'] = {'panel':0, 'channel':7, 'lower_limit':0., 'upper_limit':1.8}

DAC_dict['VBGPA'] = {'panel':3, 'channel':0, 'lower_limit':-3., 'upper_limit':0.}
DAC_dict['VBGNA'] = {'panel':3, 'channel':1, 'lower_limit':0., 'upper_limit':3.}
DAC_dict['VBGNC'] = {'panel':3, 'channel':2, 'lower_limit':-1., 'upper_limit':3.}
DAC_dict['VBGPC'] = {'panel':3, 'channel':3, 'lower_limit':-3., 'upper_limit':1.}
DAC_dict['VBGNM'] = {'panel':3, 'channel':4, 'lower_limit':0., 'upper_limit':3.}
DAC_dict['VBGPM'] = {'panel':3, 'channel':5, 'lower_limit':-3., 'upper_limit':0.}

DAC_dict['CMD_R0'] = {'panel':2, 'channel':0, 'lower_limit':0., 'upper_limit':1.8}
DAC_dict['CMD_R1'] = {'panel':2, 'channel':1, 'lower_limit':0., 'upper_limit':1.8}
DAC_dict['CMD_R2'] = {'panel':2, 'channel':2, 'lower_limit':0., 'upper_limit':1.8}
DAC_dict['CMD_R3'] = {'panel':2, 'channel':3, 'lower_limit':0., 'upper_limit':1.8}
DAC_dict['CMD_R4'] = {'panel':2, 'channel':4, 'lower_limit':0., 'upper_limit':1.8}
DAC_dict['CMD_R5'] = {'panel':2, 'channel':5, 'lower_limit':0., 'upper_limit':1.8}
DAC_dict['CMD_R6'] = {'panel':2, 'channel':6, 'lower_limit':0., 'upper_limit':1.8}

DAC_dict['ADDR_0'] = {'panel':5, 'channel':0, 'lower_limit':0., 'upper_limit':1.8}
DAC_dict['ADDR_1'] = {'panel':5, 'channel':1, 'lower_limit':0., 'upper_limit':1.8}
DAC_dict['ADDR_2'] = {'panel':5, 'channel':2, 'lower_limit':0., 'upper_limit':1.8}
DAC_dict['ADDR_4'] = {'panel':5, 'channel':4, 'lower_limit':0., 'upper_limit':1.8}
DAC_dict['ADDR_5'] = {'panel':5, 'channel':5, 'lower_limit':0., 'upper_limit':1.8}

DAC_dict['Veven_DC'] = {'panel':5, 'channel':6, 'lower_limit':0., 'upper_limit':1.8}
DAC_dict['Vodd_DC'] = {'panel':5, 'channel':7, 'lower_limit':0., 'upper_limit':1.8}
DAC_dict['CLK_DC'] = {'panel':5, 'channel':3, 'lower_limit':0., 'upper_limit':1.8}


DAC_dict['dummy'] = {'panel':1, 'channel':2, 'lower_limit':0., 'upper_limit':1.8}

