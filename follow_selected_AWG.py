# -*- coding: utf-8 -*-
"""
Created on Tue Nov  9 14:34:06 2021

@author: manip.batm
"""

from CIR7_driver import CIR7Driver
from CIR7_config import DAC_dict
from AWG_driver import AWGDriver, build_waveforms, AWG_SAMPLING_RATE_MHz
from Scope_driver import ScopeDriver
import Fastseq_elmts as fs
import numpy as np
import time

def test_clk(awg, clk_rate_MHz, Nclk, amplitude=0.9):
    ## READ INDEX
    i0 = instr.SPI_read(0xA1, 1)[0]
    time.sleep(0.1)

    ## SEND 5 CLOCKS    
    L0 = int(AWG_SAMPLING_RATE_MHz / clk_rate_MHz / 2)
    CLK = [0., 0.] + [-amplitude, amplitude] * Nclk + [0., 0.]
    CLK = np.tile(CLK, (L0, 1)).flatten("F")
    V_reset = np.zeros_like(CLK)
    
    awg.send_and_place_wf(channel=1, wf=V_reset)
    awg.send_and_place_wf(channel=2, wf=V_reset)
    awg.send_and_place_wf(channel=3, wf=CLK)
    awg.send_and_place_wf(channel=4, wf=CLK)
        
    awg.wait_opc()
    awg.run()
    awg.set_all_outputs("ON")
    awg.wait_opc()
    awg.force_trig()
    time.sleep(0.5)
    
    ## READ AGAIN
    i1 = instr.SPI_read(0xA1, 1)[0]
    time.sleep(0.1)
    return (i0, i1)

## OPEN INSTRUMENT
bitfile_path = """C:/Users/manip.batm/Documents/FPGA_Batch_2_0_5_CryoC009/Labview_2019/FPGA Bitfiles/FPGA_CIR7_main_analog_addr.lvbitx"""
ip_address = "192.168.1.21"
instr = CIR7Driver(ip_address, bitfile_path, DAC_dict)
print(instr.FPGA.fpga_vi_state)

## RESET CIR7
# instr.reset()

## DACS
DAC_val = {}
# DAC_val['MODE_ROT'] = 0. # auto-set by following functions
# DAC_val['SEL_MAT'] = 1.8
# DAC_val['OSC_VCO'] = 0.
# DAC_val['RESETN'] = 1.8

DAC_val['VBGPA'] = -2.
DAC_val['VBGNA'] = 2.
DAC_val['VBGNC'] = 0.
DAC_val['VBGPC'] = 0.
DAC_val['VBGNM'] = 0.
DAC_val['VBGPM'] = 0.

DAC_val['CMD_R0'] = 0.
DAC_val['CMD_R1'] = 0.
DAC_val['CMD_R2'] = 0.
DAC_val['CMD_R3'] = 0.
DAC_val['CMD_R4'] = 0.
DAC_val['CMD_R5'] = 0.
DAC_val['CMD_R6'] = 0.

DAC_val['ADDR_0'] = 1.8
DAC_val['ADDR_1'] = 0.
DAC_val['ADDR_2'] = 0.
DAC_val['ADDR_4'] = 0.

DAC_val['Veven_DC'] = 0.9
DAC_val['Vodd_DC'] = 0.9
DAC_val['CLK_DC'] = 0.85
instr.update_DAC(DAC_val)

## SPI
# instr.set_mode('addr')
instr.set_mode('counter')
# instr.set_mode('lmt_counter', start=0, stop=63)
# instr.set_mode('register', values=list(range(64)))
# instr.set_mode('sram', values=list(range(64))))
# instr.set_mode('direct')

instr.set_clk(int_clk=False, osc_vco=0., two_cycles=False, add_delay=True)
instr.set_output(mux_mat=False, line0=None, line1=None, column0=None, column1=None)

## PREPARE AWG
AWG_VISA = "TCPIP::192.168.1.12::4000::SOCKET" 
awg = AWGDriver(AWG_VISA)
awg.stop()
awg.set_sampling_rate(sampl_rate=AWG_SAMPLING_RATE_MHz * 1e6)

## TEST CLK RECEPTION
# lmt_counter : ok until 350+-2 kHz, ensuite progressivement 1, 2, 3 trig manqués  
# idem register & counter
# s'améliore en augmentant l'amplitude de CLK jusqu'à 3.2MHz avec 2.8V
clk_rate_MHz = 300.
real_freq = AWG_SAMPLING_RATE_MHz / int(AWG_SAMPLING_RATE_MHz / clk_rate_MHz / 2) / 2
Nclk = 5
amplitude = 1.2
print(f"CLK rate {real_freq} MHz - Amplitude +-{amplitude}V")
for k in range(4):
    (i0, i1) = test_clk(awg, clk_rate_MHz, Nclk, amplitude)
    if (i1 - i0) % 64 == Nclk:
        print("Success")
    else:
        print(f"Failed: i0={i0}, i1={i1}, i1-i0={(i1 - i0) % 64}")
    time.sleep(0.5)


# CLOSE INSTRUMENTS
awg.close()
instr.stop_seq()
instr.FPGA.close()