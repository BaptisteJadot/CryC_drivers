# -*- coding: utf-8 -*-
"""
Created on Mon Nov  8 17:56:51 2021

@author: manip.batm
"""

from CIR7_driver import CIR7Driver
from CIR7_config import DAC_dict
from AWG_driver import AWGDriver, build_waveforms, AWG_SAMPLING_RATE_MHz
from Scope_driver import ScopeDriver
import Fastseq_elmts as fs
import numpy as np
import time

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
instr.update_DAC(DAC_val)

## SPI
# instr.set_mode('addr')
# instr.set_mode('counter')
instr.set_mode('lmt_counter', start=14, stop=17)
# instr.set_mode('register', values=[14, 15, 16, 17]*16)
# instr.set_mode('sram', values=list(range(64))))
# instr.set_mode('direct')

instr.set_clk(int_clk=False, osc_vco=0., two_cycles=False, add_delay=True)
instr.set_output(mux_mat=False, line0=14//2, line1=17//2, column0=None, column1=None)

## PREPARE AWG
AWG_VISA = "TCPIP::192.168.1.12::4000::SOCKET" 
awg = AWGDriver(AWG_VISA)
awg.stop()
awg.set_sampling_rate(sampl_rate=AWG_SAMPLING_RATE_MHz * 1e6)

## INCREMENT CLOCK UNTIL ADDRESS 14
i0 = instr.SPI_read(0xA0, 1)[0]
print(i0)
if i0 != 14:
    _, _, CLK_reset = build_waveforms(values=[0.] * ((14 - i0) % 64), clk_rate_MHz=0.05, data_to_clk_us=0, current_index=0, current_val=0.)
    V_reset = np.zeros_like(CLK_reset)
    awg.send_and_place_wf(channel=1, wf=V_reset)
    awg.send_and_place_wf(channel=2, wf=V_reset)
    awg.send_and_place_wf(channel=3, wf=CLK_reset)
    awg.send_and_place_wf(channel=4, wf=V_reset)
    
    time.sleep(0.1)
    awg.run()
    time.sleep(0.1)
    awg.set_all_outputs("ON")
    time.sleep(0.1)
    awg.force_trig()
    time.sleep(0.5)
    
SPI_state = instr.SPI_dump_all(output_to_console=True)

## AWG
values = [0.25, 0.5, 0.75, 1.] * 10
clk_rate_MHz = 0.5
data_to_clk_us = 0.9
Veven, Vodd, CLK = build_waveforms(values, clk_rate_MHz, data_to_clk_us, current_index=i0, current_val=values[-1])

awg.stop()
awg.send_and_place_wf(channel=1, wf=Veven)
awg.send_and_place_wf(channel=2, wf=Vodd)
awg.send_and_place_wf(channel=3, wf=CLK)
awg.send_and_place_wf(channel=4, wf=CLK)

# start
time.sleep(0.1)
awg.run()
time.sleep(0.1)
awg.set_all_outputs("ON")
time.sleep(0.1)
awg.force_trig()
time.sleep(0.5)


# CLOSE INSTRUMENTS
awg.close()
instr.stop_seq()
instr.FPGA.close()