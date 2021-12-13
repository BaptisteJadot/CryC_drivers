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
import matplotlib.pyplot as plt
import h5py as h5

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
instr.set_mode('lmt_counter', start=0, stop=63)
# instr.set_mode('register', values=list(range(64)))
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
# SPI_state = instr.SPI_dump_all(output_to_console=True)
i0 = instr.SPI_read(0xA0, 1)[0]
i0 = (i0) % 64
print(i0)
if i0 != 0:
    # _, _, CLK_reset = build_waveforms(values=[0.] * ((14 - i0) % 64), clk_rate_MHz=0.05, data_to_clk_us=0, current_index=0, current_val=0.)
    L = int(AWG_SAMPLING_RATE_MHz / 0.02)
    CLK_reset = [0., 0.] + [0., 1.8] * ((0 - i0) % 64) + [0., 0.]
    CLK_reset = np.tile(CLK_reset, (L, 1)).flatten("F")
    V_reset = np.zeros_like(CLK_reset)
    awg.send_and_place_wf(channel=1, wf=V_reset)
    awg.send_and_place_wf(channel=2, wf=V_reset)
    awg.send_and_place_wf(channel=3, wf=CLK_reset)
    awg.send_and_place_wf(channel=4, wf=V_reset)
    
    awg.wait_opc()
    awg.run()
    awg.wait_opc()
    awg.set_all_outputs("ON")
    awg.wait_opc()
    awg.force_trig()
    time.sleep(0.5)
SPI_state = instr.SPI_dump_all(output_to_console=True)
i1 = instr.SPI_read(0xA0, 1)[0]

## AWG
values = [0.5]*64
values[14] = 0.75
values[17] = 1.
do_save = False
fname = "test_14_17_second_set.h5"
# values = values[i0:] + values[:i0]
# values = values[i0:]
clk_rate_MHz = 0.16
data_to_clk_us = 1 / clk_rate_MHz * 0.5
Veven, Vodd, CLK = build_waveforms(values, clk_rate_MHz, data_to_clk_us, current_index=i0, current_val=values[-1])

awg.stop()
awg.send_and_place_wf(channel=1, wf=Veven)
awg.send_and_place_wf(channel=2, wf=Vodd)
awg.send_and_place_wf(channel=3, wf=CLK)
awg.send_and_place_wf(channel=4, wf=CLK)

## SCOPE
SCOPE_VISA = "TCPIP::192.168.1.46::INSTR"
scope = ScopeDriver(SCOPE_VISA)
# _ = scope.check_errors()

# # config
# scope.stop()
# scope.set_trigger(source="CH4", level=1., edge="POS")
# for k in range(1, 5):
#     scope.set_vertical(chan_id=k, active=True, volts_per_div=0.25, offset_volt=1.)
# scope.set_horizontal(sec_per_div=20e-6, sec_offset=100e-6)

# arm trigger
scope.start_acq(mode="NORM", wfm_count=1)
time.sleep(1.)
# _ = scope.check_errors()

## START
# start awg
awg.run()
awg.set_all_outputs("ON")
awg.wait_opc()
awg.force_trig()
time.sleep(0.5)

## GET DATA
t, traces = scope.get_traces()
plt.figure(800);plt.clf();plt.ion()
for key, val in traces.items():
    plt.plot(t * 1e6, val, label=key)
plt.legend()
# print(t.shape)
# if len(traces.keys()) > 0:
#     print([tr.shape for tr in traces.values()])

## SAVE
# init_move_dt = np.dtype({"names":["name","value"],"formats":["S100","f8"]})
# DAC_val_arr = np.array([(key, val) for key, val in DAC_val.items()], dtype=init_move_dt)
if do_save:
    with h5.File(r"D:\Baptiste\CIR7\\4K\\All_64_with_AWG\\" + fname, "a") as f:
        grp = f.create_group("AWG")
        grp.create_dataset("Veven", data=Veven)
        grp.create_dataset("Vodd", data=Vodd)
        grp.create_dataset("CLK", data=CLK)
        grp.create_dataset("values", data=values)
        grp.attrs.create("SPI_settings", SPI_state)
        grp.attrs.create("i0", i0)
        grp.attrs.create("i1", i1)
        grp.attrs.create("clk_rate_MHz", clk_rate_MHz)
        grp.attrs.create("data_to_clk_us", data_to_clk_us)
        
        grp = f.create_group("Scope")
        grp.create_dataset("t", data=t)
        grp.create_dataset("CH1", data=traces["CH1"])
        grp.create_dataset("CH2", data=traces["CH4"])
        grp.create_dataset("CH3", data=traces["CH4"])
        grp.create_dataset("CH4", data=traces["CH4"])   
    
        f.close()


# CLOSE INSTRUMENTS
awg.close()
instr.stop_seq()
instr.FPGA.close()
scope.close()