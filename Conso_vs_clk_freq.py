# -*- coding: utf-8 -*-
"""
Created on Tue Nov 30 10:56:49 2021

@author: manip.batm
"""


from CIR7_driver import CIR7Driver
from CIR7_config import DAC_dict
from AWG_driver import AWGDriver, build_waveforms, AWG_SAMPLING_RATE_MHz
import Fastseq_elmts as fs
import numpy as np
import time
import pyvisa
import matplotlib.pyplot as plt
import h5py as h5

def read_IV(smu):
    Ia = float(smu.query("print(smua.measure.i())"))
    Va = float(smu.query("print(smua.measure.v())"))
    Ib = float(smu.query("print(smub.measure.i())"))
    Vb = float(smu.query("print(smub.measure.v())"))
    return Ia, Va, Ib, Vb

def send_clk(awg, clk_rate_MHz, amplitude=0.9):
    # stop AWG
    awg.stop()
    
    # send wf
    if clk_rate_MHz < 10:
        CLK = np.array([-amplitude]*10 + [amplitude]*10)
    else:
        CLK = np.array([-amplitude, amplitude])
    V_reset = np.zeros_like(CLK)
    awg.send_and_place_wf(channel=1, wf=V_reset)
    awg.send_and_place_wf(channel=2, wf=V_reset)
    awg.send_and_place_wf(channel=3, wf=CLK)
    awg.send_and_place_wf(channel=4, wf=CLK)
    awg.set_repetition_rate(clk_rate_MHz*1e6)
    
    # turn ON
    awg.wait_opc()
    awg.instr.write("AWGC:RMODE CONT")
    awg.run()
    awg.set_all_outputs("ON")
    awg.wait_opc()

## OPEN COMM WITH SMU
rm = pyvisa.ResourceManager()
smu = rm.open_resource("GPIB1::26::INSTR", write_termination="\n", read_termination="\n")
smu.write("beeper.beep(0.2, 880)")

## OPEN INSTRUMENT
bitfile_path = """C:/Users/manip.batm/Documents/FPGA_Batch_2_0_5_CryoC009/Labview_2019/FPGA Bitfiles/FPGA_CIR7_main_analog_addr.lvbitx"""
ip_address = "192.168.1.21"
instr = CIR7Driver(ip_address, bitfile_path, DAC_dict)
print(instr.FPGA.fpga_vi_state)

## PREPARE AWG
AWG_VISA = "TCPIP::192.168.1.12::4000::SOCKET" 
awg = AWGDriver(AWG_VISA)
awg.stop()
awg.set_sampling_rate(sampl_rate=AWG_SAMPLING_RATE_MHz * 1e6)

## RESET CIR7
# instr.reset()

## DACS
DAC_val = {}
# DAC_val['MODE_ROT'] = 0. # auto-set by following functions
# DAC_val['SEL_MAT'] = 1.8
# DAC_val['OSC_VCO'] = 0.
# DAC_val['RESETN'] = 1.8

DAC_val['VBGPA'] = 0.
DAC_val['VBGNA'] = 0.
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
# instr.set_mode('counter')
# instr.set_mode('lmt_counter', start=0, stop=63)
instr.set_mode('register', values=list(range(64)))
# instr.set_mode('sram', values=list(range(64)))
# instr.set_mode('direct')

instr.set_clk(int_clk=False, osc_vco=0., two_cycles=False, add_delay=False)
instr.set_output(mux_mat=False, line0=None, line1=None, column0=None, column1=None)
# instr.set_output(mux_mat=False, line0=14//2, line1=17//2, column0=None, column1=None)

## READ CURRENT STATE
SPI_state = instr.SPI_dump_all(output_to_console=True)
DAC_values, _ = instr.read_current_values()
instr.stop_seq()
time.sleep(0.1)

## RAMP CLK FREQUENCY AND RECORD CONSO
f_max = AWG_SAMPLING_RATE_MHz
f_CLK = np.arange(10, f_max//2+0.1, 10)
CLK_amplitude = 1.2
do_save = True
grp_name = "data_VDD18AN_register"

# 1.0V
data_chanA = np.zeros((len(f_CLK), 2))
data_chanB = np.zeros((len(f_CLK), 2))
# record power for each CLK frequency
for i, fi in enumerate(f_CLK):
    send_clk(awg, fi, 1, CLK_amplitude)
    time.sleep(5)
    tup = read_IV(smu)
    print(f"{fi*1e6:.1e} Hz \t {tup[0]*tup[1]*1e6:.2f} \t {tup[2]*tup[3]*1e6:.2f}")
    data_chanA[i, 1], data_chanA[i, 0], data_chanB[i, 1], data_chanB[i, 0] = tup

# CLOSE INSTRUMENTS
awg.stop()
awg.close()
instr.stop_seq()
instr.FPGA.close()

## SAVE
if do_save:        
    fname = "D:/Baptiste/CIR7/RT/Conso_vs_CLK_freq.h5"
    with h5.File(fname, 'a') as f:
        if grp_name not in f.keys():
            grp = f.create_group(grp_name)
        else:
            grp = f["grp_name"]
        grp.create_dataset("f_CLK", data=f_CLK)
        grp.create_dataset("data_chanA", data=data_chanA)
        grp.create_dataset("data_chanB", data=data_chanB)
        grp.attrs.create("CLK_amplitude", CLK_amplitude)
        grp.attrs.create("SPI_settings", SPI_state)
        grp.attrs.create("Comment", "ADDR5 floating, " + DAC_values.__str__())


# PLOT
runfile('D:/Baptiste/CIR7/RT/CIR7_plot_conso_vs_clk_freq.py', wdir='D:/Baptiste/CIR7/RT')
# plt.figure(533);plt.ion()
# plt.plot(f_CLK, data_chanA[:, 0]*data_chanA[:, 1]*1e6)
# plt.plot(f_CLK, data_chanB[:, 0]*data_chanB[:, 1]*1e6)
# plt.xscale("log")
