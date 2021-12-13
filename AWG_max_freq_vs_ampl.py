# -*- coding: utf-8 -*-
"""
Created on Wed Dec  1 10:43:14 2021

@author: manip.batm
"""

from CIR7_driver import CIR7Driver
from CIR7_config import DAC_dict
from AWG_driver import AWGDriver, build_waveforms, AWG_SAMPLING_RATE_MHz
import Fastseq_elmts as fs
import numpy as np
import time
import matplotlib.pyplot as plt
import h5py as h5

def test_clk(awg, clk_rate_MHz, Nclk=5, amplitude=0.9):
    ## SEND 5 CLOCKS    
    CLK = np.array([0., 0.] + [-amplitude, amplitude] * Nclk + [0., 0.])
    Vempty = np.zeros_like(CLK)
    
    awg.send_and_place_wf(channel=1, wf=Vempty)
    awg.send_and_place_wf(channel=2, wf=Vempty)
    awg.send_and_place_wf(channel=3, wf=CLK)
    awg.send_and_place_wf(channel=4, wf=CLK)
    
    awg.set_sampling_rate(clk_rate_MHz * 2 * 1e6)
    time.sleep(0.5)
    # awg.wait_opc()
    awg.run()
    awg.set_all_outputs("ON")
    time.sleep(0.5)
    # awg.wait_opc()
    awg.force_trig()
    time.sleep(0.5)
    
    ## READ SPI
    i1 = instr.SPI_read(0xA0, 1)[0]
    return i1

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
instr.reset_with_AWG(awg)

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
DAC_val['CLK_DC'] = 0.9
instr.update_DAC(DAC_val)

## SPI
# instr.set_mode('addr')
# instr.set_mode('counter')
instr.set_mode('lmt_counter', start=0, stop=63)
# instr.set_mode('register', values=list(range(64)))
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

## PARAMETERS
do_save = True
fname = "Freq_incr_vs_clk_ampl_direct_highres.h5"
CLK_amplitude = np.arange(0.3, 1.65, 0.1)
CLK_frequency = np.arange(10, 410, 5)
modes = {"counter":0, "lmt_counter":16, "register":17}
Nclk = 5

## RUN
data = np.zeros((len(CLK_frequency), len(CLK_amplitude), 3))
i0 = instr.SPI_read(0xA0, 1)[0]
for k, m in enumerate(modes.values()):
    instr.SPI_write(0xFE, [m])
    time.sleep(0.1)
    for j, Aj in enumerate(CLK_amplitude):
        for i, fi in enumerate(CLK_frequency):
            i1 = test_clk(awg, fi, Nclk, Aj)
            if (i1 - i0) % 64 == Nclk:
                print(f"{Aj:.1f}V \t {fi:.1f}MHz \t - \t Success.")
            else:
                print(f"{Aj:.1f}V \t {fi:.1f}MHz \t - \t Failed.")
            data[i, j, k] = (i1 - i0) % 64 
            i0 = i1

# CLOSE INSTRUMENTS
awg.stop()
awg.close()
instr.stop_seq()
instr.FPGA.close()

## SAVE
if do_save:        
    with h5.File("D:/Baptiste/CIR7/RT/"+fname, 'a') as f:
        if "data" in f.keys():
            raise ValueError("data already exists in h5 file.")
        else:
            grp = f.create_group("data")
            grp.create_dataset("CLK_amplitude", data=CLK_amplitude)
            grp.create_dataset("CLK_frequency", data=CLK_frequency)
            grp.create_dataset("mode", data=list(modes.values()))
            grp.create_dataset("data", data=data)
            grp.attrs.create("CLK_DC", DAC_values["CLK_DC"])
            grp.attrs.create("Nclk", Nclk)
            grp.attrs.create("SPI_settings", SPI_state)
            grp.attrs.create("Comment", "ADDR5 floating, " + DAC_values.__str__())


# PLOT
# runfile('D:/Baptiste/CIR7/RT/CIR7_plot_conso_vs_clk_freq.py', wdir='D:/Baptiste/CIR7/RT')
plt.figure(628);plt.clf();plt.ion()
plt.plot(CLK_frequency, data[:,-1,0], 'o')
plt.plot(CLK_frequency, data[:,-1,1], 'o')
plt.plot(CLK_frequency, data[:,-1,2], 'o')
