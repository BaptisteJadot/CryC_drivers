# -*- coding: utf-8 -*-
"""
Created on Tue Oct 19 09:16:57 2021

@author: manip.batm
"""


from CIR7_driver import CIR7Driver
from CIR7_config import DAC_dict
import Fastseq_elmts as fs
import numpy as np
import matplotlib.pyplot as plt
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
# instr.set_mode('lmt_counter', start=0, stop=63)
instr.set_mode('register', values=list(range(64)))
# instr.set_mode('sram', values=list(range(64))))
# instr.set_mode('direct')

instr.set_clk(int_clk=False, osc_vco=0., two_cycles=False, add_delay=False)

## INCREMENT CLOCK UNTIL ADDRESS 0
i0 = instr.SPI_read(0xA1, 1)[0]
print(i0)
while i0 != 0:
    seq = []
    seq += [fs.Panel4(address=0, clk=False, even_value=0., odd_value=0.)]
    seq += [fs.Panel4(address=0, clk=True, even_value=0., odd_value=0.)]
    seq += [fs.Panel4(address=0, clk=False, even_value=0., odd_value=0.)]
    seq += [fs.End()]
    instr.config_seq(slots={i:s for (i,s) in enumerate(seq)}, 
                        us_per_DAC=10, 
                        trig_reset_states=[True]*10, 
                        start_after=True, 
                        start_index=0)
    time.sleep(0.05)
    i0 = instr.SPI_read(0xA1, 1)[0]
    print(i0)
SPI_state = instr.SPI_dump_all(output_to_console=True)


## RUN
print(instr.get_ADC_data())
out_data = np.zeros((100,64,64), dtype=np.float)
for addr_in in range(64):        
# for addr_in in [15, 16]:        
    # update fast seq
    v = [0.2]*64
    v[addr_in] = 0.8
    seq = [fs.Trig_out(trig=[False]*4)]
    for i in range(32):
        seq += [fs.Panel4(address=0, clk=False, even_value=v[(2*i)%64], odd_value=v[(2*i-1)%64])]
        seq += [fs.Panel4(address=0, clk=True, even_value=v[(2*i)%64], odd_value=v[(2*i-1)%64])]
        seq += [fs.Panel4(address=0, clk=False, even_value=v[(2*i)%64], odd_value=v[(2*i+1)%64])]
        seq += [fs.Panel4(address=0, clk=True, even_value=v[(2*i)%64], odd_value=v[(2*i+1)%64])]
    # seq += [fs.Panel4(address=0, clk=False, even_value=v[62], odd_value=v[63])]
    # seq = [fs.Trig_out(trig=[True]*4)]
    # seq += [fs.Wait(value=1000000, precision='1ms')]
    seq += [fs.JumpFor(target=1, count=9)] # set 10 times
    seq += [fs.End()]
    seq += [fs.ADC_get(channel=2)]
    seq += [fs.JumpFor(target=len(seq)-1, count=99)] # read 10 times
    seq += [fs.End()]
    seq += [fs.ADC_get(channel=3)]
    seq += [fs.JumpFor(target=len(seq)-1, count=99)] # read 10 times
    seq += [fs.End()]
    
    instr.config_seq(slots={i:s for (i,s) in enumerate(seq)}, 
                        us_per_DAC=10, 
                        trig_reset_states=[True]*10, 
                        start_after=True, 
                        start_index=0)
    
    print(f'\n{addr_in}', end='')
    # read all 64
    for addr_out in range(64):
        # update MUX output
        if addr_out % 2 == 0:
            instr.set_output(mux_mat=True, line0=None, line1=None, column0=addr_out//2, column1=None)
            time.sleep(0.1)
            instr.start_seq(start_ind=131)
        else:
            instr.set_output(mux_mat=True, line0=None, line1=None, column0=None, column1=addr_out//2)
            time.sleep(0.1)
            instr.start_seq(start_ind=134)
        time.sleep(0.1)
        d = np.array(instr.get_ADC_data(Npts=100), dtype=np.float)
        out_data[:, addr_out, addr_in] = d
        print('.', end='')
    
# CLOSE INSTRUMENT
instr.stop_seq()
instr.FPGA.close()

## PLOT
plt.figure(356) ; plt.clf(); plt.ion()
plt.imshow(np.mean(out_data, axis=0))
# plt.plot(np.mean(out_data[:,:,15], axis=0))
# plt.plot(np.mean(out_data[:,:,16], axis=0))
plt.xlabel('addr_in')
plt.ylabel('addr_out')
cb = plt.colorbar()
cb.set_label('Vout (V)')
# plt.legend(['MUX_mat OFF', 'MUX_mat ON'])
# plt.title('SO<3> (addr 15) vs Vodd')
# plt.grid()


# ## SAVE
# # init_move_dt = np.dtype({'names':['name','value'],'formats':['S100','f8']})
# # DAC_val_arr = np.array([(key, val) for key, val in DAC_val.items()], dtype=init_move_dt)
# with h5.File('''D:\Baptiste\CIR7\\4K\\all_but_one_64.h5''', 'a') as f:
#     grp = f.create_group(f'data')
#     grp.create_dataset('Vout', data=out_data)
#     grp.attrs.create('mux_mat', True)
#     grp.attrs.create('SPI_settings', SPI_state)
#     grp.attrs.create('V0', 0.2)
#     grp.attrs.create('V1', 0.8)
#     f.close()


