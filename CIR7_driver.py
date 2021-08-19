# -*- coding: utf-8 -*-
"""
Created on Thu Aug 12 09:44:21 2021

@author: manip.batm
"""

import numpy as np
from nifpga import Session, FpgaViState
from nifpga.status import RpcConnectionErrorError, FpgaBusyInteractiveError
from FPGA_utils import build_order,uint16_to_float,float_to_uint16
# from SPI_driver import SPI_driver
import time

class CIR7_driver():
    """
    Main driver for CIR7 circuit v2.0
    """
    def __init__(self,ip_address,bitfile_path,DAC_dict={}):
        """
        Opens communication and finds the VI objects to control
        DAC_dict is a dictionnary of each DAC output (with fields 'panel', 'channel', 'lower_limit' and 'upper_limit')
        """
        self.ip_address = ip_address
        if DAC_dict == {}:
            self.DAC_dict = {str(i)+':'+str(j):{'panel':i,'channel':j,'lower_limit':-5.,'upper_limit':5.} for i in range(8) for j in range(8)} # default '0:3': {0,3,-5V,5V}]
        else:
            self.DAC_dict = DAC_dict
        try:
            self.FPGA = Session(bitfile=bitfile_path,resource='rio://'+ip_address+'/RIO0',no_run=True,reset_if_last_session_on_exit=False)
            if self.FPGA.fpga_vi_state!=FpgaViState(2): # FPGA is not running
                self.FPGA.run()
        except RpcConnectionErrorError:
            raise RuntimeError('FPGA not found')
        except FpgaBusyInteractiveError:
            raise RuntimeError('FPGA is busy')
        except:
            raise # unexpected error
        else:
            # incoming orders FIFO
            self.FPGA.orders_fifo = self.FPGA.fifos['Incoming orders']
            self.FPGA.orders_fifo.configure(1050) # configure depth
            self.FPGA.orders_fifo.start()
            # readout outputs
            self.FPGA.CIR7_values = self.FPGA.registers['CIR7 values']
            self.FPGA.DAC_values = self.FPGA.registers['DAC values']
            # SPI data fifo
            self.FPGA.SPI_data = self.FPGA.fifos['SPI data']
            self.FPGA.SPI_data.configure(512) # configure depth
            self.FPGA.SPI_data.start()
            
            # configure communication speed
            config_orders = []
            config_orders += [build_order([(2,8),(4,8),(0,8*2),(1600,32)])] # SPI speed
            self.send_orders(config_orders)
    
    def send_orders(self,orders):
        """
        Puts orders in the FPGA input buffer and waits for until they are all passed.
        orders is a list of uint64s (see FPGA doc).
        """
        #print(orders)
        while len(orders)>0:
            self.FPGA.orders_fifo.write(orders[:500],timeout_ms=10) # FIFO size is 1050 elements
            orders = orders[500:]
            irq_status = self.FPGA.wait_on_irqs(0,timeout_ms=10)
            if irq_status.timed_out:
                raise RuntimeError('Timeout while passing orders')
    
    def update_DAC(self,values):
        """
        Updates one or several DAC output(s) to the indicated value(s).
        values is a dict of type {DAC_key1:val1,DAC_key2:val2}
        """
        orders = []
        for (key, val) in values.items():
            if key not in self.DAC_dict.keys():
                raise KeyError('Unknown DAC id')
            else:
                pan = self.DAC_dict[key]['panel']
                chan = self.DAC_dict[key]['channel']
                ll = self.DAC_dict[key]['lower_limit']
                ul = self.DAC_dict[key]['upper_limit']
                if val<ll or val>ul:
                    raise ValueError('DAC value is out of limits')
                else:
                    orders.append(build_order([(1,8),(0,3*8),(pan,8),(chan,8),(float_to_uint16(val),16)]))
        self.send_orders(orders)
        
    def read_current_values(self):
        """
        Asks the FPGA for the last values sent to each DAC output.
        Returns a dict of type {DAC_key1:val1,DAC_key2:val2} and the content of the CIR7 memory.
        """
        self.FPGA.acknowledge_irqs([3])
        self.send_orders([build_order([(3,8),(0,7*8)])])
        self.FPGA.wait_on_irqs([3],timeout_ms=10)
        irq_status = self.FPGA.wait_on_irqs(3,timeout_ms=10)
        if irq_status.timed_out:
            raise RuntimeError('Timeout while reading DAC values')
        else:
            DAC_val = self.FPGA.DAC_values.read()
            CIR7_val = self.FPGA.CIR7_values.read()
            CIR7_val = [np.round(uint16_to_float(val),4) for val in CIR7_val]
            self.FPGA.acknowledge_irqs([3])
            outp_dict = {}
            for key in self.DAC_dict.keys():
                pan = self.DAC_dict[key]['panel']
                chan = self.DAC_dict[key]['channel']
                outp_dict[key] = np.round(uint16_to_float(DAC_val[pan*8+chan]),4)
            return outp_dict, CIR7_val
        
    def SPI_read(self,address,Nbytes):
        """
        Reads Nbytes from SPI.
        """
        self.send_orders([build_order([(2,8),(3,8),(0,8*3),(address,8),(Nbytes,16)])]) # read SPI
        SPI_content = self.FPGA.SPI_data.read(Nbytes,timeout_ms=2000)
        return [(d>>8,d%256) for d in SPI_content.data] # [(addr0,d0),(addr1,d1),...]
        
    def SPI_write(self,address,data):
        """
        Writes one or several byte(s) of data to the SPI.
        """
        if len(data)==0:
            data = [data]
        Nbytes = len(data)
        orders = []
        for i,di in enumerate(data):
            orders += [build_order([(2,8),(1,8),(0,8*4),(address+i,8),(di,8)])] # place in memory
        orders += [build_order([(2,8),(2,8),(0,8*3),(address,8),(Nbytes,16)])] # write order
        self.send_orders(orders)
        
    def panel4_config(self,start_ind=0,stop_ind=63,ticks_per_address=800,ticks_per_bit=10):
        """
        Configures the refresh rate & communication speed of panel 4.
        """
        if ticks_per_bit*39*2 > ticks_per_address:
            raise ValueError('Refresh rate is too low')
        orders = [build_order([(4,8),(3,8),(0,8),(0,8),(ticks_per_bit,32)])]
        orders += [build_order([(4,8),(3,8),(0,8),(1,8),(ticks_per_address,32)])]
        orders += [build_order([(4,8),(5,8),(0,8*4),(stop_ind,8),(start_ind,8)])]
        orders += [build_order([(4,8),(2,8),(0,8*6)])] # start if not already running
        self.send_orders(orders)
        
    def panel4_set_value(self,values_dict):
        """
        Updates one or several programmed value(s) in CIR7.
        """
        orders = []
        for key,val in values_dict.items():
            if not isinstance(key,int) or key not in range(64):
                raise KeyError('Panel 4 key must be an integer from 0 to 63')
            else:
                orders += [build_order([(4,8),(4,8),(0,8*3),(key,8),(float_to_uint16(val),16)])]
        self.send_orders(orders)
        
    def panel4_stop(self):
        """
        Stops panel 4 refreshing
        """
        stop_order = build_order([(4,8),(1,8),(0,8*6)])
        self.send_orders([stop_order])
        
    def panel4_start(self):
        """
        Starts panel 4 refreshing
        """
        stop_order = build_order([(4,8),(2,8),(0,8*6)])
        self.send_orders([stop_order])
        
    def config_seq(self,slots,us_per_DAC=1000,trig_reset_states=[True]*10,start_after=False,start_index=0):
        """
        Configures a fast sequence.
        slots is a dict of type {num:Slot_object}. (missing slots are not updated, use End slots to erase a previous sequence)
        """
        stop_order = build_order([(5,8),(1,8),(0,6*8)])
        trig_reset_code = sum([2**i for (i,b) in enumerate(trig_reset_states) if b])
        trig_reset_order = build_order([(5,8),(10,8),(0,38),(trig_reset_code,10)])
        us_DAC_order = build_order([(5,8),(8,8),(0,2*8),(us_per_DAC,32)])
        orders = [stop_order,trig_reset_order,us_DAC_order]
        # slots
        for (i,slot) in slots.items():
            if i not in range(0,1023):
                raise KeyError('Unknown slot id')
            else:
                #print(i,slot.gen_order(i))
                orders.append(slot.gen_order(i))
                #print('{}:{:16X}'.format(i,orders[-1]))
        if start_after:
            start_order = build_order([(5,8),(2,8),(0,38),(start_index,10)])
            orders.append(start_order)
        self.send_orders(orders)

    def update_slots(self,slots,start_after=False,start_index=0):
        """
        Modifies several slots in the sequence.
        slots is a dict of type {num:Slot_object}
        start_after indicates whether to run the sequence or not after the modification.
        e.g. update_slots({5:DAC_slot(0,0,-2),7:Timing_slot(1.0,'10us')})
        """
        start_order = build_order([(5,8),(2,8),(0,38),(start_index,10)])
        stop_order = build_order([(5,8),(1,8),(0,6*8)])
        orders = [stop_order]
        for (i,slot) in slots.items():
            if i not in range(0,1023):
                raise KeyError('Unknown slot id')
            else:
                orders.append(slot.gen_order(i))
        if start_after:
            orders.append(start_order)
        self.send_orders(orders)

    def update_slots_lowlevel(self,seq_orders,start_after=False,start_index=0):
        """
        Modifies several slots in the sequence.
        seq_orders is a list of uint64s.
        start_after indicates whether to run the sequence or not after the modification.
        """
        start_order = 0x0502<<6*8 | start_index
        stop_order = 0x0501<<6*8
        if start_after:
            self.send_orders([stop_order,*seq_orders])
        else:
            self.send_orders([stop_order,*seq_orders,start_order])

    def start_seq(self,start_ind=0):
        """
        Runs the programmed sequence from the indicated row.
        """
        self.send_orders([build_order([(5,8),(2,8),(0,38),(start_ind,10)])])

    def stop_seq(self):
        """
        Stops the sequence execution.
        """
        self.send_orders([build_order([(5,8),(1,8),(0,6*8)])])

    def restart_seq(self,restart_ADC=False):
        """
        Stops and runs the programmed sequence from the last starting index.
        restart_ADC allows to send the ADC start order immediately before (for timing performances)
        """
        if restart_ADC:
            orders = [build_order([(6,8),(2,8),(0,6*8)])] # ADC start (wait for trigger)
        else:
            orders = []
        orders.append(build_order([(5,8),(3,8),(0,6*8)])) # sequence restart
        self.send_orders(orders)
        
if __name__=="__main__":
    
    bitfile_path = """C:/Users/manip.batm/Documents/FPGA_Batch_2_0_5_CryoC009/Labview_2019/FPGA Bitfiles/FPGA_CIR7_main.lvbitx"""
    ip_address = "192.168.1.21"
    
    instr = CIR7_driver(ip_address,bitfile_path,DAC_dict={})
    print(instr.read_current_values())
    
    ans1 = instr.SPI_read(0x70,4)
    print([hex(add)+' : '+hex(val) for (add,val) in ans1])
    for (add,val) in ans1:
        print(f"{add:02x}\t{val:02x}")
    
    instr.SPI_write(0x70,[0x81,0x82,0x03,0x84])
    time.sleep(0.1)
    ans2 = instr.SPI_read(0x70,4)
    for (add,val) in ans2:
        print(f"{add:02x}\t{val:02x}")
    
    instr.panel4_config(start_ind=0,stop_ind=7,ticks_per_address=800,ticks_per_bit=10)
    values_dict = {}
    for addr in range(8):
        values_dict[addr] = 0.1*addr
    instr.panel4_set_value(values_dict)
    
    instr.FPGA.close()
    