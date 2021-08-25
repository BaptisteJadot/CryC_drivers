# -*- coding: utf-8 -*-
"""
Created on Tue Aug 24 09:33:07 2021

@author: Baptiste
"""

import numpy as np
from nifpga import Session, FpgaViState
from nifpga.status import RpcConnectionErrorError, FpgaBusyInteractiveError, FifoTimeoutError
from FPGA_utils import build_order, uint16_to_float, float_to_uint16
import time

class CIR7_driver():
    """
    Main driver for CIR7 circuit v2.0
    """

######################################
###      FPGA COMMUNICATION
######################################

    def __init__(self, ip_address, bitfile_path, DAC_dict={}):
        """
        Opens communication and finds the VI objects to control
        DAC_dict is a dictionnary of each DAC output (with fields 'panel', 'channel', 'lower_limit' and 'upper_limit')
        """
        self.ip_address = ip_address
        if DAC_dict == {}:
            self.DAC_dict = {f"{i}:{j}" : {'panel':i, 'channel':j, 'lower_limit':-5., 'upper_limit':5.} for i in range(8) for j in range(8)} # default '0:3': {0,3,-5V,5V}]
        else:
            self.DAC_dict = DAC_dict
        try:
            self.FPGA = Session(bitfile = bitfile_path, resource = 'rio://' + ip_address + '/RIO0', no_run=False, reset_if_last_session_on_exit=False)
            if self.FPGA.fpga_vi_state != FpgaViState(2): # FPGA is not running
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
            config_orders += [build_order([(2, 8), (4, 8), (0, 16), (1600, 32)])] # SPI speed
            self.send_orders(config_orders)
    
    def send_orders(self, orders):
        """
        Puts orders in the FPGA input buffer and waits for until they are all passed.
        orders is a list of uint64s (see FPGA doc).
        """
        #print(orders)
        while len(orders) > 0:
            self.FPGA.orders_fifo.write(orders[:500], timeout_ms=10) # FIFO size is 1050 elements
            orders = orders[500:]
            irq_status = self.FPGA.wait_on_irqs(0, timeout_ms=10)
            if irq_status.timed_out:
                raise RuntimeError('Timeout while passing orders')

######################################
###            DC DACS
######################################
    
    def update_DAC(self, values):
        """
        Updates one or several DAC output(s) to the indicated value(s).
        values is a dict of type {DAC_key1:val1, DAC_key2:val2}.
        Direct jump to value for CIR7.
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
                if val < ll or val > ul:
                    raise ValueError('DAC value is out of limits')
                else:
                    orders.append(build_order([(1, 8), (0, 24), (pan, 8), (chan, 8), (float_to_uint16(val), 16)]))
        self.send_orders(orders)
        
    def read_current_values(self):
        """
        Asks the FPGA for the last values sent to each DAC output.
        Returns a dict of type {DAC_key1:val1, DAC_key2:val2} and the content of the CIR7 memory.
        """
        self.FPGA.acknowledge_irqs([3])
        self.send_orders([build_order([(3, 8), (0, 56)])])
        self.FPGA.wait_on_irqs([3], timeout_ms=10)
        irq_status = self.FPGA.wait_on_irqs(3, timeout_ms=10)
        if irq_status.timed_out:
            raise RuntimeError('Timeout while reading DAC values')
        else:
            DAC_val = self.FPGA.DAC_values.read()
            CIR7_val = self.FPGA.CIR7_values.read()
            CIR7_val = [np.round(uint16_to_float(val), 4) for val in CIR7_val]
            self.FPGA.acknowledge_irqs([3])
            outp_dict = {}
            for key in self.DAC_dict.keys():
                DAC_id = self.DAC_dict[key]['panel'] * 8 + self.DAC_dict[key]['channel']
                outp_dict[key] = np.round(uint16_to_float(DAC_val[DAC_id]),4)
            return outp_dict, CIR7_val


######################################
###             SPI
######################################
        
    def SPI_read(self, address, Nbytes):
        """
        Read Nbytes from SPI.
        """
        read_order = build_order([(2, 8), (3, 8), (0, 24), (address, 8), (Nbytes, 16)])
        self.send_orders([read_order]) # read Nbytes from SPI
        SPI_content = self.SPI_data.read(Nbytes, timeout_ms=2000) # read from memory content
        if SPI_content.elements_remaining > 0:
            raise RuntimeError('Found extra elements in SPI fifo')
        elif [d >> 8 for d in SPI_content.data] != list(range(address, address + Nbytes)): # check addresses
            raise RuntimeError('SPI content was incorrect')
        return [d % 256 for d in SPI_content.data] # remove address flag, only return data
        
    def SPI_write(self, address, data):
        """
        Write one or several byte(s) of data to the SPI.
        """
        if len(data) == 0:
            data = [data]
        Nbytes = len(data)
        orders = []
        for (i, di) in enumerate(data):
            orders += [build_order([(2, 8), (1, 8), (0, 32), (address + i, 8), (di, 8)])] # place in memory
        orders += [build_order([(2, 8), (2, 8), (0, 24), (address, 8), (Nbytes, 16)])] # write order
        self.send_orders(orders)

    def dump_all(self):
        """
        Prints whole SPI content.
        """
        print("Reg00 to Reg63", self.SPI_read(0x00, 64))
        print("Min&Max addresses of loop counter", self.SPI_read(0x40, 2))
        print("SRAM address, data & ctrl", self.SPI_read(0x50, 3))
        print("Output buffers configuration", self.SPI_read(0x70, 4))
        print("BIST clear", self.SPI_read(0x80, 1))
        print("Modes and soft-reset", self.SPI_read(0xFE, 2))
        print("Counter / Register / SRAM indexes", self.SPI_read(0xA0, 3))
        print("Selected", self.SPI_read(0x90, 8))
        
######################################
###     PANEL4 AUTO-REFRESH
######################################

    def panel4_config(self, start_ind=0, stop_ind=63, ticks_per_address=800, ticks_per_bit=10):
        """
        Configures the refresh rate & communication speed of panel 4.
        """
        if ticks_per_bit * 39 * 2 > ticks_per_address:
            raise ValueError('Refresh rate is too low')
        orders = [build_order([(4, 8), (3, 8), (0, 8), (0, 8), (ticks_per_bit, 32)])]
        orders += [build_order([(4, 8), (3, 8), (0, 8), (1, 8), (ticks_per_address, 32)])]
        orders += [build_order([(4, 8), (5, 8), (0, 32), (stop_ind, 8), (start_ind, 8)])]
        orders += [build_order([(4, 8), (2, 8), (0, 48)])] # start if not already running
        self.send_orders(orders)
        
    def panel4_set_value(self, values_dict):
        """
        Updates one or several programmed value(s) in CIR7.
        """
        orders = []
        for key, val in values_dict.items():
            if not isinstance(key, int) or key not in range(64):
                raise KeyError('Panel 4 key must be an integer from 0 to 63')
            else:
                orders += [build_order([(4, 8), (4, 8), (0, 24), (key, 8), (float_to_uint16(val), 16)])]
        self.send_orders(orders)
        
    def panel4_stop(self):
        """
        Stops panel 4 refreshing
        """
        stop_order = build_order([(4, 8), (1, 8), (0, 48)])
        self.send_orders([stop_order])
        
    def panel4_start(self):
        """
        Starts panel 4 refreshing
        """
        stop_order = build_order([(4, 8), (2, 8), (0, 48)])
        self.send_orders([stop_order])
        
######################################
###     FAST-SEQUENCE
######################################

    def config_seq(self, slots, us_per_DAC=1000, trig_reset_states=[True]*10, start_after=False, start_index=0):
        """
        Configures a fast sequence.
        slots is a dict of type {num:Slot_object}. (missing slots are not updated, use End slots to erase a previous sequence)
        """
        stop_order = build_order([(5, 8), (1, 8), (0, 48)])
        trig_reset_code = s = sum([b << i for (i, b) in enumerate(trig_reset_states)])
        trig_reset_order = build_order([(5, 8), (10, 8), (0, 38), (trig_reset_code, 10)])
        us_DAC_order = build_order([(5, 8), (8, 8), (0, 16), (us_per_DAC, 32)])
        orders = [stop_order, trig_reset_order, us_DAC_order]
        # slots
        for (i, slot) in slots.items():
            if i not in range(0, 1023):
                raise KeyError('Unknown slot id')
            else:
                orders += [build_order([(5, 8), (4, 8), (i, 8), (slot.gen_order(), 40)])]
                print('{}:{:10X}'.format(i, slot.gen_order()))
        if start_after:
            start_order = build_order([(5, 8), (2, 8), (0, 38), (start_index, 10)])
            orders.append(start_order)
        self.send_orders(orders)

    def update_slots(self, slots, start_after=False, start_index=0):
        """
        Modifies several slots in the sequence.
        slots is a dict of type {num:Slot_object}
        start_after indicates whether to run the sequence or not after the modification.
        e.g. update_slots({5:Wait(1.0, '10us'), 7:End()})
        """
        stop_order = build_order([(5, 8), (1, 8), (0, 48)])
        orders = [stop_order]
        for (i, slot) in slots.items():
            if i not in range(0, 1023):
                raise KeyError('Unknown slot id')
            else:
                orders += [build_order([(5, 8), (4, 8), (i, 8), (slot.gen_order(), 40)])]
        if start_after:
            start_order = build_order([(5, 8), (2, 8), (0, 38), (start_index, 10)])
            orders.append(start_order)
        self.send_orders(orders)

    def update_slots_lowlevel(self, seq_orders, start_after=False, start_index=0):
        """
        Modifies several slots in the sequence.
        seq_orders is a list of uint64s.
        start_after indicates whether to run the sequence or not after the modification.
        """
        stop_order = build_order([(5, 8), (1, 8), (0, 48)])
        if start_after:
            start_order = build_order([(5, 8), (2, 8), (0, 38), (start_index, 10)])
            self.send_orders([stop_order, *seq_orders, start_order])
        else:
            self.send_orders([stop_order, *seq_orders])

    def start_seq(self, start_ind=0):
        """
        Runs the programmed sequence from the indicated row.
        """
        self.send_orders([build_order([(5, 8), (2, 8), (0, 38), (start_ind, 10)])])

    def stop_seq(self):
        """
        Stops the sequence execution.
        """
        self.send_orders([build_order([(5, 8), (1, 8), (0, 48)])])

    def restart_seq(self, restart_ADC=False):
        """
        Stops and runs the programmed sequence from the last starting index.
        restart_ADC allows to send the ADC start order immediately before (for timing performances)
        """
        if restart_ADC:
            orders = [build_order([(6, 8), (2, 8), (0, 48)])] # ADC start (wait for trigger)
        else:
            orders = []
        orders.append(build_order([(5, 8), (3, 8), (0, 48)])) # sequence restart
        self.send_orders(orders)
        
if __name__=="__main__":
    
    bitfile_path = """C:/Users/manip.batm/Documents/FPGA_Batch_2_0_5_CryoC009/Labview_2019/FPGA Bitfiles/FPGA_CIR7_main.lvbitx"""
    ip_address = "192.168.1.21"
    
    instr = CIR7_driver(ip_address, bitfile_path, DAC_dict={})
    print(instr.read_current_values())
    
    ans1 = instr.SPI_read(0x70, 4)
    print('First SPI read : ')
    for (i, val) in enumerate(ans1):
        print(f"{0x70+i:02x}\t{val:02x}")
    
    instr.SPI_write(0x70, [0x81, 0x82, 0x03, 0x84])
    print('Writing to SPI ...')
    time.sleep(0.1)
    ans2 = instr.SPI_read(0x70, 4)
    print('Second SPI read : ')
    for (i, val) in enumerate(ans1):
        print(f"{0x70+i:02x}\t{val:02x}")
    
    instr.FPGA.close()
    