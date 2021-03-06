# -*- coding: utf-8 -*-
"""
Created on Tue Aug 24 09:33:07 2021

@author: Baptiste
"""

import numpy as np
from nifpga import Session, FpgaViState
from nifpga.status import RpcConnectionErrorError, FpgaBusyInteractiveError, FifoTimeoutError
from FPGA_utils import build_order, uint16_to_float, float_to_uint16
import Fastseq_elmts as fs
import time

class CIR7Driver():
    """
    Main driver for CIR7 circuit v2.0
    """

######################################
###      FPGA COMMUNICATION
######################################

    def __init__(self, ip_address, bitfile_path, DAC_dict={}):
        """
        Open communication and finds the VI objects to control.
        DAC_dict is a dictionnary of each DAC output (with fields 'panel', 'channel', 'lower_limit' and 'upper_limit').
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
            self.FPGA.SPI_fifo = self.FPGA.fifos['SPI data']
            self.FPGA.SPI_fifo.configure(512) # configure depth
            self.FPGA.SPI_fifo.start()
            # ADC data fifo
            self.FPGA.ADC_fifo = self.FPGA.fifos['ADC data']
            self.FPGA.ADC_fifo.configure(10000) # configure depth
            self.FPGA.ADC_fifo.start()
            
            # configure communication speed
            SPI_ticks_per_bit = 4000
            config_orders = []
            config_orders += [build_order([(2, 8), (4, 8), (0, 16), (SPI_ticks_per_bit, 32)])] # SPI speed
            config_orders += [build_order([(5, 8), (7, 8), (0, 16), (int(np.ceil(SPI_ticks_per_bit*16/80)), 32)])] # us per SPI byte in fastseq
            self.send_orders(config_orders)
    
    def send_orders(self, orders):
        """
        Put orders in the FPGA input buffer and waits for until they are all passed.
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
        Update one or several DAC output(s) to the indicated value(s).
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
        Ask the FPGA for the last values sent to each DAC output.
        Return a dict of type {DAC_key1:val1, DAC_key2:val2} and the content of the CIR7 memory.
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
        # print(f'Reading {Nbytes} bytes from {address:02X}')
        SPI_content = self.FPGA.SPI_fifo.read(Nbytes, timeout_ms=2000) # read from memory content
        if SPI_content.elements_remaining > 0:
            raise RuntimeError('Found extra elements in SPI fifo')
        elif [d >> 8 for d in SPI_content.data] != list(range(address, address + Nbytes)): # check addresses
            raise RuntimeError('SPI content was incorrect')
        return [d % 256 for d in SPI_content.data] # remove address flag, only return data
    
    def SPI_empty_buffer(self):
        """
        Return the content of the SPI output fifo.
        """
        SPI_content = self.FPGA.SPI_fifo.read(0, timeout_ms=10) # first read to get number of elements
        SPI_content = self.FPGA.SPI_fifo.read(SPI_content.elements_remaining, timeout_ms=2000) # read from memory content
        return [(d >> 8, d % 256) for d in SPI_content.data] # list of (addr, data)
        
    def SPI_write(self, address, data):
        """
        Write one or several byte(s) of data to the SPI.
        """
        try:
            iter(data)
        except TypeError:
            data = [data]
        Nbytes = len(data)
        orders = []
        for (i, di) in enumerate(data):
            orders += [build_order([(2, 8), (1, 8), (0, 32), (address + i, 8), (di, 8)])] # place in memory
        orders += [build_order([(2, 8), (2, 8), (0, 24), (address, 8), (Nbytes, 16)])] # write order
        # print(f'Writing {[hex(d) for d in data]} to {address:02X}')
        self.send_orders(orders)

    def SPI_dump_all(self, output_to_console=True):
        """
        Print whole SPI content.
        """
        out_str = f'Reg00 to Reg63, {self.SPI_read(0x00, 64)}\n'
        out_str += f'Min&Max addresses of loop counter, {self.SPI_read(0x40, 2)}\n'
        out_str += f'SRAM address, data & ctrl, {self.SPI_read(0x50, 3)}\n'
        out_str += f'Output buffers configuration, {self.SPI_read(0x70, 4)}\n'
        out_str += f'BIST clear, {self.SPI_read(0x80, 1)}\n'
        out_str += f'Modes and soft-reset, {self.SPI_read(0xFE, 2)}\n'
        out_str += f'Counter / Register / SRAM indexes, {self.SPI_read(0xA0, 3)}\n'
        out_str += f'Selected, {self.SPI_read(0x90, 8)}'
        if output_to_console:
            print(out_str)
        return out_str

    def set_mode(self, mode='addr', **kwargs):
        """
        Set CIR7's addressing mode.
        Authorized uses : 
            set_mode('addr')
            set_mode('counter')
            set_mode('lmt_counter', start=0, stop=63)
            set_mode('register', values=[0, 1, 2])
            set_mode('sram', values=[0, 1, 2])
            set_mode('direct')
        """
        if mode == 'addr':
            SPI_code = 0x00
            counter_control = False

        elif mode == 'counter':
            SPI_code = 0x00
            counter_control = True

        elif mode == 'lmt_counter':
            if 'start' not in kwargs.keys() or 'stop' not in kwargs.keys():
                raise KeyError('start/stop parameter not found')
            else:
                self.SPI_write(0x40, [kwargs['start'], kwargs['stop']]) # update bounds
                SPI_code = 0 | (1 << 4)
                counter_control = True

        elif mode == 'register':
            if 'values' not in kwargs.keys():
                raise KeyError('values parameter not found')
            else:
                self.SPI_write(0x00, kwargs['values']) # update registers
                SPI_code = 1 | (1 << 4)
                counter_control = True

        elif mode == 'sram':
            if 'values' not in kwargs.keys():
                raise KeyError('values parameter not found')
            else:
                self.SPI_sram_write(0x00, kwargs['values'], sel_mem='both') # update SRAM
                SPI_code = 2 | (1 << 4)
                counter_control = True

        elif mode == 'direct':
            SPI_code = 1 << 5
            counter_control = False
        
        else:
            raise KeyError('Unknown mode')
        
        mode_mask = 0b00110011
        prev_code = self.SPI_read(0xFE, 1)[0] # read to keep clock part
        new_code = (SPI_code & mode_mask) | (prev_code & ~mode_mask)
        self.SPI_write(0xFE, new_code)
        if not counter_control:
            raise RuntimeError('Cannot use address mode')
        self.update_DAC({'MODE_ROT':1.8 if counter_control else 0.})

    def set_clk(self, int_clk=False, osc_vco=0., two_cycles=False, add_delay=False):
        """
        Set the clock.        
        """     
        SPI_code = self.SPI_read(0xFE, 1)[0] # read to keep mode part
        SPI_code &= ~0b11001000 # clear clock part
        SPI_code |= int_clk << 3 | two_cycles << 6 | add_delay << 7 # replace clk part
        self.SPI_write(0xFE, SPI_code)
        self.update_DAC({'OSC_VCO':osc_vco})

    def set_output(self, mux_mat=False, line0=None, line1=None, column0=None, column1=None, CC_code=0):
        """
        Select the lines/column to observe.
        """
        SPI_codes = []
        for i in [line0, line1, column0, column1]:
            if i is None:
                SPI_codes += [0x00] # off
            else:
                SPI_codes += [0x80 | i%32 | (CC_code<<4)] # CEN ON, CC1/2 OFF, ADDRESS 5 LSBits
        self.SPI_write(0x70, SPI_codes)
        self.update_DAC({'SEL_MAT':1.8 if mux_mat else 0.})

    def reset(self):
        """
        Perform a hard and a soft reset.
        """        
        # hard reset   
        self.update_DAC({'OSC_VCO':0.46}) # 100MHz
        self.update_DAC({'RESETN':0.})
        time.sleep(0.1)
        self.update_DAC({'RESETN':1.8})
        time.sleep(0.1)
        self.update_DAC({'OSC_VCO':0.}) # disable
        
        ## soft reset with EXT CLK
        # # program 10 clocks in fastseq
        reset_seq = []
        # reset_seq += [fs.Trig_out(trig=[False]*4)]
        reset_seq += [fs.Panel4(address=0, clk=False)]
        reset_seq += [fs.Panel4(address=0, clk=True)]
        reset_seq += [fs.JumpFor(target=0, count=10)]
        reset_seq += [fs.End()]
        self.config_seq(slots={i:s for i,s in enumerate(reset_seq)}, us_per_DAC=10) # 50kHz clock
        self.SPI_write(0xFF, 0) 
        time.sleep(0.1)
        self.start_seq(start_ind=0)
        time.sleep(0.1)
        self.SPI_write(0xFF, 1)
        time.sleep(0.1)
        self.start_seq(start_ind=0)
        time.sleep(0.1)
        self.stop_seq()
        
        ## soft reset with INT CLK
        # self.stop_seq()
        # self.update_DAC({'OSC_VCO':0.}) # disable
        # self.SPI_write(0xFF, 0)
        # self.set_clk(int_clk=True, osc_vco=0.)
        # self.update_DAC({'OSC_VCO':0.46}) # 100MHz
        # time.sleep(0.1)
        # self.update_DAC({'OSC_VCO':0.}) # disable
        # self.SPI_write(0xFF, 1)
        # self.update_DAC({'OSC_VCO':0.46}) # 100MHz
        # time.sleep(0.1)
        # self.update_DAC({'OSC_VCO':0.}) # disable
        # self.set_clk(int_clk=False, osc_vco=0.) # 0MHz
        
    def reset_with_AWG(self, awg):
        """
        Perform a hard and a soft reset.
        """
        # hard reset   
        self.update_DAC({'OSC_VCO':0.46}) # 100MHz
        self.update_DAC({'RESETN':0.})
        time.sleep(0.1)
        self.update_DAC({'RESETN':1.8})
        time.sleep(0.1)
        self.update_DAC({'OSC_VCO':0.}) # disable
        
        ## soft reset with EXT CLK
        # stop AWG
        awg.stop()
        CLK = np.array([0., 0.] + [-0.9, 0.9] * 5 + [0., 0.])
        V_empty = np.zeros_like(CLK)
        awg.send_and_place_wf(channel=1, wf=V_empty)
        awg.send_and_place_wf(channel=2, wf=V_empty)
        awg.send_and_place_wf(channel=3, wf=CLK)
        awg.send_and_place_wf(channel=4, wf=V_empty)
        awg.set_sampling_rate(10e6)
        # run
        awg.wait_opc()
        awg.run()
        awg.set_all_outputs("ON")
        awg.wait_opc()
        
        # send soft reset
        self.SPI_write(0xFF, 0) 
        time.sleep(0.1)
        # send soft CLK
        awg.force_trig()
        # disable soft reset
        self.SPI_write(0xFF, 1) 
        time.sleep(0.1)
        # send soft CLK
        awg.force_trig()
        time.sleep(0.1)
        awg.stop()

    def SPI_sram_write(self, addr, data, sel_mem='both'):
        """
        Write one or several byte(s) into the SRAM memories.
        sel_mem is 'ctl', 'obs' or 'both'.
        """
        try:
            iter(data)
        except TypeError:
            data = [data]
            
        # SRAM COMM NEED TRIG
        seq = []
        seq += [fs.Panel4(clk=False, even_value=0., odd_value=0.)]
        seq += [fs.Panel4(clk=True, even_value=0., odd_value=0.)]
        seq += [fs.JumpFor(target=0, count=0)] # play infinitely
        seq += [fs.End()]
        self.config_seq(slots={i:s for (i,s) in enumerate(seq)}, 
                            us_per_DAC=10, 
                            trig_reset_states=[True]*10, 
                            start_after=True, 
                            start_index=0)
        
        # SEND SRAM VALUES
        sram_ctrl = {'both':0x00, 'ctl':0x02, 'obs':0x01}[sel_mem] # WEN LOW, CSN LOW to activate
        self.SPI_write(0x52, [0x07])
        
        for i, d in enumerate(data):
            # SRAM address, data, ctrl
            self.SPI_write(0x50, [addr+i, d, sram_ctrl])
            self.SPI_write(0x52, [0x07])
            
        self.SPI_write(0x50, [0x00])
        # STOP TRIG
        self.stop_seq()
        

    def SPI_sram_read(self, addr, Nbytes, sel_mem='both'):
        """
        Read one or several byte(s) from the SRAM memories.
        sel_mem is 'ctl', 'obs' or 'both'.
        """
        # SRAM COMM NEED TRIG
        seq = []
        seq += [fs.Panel4(clk=False, even_value=0., odd_value=0.)]
        seq += [fs.Panel4(clk=True, even_value=0., odd_value=0.)]
        seq += [fs.JumpFor(target=0, count=0)] # play infinitely
        seq += [fs.End()]
        self.config_seq(slots={i:s for (i,s) in enumerate(seq)}, 
                            us_per_DAC=10, 
                            trig_reset_states=[True]*10, 
                            start_after=True, 
                            start_index=0)
        
        # RECEIVE SRAM VALUES
        sram_ctrl = {'both':0x04, 'ctl':0x06, 'obs':0x05}[sel_mem] # WEN HIGH, CSN LOW/HIGH
        self.SPI_write(0x52, sram_ctrl) # set ctrl
        ctl_content = []
        obs_content = []
        for i in range(Nbytes):
            self.SPI_write(0x50, addr + i) # set address
            r = self.SPI_read(0x60, 2) # read two bytes
            ctl_content.append(r[0])
            obs_content.append(r[1])
        self.SPI_write(0x52, 0x07) # reset ctrl
        
        # STOP TRIG
        self.stop_seq()
        return ctl_content, obs_content


######################################
###     PANEL4 AUTO-REFRESH
######################################

    # def panel4_config(self, start_ind=0, stop_ind=63, ticks_per_address=800, ticks_per_bit=10):
    #     """
    #     Configures the refresh rate & communication speed of panel 4.
    #     """
    #     if ticks_per_bit * 39 * 2 > ticks_per_address:
    #         raise ValueError('Refresh rate is too low')
    #     orders = [build_order([(4, 8), (3, 8), (0, 8), (0, 8), (ticks_per_bit, 32)])]
    #     orders += [build_order([(4, 8), (3, 8), (0, 8), (1, 8), (ticks_per_address, 32)])]
    #     orders += [build_order([(4, 8), (5, 8), (0, 32), (stop_ind, 8), (start_ind, 8)])]
    #     orders += [build_order([(4, 8), (2, 8), (0, 48)])] # start if not already running
    #     self.send_orders(orders)
        
    # def panel4_set_value(self, values_dict):
    #     """
    #     Updates one or several programmed value(s) in CIR7.
    #     """
    #     orders = []
    #     for key, val in values_dict.items():
    #         if not isinstance(key, int) or key not in range(64):
    #             raise KeyError('Panel 4 key must be an integer from 0 to 63')
    #         else:
    #             orders += [build_order([(4, 8), (4, 8), (0, 24), (key, 8), (float_to_uint16(val), 16)])]
    #     self.send_orders(orders)
        
    # def panel4_stop(self):
    #     """
    #     Stops panel 4 refreshing
    #     """
    #     stop_order = build_order([(4, 8), (1, 8), (0, 48)])
    #     self.send_orders([stop_order])
        
    # def panel4_start(self):
    #     """
    #     Starts panel 4 refreshing
    #     """
    #     stop_order = build_order([(4, 8), (2, 8), (0, 48)])
    #     self.send_orders([stop_order])
        
        
######################################
###     FAST-SEQUENCE
######################################

    def config_seq(self, slots, us_per_DAC=1000, trig_reset_states=[True]*10, start_after=False, start_index=0):
        """
        Configures a fast sequence.
        slots is a dict of type {num:Slot_object}. (missing slots are not updated, use End slots to erase a previous sequence)
        """
        stop_order = build_order([(5, 8), (1, 8), (0, 48)])
        trig_reset_code = sum([b << i for (i, b) in enumerate(trig_reset_states)])
        trig_reset_order = build_order([(5, 8), (10, 8), (0, 38), (trig_reset_code, 10)])
        us_DAC_order = build_order([(5, 8), (8, 8), (0, 16), (us_per_DAC, 32)])
        # us_per_SPI_order = build_order([(5, 8), (7, 8), (0, 16), (150, 32)])
        orders = [stop_order, trig_reset_order, us_DAC_order]
        # slots
        for (i, slot) in slots.items():
            if i not in range(0, 4096):
                raise KeyError('Slot id is not valid')
            else:
                orders += [build_order([(5, 8), (0x8, 4), (i, 12), (slot.gen_order(), 40)])]
                # print('{}:{:010X}'.format(i, slot.gen_order()))
        if start_after:
            start_order = build_order([(5, 8), (2, 8), (0, 36), (start_index, 12)])
            orders.append(start_order)
        # print(orders)
        self.send_orders(orders)

    def update_slots(self, slots, stop_before=True, start_after=False, start_index=0):
        """
        Modifies several slots in the sequence.
        slots is a dict of type {num:Slot_object}
        stop_before indicates whether to stop the sequence or not before the modification.
        start_after indicates whether to run the sequence or not after the modification.
        e.g. update_slots({5:Wait(1.0, '10us'), 7:End()})
        """
        orders = []
        if stop_before:
            stop_order = build_order([(5, 8), (1, 8), (0, 48)])
            orders = [stop_order]
            
        for (i, slot) in slots.items():
            if i not in range(0, 4096):
                raise KeyError('Unknown slot id')
            else:
                orders += [build_order([(5, 8), (0x8, 4), (i, 12), (slot.gen_order(), 40)])]
                
        if start_after:
            start_order = build_order([(5, 8), (2, 8), (0, 36), (start_index, 12)])
            orders.append(start_order)
            
        self.send_orders(orders)

    def update_slots_lowlevel(self, seq_orders, stop_before=True, start_after=False, start_index=0):
        """
        Modifies several slots in the sequence.
        seq_orders is a list of uint64s.
        stop_before indicates whether to stop the sequence or not before the modification.
        start_after indicates whether to run the sequence or not after the modification.
        """
        orders = []
        if stop_before:
            stop_order = build_order([(5, 8), (1, 8), (0, 48)])
            orders = [stop_order]
        orders += seq_orders                
        if start_after:
            start_order = build_order([(5, 8), (2, 8), (0, 36), (start_index, 12)])
            orders += [start_order]
        self.send_orders(orders)

    def start_seq(self, start_ind=0):
        """
        Runs the programmed sequence from the indicated row.
        """
        self.send_orders([build_order([(5, 8), (2, 8), (0, 36), (start_ind, 12)])])

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

    def get_ADC_data(self, Npts=None, timeout_ms=2000):
        """
        Reads Npts (all if None) from ADC output buffer.
        """
        if Npts is None:
            ADC_content = self.FPGA.ADC_fifo.read(0, timeout_ms=timeout_ms) # empty read to get data size
            Npts = ADC_content.elements_remaining
        ADC_content = self.FPGA.ADC_fifo.read(Npts, timeout_ms=timeout_ms) # read Npts
        return ADC_content.data
        
if __name__=="__main__":
    from CIR7_config import DAC_dict
    
    bitfile_path = """C:/Users/manip.batm/Documents/FPGA_Batch_2_0_5_CryoC009/Labview_2019/FPGA Bitfiles/FPGA_CIR7_main_analog_addr.lvbitx"""
    ip_address = "192.168.1.21"
    
    instr = CIR7Driver(ip_address, bitfile_path, DAC_dict)
    print(instr.read_current_values()[0])
    
    # seq = []
    # seq += [fs.Trig_out(trig=[False]*4)]
    # seq += [fs.Panel4(address=0, clk=False)]
    # seq += [fs.Panel4(address=0, clk=True)]
    # seq += [fs.JumpFor(target=0, count=10)]
    # seq += [fs.End()]
    # # seq = [fs.End()]*4096
    
    # for i, s in enumerate(seq):
    #     print(f"{i}: {s}")
    # instr.config_seq({i:s for i,s in enumerate(seq)}, us_per_DAC=10, start_after=True)
    
    instr.reset()
    
    ans1 = instr.SPI_read(0x70, 4)
    print('First SPI read : ')
    for (i, val) in enumerate(ans1):
        print(f"{0x70+i:02x}\t{val:02x}")
        
    ans3 = instr.SPI_read(0x00, 64)
    print('Registers : ')
    for (i, val) in enumerate(ans3):
        print(f"{i:02x}\t{val:02x}")
        
    ans4 = instr.SPI_read(0xFE, 2)
    print(f"Mode {ans4[0]:02x}\tReset {ans4[1]:02x}")
    
    instr.SPI_write(0x70, [0x81, 0x82, 0x03, 0x84])
    print('Writing to SPI ...')
    time.sleep(0.1)
    ans2 = instr.SPI_read(0x70, 4)
    print('Second SPI read : ')
    for (i, val) in enumerate(ans1):
        print(f"{0x70+i:02x}\t{val:02x}")
        
    ans4 = instr.SPI_read(0xFE, 2)
    print(f"Mode {ans4[0]:02x}\tReset {ans4[1]:02x}")
        
    _ = instr.SPI_dump_all()
    
    instr.FPGA.close()
    