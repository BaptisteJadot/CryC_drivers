# -*- coding: utf-8 -*-
"""
Created on Fri Nov  5 14:52:34 2021

@author: manip.batm
"""

import pyvisa
import time
import numpy as np

class AWGDriver():
    """
    Driver for Tektronik 5014.
    """
    def __init__(self, AWG_VISA):
        rm = pyvisa.ResourceManager()
        self.instr = rm.open_resource(
            AWG_VISA,
            write_termination="\n",
            read_termination="\n",
        )
        self.chan_ampl = {}
        self.chan_offset = {}
        self.sampling_rate = None
        self.repetition_rate = None
        self.wf_length = None
        self.instr.write("AWGC:RMODE TRIG")
        for c in range(1, 5):
            self.set_amplitude_and_offset(channel=c, amplitude=2., offset=0)

    def close(self):
        self.instr.close()

    def wait_opc(self):
        _ = self.instr.query("*OPC?")

    def stop(self):
        self.instr.write("AWGC:STOP")

    def run(self):
        self.instr.write("AWGC:RUN")

    def set_output(self, channel=1, state="OFF"):
        self.instr.write(f"SOUR{channel}:OUTP {state}")
    
    def set_all_outputs(self, state="OFF"):
        for k in range(1, 5):
            self.set_output(k, state)
    
    def set_amplitude_and_offset(self, channel, amplitude=4.5, offset=0.):
        self.chan_ampl[channel] = amplitude
        self.chan_offset[channel] = offset
        self.instr.write(f"SOUR{channel}:VOLT:AMPL {amplitude}")
        self.instr.write(f"SOUR{channel}:VOLT:OFFS {offset}")
        
    def set_sampling_rate(self, sampl_rate=1.2e9):
        self.sampling_rate = sampl_rate
        if self.wf_length is not None:
            self.repetition_rate = self.sampling_rate / self.wf_length
        self.instr.write(f"SOUR:FREQ {sampl_rate}")
    
    def set_repetition_rate(self, rep_rate=1e6):
        self.repetition_rate = rep_rate
        if self.wf_length is not None:
            self.sampling_rate = self.repetition_rate * self.wf_length
        self.instr.write(f"AWGC:RRATE {rep_rate}")
        
    def clear_waveforms(self, name="ALL"):
        self.instr.write(f"WLIST:WAV:DEL {name}")
        
    def package_wf(self, channel, wf_float, m1, m2):
        ampl = self.chan_ampl[channel]
        offs = self.chan_offset[channel]
        wf_norm = (wf_float - offs) / ampl
        if np.any(np.abs(wf_norm) > 1.):
            raise ValueError(f"Amplitude is too large for channel {channel}")
        else:
            m1 = m1.astype(np.bool)
            m2 = m2.astype(np.bool)
            wf_uint16 = np.uint16(np.round(wf_norm * 8191 + 8191.5)
                                + np.round(16384 * m1) 
                                + np.round(32768 * m2))
            return wf_uint16
        
    def send_and_place_wf(self, channel, wf, m1=None, m2=None):
        # convert waveform to uint16
        if m1 is None:
            m1 = np.zeros(wf.shape, dtype=np.bool)
        if m2 is None:
            m2 = np.zeros(wf.shape, dtype=np.bool)
        wf_uint16 = self.package_wf(channel, wf, m1, m2)
        
        # update repetition rate
        self.wf_length = len(wf_uint16)
        if self.sampling_rate is not None:
            self.repetition_rate = self.sampling_rate / self.wf_length
        elif self.repetition_rate is not None:
            self.sampling_rate = self.repetition_rate / self.wf_length
            
        # transfer wf
        wfm_name = f"CH{channel}_wfm"
        self.instr.write(f"WLIST:WAV:DEL \"{wfm_name}\"") # delete waveform if existing
        self.instr.write(f"WLIST:WAV:NEW \"{wfm_name}\", {len(wf_uint16)}, INT") # create empty wf
        block_data = pyvisa.util.to_ieee_block(wf_uint16, datatype='h')
        self.instr.write_binary_values(f"WLIST:WAV:DATA \"{wfm_name}\", ", wf_uint16, datatype="h") # transfer binary data
        self.instr.write(f"SOUR{channel}:WAV \"{wfm_name}\"") # set waveform as active

if __name__=="__main__":

    AWG_VISA = "TCPIP::192.168.1.12::4000::SOCKET" 
    awg = AWGDriver(AWG_VISA)
    
    # config
    # awg.set_sampling_rate(sampl_rate=1.2e9)
    awg.set_repetition_rate(rep_rate=1e6)
    
    sine = np.sin(np.linspace(0, 6, 601) * np.pi)
    awg.send_and_place_wf(channel=1, wf=sine, m1=None, m2=None)

    # close instrument
    awg.close()