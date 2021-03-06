# -*- coding: utf-8 -*-
"""
Created on Fri Nov  5 14:52:34 2021

@author: manip.batm
"""

import pyvisa
import time
import numpy as np

AWG_SAMPLING_RATE_MHz = 1200

def build_clk(Nclk, clk_rate_MHz):
    # check arguments
    L0 = int(np.round(AWG_SAMPLING_RATE_MHz / (2 * clk_rate_MHz)))
    if L0 < 1:
        raise ValueError("CLK frequency is too high.")
    elif L0 * (2 * clk_rate_MHz) != AWG_SAMPLING_RATE_MHz:
        print(f"CLK frequency rounded to {AWG_SAMPLING_RATE_MHz / (2 * L0)} MHz.")
    
    # build
    CLK = [0., 0.] + [-1.2, 1.2] * Nclk + [0., 0.]
    CLK = np.tile(CLK, (L0, 1)).flatten("F")
    return CLK, L0
    
def build_waveforms(values=[0.5]*64, clk_rate_MHz=0.05, data_to_clk_us=5., current_index=63, current_val=0.):
    ## CLK
    CLK, L0 = build_clk(len(values), clk_rate_MHz)
    # addr_change = (np.where(np.diff(CLK) > 0.)[0] + 1).tolist() # indexes of rising CLK edge
    
    ## DATA - CLK DELAY
    Ldc = int(np.round(data_to_clk_us * AWG_SAMPLING_RATE_MHz))
    if abs(Ldc) >= 2 * L0:
        print("DATA to CLK time is greater than 1 CLK period")
    if Ldc != data_to_clk_us * AWG_SAMPLING_RATE_MHz:
        print(f"DATA to CLK time rounded to {Ldc / AWG_SAMPLING_RATE_MHz} us.")

    ## BUILD WAVEFORMS
    V0 = np.tile(values[::2], (4*L0, 1)).flatten("F")
    V1 = np.tile(values[1::2], (4*L0, 1)).flatten("F")
    if len(values) % 2 == 0:
        V0 = np.concatenate((np.array([V0[0]]*(3 * L0 - Ldc)), 
                         V0,
                         np.array([V0[-1]]*(L0 + Ldc)))) # add before and after
        V1 = np.concatenate((np.array([current_val]*(5 * L0 - Ldc)), 
                         V1[:-(2 * L0)], 
                         np.array([V1[-1]]*(L0 + Ldc)))) # add before and after
    else:
        V0 = np.concatenate((np.array([V0[0]]*(3 * L0 - Ldc)), 
                         V0[:-(2 * L0)],
                         np.array([V0[-1]]*(L0 + Ldc)))) # add before and after
        V1 = np.concatenate((np.array([current_val]*(5 * L0 - Ldc)), 
                         V1, 
                         np.array([V1[-1]]*(L0 + Ldc)))) # add before and after
    
    # correct for i0 parity
    if current_index % 2 != 0:
        Veven = np.array(V0)
        Vodd = np.array(V1)
    else:
        Vodd = np.array(V1)
        Veven = np.array(V0)

    return Veven, Vodd, CLK

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
            self.set_amplitude_and_offset(channel=c, amplitude=2.5, offset=0)

    def close(self):
        self.instr.close()

    def wait_opc(self):
        _ = self.instr.query("*OPC?")

    def stop(self):
        self.instr.write("AWGC:STOP")

    def run(self):
        self.instr.write("AWGC:RUN")
        
    def force_trig(self):
        self.instr.write("*TRG")

    def set_output(self, channel=1, state="OFF"):
        self.instr.write(f"OUTP{channel} {state}")
    
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
        self.instr.write_binary_values(f"WLIST:WAV:DATA \"{wfm_name}\", ", wf_uint16, datatype="h") # transfer binary data
        self.instr.write(f"SOUR{channel}:WAV \"{wfm_name}\"") # set waveform as active

if __name__=="__main__":
    
    # runfile("follow_selected_AWG.py")
    
    AWG_VISA = "TCPIP::192.168.1.12::4000::SOCKET" 
    awg = AWGDriver(AWG_VISA)
    
    # config
    # awg.set_sampling_rate(sampl_rate=1.2e9)
    awg.set_repetition_rate(rep_rate=1e6)
    
    sine = np.sin(np.linspace(0, 6, 601) * np.pi)
    awg.send_and_place_wf(channel=1, wf=sine, m1=None, m2=None)
    awg.send_and_place_wf(channel=2, wf=-sine, m1=None, m2=None)
    awg.send_and_place_wf(channel=3, wf=sine, m1=None, m2=None)
    awg.send_and_place_wf(channel=4, wf=-sine, m1=None, m2=None)
    
    # close instrument
    awg.close()