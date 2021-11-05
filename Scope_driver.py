# -*- coding: utf-8 -*-
"""
Created on Fri Nov  5 10:36:55 2021

@author: Baptiste
"""

from numpy.lib.utils import source
import pyvisa
import pyvisa.resources
import time
import numpy as np

class ScopeDriver():
    """
    Driver for R&S RTA4004. Adapted from Matthieu D.
    """

######################################
###      OPEN COMMUNICATION
######################################

    def __init__(self, SCOPE_VISA):
        rm = pyvisa.ResourceManager()
        self.instr = rm.open_resource(
            SCOPE_VISA,
            write_termination="\n",
            read_termination="\n",
        )
        # Waveform transfer settings
        self.instr.write("FORM REAL")  # Set REAL data format
        self.instr.write("FORM:BORD LSBF") 
    
    def stop(self):
        self.instr.write("STOP")

    def check_errors(self):
        ans = self.instr.query("SYST:ERR:ALL?")
        print(ans)
        return ans

    def close(self):
        self.instr.close()

    def set_trigger(self, source="CH1", level=1., edge="POS"):
        self.instr.write("STOP")
        self.instr.write("TRIG:A:TYPE EDGE") # trig type
        self.instr.write(f"TRIG:A:SOUR {source}") # CH1, CH2, CH3, CH4, EXT
        self.instr.write(f"TRIG:A:LEV1 {level}") # value in Volts
        self.instr.write(f"TRIG:A:EDGE:SLOP {edge}") # POS, NEG, EITH
    
    def set_vertical(self, chan_id=1, active=True, volts_per_div=1., offset_volt=0.):
        self.instr.write(f"CHAN{chan_id}:STAT {'ON' if active else 'OFF'}")
        self.instr.write(f"CHAN{chan_id}:TYPE HRES")
        self.instr.write(f"CHAN{chan_id}:COUP DCLimit") # coupling DC 1Mohms
        self.instr.write(f"CHAN{chan_id}:SCAL {volts_per_div}")
        self.instr.write(f"CHAN{chan_id}:OFFS {offset_volt}")
        self.instr.write(f"CHAN{chan_id}:POS 0")    
        
    def set_horizontal(self, sec_per_div=100e-6, sec_offset=0.):
        self.instr.write(f"TIM:SCAL {sec_per_div}")
        self.instr.write(f"TIM:POS {sec_offset}")

    def start_acq(self, mode="NORM", wfm_count=1):
        self.instr.write("STOP")
        self.instr.write(f"TRIG:A:MODE {mode}") # NORM, AUTO
        if mode == "AUTO" or wfm_count == 0:
            self.instr.write("RUNC") # run continous
        else:
            self.instr.write(f"ACQ:NSIN:COUN {wfm_count}") # acquire N traces
            self.instr.write("RUNS") # run single

    def get_traces(self, active_channels=["CH1", "CH2", "CH3", "CH4"]):
        header = self.instr.query("CHAN:DATA:HEAD?")
        start, stop, n, _ = header.split(",")
        t = np.linspace(float(start), float(stop), int(n))
        traces = {}
        for chan_id in active_channels:
            i = ["", "CH1", "CH2", "CH3", "CH4"].index(chan_id)
            old = self.instr.timeout
            self.instr.timeout = 5000 # special timeout
            traces[chan_id] = self.instr.query_binary_values(
                f"CHAN{i}:DATA?", container=np.ndarray
            )
            self.instr.timeout = old # reset timeout
        return t, traces

if __name__=="__main__":

    SCOPE_VISA = "TCPIP::192.168.1.46::INSTR"
    scope = ScopeDriver(SCOPE_VISA)
    _ = scope.check_errors()
    
    # config
    scope.stop()
    scope.set_trigger(source="EXT", level=1., edge="NEG")
    for k in range(1, 5):
        scope.set_vertical(chan_id=k, active=True, volts_per_div=0.5, offset_volt=0.)
    scope.set_horizontal(sec_per_div=100e-6, sec_offset=0.)

    # arm trigger
    scope.start_acq(mode="NORM", wfm_count=1)
    time.sleep(0.5)
    _ = scope.check_errors()

    # simulate trig
    scope.instr.write("*TRG")
    time.sleep(0.5)
    _ = scope.check_errors()

    # get data
    t, traces = scope.get_traces(active_channels=["CH1", "CH2", "CH3", "CH4"])
    print(t.shape)
    if len(traces.keys()) > 0:
        print([tr.shape for tr in traces.values()])

    # close instrument
    scope.close()