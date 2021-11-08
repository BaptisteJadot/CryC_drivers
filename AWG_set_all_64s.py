# -*- coding: utf-8 -*-
"""
Created on Fri Nov  5 21:59:39 2021

@author: manip.batm
"""

from AWG_driver import AWGDriver
import numpy as np

AWG_SAMPLING_RATE = 1.2e9
VACT_CHAN = 1
VEVEN_CHAN = 2
VODD_CHAN = 3
CLK_CHAN = 4

def build_waveforms(clk_rate=50e3, clk_offset=0., values=[0.5]*64, start_index=0):
    N = int(np.round(AWG_SAMPLING_RATE / clk_rate))
    Veven, Vodd, Vact, CLK = np.zeros((N*64, 1), dtype=np.float)
    Vact = np.tile