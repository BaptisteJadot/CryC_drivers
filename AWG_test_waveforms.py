# -*- coding: utf-8 -*-
"""
Created on Fri Nov  5 21:59:39 2021

@author: manip.batm
"""

from AWG_driver import AWGDriver, build_waveforms
import numpy as np

def check_sequence(expected_values, Veven, Vodd, CLK, current_index=63):
    addr_change = (np.where(np.diff(CLK) > 0.)[0] + 1).tolist() # indexes of rising CLK edge
    even_addr = current_index % 2 == 0
    for i in range(addr_change[0], len(CLK)):
        if i in addr_change:
            even_addr = not even_addr
            current_val = Veven[i] if even_addr else Vodd[i]
        if even_addr and Veven[i] != current_val:
            print(f"Error at index {i}: even address, data {Veven[i]}")
            return False
        elif not(even_addr) and Vodd[i] != current_val:
            print(f"Error at index {i}: odd address, data {Vodd[i]}")
            return False
    return True

if __name__=="__main__":
    import matplotlib.pyplot as plt
    
    clk_rate_MHz = 0.05
    addr_list = [14, 15, 16, 17]
    values = [0.25, 0.5, 0.75, 1.]
    clk_rate_MHz = 0.05
    data_to_clk_us = 5.
    
    Veven, Vodd, CLK = build_waveforms(values, clk_rate_MHz, data_to_clk_us, current_index=addr_list[-1], current_val=values[-1])
    print(check_sequence(values, Veven, Vodd, CLK, current_index=addr_list[-1]))

    plt.figure(333);plt.clf();plt.ion()
    x = np.arange(len(CLK)) / AWG_SAMPLING_RATE_MHz
    plt.scatter(x, Veven)
    plt.scatter(x, Vodd)
    plt.plot(x, CLK / 2., 'k')