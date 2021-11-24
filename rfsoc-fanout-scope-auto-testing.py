"""Test the fanout outputs.

The 4 channels of a RTA4004 are expected to be connected to the on the 4 SMA on
the front panel of the fanout (from left to right).

"""
import os
from typing import cast
from time import sleep

import matplotlib.pyplot as plt
import numpy as np
import h5py
import pyvisa
import pyvisa.resources

# IP address of the RFSOC
RFSOC_IP = "192.168.0.10"

# VISA resource name of the oscilloscope (RTA4004)
SCOPE_VISA = "USB0::0x0AAD::0x01D6::102837::0::INSTR"

# From 1 to test only 4 DACs to 14 to test all dacs
TEST_DACS = 14

PATH = r"C:\Users\matthieu.dartiailh\Documents\Work\Neel\RFSOC\tests\fanout_data_20211104.h5"

# --- Utilities


def execute_sequence(rfsoc, seq):
    rfsoc.write(f"SEQ 0,{seq}")
    rfsoc.write("SEQ:STOP")
    rfsoc.write("SEQ:START")

    while True:
        resp = rfsoc.query("SEQ:END?")
        if resp == "1":
            break
        sleep(0.1)


def rfsoc_set_constant(rfsoc, value):
    # Put everything in a reasonable starting position.
    channels = list(range(1, 51))

    seq = ""
    # Send all DACs to 0
    for i, dac in enumerate(channels):
        is_final = (1 << 31) if i + 1 == len(channels) else 0
        seq += f"{0x1100},{(is_final + (dac << 16)) + value},"
    seq += "1,199,"

    # Send all ADC DACs to 0
    for i, dac in enumerate(range(1, 5)):
        is_final = (1 << 31) if i + 1 == 5 else 0
        seq += f"{0x1101},{(is_final + (dac << 16)) +value},"
    seq += "1,199,0,0"

    execute_sequence(rfsoc, seq)


def rfsoc_fanout_reset(rfsoc):
    # Put everything in a reasonable starting position.
    channels = list(range(1, 51))

    # Send all DACs to 0 and end all ADC DACs to 0
    rfsoc_set_constant(rfsoc, 0b0111111111111111)

    seq = ""
    # Connect all the fanout outputs
    mask = 0
    for i in range(1, 32):
        if i in channels:
            mask += 1 << (i - 1)
    seq += f"{0x1103},{mask},"
    mask = 0
    for i in range(32, 51):
        if i in channels:
            mask += 1 << (i - 32)
    seq += f"{0x1104},{mask},"

    # Set the LED OFF and disconnect all ADC from the DACs they can access
    val = (
        (1 << 31)
        + (0b1111 << 16)  # Set all ADC to run as voltage amplifiers
        + (15 << 12)
        + (15 << 8)
        + (15 << 4)
        + 15
    )
    seq += f"{0x1102},{val},1,999,0,0"

    execute_sequence(rfsoc, seq)


def scope_init(scope):
    scope.write("STOP")
    scope.write("ACQ:NSIN:COUN 1")
    scope.write("TRIG:A:MODE NORM")

    # Enable each channel and configure it
    for i in range(1, 5):
        scope.write(f"CHAN{i}:STAT ON")
        scope.write(f"CHAN{i}:TYPE HRES")
        scope.write(f"CHAN{i}:OFFS 0")
        scope.write(f"CHAN{i}:POS 0")
        scope.write(f"CHAN{i}:COUP DCLimit")
        scope.write(f"MEAS{i}:SOUR CH{i}")
        scope.write(f"MEAS{i}:STAT OFF")

    # Trigger setting
    scope.write("TRIG:A:MODE NORM")
    scope.write("TRIG:A:SOUR CH1")
    scope.write("TRIG:A:TYPE EDGE")

    # Measurement settings
    scope.write("REFL:REL:MODE TEN")

    # Waveform transfer settings
    scope.write("FORM REAL")  # Set REAL data format
    scope.write("FORM:BORD LSBF")

    sleep(1)
    print(scope.query("SYST:ERR:ALL?"))


def prepare_constant_measurement(scope, expected_value, scale):
    scope.write("TIM:SCAL 1e-3")
    scope.write("TIM:POS 0")
    for i in range(1, 5):
        scope.write(f"CHAN{i}:OFFS {expected_value}")
        scope.write(f"CHAN{i}:SCAL {scale}")
        scope.write(f"MEAS{i}:MAIN MEAN")


def retrieve_constant_measurement(scope: pyvisa.resources.MessageBasedResource):
    mean = {}
    std = {}
    traces = {}
    times = {}
    if PATH:
        header = scope.query("CHAN:DATA:HEAD?")
        start, stop, n, _ = header.split(",")
        t = np.linspace(float(start), float(stop), int(n))
    for j in range(1, 5):
        mean[j] = float(scope.query(f"MEAS{j}:RES? MEAN"))
        std[j] = float(scope.query(f"MEAS{j}:RES? STDD"))
        if PATH:
            old = scope.timeout
            scope.timeout = 5000
            times[j] = t
            traces[j] = scope.query_binary_values(
                f"CHAN{j}:DATA?", container=np.ndarray
            )
            scope.timeout = old

    return (mean, std, times, traces)


def execute_constant_measurement(rfsoc, scope):
    value = {}
    std = {}
    times = {}
    traces = {}
    for i in range(TEST_DACS):
        # Connect each ADC to the right DAC
        val = (
            (1 << 31)
            + (1 << 28)
            + (0b1111 << 16)  # Set all ADC to run as voltage amplifiers
            + (i << 12)
            + (i << 8)
            + (i << 4)
            + i
        )
        execute_sequence(rfsoc, f"{0x1102},{val},1,{250_000_000 - 1:d},0,0")
        sleep(1)
        scope.write("RUNS")
        scope.write("*TRG")
        sleep(1)
        mean, s, t, tr = retrieve_constant_measurement(scope)
        for j in range(1, 5):
            if (index := (j - 1) * 14 + i + 1) < 50:
                value[index] = mean[j]
                std[index] = s[j]
                if PATH:
                    times[index] = t[j]
                    traces[index] = tr[j]

    return value, std, times, traces


def make_square_sequence(value: int):
    wav_seq = ""
    # Set all fanouts to the max positive value at the same time
    for i, dac in enumerate(channels):
        is_final = (1 << 31) if i + 1 == len(channels) else 0
        wav_seq += f"{0x1100},{is_final + (dac << 16) + value},"
    wav_seq += "1,999,"
    # Send everybody back to 0
    for i, dac in enumerate(channels):
        is_final = (1 << 31) if i + 1 == len(channels) else 0
        wav_seq += f"{0x1100},{is_final + (dac << 16) + 0b0111111111111111},"
    wav_seq += "0,0"
    return wav_seq


def prepare_square_measurement(scope, expected_value):
    # Time scale
    scope.write("TIM:SCAL 1e-6")
    scope.write("TIM:POS 3e-6")

    # Vertical settings
    for i in range(1, 5):
        scope.write(f"CHAN{i}:SCAL {abs(expected_value) / 5}")
        scope.write(f"CHAN{i}:OFFS {expected_value / 2}")
    scope.write(f"TRIG:A:LEV1 {0.2 * expected_value}")
    scope.write(f"TRIG:A:EDGE:SLOP {'POS' if expected_value > 0 else 'NEG'}")
    sleep(1)


def retrieve_square_measurement(scope, expected_value):
    value = {}
    overshoot = {}
    slewrate = {}
    times = {}
    traces = {}
    is_pos = expected_value > 0
    if PATH:
        header = scope.query("CHAN:DATA:HEAD?")
        start, stop, n, _ = header.split(",")
        t = np.linspace(float(start), float(stop), int(n))
    for j in range(1, 5):
        value[j] = float(scope.query(f"MEAS{j}:RES? {'HIGH' if is_pos else 'LOW'}"))
        overshoot[j] = float(scope.query(f"MEAS{j}:RES? {'POV' if is_pos else 'NOV'}"))
        slewrate[j] = (
            float(scope.query(f"MEAS{j}:RES? {'SRR' if is_pos else 'SRF'}")) / 1e6
        )
        if PATH:
            times[j] = t
            old = scope.timeout
            scope.timeout = 5000
            traces[j] = scope.query_binary_values(
                f"CHAN{j}:DATA?", container=np.ndarray
            )
            scope.timeout = old

    return value, overshoot, slewrate, times, traces


def execute_square_measurement(rfsoc, scope, wav_seq, expected_value):
    value = {}
    overshoot = {}
    slewrate = {}
    times = {}
    traces = {}
    for i in range(TEST_DACS):
        # Connect each ADC to the right DAC
        val = (
            (1 << 31)
            + (1 << 28)
            + (0b1111 << 16)  # Set all ADC to run as voltage amplifiers
            + (i << 12)
            + (i << 8)
            + (i << 4)
            + i
        )
        execute_sequence(rfsoc, f"{0x1102},{val},1,{250_000_000 - 1:d},0,0")
        scope.write("RUNS")
        sleep(0.1)
        execute_sequence(rfsoc, wav_seq)
        sleep(0.1)
        val, over, slew, t, tr = retrieve_square_measurement(scope, expected_value)
        for j in range(1, 5):
            if (index := (j - 1) * 14 + i + 1) <= 50:
                value[index] = val[j]
                overshoot[index] = over[j]
                slewrate[index] = slew[j]
                if PATH:
                    times[index] = t[j]
                    traces[index] = tr[j]

    return value, overshoot, slewrate, times, traces


# --- Script
if not PATH or not os.path.exists(PATH):
    if PATH:
        print(
            "Specified file does not exist. Starting measurement sequence to create one"
        )

    # RFSOC set up
    rm = pyvisa.ResourceManager()
    rfsoc = cast(
        pyvisa.resources.MessageBasedResource,
        rm.open_resource(
            f"TCPIP::{RFSOC_IP}::5001::SOCKET",
            read_termination="\r\n",
            write_termination="\r\n",
            timeout=2000,
        ),
    )
    if not int(rfsoc.query("PLLINIT?")):
        rfsoc.write("DAC:RELAY:ALL 0")
        rfsoc.write("PLLINIT")
        sleep(5)
        rfsoc.write("DAC:RELAY:ALL 1")

    rfsoc.write("SEQ:STOP")
    rfsoc_fanout_reset(rfsoc)

    # Scope set up
    scope = cast(
        pyvisa.resources.MessageBasedResource,
        rm.open_resource(
            SCOPE_VISA,
            write_termination="\n",
            read_termination="\n",
        ),
    )
    scope_init(scope)

    # Perform a high quality measurement of the zero for each channel.
    prepare_constant_measurement(scope, 0, 0.001)
    zeros, zeros_std, zeros_times, zeros_traces = execute_constant_measurement(
        rfsoc, scope
    )

    print("Zeros (V):")
    print({k: f"{zeros[k]} pm {zeros_std[k]}" for k in sorted(zeros)})

    # Extract positive reference level using a coarse measurement
    channels = list(range(1, 51))
    prepare_square_measurement(scope, 5)
    (
        pos_reference,
        pos_overshoot,
        pos_slewrate,
        pos_times,
        pos_traces,
    ) = execute_square_measurement(
        rfsoc, scope, make_square_sequence(0b1111111111111111), 5
    )

    print("Positive references (V):")
    print({k: pos_reference[k] for k in sorted(pos_reference)})
    print("Positive overshoots (%):")
    print({k: pos_overshoot[k] for k in sorted(pos_overshoot)})
    print("Positive slewrate (V/µs):")
    print({k: pos_slewrate[k] for k in sorted(pos_slewrate)})
    print()

    # Perform a high res measurement of the positive reference
    # We use the average value and a somewhat larger scale to be sure not to miss
    # the value and also because the oscilloscope does not support large offset at
    # small scale
    rfsoc_set_constant(rfsoc, 0b1111111111111111)
    prepare_constant_measurement(scope, np.average(list(pos_reference.values())), 0.1)
    (
        pos_ref_fine,
        pos_ref_fine_std,
        pos_ref_fine_times,
        pos_ref_fine_traces,
    ) = execute_constant_measurement(rfsoc, scope)

    print("Pos fine reference (V):")
    print(
        {k: f"{pos_ref_fine[k]} pm {pos_ref_fine_std[k]}" for k in sorted(pos_ref_fine)}
    )
    print()

    # Extract negative reference level using a coarse measurement
    rfsoc_set_constant(rfsoc, 0b0111111111111111)
    prepare_square_measurement(scope, -5)
    (
        neg_reference,
        neg_overshoot,
        neg_slewrate,
        neg_times,
        neg_traces,
    ) = execute_square_measurement(rfsoc, scope, make_square_sequence(0), -5)
    print("Negative references (V):")
    print({k: neg_reference[k] for k in sorted(neg_reference)})
    print("Negative overshoots (%):")
    print({k: neg_overshoot[k] for k in sorted(neg_overshoot)})
    print("Negative slewrate (V/µs):")
    print({k: neg_slewrate[k] / 1e6 for k in sorted(neg_slewrate)})
    print()

    # Perform a high res measurement of the negative reference
    # We use the average value and a somewhat larger scale to be sure not to miss
    # the value and also because the oscilloscope does not support large offset at
    # small scale
    rfsoc_set_constant(rfsoc, 0)
    prepare_constant_measurement(scope, np.average(list(neg_reference.values())), 0.1)
    (
        neg_ref_fine,
        neg_ref_fine_std,
        neg_ref_fine_times,
        neg_ref_fine_traces,
    ) = execute_constant_measurement(rfsoc, scope)

    print("Neg fine reference (V):")
    print(
        {k: f"{neg_ref_fine[k]} pm {neg_ref_fine_std[k]}" for k in sorted(neg_ref_fine)}
    )
    print()

    if PATH:
        with h5py.File(PATH, "w") as f:
            # Store square shape measurements
            c = f.create_group("coarse")
            #   Positive square
            cp = c.create_group("pos")
            cp["dac"] = np.array(sorted(pos_reference))
            cp["reference"] = np.array(
                [pos_reference[k] for k in sorted(pos_reference)]
            )
            cp["overshoot"] = np.array(
                [pos_overshoot[k] for k in sorted(pos_overshoot)]
            )
            cp["slewrate"] = np.array([pos_slewrate[k] for k in sorted(pos_slewrate)])
            cp["times"] = np.array([pos_times[k] for k in sorted(pos_times)])
            cp["traces"] = np.array([pos_traces[k] for k in sorted(pos_traces)])
            #    Negative square
            cn = c.create_group("neg")
            cn["dac"] = np.array(sorted(neg_reference))
            cn["reference"] = np.array(
                [neg_reference[k] for k in sorted(neg_reference)]
            )
            cn["overshoot"] = np.array(
                [neg_overshoot[k] for k in sorted(neg_overshoot)]
            )
            cn["slewrate"] = np.array([neg_slewrate[k] for k in sorted(neg_slewrate)])
            cn["times"] = np.array([neg_times[k] for k in sorted(neg_times)])
            cn["traces"] = np.array([neg_traces[k] for k in sorted(neg_traces)])

            # Result from measurements carried out with a finer resolution
            ff = f.create_group("fine")
            # 0
            z = ff.create_group("zero")
            z["dac"] = np.array(sorted(zeros))
            z["value"] = np.array([zeros[k] for k in sorted(zeros)])
            z["std"] = np.array([zeros_std[k] for k in sorted(zeros_std)])
            z["times"] = np.array([zeros_times[k] for k in sorted(zeros_times)])
            z["traces"] = np.array([zeros_traces[k] for k in sorted(zeros_traces)])
            # Positive reference
            p = ff.create_group("pos")
            p["dac"] = np.array(sorted(pos_ref_fine))
            p["value"] = np.array([pos_ref_fine[k] for k in sorted(pos_ref_fine)])
            p["std"] = np.array([pos_ref_fine_std[k] for k in sorted(pos_ref_fine_std)])
            p["times"] = np.array(
                [pos_ref_fine_times[k] for k in sorted(pos_ref_fine_times)]
            )
            p["traces"] = np.array(
                [pos_ref_fine_traces[k] for k in sorted(pos_ref_fine_traces)]
            )
            # Negative reference
            n = ff.create_group("neg")
            n["dac"] = np.array(sorted(neg_ref_fine))
            n["value"] = np.array([neg_ref_fine[k] for k in sorted(neg_ref_fine)])
            n["std"] = np.array([neg_ref_fine_std[k] for k in sorted(neg_ref_fine_std)])
            n["times"] = np.array(
                [neg_ref_fine_times[k] for k in sorted(neg_ref_fine_times)]
            )
            n["traces"] = np.array(
                [neg_ref_fine_traces[k] for k in sorted(neg_ref_fine_traces)]
            )

else:
    print("Found an existing datafile, skipping measurements.")

if os.path.exists(PATH):
    with h5py.File(PATH, "r") as f:

        # Square for 1, 15, 29, 43
        fig, (ax1, ax2) = plt.subplots(2, 1, sharex=True, constrained_layout=True)
        fig.suptitle("Square shape DAC 1 vs others")

        for i in (0, 14, 28, 42):
            ax1.plot(
                f["coarse"]["pos"]["times"][i] * 1e6,
                f["coarse"]["pos"]["traces"][i],
                label=f"DAC {i + 1}",
            )
        ax1.legend()
        ax1.set_ylabel("Voltage (V)")

        for i in (0, 14, 28, 42):
            ax2.plot(
                f["coarse"]["neg"]["times"][i] * 1e6, f["coarse"]["neg"]["traces"][i]
            )
        ax2.set_ylabel("Voltage (V)")
        ax2.set_xlabel("Time (µs)")

        # Noise at 0, pos and neg for the sames
        fig, (ax1, ax2, ax3) = plt.subplots(3, 1, sharex=True, constrained_layout=True)
        fig.suptitle("Constant value DAC 1 vs others")

        for i in (0, 28):
            ax1.plot(
                f["fine"]["zero"]["times"][i] * 1e3,
                f["fine"]["zero"]["traces"][i],
                label=f"DAC {i + 1}",
            )
        ax1.legend()
        ax1.set_ylabel("Voltage (V)")

        for i in (0, 28):
            ax2.plot(f["fine"]["pos"]["times"][i] * 1e3, f["fine"]["pos"]["traces"][i])

        for i in (0, 28):
            ax3.plot(f["fine"]["neg"]["times"][i] * 1e3, f["fine"]["neg"]["traces"][i])

        ax3.set_xlabel("Time (ms)")

        # Plots with error bars of the stable values at 0, pos, neg
        fig, (ax1, ax2, ax3) = plt.subplots(3, 1, sharex=True, constrained_layout=True)
        fig.suptitle("Stable values (3$\sigma$)")
        for ax, v in zip((ax1, ax2, ax3), ("zero", "pos", "neg")):
            ax.errorbar(
                f["fine"][v]["dac"][:],
                f["fine"][v]["value"][:],
                3 * f["fine"][v]["std"][:],
                fmt="+",
            )
            ax.set_ylabel("Voltage (V)")
        ax3.set_xlabel("DAC")

        # Plot of the positive and negative slew rates
        fig, (ax1, ax2) = plt.subplots(2, 1, sharex=True, constrained_layout=True)
        fig.suptitle(
            "Slew rates\n"
            f'DAC 1: pos {f["coarse"]["pos"]["slewrate"][0]}, '
            f'neg {f["coarse"]["neg"]["slewrate"][0]} V/µs'
        )
        for ax, v in zip((ax1, ax2), ("pos", "neg")):
            ax.plot(f["coarse"][v]["dac"][1:], f["coarse"][v]["slewrate"][1:], "+")
            ax.set_ylabel("Slew rate (V/µs)")
        ax2.set_xlabel("DAC")

        plt.show()
