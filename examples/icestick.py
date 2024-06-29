#!/usr/bin/env python3

import argparse
from migen import Module
from migen.build.generic_platform import Subsignal, Pins
from migen.build.platforms.icestick import Platform
from migen.fhdl.specials import TSTriple
from regmap.core.i2c_sequencer import I2cSequencer
from regmap.core.i2c import I2cPads, I2cOperation, I2cByteOperation
from regmap.core.models import I2cBus
from regmap.devices.temp import LM75
from regmap.devices.gpio import PCF8574
from regmap.devices.ads131 import ADS131M04
from regmap.devices.eeprom import EEPROM_MAC
from regmap.core.spi import SpiMaster
from regmap.devices.tmp182x import TMP1826
from regmap.core.onewire import OneWireBitOperation, OneWireByteOperation


_ios = [
    ("i2c", 0,
        Subsignal("sda", Pins("PMOD:3")),
        Subsignal("scl", Pins("PMOD:2")),
    ),
    ("spi", 0,
        Subsignal("cs", Pins("PMOD:0")),
        Subsignal("sclk", Pins("PMOD:4")),
        Subsignal("mosi", Pins("PMOD:1")),
        Subsignal("miso", Pins("PMOD:5")),
        Subsignal("miso_cpy", Pins("PMOD:7")),
    ),
    ("onewire", 0,
        Subsignal("io", Pins("GPIO1:0")),
    ),
]


class Top(Module):
    def __init__(self, platform, i2c_sequencer=False, i2c=False, spi=False, onewire=False):
        platform.add_extension(_ios)
        sys_clk_freq = 12E6

        if i2c_sequencer:
            bus = I2cBus()
            sensor = LM75(bus)
            gpio = PCF8574(bus)

            transactions = [
                sensor.configuration.write(0b00000100),  # OS active high
                sensor.thyst.write(75 << 8),  # OS inactive when temp falls under 75°C
                sensor.tos.write(80 << 8),  # OS active when temp rises above 80°C
                sensor.temperature.read(),
                gpio.io.write(0xF0),
                gpio.io.read(),
            ]

            self.submodules.i2c_sequencer = I2cSequencer(platform.request("i2c"), transactions, 12E6)

        if i2c:
            bus = I2cBus()
            eui = EEPROM_MAC(bus)
            pads = I2cPads(platform.request("i2c"))
            op_b = I2cByteOperation(pads, 100)
            op = I2cOperation()
            self.submodules += eui, pads, op, op_b
            self.comb += [
                op.source.connect(op_b.sink),
                op_b.source.connect(eui.i2c_sink),
                eui.i2c_source.connect(op.sink),
            ]

        if spi:
            pads = platform.request("spi")
            self.submodules.spi = SpiMaster(
                sys_fcy=sys_clk_freq, min_fcy=4E6, pads=pads, dw=24, sclk_output_idle=True)
            self.submodules.ads = ADS131M04(config=(4, [
                    0x7777,  # reg=0x04; Gain=128 for all channels
                    0x0000,  # reg=0x05; reserved
                    (0b0011 << 9) | (0b1 << 8),  # reg=0x06; chop mode enable, delay=16
                ]))
            self.comb += [
                self.spi.source.connect(self.ads.spi_sink),
                self.ads.spi_source.connect(self.spi.sink),
                pads.cs.eq(~self.spi.busy),
            ]

        if onewire:
            pad = platform.request("onewire").io
            self.specials.onewire_pad = onewire_pad = TSTriple().get_tristate(pad)
            tmp = TMP1826()
            ow_op = OneWireBitOperation(onewire_pad, sys_clk_freq)
            ow_b_op = OneWireByteOperation()
            self.submodules += tmp, ow_op, ow_b_op
            self.comb += [
                ow_op.source.connect(ow_b_op.sink_bit),
                ow_b_op.source_bit.connect(ow_op.sink),
                tmp.source.connect(ow_b_op.sink),
                ow_b_op.source.connect(tmp.sink),
            ]



if __name__ == '__main__':
    parser = argparse.ArgumentParser("Icestick NewMot demo")
    parser.add_argument("--build", "-b", action="store_true", help="build the FPGA")
    parser.add_argument("--flash", "-f", action="store_true", help="flash the FPGA")
    parser.add_argument("--i2c-sequencer", action="store_true", help="enable i2c sequencer")
    parser.add_argument("--i2c", action="store_true", help="enable i2c device")
    parser.add_argument("--spi", action="store_true", help="enable spi device")
    parser.add_argument("--onewire", action="store_true", help="enable onewire device")
    args = parser.parse_args()

    plat = Platform()

    soc = Top(
        platform=plat,
        i2c_sequencer=args.i2c_sequencer,
        i2c=args.i2c,
        spi=args.spi,
        onewire=args.onewire,
    )

    if args.build:
        plat.build(soc, build_dir="build/icestick")
    if args.flash:
        plat.create_programmer().flash(0, "build/icestick/top.bin")
