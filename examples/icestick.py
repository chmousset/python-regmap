#!/usr/bin/env python3

import argparse
from migen import Module
from migen.build.generic_platform import Subsignal, Pins
from migen.build.platforms.icestick import Platform
from regmap.core.i2c import I2cSequencer
from regmap.core.models import I2cBus
from regmap.devices.temp import LM75
from regmap.devices.gpio import PCF8574
from regmap.devices.ads131 import ADS131M04
from regmap.core.spi import SpiMaster


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
]


class Top(Module):
    def __init__(self, platform, i2c=False, spi=False):
        platform.add_extension(_ios)

        if i2c:
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

            self.submodules.i2c = I2cSequencer(platform.request("i2c"), transactions, 12E6)

        if spi:
            pads = platform.request("spi")
            self.submodules.spi = SpiMaster(
                sys_fcy=12E6, min_fcy=4E6, pads=pads, dw=24, sclk_output_idle=True)
            self.submodules.ads = ADS131M04(4)
            self.comb += [
                self.spi.source.connect(self.ads.spi_sink),
                self.ads.spi_source.connect(self.spi.sink),
                pads.cs.eq(~self.spi.busy),
            ]



if __name__ == '__main__':
    parser = argparse.ArgumentParser("Icestick NewMot demo")
    parser.add_argument("--build", "-b", action="store_true", help="build the FPGA")
    parser.add_argument("--flash", "-f", action="store_true", help="flash the FPGA")
    parser.add_argument("--i2c", action="store_true", help="enable i2c device")
    parser.add_argument("--spi", action="store_true", help="enable spi device")
    args = parser.parse_args()

    plat = Platform()

    soc = Top(
        platform=plat,
        i2c=args.i2c,
        spi=args.spi,
    )

    if args.build:
        plat.build(soc, build_dir="build/icestick")
    if args.flash:
        plat.create_programmer().flash(0, "build/icestick/top.bin")
