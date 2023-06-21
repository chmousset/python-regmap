#!/usr/bin/env python3

import argparse
from migen import Module
from migen.build.generic_platform import Subsignal, Pins
from migen.build.platforms.icestick import Platform
from regmap.core.i2c import I2cSequencer
from regmap.core.models import I2cBus
from regmap.devices.temp import LM75


_ios = [
    ("i2c", 0,
        Subsignal("sda", Pins("PMOD:3")),
        Subsignal("scl", Pins("PMOD:2")),
    ),
]


class Top(Module):
    def __init__(self, platform):
        platform.add_extension(_ios)

        bus = I2cBus()
        sensor = LM75(bus)

        transactions = [
            sensor.configuration.write(0b00000100),  # OS active high
            sensor.thyst.write(75 << 8),  # OS inactive when temp falls under 75°C
            sensor.tos.write(80 << 8),  # OS active when temp rises above 80°C
            sensor.temperature.read(),
        ]

        self.submodules.i2c = I2cSequencer(platform.request("i2c"), transactions, 12E6)


if __name__ == '__main__':
    parser = argparse.ArgumentParser("Icestick NewMot demo")
    parser.add_argument("--build", "-b", action="store_true", help="build the FPGA")
    parser.add_argument("--flash", "-f", action="store_true", help="flash the FPGA")
    args = parser.parse_args()

    plat = Platform()

    soc = Top(platform=plat)

    if args.build:
        plat.build(soc, build_dir="build/icestick")
    if args.flash:
        plat.create_programmer().flash(0, "build/icestick/top.bin")
