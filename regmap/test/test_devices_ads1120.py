import unittest
from migen import Module, Record
from migen.sim.core import run_simulation, passive
from regmap.core.spi import SpiMaster
from regmap.devices.ads1120 import ADS1120, ADS1120Config
from regmap.test.utils import assert_eq_before, timeout


class TestEeprom(unittest.TestCase):
    def test(self):
        pads = Record([("mosi", 1), ("miso", 1), ("sck", 1), ("sclk", 1), ("cs", 1)])
        dut = Module()
        dut.submodules.spi_master = master = SpiMaster(20E6, 4E6, pads, 24)
        dut.submodules.ads1120 = ADS1120()
        dut.comb += [
            pads.cs.eq(~master.busy),
            master.source.connect(dut.ads1120.spi_sink),
            dut.ads1120.spi_source.connect(master.sink),
            dut.ads1120.config.eq(0x12345678),
            dut.ads1120.drdy_n.eq(pads.miso),
        ]

        simple_config = ADS1120Config(mux_drdy=True).to_w32()
        print(f"ADS1120 config: {simple_config:08X}")

        @passive
        def stim():
            yield
            yield pads.miso.eq(1)
            for _ in range(500):
                yield
            yield pads.miso.eq(0)
            for _ in range(10):
                yield
            yield pads.miso.eq(1)
            for _ in range(300):
                yield
            yield pads.miso.eq(0)

        def check():
            yield from assert_eq_before(dut.ads1120.source.valid, 1, 200)
            yield
            yield from assert_eq_before(dut.ads1120.source.valid, 1, 1000)
            yield

        run_simulation(dut,
            [
                timeout(2000),
                stim(),
                check(),
            ],
            vcd_name="out/test_devices_ads1120.vcd")
