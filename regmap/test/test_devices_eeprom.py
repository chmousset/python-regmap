import unittest
from migen import Module
from migen.sim.core import run_simulation
from regmap.core.i2c import I2cByteOperation, I2cOperation

from regmap.devices.eeprom import EEPROM_MAC
from regmap.test.utils import assert_eq_before, timeout
from regmap.test.test_core_i2c import I2cMem, I2cBus, IoTri


class TestEeprom(unittest.TestCase):
    def test_read_uid(self):
        # create our DUT toplevel
        top = Module()

        # Create bus with a physical EEPROM simulator
        top.submodules.pads = pads = IoTri({"sda": 1, "scl": 1})
        top.submodules.mem_sim = mem_sim = I2cMem(mem={0xFA + i: i for i in range(6)})
        top.submodules.bus = bus = I2cBus([pads, mem_sim])

        top.submodules.op_b = op_b = I2cByteOperation(pads, 1)
        top.submodules.op = op = I2cOperation()
        top.submodules.mem = mem = EEPROM_MAC(bus, 0x50>>1)
        top.comb += [
            op.source.connect(op_b.sink),
            op_b.source.connect(mem.i2c_sink),
            mem.i2c_source.connect(op.sink),
        ]

        def stim():
            yield

        def check():
            yield
            yield from assert_eq_before(mem.eui.valid, 1, 2000)
            eui = (yield mem.eui.eui)
            print(f"EUI={eui:012X}")
            for _ in range(150):
                yield
            assert eui == 0x000102030405

        run_simulation(top,
            [
                timeout(2000),
                stim(),
                check(),
                mem_sim.edge_detect(),
                mem_sim.sim(),
            ],
            vcd_name="out/test_devices_eeprom_eeprom_mac_read.vcd")
