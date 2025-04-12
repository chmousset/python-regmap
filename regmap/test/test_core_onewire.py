import unittest
from math import ceil
from migen import Signal, run_simulation, Record, Module, C, If
from regmap.core.onewire import OneWireBitOperation, OneWireByteOperation, OneWireTiming, one_wire_timing
from regmap.devices.tmp182x import TMP1826


class IoTri(Module):
    def __init__(self, input_value):
        self.i = i = Signal()
        self.o = o = Signal()
        self.oe = oe = Signal()

        self.comb += [
            If(oe,
                i.eq(o),
            ).Else(
                i.eq(input_value),
            ),
        ]

class TestCoreBitOperation(unittest.TestCase):
    def push_bit(self, dut, bit):
        yield (dut.sink.valid.eq(1))
        yield (dut.sink.data.eq(bit))
        while (yield dut.sink.ready) == 0:
            yield
        yield
        yield (dut.sink.valid.eq(0))

    def pop_bit(self, dut, expected_value):
        yield dut.source.ready.eq(1)
        while (yield dut.source.valid) == 0:
            yield
        self.assertEqual((yield dut.source.data), expected_value)
        yield

    def wait_done(self, dut):
        yield
        while (yield dut.busy) == 1:
            yield

    def _test_single_bit(self, bit):
        for input_value in [0, 1]:
            pad = IoTri(input_value)
            print(f"{input_value}")
            dut = OneWireBit(
                pad_tri=pad,
                tSTART=C(10),
                tSAMP=C(10),
                tREC=C(20),
            )
            dut.submodules.pad = pad

            run_simulation(dut, [
                    self.push_bit(dut, bit),
                    self.wait_done(dut),
                    self.pop_bit(dut, bit & input_value),
                ],
                vcd_name=f"out/test_core_onewire_bit_{bit}_{input_value}.vcd")

    def _test_bit_one(self):
        self._test_single_bit(1)

    def _test_bit_zero(self):
        self._test_single_bit(0)


class TestOneWireTiming(unittest.TestCase):
    def test_timings(self):
        fcy = 1E6
        dut = OneWireTiming(fcy)
        print(vars(dut))

        def test(dut):
            for speed, operations in one_wire_timing.items():
                print(f"speed= {speed}")
                yield dut.overdrive.eq(1 if speed == "overdrive" else 0)
                for operation, segments in operations.items():
                    print(f"  operation= {operation} => {segments}")
                    yield dut.reset_cmd.eq(1 if operation == "reset" else 0)
                    yield
                    for segment, value in segments.items():
                        print(f"    segment= {segment} => {value}")
                        self.assertEqual(
                            (yield getattr(dut, segment)),
                            ceil(fcy * value)
                        )

        run_simulation(dut, test(dut), vcd_name="out/test_core_onewire_timings.vcd")


class Dut(Module):
    def __init__(self, onewire_pad, sys_clk_freq):
        tmp = TMP1826()
        ow_bit = OneWireBitOperation(onewire_pad, sys_clk_freq)
        ow_byte = OneWireByteOperation()
        self.submodules += tmp, ow_bit, ow_byte
        self.comb += [
            ow_bit.source.connect(ow_byte.sink_bit),
            ow_byte.source_bit.connect(ow_bit.sink),
            tmp.source.connect(ow_byte.sink),
            ow_byte.source.connect(tmp.sink),
        ]

class TestTMP182(unittest.TestCase):
    def testTmp(self):
        pad = Record([("o", 1), ("oe", 1), ("i", 1)])
        fclk = 2E6

        dut = Dut(pad, fclk)

        def wait(cycles=1000):
            for _ in range(cycles):
                yield

        run_simulation(dut, wait(), vcd_name="out/test_device_onewire_tmp182.vcd")
