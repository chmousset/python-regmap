import unittest
import inspect
from math import ceil
from migen import *
from regmap.core.i2c2 import *


class IoTri(Module):
    def __init__(self, input_value):
        signals = ["sda", "scl"]
        for s in signals:
            self.t = t = TSTriple(1)
            setattr(self, s, Signal(name=s + ""))
            setattr(self, s+"_i", Signal(name=s + "_i", reset=1))
            setattr(self, s+"_external_i", Signal(name=s + "_pad", reset=input_value.get(s, 1)))
            sig_out = getattr(self, s)
            sig_in = getattr(self, s+"_i")
            external_sig_in = getattr(self, s+"_external_i")
            self.comb += [
                If(~sig_out,
                    external_sig_in.eq(0),
                    sig_in.eq(0),
                ).Else(
                    sig_in.eq(external_sig_in),
                ),
            ]


class TestTimer(unittest.TestCase):
    def test(self):
        clk_div = [4, 6]
        dut= I2CTimer(clk_div)
        def assert_done(time):
            for _ in range(time):
                self.assertEqual((yield dut.done), 0)
                yield
            self.assertEqual((yield dut.done), 1)

        def bench():
            for i, period in enumerate(clk_div):
                yield dut.speed.eq(i)
                yield
                yield dut.wait.eq(1)
                yield from assert_done(period + 1)
                yield dut.wait.eq(0)
                yield
                yield
                yield

        run_simulation(dut, bench(), vcd_name="out/test_core_i2c_timer.vcd")


class TestCoreBitOperation(unittest.TestCase):
    def push_start(self, dut):
        yield (dut.sink.valid.eq(1))
        yield (dut.sink.first.eq(1))
        while (yield dut.sink.ready) == 0:
            yield
        yield
        yield (dut.sink.valid.eq(0))
        yield (dut.sink.first.eq(0))

    def push_stop(self, dut):
        yield (dut.sink.valid.eq(1))
        yield (dut.sink.last.eq(1))
        while (yield dut.sink.ready) == 0:
            yield
        yield
        yield (dut.sink.valid.eq(0))
        yield (dut.sink.last.eq(0))

    def push_bit(self, dut, bit):
        yield (dut.sink.valid.eq(1))
        yield (dut.sink.data.eq(bit))
        while (yield dut.sink.ready) == 0:
            yield
        yield
        yield (dut.sink.valid.eq(0))

    def pop_start(self, dut):
        yield dut.source.ready.eq(1)
        while (yield dut.source.valid) == 0:
            yield
        self.assertEqual((yield dut.source.first), 1)
        self.assertEqual((yield dut.source.last), 0)
        yield

    def pop_stop(self, dut):
        yield dut.source.ready.eq(1)
        extra_bits = 0;
        while True:
            while (yield dut.source.valid) == 0:
                yield
            self.assertGreaterEqual(1, extra_bits)
            if (yield dut.source.last) == 0:
                extra_bits += 1
            else:
                yield
                return
            self.assertEqual((yield dut.source.first), 0)
            yield

    def pop_bit(self, dut, expected_value):
        yield dut.source.ready.eq(1)
        while (yield dut.source.valid) == 0:
            yield
        self.assertEqual((yield dut.source.data), expected_value)
        yield

    def wait_done(self, dut):
        yield
        while (yield dut.busy) == 0:
            yield
        while (yield dut.busy) == 1:
            yield
        yield
        yield

    @passive
    def timeout(self, cycles):
        for _ in range(cycles):
            yield
        raise Exception(f"timeout after {cycles} cycles")

    def _test_single_bit(self, bit):
        for input_value in [0, 1]:
            pad = IoTri({"sda": input_value, "scl":1})
            print(f"{input_value}")
            dut = I2cBitOperation(
                pads_tri=pad,
                clk_div=[4, 8],
            )
            dut.submodules.pad = pad

            run_simulation(dut, [
                    self.push_bit(dut, bit),
                    self.wait_done(dut),
                    self.pop_bit(dut, bit & input_value),
                    self.timeout(50),
                ],
                vcd_name=f"out/test_core_i2c_bit_{bit}_{input_value}.vcd")

    def test_bit_one(self):
        self._test_single_bit(1)

    def test_bit_zero(self):
        self._test_single_bit(0)

    def test_start(self):
        pad = IoTri({})
        dut = I2cBitOperation(
            pads_tri=pad,
            clk_div=[4, 8],
        )
        dut.submodules.pad = pad

        run_simulation(dut, [
                self.push_start(dut),
                self.wait_done(dut),
                self.pop_start(dut),
            ],
            vcd_name=f"out/test_core_i2c_bit_start.vcd")

    def test_stop(self):
        pad = IoTri({"sda": 1, "scl":1})
        dut = I2cBitOperation(
            pads_tri=pad,
            clk_div=[4, 8],
        )
        dut.submodules.pad = pad

        run_simulation(dut, [
                self.push_stop(dut),
                self.wait_done(dut),
                self.pop_stop(dut),
            ],
            vcd_name=f"out/test_core_i2c_bit_stop.vcd")

    def test_startstop(self):
        pad = IoTri({"sda": 1, "scl":1})
        dut = I2cBitOperation(
            pads_tri=pad,
            clk_div=[4, 8],
        )
        dut.submodules.pad = pad
        def stim():
            yield
            yield from self.push_start(dut)
            yield
            yield from self.push_stop(dut)

        def check():
            yield from self.pop_start(dut)
            yield
            yield from self.pop_stop(dut)

        run_simulation(dut, [
                self.timeout(50),
                stim(),
                # self.wait_done(dut),
                check(),
            ],
            vcd_name=f"out/test_core_i2c_bit_start_stop.vcd")


class TestCoreByteOperation(TestCoreBitOperation):
    def push_byte(self, dut, b):
        yield (dut.sink.valid.eq(1))
        yield (dut.sink.data.eq(b))
        while (yield dut.sink.ready) == 0:
            yield
        yield
        yield (dut.sink.valid.eq(0))

    def pop_byte(self, dut, expected_byte, expected_ack=None):
        yield dut.source.ready.eq(1)
        while (yield dut.source.valid) == 0:
            yield
        self.assertEqual((yield dut.source.first), 0)
        self.assertEqual((yield dut.source.last), 0)
        self.assertEqual((yield dut.source.data), expected_byte)
        if expected_ack is not None:
            self.assertEqual((yield dut.source.ack), expected_ack)
        yield
        yield dut.source.ready.eq(0)

    def testStartStop(self):
        pad = IoTri({"sda": 1, "scl":1})
        dut = I2cByteOperation(pad, [4, 8])
        dut.submodules.pad = pad
        def stim():
            yield
            yield from self.push_start(dut)
            yield
            yield from self.push_stop(dut)

        def check():
            yield from self.pop_start(dut)
            yield
            yield from self.pop_stop(dut)

        run_simulation(dut, [
                self.timeout(50),
                stim(),
                # self.wait_done(dut),
                check(),
            ],
            vcd_name=f"out/test_core_i2c_byte_start_stop.vcd")

    def testStartDataStop(self):
        pad = IoTri({"sda": 1, "scl":1})
        dut = I2cByteOperation(pad, [4, 8])
        dut.submodules.pad = pad
        def stim():
            yield
            yield from self.push_start(dut)
            yield
            yield from self.push_byte(dut, 0x42)
            yield
            yield from self.push_stop(dut)

        def check():
            yield from self.pop_start(dut)
            yield from self.pop_byte(dut, 0x42, 1)
            yield from self.pop_stop(dut)

        run_simulation(dut, [
                self.timeout(250),
                stim(),
                # self.wait_done(dut),
                check(),
            ],
            vcd_name=f"out/test_core_i2c_byte_start_data_stop.vcd")


class TestI2cOperation(TestCoreByteOperation):
    def push_op(self, dut, start, stop, address, read, data, address_10b=False):
        yield dut.sink.first.eq(start)
        yield dut.sink.last.eq(stop)
        yield dut.sink.address.eq(address)
        yield dut.sink.read.eq(read)
        yield dut.sink.data.eq(data)
        yield dut.sink.address10.eq(1 if address_10b else 0)
        yield dut.sink.valid.eq(1)
        while (yield dut.sink.ready) == 0:
            yield
        yield
        yield (dut.sink.valid.eq(0))

    def test_r8b(self):
        dut = I2cOperation()
        def stim():
            yield
            yield from self.push_op(dut, 1, 1, 0x42, 1, 0x69)
            yield

        def check():
            yield from self.pop_start(dut)
            yield from self.pop_byte(dut, (0x42 << 1) | 0b1)
            yield from self.pop_byte(dut, 0xFF)
            yield from self.pop_stop(dut)

        run_simulation(dut, [
                self.timeout(250),
                stim(),
                # self.wait_done(dut),
                check(),
            ],
            vcd_name=f"out/test_core_i2c_operation_r8_0x42_0x69.vcd")

    def test_w8b(self):
        dut = I2cOperation()
        def stim():
            yield
            yield from self.push_op(dut, 1, 1, 0x42, 0, 0x69)
            yield

        def check():
            yield from self.pop_start(dut)
            yield from self.pop_byte(dut, 0x42 << 1)
            yield from self.pop_byte(dut, 0x69)
            yield from self.pop_stop(dut)

        run_simulation(dut, [
                self.timeout(250),
                stim(),
                # self.wait_done(dut),
                check(),
            ],
            vcd_name=f"out/test_core_i2c_operation_w8_0x42_0x69.vcd")

    def test_w8b_16(self):
        dut = I2cOperation()
        def stim():
            yield
            yield from self.push_op(dut, 1, 0, 0x42, 0, 0x69)
            yield
            yield from self.push_op(dut, 0, 0, 0, 0, 0xde)
            yield
            yield from self.push_op(dut, 0, 1, 0, 0, 0xad)
            yield

        def check():
            yield from self.pop_start(dut)
            yield from self.pop_byte(dut, 0x42 << 1)
            yield from self.pop_byte(dut, 0x69)
            yield from self.pop_byte(dut, 0xde)
            yield from self.pop_byte(dut, 0xad)
            yield from self.pop_stop(dut)

        run_simulation(dut, [
                self.timeout(250),
                stim(),
                # self.wait_done(dut),
                check(),
            ],
            vcd_name=f"out/test_core_i2c_operation_w8_0x42_0x69_0xde_0xad.vcd")

    def test_r10b(self):
        dut = I2cOperation()
        def stim():
            yield
            yield from self.push_op(dut, 1, 1, 0x42, 1, 0x69, True)
            yield

        def check():
            yield from self.pop_start(dut)
            yield from self.pop_byte(dut, 0b11110001)
            yield from self.pop_byte(dut, 0x42)
            yield from self.pop_byte(dut, 0xFF)
            yield from self.pop_stop(dut)

        run_simulation(dut, [
                self.timeout(500),
                stim(),
                check(),
            ],
            vcd_name=f"out/test_core_i2c_operation_r10_0x42_0x69.vcd")
