import unittest
from migen import Module, Signal
from migen.sim.core import run_simulation, passive
from regmap.core.i2c import (
    I2CTimer, I2cBitOperation, I2cBitOperationRTx, I2cByteOperationRx, I2cByteOperation,
    I2cOperation
)


class IoTri(Module):
    def __init__(self, input_value):
        signals = ["sda", "scl"]
        for s in signals:
            setattr(self, s + "_o", Signal(name=s + "_o"))
            setattr(self, s + "_i", Signal(name=s + "_i", reset=1))
            setattr(self, s + "_external_i", Signal(name=s + "_external_i"))
            setattr(self, s + "_external_o", Signal(name=s + "_external_o",
                reset=input_value.get(s, 1)))
            sig_o = getattr(self, s + "_o")
            sig_i = getattr(self, s + "_i")
            external_sig_i = getattr(self, s + "_external_i")
            external_sig_o = getattr(self, s + "_external_o")
            pad = Signal(name=s + "_pad")
            self.comb += [
                pad.eq(sig_o & external_sig_o),
                external_sig_i.eq(pad),
                sig_i.eq(pad),
            ]


class TestTimer(unittest.TestCase):
    def test(self):
        clk_div = [4, 6]
        dut = I2CTimer(clk_div)

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


class TestCoreCommon(unittest.TestCase):
    def push_start(self, dut):
        yield (dut.sink.valid.eq(1))
        yield (dut.sink.first.eq(1))
        yield
        while (yield dut.sink.ready) == 0:
            yield
        yield (dut.sink.valid.eq(0))
        yield (dut.sink.first.eq(0))

    def push_stop(self, dut):
        yield (dut.sink.valid.eq(1))
        yield (dut.sink.last.eq(1))
        yield
        while (yield dut.sink.ready) == 0:
            yield
        yield (dut.sink.valid.eq(0))
        yield (dut.sink.last.eq(0))

    def push_bit(self, dut, bit):
        yield (dut.sink.valid.eq(1))
        yield (dut.sink.data.eq(bit))
        yield
        while (yield dut.sink.ready) == 0:
            yield
        yield (dut.sink.valid.eq(0))

    def pop_start(self, dut):
        yield dut.source.ready.eq(1)
        while (yield dut.source.valid) == 0:
            yield
        self.assertEqual((yield dut.source.first), 1)
        self.assertEqual((yield dut.source.last), 0)
        yield

    def pop_restart(self, dut):
        yield dut.source.ready.eq(1)
        extra_bits = 0
        while True:
            while (yield dut.source.valid) == 0:
                yield
            self.assertGreaterEqual(1, extra_bits)
            if (yield dut.source.first) == 0:
                extra_bits += 1
            else:
                yield
                return
            self.assertEqual((yield dut.source.last), 0)
            yield

    def pop_stop(self, dut):
        yield dut.source.ready.eq(1)
        extra_bits = 0
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

    def push_byte(self, dut, b, write):
        yield (dut.sink.valid.eq(1))
        yield (dut.sink.data.eq(b))
        yield (dut.sink.write.eq(write))
        yield
        while (yield dut.sink.ready) == 0:
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

    def push_op(self, dut, start, stop, address, read, write, data, address_10b=False):
        yield dut.sink.first.eq(start)
        yield dut.sink.last.eq(stop)
        yield dut.sink.address.eq(address)
        yield dut.sink.read.eq(read)
        yield dut.sink.write.eq(write)
        yield dut.sink.data.eq(data)
        yield dut.sink.address10.eq(1 if address_10b else 0)
        yield dut.sink.valid.eq(1)
        yield
        while (yield dut.sink.ready) == 0:
            yield
        yield (dut.sink.valid.eq(0))


class TestCoreBitOperation(TestCoreCommon):
    def _test_single_bit(self, bit):
        for input_value in [0, 1]:
            pad = IoTri({"sda": input_value, "scl": 1})
            print(f"{input_value}")
            dut = I2cBitOperation(
                pads_tri=pad,
                clk_div=[4, 8],
            )
            dut.submodules.pad = pad

            run_simulation(dut,
                [
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

        run_simulation(dut,
            [
                self.push_start(dut),
                self.wait_done(dut),
                self.pop_start(dut),
            ],
            vcd_name="out/test_core_i2c_bit_start.vcd")

    def test_stop(self):
        pad = IoTri({"sda": 1, "scl": 1})
        dut = I2cBitOperation(
            pads_tri=pad,
            clk_div=[4, 8],
        )
        dut.submodules.pad = pad

        run_simulation(dut,
            [
                self.push_stop(dut),
                self.wait_done(dut),
                self.pop_stop(dut),
            ],
            vcd_name="out/test_core_i2c_bit_stop.vcd")

    def test_startstop(self):
        pad = IoTri({"sda": 1, "scl": 1})
        dut = I2cBitOperation(
            pads_tri=pad,
            clk_div=[4, 8],
        )
        dut.submodules.pad = pad

        def stim():
            yield
            yield from self.push_start(dut)
            yield from self.push_stop(dut)

        def check():
            yield from self.pop_start(dut)
            yield from self.pop_stop(dut)

        run_simulation(dut,
            [
                self.timeout(50),
                stim(),
                # self.wait_done(dut),
                check(),
            ],
            vcd_name="out/test_core_i2c_bit_start_stop.vcd")

    def test_restart(self):
        pad = IoTri({"sda": 1, "scl": 1})
        dut = I2cBitOperation(
            pads_tri=pad,
            clk_div=[4, 8],
        )
        dut.submodules.pad = pad

        def stim():
            yield
            yield from self.push_start(dut)
            yield from self.push_start(dut)
            yield from self.push_stop(dut)

        def check():
            yield from self.pop_start(dut)
            yield from self.pop_restart(dut)
            yield from self.pop_stop(dut)

        run_simulation(dut,
            [
                self.timeout(51),
                stim(),
                # self.wait_done(dut),
                check(),
            ],
            vcd_name="out/test_core_i2c_bit_restart.vcd")


class TestCoreByteOperationRx(TestCoreCommon):
    def testStartStop(self):
        dut = I2cByteOperationRx()

        def stim():
            yield
            yield from self.push_start(dut)
            yield from self.push_stop(dut)

        def check():
            yield from self.pop_start(dut)
            yield from self.pop_stop(dut)

        run_simulation(dut,
            [
                self.timeout(50),
                stim(),
                # self.wait_done(dut),
                check(),
            ],
            vcd_name="out/test_core_i2c_byte_rx_start_stop.vcd")

    def testStartDataStop(self):
        dut = I2cByteOperationRx()

        def stim():
            yield
            yield from self.push_start(dut)
            for i in range(8):
                yield from self.push_bit(dut, 1 if (0x42 & (1 << (7 - i))) else 0)
            yield from self.push_bit(dut, 1)  # ACK
            yield from self.push_stop(dut)

        def check():
            yield from self.pop_start(dut)
            yield from self.pop_byte(dut, 0x42, 1)
            yield from self.pop_stop(dut)

        run_simulation(dut,
            [
                self.timeout(300),
                stim(),
                # self.wait_done(dut),
                check(),
            ],
            vcd_name="out/test_core_i2c_byte_rx_start_data_stop.vcd")


class TestCoreByteOperation(TestCoreCommon):
    def testStartStop(self):
        pad = IoTri({"sda": 1, "scl": 1})
        dut = I2cByteOperation(pad, [4, 8])
        dut.submodules.pad = pad

        def stim():
            yield
            yield from self.push_start(dut)
            yield from self.push_stop(dut)

        def check():
            yield from self.pop_start(dut)
            yield from self.pop_stop(dut)

        run_simulation(dut,
            [
                self.timeout(50),
                stim(),
                # self.wait_done(dut),
                check(),
            ],
            vcd_name="out/test_core_i2c_byte_start_stop.vcd")

    def testStartDataStop(self):
        pad = IoTri({"sda": 1, "scl": 1})
        dut = I2cByteOperation(pad, [4, 8])
        dut.submodules.pad = pad

        def stim():
            yield
            yield from self.push_start(dut)
            yield from self.push_byte(dut, 0x42, 1)
            yield from self.push_stop(dut)

        def check():
            yield from self.pop_start(dut)
            yield from self.pop_byte(dut, 0x42, 1)
            yield from self.pop_stop(dut)

        run_simulation(dut,
            [
                self.timeout(250),
                stim(),
                # self.wait_done(dut),
                check(),
            ],
            vcd_name="out/test_core_i2c_byte_start_data_stop.vcd")

    def testStartDataStopRead(self):
        pad = IoTri({"sda": 1, "scl": 1})
        dut = I2cByteOperation(pad, [4, 8])
        dut.submodules.pad = pad

        def stim():
            yield
            yield from self.push_start(dut)
            yield from self.push_byte(dut, 0x42, 0)
            yield from self.push_stop(dut)

        def check():
            yield from self.pop_start(dut)
            yield from self.pop_byte(dut, 0x42, 0)
            yield from self.pop_stop(dut)

        run_simulation(dut,
            [
                self.timeout(250),
                stim(),
                # self.wait_done(dut),
                check(),
            ],
            vcd_name="out/test_core_i2c_byte_start_data_stop_read.vcd")


class TestI2cOperation(TestCoreCommon):
    def test_r8b(self):
        dut = I2cOperation()

        def stim():
            yield
            yield from self.push_op(dut, 1, 1, 0x42, 1, 0, 0x69)
            yield

        def check():
            yield from self.pop_start(dut)
            yield from self.pop_byte(dut, (0x42 << 1) | 0b1)
            yield from self.pop_byte(dut, 0xFF)
            yield from self.pop_stop(dut)

        run_simulation(dut,
            [
                self.timeout(250),
                stim(),
                # self.wait_done(dut),
                check(),
            ],
            vcd_name="out/test_core_i2c_operation_r8_0x42_0x69.vcd")

    def test_w8b(self):
        dut = I2cOperation()

        def stim():
            yield
            yield from self.push_op(dut, 1, 1, 0x42, 0, 1, 0x69)
            yield

        def check():
            yield from self.pop_start(dut)
            yield from self.pop_byte(dut, 0x42 << 1)
            yield from self.pop_byte(dut, 0x69)
            yield from self.pop_stop(dut)

        run_simulation(dut,
            [
                self.timeout(250),
                stim(),
                # self.wait_done(dut),
                check(),
            ],
            vcd_name="out/test_core_i2c_operation_w8_0x42_0x69.vcd")

    def test_w8b_16(self):
        dut = I2cOperation()

        def stim():
            yield
            yield from self.push_op(dut, 1, 0, 0x42, 0, 1, 0x69)
            yield from self.push_op(dut, 0, 0, 0, 0, 1, 0xde)
            yield from self.push_op(dut, 0, 1, 0, 0, 1, 0xad)
            yield

        def check():
            yield from self.pop_start(dut)
            yield from self.pop_byte(dut, 0x42 << 1)
            yield from self.pop_byte(dut, 0x69)
            yield from self.pop_byte(dut, 0xde)
            yield from self.pop_byte(dut, 0xad)
            yield from self.pop_stop(dut)

        run_simulation(dut,
            [
                self.timeout(250),
                stim(),
                # self.wait_done(dut),
                check(),
            ],
            vcd_name="out/test_core_i2c_operation_w8_0x42_0x69_0xde_0xad.vcd")

    def test_r10b(self):
        dut = I2cOperation()

        def stim():
            yield
            yield from self.push_op(dut, 1, 1, 0x42, 1, 0, 0x69, True)
            yield

        def check():
            yield from self.pop_start(dut)
            yield from self.pop_byte(dut, 0b11110001)
            yield from self.pop_byte(dut, 0x42)
            yield from self.pop_byte(dut, 0xFF)
            yield from self.pop_stop(dut)

        run_simulation(dut,
            [
                self.timeout(500),
                stim(),
                check(),
            ],
            vcd_name="out/test_core_i2c_operation_r10_0x42_0x69.vcd")


class TestI2cBit_OperationRTx(TestCoreCommon):
    def test_start_stop(self):
        pad = IoTri({"sda": 1, "scl": 1})
        dut = I2cBitOperationRTx(pad, 2E6)
        dut.submodules.pad = pad

        def stim():
            yield
            yield dut.source.ready.eq(1)
            yield dut.sink.write.eq(1)
            yield from self.push_start(dut)
            yield from self.push_bit(dut, 1)
            yield from self.push_bit(dut, 0)
            yield from self.push_bit(dut, 1)
            yield from self.push_stop(dut)
            timeout = 20
            while (yield dut.busy) == 1:
                timeout -= 1
                if timeout == 0:
                    raise Exception("Still busy")
                yield
            yield

        def check():
            yield from self.pop_start(dut)
            yield from self.pop_bit(dut, 1)
            yield from self.pop_bit(dut, 0)
            yield from self.pop_bit(dut, 1)
            yield from self.pop_stop(dut)

        run_simulation(dut, [stim(), check()],
            vcd_name="out/test_core_OperationRTx_start_bit_stop.vcd")

    def test_arb_lost(self):
        pad = IoTri({"sda": 1, "scl": 1})
        dut = I2cBitOperationRTx(pad, 2E6)
        dut.submodules.pad = pad

        def stim():
            yield
            yield dut.source.ready.eq(1)
            yield dut.sink.write.eq(1)
            yield from self.push_start(dut)

            yield from self.push_bit(dut, 1)
            yield dut.pad.sda_external_o.eq(0)  # force SDA to 0: the dut doesn't control the bus

            #  Wait for the arbitration lost condition to be detected
            timeout = 20
            while (yield dut.arb_lost) == 0:
                timeout -= 1
                if timeout == 0:
                    raise Exception("Arbitration lost not detected")
                yield

            self.assertEqual(1, (yield dut.pad.sda_o))  # the dut should release the bus
            self.assertEqual(1, (yield dut.pad.scl_o))  # the dut should release the bus
            self.assertEqual(1, (yield dut.busy))

            # at this stage, SCL=1 and SDA=0. Generate a Stop
            for _ in range(10):
                yield
            yield dut.pad.sda_external_o.eq(1)

            for _ in range(4):
                yield
            self.assertEqual(0, (yield dut.busy))  # Stop condition detected

        def check():
            yield from self.pop_start(dut)
            yield from self.pop_bit(dut, 0)
            yield from self.pop_stop(dut)

        run_simulation(dut, [stim(), check()],
            vcd_name="out/test_core_OperationRTx_arbitration_lost.vcd")

    def test_clk_stretch(self):
        pad = IoTri({"sda": 1, "scl": 1})
        dut = I2cBitOperationRTx(pad, 2E6)
        dut.submodules.pad = pad

        def stim():
            yield
            yield dut.source.ready.eq(1)
            yield dut.sink.write.eq(1)
            yield from self.push_start(dut)
            yield from self.push_bit(dut, 1)

            yield dut.pad.scl_external_o.eq(0)  # force SCL to 0: clock stretch
            for _ in range(40):
                yield

            self.assertEqual(1, (yield dut.pad.scl_o))  # the dut should not control the bus
            self.assertEqual(1, (yield dut.busy))
            yield dut.pad.scl_external_o.eq(1)

            yield from self.push_stop(dut)
            timeout = 20
            while (yield dut.busy) == 1:
                timeout -= 1
                if timeout == 0:
                    raise Exception("Still busy")
                yield
            yield

        def check():
            yield from self.pop_start(dut)
            yield from self.pop_bit(dut, 1)
            yield from self.pop_stop(dut)

        run_simulation(dut, [stim(), check()],
            vcd_name="out/test_core_OperationRTx_clock_stretch.vcd")

    def test_reset(self):
        pad = IoTri({"sda": 1, "scl": 1})
        dut = I2cBitOperationRTx(pad, 2E6)
        dut.submodules.pad = pad

        def stim():
            yield
            yield dut.source.ready.eq(1)
            yield dut.sink.write.eq(1)
            yield from self.push_start(dut)
            yield from self.push_bit(dut, 1)

            self.assertEqual(1, (yield dut.busy))

            yield dut.reset.eq(1)
            yield
            yield dut.reset.eq(0)
            yield
            yield
            self.assertEqual(0, (yield dut.busy))

        def check():
            yield from self.pop_start(dut)

        run_simulation(dut, [stim(), check()],
            vcd_name="out/test_core_OperationRTx_reset.vcd")

    def test_slave(self):
        pad = IoTri({"sda": 1, "scl": 1})
        dut = I2cBitOperationRTx(pad, 2E6)
        dut.submodules.pad = pad
        val = 0x42

        def b4():
            for _ in range(5):
                yield

        def start(dut):
            yield from b4()
            yield dut.pad.sda_external_o.eq(0)
            yield from b4()
            yield dut.pad.scl_external_o.eq(0)

        def stop(dut):
            yield from b4()
            yield dut.pad.sda_external_o.eq(0)
            yield from b4()
            yield dut.pad.scl_external_o.eq(1)
            yield from b4()
            yield dut.pad.sda_external_o.eq(1)

        def bit(dut, bit):
            yield from b4()
            yield dut.pad.sda_external_o.eq(bit)
            yield from b4()
            yield dut.pad.scl_external_o.eq(1)
            yield from b4()
            yield from b4()
            yield dut.pad.scl_external_o.eq(0)

        def byte(dut, b):
            for i in range(8):
                yield from bit(dut, 1 if b & (1 << (7 - i)) else 0)
            yield from bit(dut, 1)  # Ack

        def stim():
            yield
            yield dut.source.ready.eq(1)
            self.assertEqual(0, (yield dut.busy))

            yield from start(dut)
            self.assertEqual(1, (yield dut.busy))

            yield from byte(dut, val)
            yield from stop(dut)
            yield
            yield
            self.assertEqual(0, (yield dut.busy))

        def check():
            yield from self.pop_start(dut)
            for i in range(8):
                yield from self.pop_bit(dut, 1 if val & (1 << (7-i)) else 0)
            yield from self.pop_bit(dut, 1)  # ack
            yield from self.pop_stop(dut)

        run_simulation(dut, [stim(), check()],
            vcd_name="out/test_core_OperationRTx_slave.vcd")
