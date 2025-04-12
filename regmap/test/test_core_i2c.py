import unittest
from migen import Module, Signal, If
from migen.sim.core import run_simulation, passive
from regmap.core.i2c import (
    I2CTimer, I2cBitOperation, I2cBitOperationRTx, I2cByteOperationRx, I2cByteOperation,
    I2cOperation, I2cByteOperationRTx
)
from regmap.test.utils import ep_pop, ep_push, timeout,assert_eq_before


class I2cDevice(Module):
    def __init__(self):
        self.sda_i = Signal(reset=1)
        self.sda_o = Signal(reset=1)
        self.scl_i = Signal(reset=1)
        self.scl_o = Signal(reset=1)

    @passive
    def edge_detect(self):
        step = 0
        last_sda = (yield self.sda_i)
        last_scl = (yield self.scl_i)
        while True:
            sda = (yield self.sda_i)
            scl = (yield self.scl_i)
            step += 1

            self.sda_rising = (last_sda, sda) == (0, 1)
            self.sda_falling = (last_sda, sda) == (1, 0)
            self.scl_rising = (last_scl, scl) == (0, 1)
            self.scl_falling = (last_scl, scl) == (1, 0)
            self.is_start = (scl == 1) and self.sda_falling
            self.is_stop = (scl == 1) and self.sda_rising

            last_sda = sda
            last_scl = scl
            yield


    def wait_idle(self):
        while ((yield self.sda_i), (yield self.scl_i)) != (1,1):
            yield

    def wait_start(self):
        while True:
            if self.is_start:
                break
            yield

    def wait_stop(self):
        while True:
            if self.is_stop:
                break
            yield

    def wait_bit(self):
        while True:
            yield
            if self.scl_rising:
                self.bit = (yield self.sda_i)
                return
            if self.is_stop or self.is_start:
                return

    def wait_byte(self, ack_address=None, ack=None):
        self.byte = 0
        for _ in range(8):
            yield from self.wait_bit()
            if self.is_stop or self.is_start:
                return
            yield
            self.byte = (self.byte << 1) + self.bit
        if ack is not None:
            yield from self.send_ack(ack)
        elif ack_address is not None:
            yield from self.send_ack((self.byte & 0xFE) == ack_address)

    def send_ack(self, ack=True):
        yield from self.send_bit(0 if ack else 1)
        yield
        yield from self.send_bit(1)

    def send_bit(self, bit):  # transmit a bit
        while not self.scl_falling:
            if self.is_stop or self.is_start:
                return self.sda_o.eq(1)
            yield
        yield self.sda_o.eq(bit)

    def send_byte(self, byte):
        for bit in range(7, -1, -1):
            value = 1 if byte & (1<<bit) else 0
            yield from self.send_bit(value)
            if self.is_stop:
                break
            yield from self.wait_bit()


class I2cMem(I2cDevice):
    def __init__(self, address=0x50, address_bytes=1, mem={}):
        super().__init__()
        self.mem_address = 0
        self.address = address
        self.address_bytes = address_bytes
        self.mem = mem

    @passive
    def sim(self):
        yield # update last
        while True:
            yield from self.wait_start()
            yield
            yield from self.wait_byte(ack_address=self.address)
            if self.is_stop or self.is_start:
                continue
            if self.byte & 0xfe == self.address:
                if self.byte & 0x01:  # read operation
                    while True:
                        data = self.mem.get(self.mem_address, 0)
                        yield from self.send_byte(data)
                        self.mem_address = (self.mem_address + 1) % (256**self.address_bytes)
                        if self.is_stop or self.is_start:
                            break
                        yield from self.wait_bit()  # ACK
                        if self.bit:  # NACK: now we're passive
                            yield self.sda_o.eq(1)
                            yield from self.wait_stop()
                            break
                else:  # write operation
                    self.mem_address = 0
                    for _ in range(self.address_bytes):
                        yield from self.wait_byte(ack=True)
                        if self.is_stop or self.is_start:
                            break
                        self.mem_address = self.mem_address << 8 | self.byte

                    while True:
                        yield from self.wait_byte(ack=self.mem_address in self.mem.keys())
                        if self.is_stop or self.is_start:
                            break
                        if self.mem_address not in self.mem.keys():
                            yield self.sda_o.eq(1)  # go passive
                            yield from self.wait_stop()
                            break

                        self.mem.update({self.mem_address: self.byte})
                        self.mem_address = (self.mem_address + 1) % (256**self.address_bytes)



class I2cBus(Module):
    def __init__(self, pads_list):
        self.sda_bus = sda = Signal(reset=1)
        self.scl_bus = scl = Signal(reset=1)
        for pads in pads_list:
            self.comb += [
                pads.sda_i.eq(sda),
                pads.scl_i.eq(scl),
                If(~pads.sda_o,
                    sda.eq(0),
                ),
                If(~pads.scl_o,
                    scl.eq(0),
                ),
            ]


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

    def push_byte(self, dut, b, write):
        yield (dut.sink.valid.eq(1))
        yield (dut.sink.data.eq(b))
        yield (dut.sink.write.eq(write))
        yield
        while (yield dut.sink.ready) == 0:
            yield
        yield (dut.sink.valid.eq(0))

    def pop_byte(self, dut, expected_byte=None, expected_ack=None):
        yield dut.source.ready.eq(1)
        while (yield dut.source.valid) == 0:
            yield
        self.assertEqual((yield dut.source.first), 0)
        self.assertEqual((yield dut.source.last), 0)
        if expected_byte:
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
                    timeout(50),
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

        def stim():
            yield dut.source.ready.eq(1)

        run_simulation(dut,
            [
                stim(),
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
                timeout(50),
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
                timeout(51),
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
                timeout(50),
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
                timeout(300),
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
                timeout(50),
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
                timeout(250),
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
                timeout(250),
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
                timeout(250),
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
                timeout(250),
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
                timeout(250),
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
                timeout(500),
                stim(),
                check(),
            ],
            vcd_name="out/test_core_i2c_operation_r10_0x42_0x69.vcd")


    def test_mem_w69_16(self):
        pads = IoTri({"sda": 1, "scl": 1})
        op = I2cOperation()
        byte_op = I2cByteOperation(pads, 2)
        mem = I2cMem(mem={0x69: 0x00, 0x6A: 0x01})
        bus = I2cBus([pads, mem])
        top = Module()
        top.comb += [ op.source.connect(byte_op.sink) ]
        top.submodules += op, byte_op, mem, bus

        def stim():
            yield byte_op.source.ready.eq(1)
            yield
            yield from self.push_op(op, 1, 0, 0x50>>1, 0, 1, 0x69)
            yield from self.push_op(op, 0, 0, 0, 0, 1, 0xde)
            yield from self.push_op(op, 0, 1, 0, 0, 1, 0xad)
            # yield from self.push_op(op, 0, 1, 0, 0, 1, 0xbe)
            yield

        def check():
            yield
            while mem.is_stop == 0:
                yield
            yield
            yield

        run_simulation(top,
            [
                timeout(650),
                stim(),
                # self.wait_done(dut),
                check(),
                mem.edge_detect(),
                mem.sim(),
            ],
            vcd_name="out/test_core_i2c_operation_mem_w69_0xde_0xad.vcd")
        assert(mem.mem == {0x69: 0xde, 0x6A: 0xad})


class TestI2cBit_OperationRTx(TestCoreCommon):
    def test_start_stop(self):
        pad = IoTri({"sda": 1, "scl": 1})
        dut = I2cBitOperationRTx(pad, 2E6)
        dut.submodules.pad = pad

        def stim():
            for _ in range(2):
                yield
                yield dut.source.ready.eq(1)
                yield dut.sink.write.eq(1)
                yield from self.push_start(dut)
                yield from self.push_bit(dut, 1)
                yield from self.push_bit(dut, 0)
                yield from self.push_bit(dut, 1)
                yield from self.push_stop(dut)
                yield from assert_eq_before(dut.busy, 0, 30)

        def check():
            for _ in range(2):
                yield from self.pop_start(dut)
                yield from self.pop_bit(dut, 1)
                yield from self.pop_bit(dut, 0)
                yield from self.pop_bit(dut, 1)
                yield from self.pop_stop(dut)

        run_simulation(dut, [stim(), check(), timeout(220)],
            vcd_name="out/test_core_I2cOperationRTx_start_bit_stop.vcd")

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
            yield from assert_eq_before(dut.arb_lost, 1, 20)
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
            vcd_name="out/test_core_I2cOperationRTx_arbitration_lost.vcd")

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
            timeout = 30
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
            vcd_name="out/test_core_I2cOperationRTx_clock_stretch.vcd")

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
            vcd_name="out/test_core_I2cOperationRTx_reset.vcd")

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
            vcd_name="out/test_core_I2cOperationRTx_slave.vcd")


class TestI2cByteOperationRTx(TestCoreCommon):
    def test_master_w_r(self):
        dut = I2cByteOperationRTx()

        def stim():
            for _ in range(2):
                # Start
                yield from ep_push(dut.master_sink, {"first": 1}, timeout=20)

                # write one byte
                yield from ep_push(dut.master_sink, {"write": 1, "data": 0x42}, timeout=20)
                yield from ep_pop(dut.master_source, {"is_ack": 1, "ack": 0}, timeout=20)

                # read one byte
                yield from ep_push(dut.master_sink, {}, timeout=20)
                yield from ep_pop(dut.master_source, {"data": 0xDE}, timeout=40)
                yield from ep_push(dut.master_sink, {"is_ack": 1}, timeout=20)

                # Stop
                yield from ep_push(dut.master_sink, {"last": 1}, timeout=40)

        def check():
            for _ in range(2):
                # yield
                yield from ep_pop(dut.bit_source, {"first": 1}, timeout=20)
                yield from ep_push(dut.bit_sink, {"first": 1}, timeout=20)
                yield

                for i in range(8):
                    bit = 0b1 & (0x42 >> (7 - i))
                    yield from ep_pop(dut.bit_source, {"data": bit}, timeout=20)
                    yield from ep_push(dut.bit_sink, {"data": bit}, timeout=20)
                yield from ep_pop(dut.bit_source, {"data": 1}, timeout=20)
                yield from ep_push(dut.bit_sink, {"data": 0}, timeout=20)

                for i in range(8):
                    bit = 0b1 & (0xDE >> (7 - i))
                    yield from ep_pop(dut.bit_source, {"write": 0}, timeout=20)
                    yield from ep_push(dut.bit_sink, {"data": bit}, timeout=20)
                yield from ep_pop(dut.bit_source, timeout=20)
                yield from ep_push(dut.bit_sink, {"data": 1}, timeout=20)


                yield from ep_pop(dut.bit_source, {"last": 1}, timeout=20)
                yield from ep_push(dut.bit_sink, {"last": 1}, timeout=20)

                yield

        run_simulation(dut, [stim(), check()],
            vcd_name="out/test_core_I2cByteOperationRTx_master_w_r.vcd")

    def test_master_w_noack(self):
        dut = I2cByteOperationRTx()

        def stim():
            # Start
            yield from ep_push(dut.master_sink, {"first": 1}, timeout=20)

            # write one byte
            yield from ep_push(dut.master_sink, {"write": 1, "data": 0x42}, timeout=20)
            yield from ep_pop(dut.master_source, {"is_ack": 1, "ack": 1}, timeout=20)

            for _ in range(2):
                yield
            self.assertEqual((yield dut.busy), 0)

        def check():
            # yield
            yield from ep_pop(dut.bit_source, {"first": 1}, timeout=20)
            yield from ep_push(dut.bit_sink, {"first": 1}, timeout=20)
            yield

            for i in range(8):
                bit = 0b1 & (0x42 >> (7 - i))
                yield from ep_pop(dut.bit_source, {"data": bit}, timeout=20)
                yield from ep_push(dut.bit_sink, {"data": bit}, timeout=20)
            yield from ep_pop(dut.bit_source, {"data": 1}, timeout=20)
            yield from ep_push(dut.bit_sink, {"data": 1}, timeout=20)

            # since the ACK=1, the write stops there
            yield from ep_pop(dut.bit_source, {"last": 1}, timeout=20)
            yield from ep_push(dut.bit_sink, {"last": 1}, timeout=20)

            yield

        run_simulation(dut, [stim(), check()],
            vcd_name="out/test_core_I2cByteOperationRTx_master_w_noack.vcd")
