import unittest
from migen import Module, Signal
from migen.sim.core import run_simulation, passive
from litex.soc.interconnect import stream
from regmap.core.i2c_sequencer import I2cRegmap
from regmap.core.models import I2cReg, I2cDevice
from regmap.test.utils import ep_push, ep_pop


@passive
def rtx_model(sink, source, ack_write=True, echo_write=True):
    is_write = False
    cnt = 0x42
    while True:
        yield
        yield from ep_pop(source)
        if (yield source.first):
            print("pop Start")
            continue
        if (yield source.last):
            print("pop Stop")
            continue
        if (yield source.is_ack):
            ack = yield source.ack
            print(f"pop Ack={ack}")
            if is_write:
                ack = 0 if ack_write else 1
                print(f"push Ack={ack}")
                yield from ep_push(sink, {"is_ack": 1, "ack": ack})
                continue
        if (yield source.write):
            data = yield source.data
            print(f"pop Write={data}")
            is_write = True
            if echo_write:
                yield from ep_push(sink, {"data": (yield source.data)})
            else:
                yield from ep_push(sink, {"data": ~(yield source.data)})
        else:
            is_write = False
            print(f"pop Read")
            yield from ep_push(sink, {"data": cnt})
            cnt += 1


class TestBasic(unittest.TestCase):
    def test(self):
        dev = I2cDevice(None, 0x12)
        reg_A = I2cReg(dev, 0xde, 2)
        reg_B = I2cReg(dev, 0xde, 2)
        dut = I2cRegmap()
        dut.add_transaction(reg_A.write(0xadbe))
        reg_B.add_read_stream()
        dut.add_transaction(reg_B.read())

        def bench():
            yield dut.enable.eq(1)
            yield reg_B.source.ready.eq(1)
            for _ in range(100):
                yield
            raise NotImplementedError("TODO: checks")

        run_simulation(dut, [bench(), rtx_model(dut.sink, dut.source)],
            vcd_name="out/test_core_i2c_sequencer.vcd")
