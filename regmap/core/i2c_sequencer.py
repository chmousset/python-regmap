# I2C Sequencer provides an easy way to perform writes to I2C devices.
# Logic usage is minimum, but requires ROM block to store transactions.
# It does not support clock stretching or multi-master, so it's intended for very simple
# use cases where readback isn't required.
from math import ceil
from migen import Module, If, Signal
from migen.genlib.misc import WaitTimer
from migen.genlib.io import DDROutput
from migen.fhdl.specials import Memory, Tristate


class SdaScl:
    def __init__(self):
        self.sda = 1
        self.scl = 1
        self.transitions = []

    def add_transition(self, seq):
        sda = seq[0]
        scl = seq[1]
        next_sda = self.sda if sda == 'x' else int(sda)
        next_scl = int(scl)
        self.transitions += [(next_sda << 1) | next_scl]
        self.sda = next_sda
        self.scl = next_scl

    def idle(self):
        for _ in range(4):
            self.add_transition("11")

    def start(self):
        self.idle()
        self.add_transition("01")

    def restart(self):
        self.add_transition("10")
        self.add_transition("11")
        self.add_transition("01")

    def stop(self):
        # self.add_transition("x0")
        self.add_transition("00")
        self.add_transition("01")
        self.add_transition("11")
        self.idle()

    def bit(self, value):
        if value:
            self.bit_one()
        else:
            self.bit_zero()

    def byte(self, value):
        for shift in range(7, -1, -1):
            self.bit(1 & (value >> shift))

    def bit_one(self):
        # self.add_transition("x0")
        self.add_transition("10")
        self.add_transition("11")

    def bit_zero(self):
        # self.add_transition("x0")
        self.add_transition("00")
        self.add_transition("01")


class I2cSequencer(Module):
    """Autonomous I2C sequencer.

    Useful to configure devices at startup. It uses a memory to store the predefined transactions.

    Example:

    bus = I2cBus()
    sensor = LM75(bus)
    transactions = [
        gpio.io.write(0xF0),
        gpio.io.read(),
    ]
    self.submodules.i2c = I2cSequencer(platform.request("i2c"), transactions, 12E6)
    """
    def __init__(self, pads, transactions, fclk):

        # # #

        # tristate the IOs
        sda, scl = Signal(), Signal()
        self.sda_i, self.scl_i = Signal(), Signal()
        self.specials.sda_tri = Tristate(pads.sda, 0, ~sda, self.sda_i)
        self.specials.scl_tri = Tristate(pads.scl, 0, ~scl, self.scl_i)

        # Transform transactions to SDA/SCL bit values
        sdascl = SdaScl()
        for tr in transactions:
            sdascl.start()
            if len(tr.write):
                sdascl.byte(tr.device.address << 1)
                sdascl.bit(1)  # ACK
                for data in tr.write:
                    sdascl.byte(data)
                    sdascl.bit(1)  # ACK
                if tr.read:
                    sdascl.restart()
            if tr.read:
                sdascl.byte((tr.device.address << 1) + 1)
                sdascl.bit(1)  # ACK
                for _ in range(tr.read):
                    sdascl.byte(0xFF)
                    sdascl.bit(0)  # ACK
            sdascl.stop()

        # clock divisor
        clkdiv = ceil(fclk / 100E3 / 2)
        self.submodules.wait = wait = WaitTimer(clkdiv)
        self.comb += wait.wait.eq(~wait.done)

        # memory to store the transactions
        count = len(sdascl.transitions)
        self.specials.mem = mem = Memory(width=2, depth=count, init=sdascl.transitions)
        self.specials.rp = rp = mem.get_port()

        # the module continuously goes through the memory, generating the desired pattern
        turnaround = Signal(4, reset=15)
        self.sync += [
            If(wait.done,
                If(rp.adr == count - 1,
                    If(turnaround,
                        turnaround.eq(turnaround - 1),
                    ).Else(
                        turnaround.eq(turnaround.reset),
                        rp.adr.eq(0),
                    )
                ).Else(
                    rp.adr.eq(rp.adr + 1),
                ),
            ),
        ]
        self.comb += [
            scl.eq(rp.dat_r[0]),
        ]
        self.sync += [
            sda.eq(rp.dat_r[1]),
        ]
