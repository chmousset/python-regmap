from math import ceil
from migen import Module, If, Signal
from migen.genlib.misc import WaitTimer
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

    def idle(self):
        for _ in range(4):
            self.add_transition("11")

    def start(self):
        self.idle()
        self.add_transition("01")
        self.add_transition("00")
        self.add_transition("00")
        self.add_transition("00")

    def restart(self):
        self.add_transition("x0")
        self.add_transition("01")
        self.add_transition("11")
        self.add_transition("01")
        self.add_transition("00")
        self.add_transition("00")
        self.add_transition("00")

    def stop(self):
        self.add_transition("x0")
        self.add_transition("00")
        self.add_transition("01")
        self.idle()

    def bit(self, value):
        if value:
            self.bit_one()
        else:
            self.bit_zero()

    def byte(self, value):
        print(f"W 0b{value:08b}")
        for shift in range(7, -1, -1):
            print(f"  [{shift}] = 1 & (value >> shift)")
            self.bit(1 & (value >> shift))

    def bit_one(self):
        self.add_transition("10")
        self.add_transition("11")

    def bit_zero(self):
        self.add_transition("00")
        self.add_transition("01")


class I2cSequencer(Module):
    def __init__(self, pads, transactions, fclk):

        # # #

        # tristate the IOs
        sda, scl = Signal(), Signal()
        self.specials += Tristate(pads.sda, 0, ~sda)
        self.specials += Tristate(pads.scl, 0, ~scl)
        # self.comb += [
        #     pads.sda.eq(sda),
        #     pads.scl.eq(scl),
        # ]

        # Transform transactions to SDA/SCL bit values
        sdascl = SdaScl()
        for tr in transactions:
            sdascl.start()
            sdascl.byte(tr.device.address << 1)
            sdascl.bit(1)  # ACK
            for data in tr.write:
                sdascl.byte(data)
                sdascl.bit(1)  # ACK
            if tr.read:
                sdascl.restart()
                sdascl.byte((tr.device.address << 1) + 1)
                sdascl.bit(1)  # ACK
                for _ in range(tr.read):
                    sdascl.byte(0xFF)
                    sdascl.bit(0)  # ACK
            sdascl.stop()
        print(sdascl.transitions)

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
            sda.eq(rp.dat_r[1]),
            scl.eq(rp.dat_r[0]),
        ]
