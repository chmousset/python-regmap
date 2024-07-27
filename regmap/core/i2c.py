from migen import *
from litex.soc.interconnect import stream
from migen.genlib.cdc import MultiReg
from math import ceil
from litex.gen.genlib.misc import WaitTimer


i2c_bit_layout = [
    ("data", 1),
]

i2c_bit_operation_layout = i2c_bit_layout + [
    ("write", 1),
]

i2c_byte_layout = [
    ("data", 8),
    ("ack", 1),
]

i2c_byte_operation_layout = [
    ("data", 8),
    ("write", 1),
]

i2c_operation_layout = [
    ("data", 8),
    ("address", 10),
    ("read", 1),
    ("write", 1),
    ("address10", 1),
]


class I2cPads(Module):
    """This Tristate controls the pad when the output is 0; else it's an input"""
    def __init__(self, pads):
        # Inputs
        self.sda_o, self.scl_o = Signal(reset=1), Signal(reset=1)

        # Outputs
        self.sda_i, self.scl_i = Signal(), Signal()

        # # #
        sda_i, scl_i = Signal(), Signal()
        self.specials += Tristate(pads.sda, self.sda, ~self.sda, sda_i)
        self.specials += MultiReg(sda_i, self.sda_i),
        self.specials += Tristate(pads.scl, self.scl, ~self.scl, scl_i)
        self.specials += MultiReg(scl_i, self.scl_i),


class I2CTimer(Module):
    """WaitTimer, but reconfigurable on the fly for multiple periods."""
    def __init__(self, clk_div):
        # inputs
        self.speed = speed = Signal(max=len(clk_div) + 1)
        self.wait = wait = Signal()

        # outputs
        self.done = done = Signal()

        # # #
        cnt = Signal(max=max(clk_div) + 1, reset=max(clk_div))
        print(f"cnt={cnt}")
        self.comb += done.eq(cnt == 0)
        self.sync += [
            If(~wait,
                Case(speed, {k: [cnt.eq(div)] for k, div in enumerate(clk_div)}).makedefault(0)
            ).Elif(~done,
                cnt.eq(cnt - 1),
            ),
        ]


class I2cBitOperationRTx(Module):
    """Control sda and scl pads to to perform bit-level operations.

    I2C can have multiple states, making each transitions relatively complex to manage.
    In particular, since the conditions are based on the state of the pads, which result from the
    logic AND result of multiple devices' outputs, reaching certain conditions can be tricky.
    This module should be able to manage them all (including clock stretching).

    Parameters:
    - pads_tri: I2cPads object
    - sys_clk: clock domain frequency
    - fscl: Vec[float] - list of desired clock speeds
    - timeout: None or float - Timeout (seconds) after which clock stretching is marked as default

    Inputs:
    - sink: stream.Endpoint(i2c_bit_operation_layout) - defines which operation to perform:
      - first=1 => perform a I2C Start condition
      - last=1 => perform a I2C Stop condition
      - first=last=0 => perform a bit transmission
    - speed: Signal() - select the transmit speed, in order of `fscl` parameter

    Outputs:
    - source: stream.Endpoint(i2c_bit_layout) - data received from the I2C bus
      - first=1 => Start condition detected
      - last=1 => Stop condition detected
      - first=last=0 => bit transmission has been detected
      NOTE: every bit sent will be echoed on this stream
    - arb_lost: If set, arbitration has been lost. If sink.valid is set when SCL falls, start slave
      mode
    - noise: illegal transitions on SCL or SDA have been detected.
    - busy: reflects the state of the I2C bus. Set both in Slave or Master mode.

    """
    def __init__(self, pads_tri, sys_clk, fscl=[100E3, 400E3, 1E6], timeout=10E-3):
        # inputs
        self.sink = sink = stream.Endpoint(i2c_bit_operation_layout)
        self.speed = speed = Signal(max=len(fscl))
        self.reset = reset = Signal()

        # outputs
        self.busy = busy = Signal()
        self.arb_lost = arb_lost = Signal()
        self.source = source = stream.Endpoint(i2c_bit_layout)
        self.noise = noise = Signal()

        # # #
        sda_i = pads_tri.sda_i
        sda_o = pads_tri.sda_o
        scl_i = pads_tri.scl_i
        scl_o = pads_tri.scl_o
        bit = Signal()
        is_write = Signal()

        tp4 = [max(0, int(ceil(sys_clk / f / 4) - 1)) for f in fscl]
        th =  [max(0, int(ceil(sys_clk / f / 2) - 3)) for f in fscl]
        self.submodules.wtp4 = wtp4 = I2CTimer(tp4)
        self.submodules.wth = wth = I2CTimer(th)
        self.submodules.wtto = wtto = WaitTimer(int(sys_clk * timeout))
        self.comb += [
            wtp4.speed.eq(speed),
            wth.speed.eq(speed),
        ]
        pads_tri.sda_o.reset = 1
        pads_tri.scl_o.reset = 1

        # # # RX logic
        prev_sda = Signal()
        prev_scl = Signal()

        self.sync += [
            prev_sda.eq(sda_i),
            prev_scl.eq(scl_i),
        ]
        self.comb += [
            If(prev_scl & scl_i & prev_sda & ~sda_i,
                source.valid.eq(1),
                source.first.eq(1),
            ).Elif(prev_scl & scl_i & ~prev_sda & sda_i,
                source.valid.eq(1),
                source.last.eq(1),
            ).Elif(scl_i & ~prev_scl & (sda_i == prev_sda),
                source.valid.eq(1),
                source.data.eq(sda_i)
            ).Elif((scl_i ^ prev_scl) | (sda_i ^ prev_sda),
                noise.eq(1),
            ),
        ]

        # # # TX Logic
        self.submodules.fsm = fsm = FSM("RESET")
        fsm.act("IDLE",
            NextValue(arb_lost, 0),
            sink.ready.eq(1),
            wtp4.wait.eq(0),
            wtto.wait.eq(0),
            wth.wait.eq(0),
            If(source.valid & source.first,  # should not check valid
                NextState("ARB_LOST"),
                NextValue(busy, 1),
            ).Elif(sink.valid,
                NextValue(busy, 1),
                If(sink.first,
                    NextState("START"),
                ).Elif(sink.last,
                    NextState("STOP"),
                ).Else(
                    NextState("BIT_SETUP"),
                    NextValue(bit, sink.data),
                    NextValue(is_write, sink.write),
                ),
            ),
        )
        fsm.act("START",
            wtp4.wait.eq(scl_i),
            If(~sda_i | ~scl_i,
                NextState("ARB_LOST"),
            ),
            If(wtp4.done,
                wtp4.wait.eq(0),
                NextValue(sda_o, 0),
                NextState("START2"),
            ),
        )
        fsm.act("START2",
            wtp4.wait.eq(1),
            If(wtp4.done,
                wtp4.wait.eq(0),
                NextValue(scl_o, 0),
                NextState("IDLE"),
            ),
        )
        fsm.act("STOP",
            wtp4.wait.eq(~scl_i),  # wait for SCL to be 0 for Tp/4
            If(wtp4.done,
                wtp4.wait.eq(0),
                NextValue(sda_o, 0),  # SDA should be at 0 to create a rising edge
                NextState("STOP2"),
            ),
        )
        fsm.act("STOP2",
            wtp4.wait.eq(1),
            If(wtp4.done,
                wtp4.wait.eq(0),
                NextValue(scl_o, 1),
                NextState("STOP3"),
            ),
        )
        fsm.act("STOP3",
            wtp4.wait.eq(1),
            If(wtp4.done,
                wtp4.wait.eq(0),
                NextValue(sda_o, 1),
                NextState("IDLE"),
                NextValue(busy, 0),
            ),
        )
        fsm.act("BIT_SETUP",  # at this stage, we already pull SCL down
            wtp4.wait.eq(1),
            If(wtp4.done,  # wait T_HOLD before outputting data
                wtp4.wait.eq(0),
                If(is_write,
                    NextValue(sda_o, bit),  # drive SDA only for write operations
                ),
                NextState("BIT_HOLD"),
            ),
        )
        fsm.act("BIT_HOLD",
            wtp4.wait.eq(1),
            If(wtp4.done & source.ready,  # hold befrore generating rising edge. Clock stretch if consumer not ready
                wtp4.wait.eq(0),
                NextValue(scl_o, 1),
                NextState("BIT_READ"),
            ),
        )
        fsm.act("BIT_READ",
            wth.wait.eq(scl_i),  # wait for rising edge
            wtto.wait.eq(1),  # here another device can hold SCL to clock-stretch
            If(scl_i,  # sample data
                source.valid.eq(1),
                source.data.eq(sda_i),
                NextValue(arb_lost, is_write & (bit ^ sda_i)),  # in write operation, we should hold the bus
                NextState("BIT_HIGH"),
            ).Elif(wtto.done,
                NextState("TIMEOUT"),
            ),
        )
        fsm.act("BIT_HIGH",
            wth.wait.eq(1),
            If(wth.done,
                If(arb_lost,
                    NextState("ARB_LOST"),
                ).Else(
                    NextState("IDLE"),
                    NextValue(scl_o, 0),  # setup SCL for next bit or STOP
                )
            ),
        )
        fsm.act("TIMEOUT",
            # TODO: handle blocked bus condition
            NextValue(busy, 0),
            NextState("IDLE"),
        )
        fsm.act("ARB_LOST",
            wtto.wait.eq(~source.valid),  # no activity: bus is blocked
            If(wtto.done,
                NextState("TIMEOUT"),
            ).Elif(source.ready & source.valid & source.last,  # wait for a STOP
                NextState("IDLE"),
                NextValue(busy, 0),
            ).Elif(sink.valid & prev_scl & scl_i,  # on falling edge, we can setup data to send as a slave
                NextState("SLAVE_BIT_SETUP"),
            ),
        )
        fsm.act("SLAVE_BIT_SETUP",
            If(~scl_i,
                NextValue(scl_o, 0),  # clock stretch as soon as SCL goes low
            ),
            wtto.wait.eq(1),
            wtp4.wait.eq(~scl_i),
            sink.ready.eq(wtp4.done),
            If(wtto.done,  # timeout on clock stretching: release the I2C bus
                wtp4.wait.eq(0),
                sink.ready.eq(0),
                NextValue(sda_o, 1),  # put SDA recessive
                NextState("SLAVE_TIMEOUT_RECOVERY"),
            ),
            If(sink.valid & sink.ready,
                wtp4.wait.eq(0),
                NextState("SLAVE_BIT_HOLD"),
                NextValue(sda_o, sink.data),
            ),
            If(source.valid & source.last,  # on Last, exit slave mode
                NextState("IDLE"),
            ),
        )
        fsm.act("SLAVE_BIT_HOLD",
            wtp4.wait.eq(1),
            If(wtp4.done,
                NextValue(scl_o, 1),  # release SCL for master control
            ),
        )
        fsm.act("SLAVE_BIT_TRANSMIT",
            If(scl_i,  # rising edge: prepare to send next bit
                NextState("SLAVE_BIT_SETUP"),
            ),
        )
        fsm.act("SLAVE_TIMEOUT_RECOVERY",
            wtp4.wait.eq(1),
            If(wtp4.done,
                NextValue(scl_o, 0),
            ),
            If(source.valid & source.last,
                NextState("IDLE"),
                wtp4.wait.eq(0),
                NextValue(busy, 0),
            ),
        )
        fsm.act("RESET",
            NextValue(sda_o, 1),
            NextValue(scl_o, 1),
            NextValue(busy, 0),
            NextState("IDLE"),
        )

    def do_finalize(self):
        # hackish reset injection
        self.fsm.finalize()
        self.fsm.comb += [
            If(self.reset,
                self.fsm.next_state.eq(self.fsm.state.reset),
            ),
        ]



class I2cBitOperationTx(Module):
    """Control sda and scl pads to to perform bit-level operations.

    I2C can have multiple states, making each transitions relatively complex to manage.
    In particular, since the conditions are based on the state of the pads, which result from the
    or-ing of multiple devices' outputs, reaching certain conditions can be tricky.
    This module should be able to manage them all (including clock stretching).

    Inputs:
    - sink: stream.Endpoint(i2c_bit_operation_layout) - defines which operation to perform:
      - first=1 => perform a I2C Start condition
      - last=1 => perform a I2C stop condition
      - first=last=0 => perform a bit transmission
    """
    def __init__(self, pads_tri, clk_div, with_clk_stretch=False):
        # inputs
        self.sink = sink = stream.Endpoint(i2c_bit_operation_layout)

        # outputs
        self.busy = busy = Signal(reset=1)

        # # #
        bit = Signal()
        pads = pads_tri
        if type(clk_div) is int:
            clk_div = [clk_div]
        self.submodules.wt = wt = I2CTimer(clk_div)
        self.speed = wt.speed
        pads.sda_o.reset = 1
        pads.scl_o.reset = 1

        self.submodules.fsm = fsm = FSM("IDLE")
        fsm.act("IDLE",
            busy.eq(0),
            wt.wait.eq(1),
            sink.ready.eq(wt.done),
            If(sink.ready & sink.valid,
                wt.wait.eq(0),
                If(sink.first,
                    NextState("START"),
                ).Elif(sink.last,
                    NextState("STOP"),
                ).Else(
                    NextState("BIT"),
                    NextValue(bit, sink.data),
                ),
            )
        )
        fsm.act("START",
            # |   Actual  |           Next          |
            # | SDA | SCL | SDA | SCL |    State    |
            # |-----|-----|-----|-----|-------------|
            # |  0  |  0  |  1  |     |             |
            # |  0  |  1  |     |  0  |             |
            # |  1  |  0  |     |  1  |             |
            # |  1  |  1  |  0  |     | START2/IDLE |

            If(wt.done,
                If(~pads.sda_i & pads.scl_i,
                    NextValue(pads.scl_o, 0),
                ).Elif(pads.sda_i & ~pads.scl_i,
                    NextValue(pads.scl_o, 1),
                ).Elif(~pads.sda_i & ~pads.scl_i,
                    NextValue(pads.sda_o, 1),
                ).Else(# both at 1
                    NextValue(pads.sda_o, 0),
                    NextState("IDLE"),
                ),
            ).Else(
                wt.wait.eq(1),
            ),
        )
        fsm.act("STOP",
            # |     Actual          |           Next          |
            # | SDA_i | SDA_O | SCL | SDA | SCL |    State    |
            # |-------|-------|-----|-----|-----|-------------|
            # |   0   |   0   |  1  |  1  |     |             |
            # |   0   |   0   |  0  |     |  1  |             |
            # |   0   |   1   |  1  |     |  0  |             |
            # |   0   |   1   |  0  |  0  |     |             |
            # |   1   |   0   |  1  |     |     |  HW_FAULT   |
            # |   1   |   0   |  0  |     |     |  HW_FAULT   |
            # |   1   |   1   |  1  |     |     |    IDLE     |
            # |   1   |   1   |  0  |  0  |     |             |

            If(wt.done,
                If(~pads.scl_i & pads.scl_o,
                    # Clock stretching by a device, let's wait it's over
                    wt.wait.eq(1),
                ).Elif(~pads.sda_i & pads.scl_i,
                    NextValue(pads.sda_o, 1),
                    NextState("IDLE"),
                ).Elif(pads.sda_i & ~pads.scl_i,
                    NextValue(pads.sda_o, 0),
                ).Elif(~pads.sda_i & ~pads.scl_i,
                    NextValue(pads.scl_o, 1),
                ).Else(# both at 1
                    NextValue(pads.scl_o, 0),
                ),
            ).Else(
                wt.wait.eq(1),
            ),
        )
        if with_clk_stretch:
            fsm.act("BIT",
                # |   |       Actual        |           Next          |
                # | b | SDA | SCL_i | SCL_0 | SDA | SCL |    State    |
                # |---|-----|-------|-------|-----|-----|-------------|
                # | 1 |  0  |   0   |   0   |  1  |     |             |x
                # | x |  x  |   1   |   1   |     |  0  |             |x
                # | 0 |  1  |   0   |   0   |  0  |     |             |x
                # | x |  b  |   0   |   0   |     |  1  |    IDLE     |
                # | 1 |  0  |   1   |   0   |     |     |  HW_FAULT   |
                # | x |  x  |   0   |   1   |     |     |             |
                # | 0 |  1  |   1   |   0   |     |     |  HW_FAULT   |
                # | x |  b  |   1   |   0   |     |     |  HW_FAULT   |
                If(wt.done,
                    If((bit ^ pads.sda) & ~pads.scl_i & ~pads.scl,  # Setup
                        NextValue(pads.sda_o, bit),
                    ).Elif(pads.scl_i & pads.scl,  # Falling edge
                        NextValue(pads.scl_o, 0),
                    ).Elif((bit == pads.sda) & ~pads.scl_i,  # rising edge
                        NextValue(pads.scl_o, 1),
                        NextState("IDLE"),
                    ).Elif(~pads.scl_i & pads.scl,  # clock stretching
                        wt.wait.eq(1),
                    ).Else(
                        # wt.wait.eq(1),  # more setup time required
                        # NextState("HW_FAULT"),
                    ),
                ).Else(
                    wt.wait.eq(1),
                ),
            )
        else:
            fsm.act("BIT",
                # |   |   Actual  |           Next          |
                # | b | SDA | SCL | SDA | SCL |    State    |
                # |---|-----|-----|-----|-----|-------------|
                # | 1 |  0  |  0  |  1  |     |             |
                # | x |  x  |  1  |     |  0  |             |
                # | 0 |  1  |  0  |  0  |     |             |
                # | 0 |  1  |  0  |  0  |     |             |
                # | 0 |  0  |  0  |     |  1  |    IDLE     |
                # | 1 |  1  |  0  |     |  1  |    IDLE     |
                If(wt.done,
                    If((bit ^ pads.sda_o) & ~pads.scl_o,  # Setup
                        NextValue(pads.sda_o, bit),
                    ).Elif(pads.scl_o,  # Falling edge
                        NextValue(pads.scl_o, 0),
                    ).Elif((bit == pads.sda_o) & ~pads.scl_i,  # rising edge
                        NextValue(pads.scl_o, 1),
                        NextState("IDLE"),
                    ).Elif(~pads.scl_i & pads.scl_o,  # clock stretching
                        wt.wait.eq(1),
                    ).Else(
                        # wt.wait.eq(1),  # more setup time required
                        # NextState("HW_FAULT"),
                    ),
                ).Else(
                    wt.wait.eq(1),
                ),
            )


class I2cBitOperationRx(Module):
    def __init__(self, pads_tri):
        # outputs
        self.source = source = stream.Endpoint(i2c_bit_layout)
        self.busy = busy = Signal()
        self.noise = noise = Signal()

        # # #
        prev_sda = Signal()
        prev_scl = Signal()
        sda = pads_tri.sda_i
        scl = pads_tri.scl_i

        self.sync += [
            prev_sda.eq(sda),
            prev_scl.eq(scl),
        ]
        self.comb += [
            If(prev_scl & scl & prev_sda & ~sda,
                source.valid.eq(1),
                source.first.eq(1),
            ).Elif(prev_scl & scl & ~prev_sda & sda,
                source.valid.eq(1),
                source.last.eq(1),
            ).Elif(scl & ~prev_scl & (sda == prev_sda),
                source.valid.eq(1),
                source.data.eq(sda)
            ).Elif((scl ^ prev_scl) | (sda ^ prev_sda),
                noise.eq(1),
            ),
        ]


class I2cBitOperation(Module):
    def __init__(self, pads_tri, clk_div):

        # # #
        self.submodules.tx = I2cBitOperationTx(pads_tri, clk_div)
        self.submodules.rx = I2cBitOperationRx(pads_tri)
        self.sink = self.tx.sink
        self.source = self.rx.source
        self.busy = self.tx.busy
        self.speed = self.tx.speed


class I2cByteOperationTx(Module):
    """Specialized gearbox to serialize byte-level I2C operations.

    This module partially handles multi-Master scenario thanks to the `busy` input.
    If `busy` is set when idle, this will block serialization until cleared.
    """
    def __init__(self):
        # inputs
        self.sink = sink = stream.Endpoint(i2c_byte_operation_layout)
        self.busy = busy = Signal()

        # outputs
        self.source = source = stream.Endpoint(i2c_bit_operation_layout)

        # # #
        write = Signal()
        self.submodules.gb = gb = stream.Gearbox(8, 1)
        self.submodules.fsm = fsm = FSM("IDLE")
        fsm.act("IDLE",
            If(busy,
                NextState("SLAVE"),
            ).Else(
                sink.ready.eq(source.ready),
                If(sink.first | sink.last,
                    sink.connect(source, keep=["ready", "valid", "first", "last"]),
                    If(sink.valid & sink.ready,
                        NextState("IDLE"),
                    ),
                ).Else(
                    sink.connect(gb.sink, keep=["ready", "valid", "data", "first", "last"]),
                    NextValue(write, sink.write),
                    gb.source.connect(source),
                    If(sink.valid & sink.ready,
                        NextState("TXDATA"),
                    ),
                ),
            ),
        )
        fsm.act("SLAVE",
            If(~busy,
                NextState("IDLE"),
            ),
        )
        # fsm.act("RDATA",
        #     sink.connect(gb.sink, keep=["ready", "valid", "data", "first", "last"]),
        #     If(sink.valid & sink.ready,
        #         If(sink.last,
        #             NextState("IDLE"),
        #         ).Else(
        #             NextState("TXDATA"),
        #         ),
        #     ),
        # )

        fsm.act("TXDATA",
            gb.source.connect(source),
            If(~gb.source.valid,
                NextState("ACK"),
            ),
        )
        fsm.act("ACK",
            # reading ACK needs to send a recessive 1. If the Master is reading, it then generates an ACK
            source.valid.eq(1),
            source.data.eq(write),
            If(source.ready,
                NextState("IDLE"),
            ),
        )


class I2cByteOperationRx(Module):
    """Special Gearbox to deserialize bit-level I2C operations.

    The source's payload is only valid if both last and first are cleared.
    If a transfer is aborted with a STOP, or restarted early, the gearbox resets.

    NOTE: This module doesn't introduce latency, but doesn't buffer its data to mitigate
    Backpressure. If sink.valid is set but source.ready is cleared, an undetermined amount of bits
    will be lost and source.ack will be invalid.

    NOTE: This module will only produce an output if a start (first) has been received first.
    This prevents spurious transitions to be interpreted as a valid transfer.

    In other words:
    - source.valid is set if sink.valid is set and:
      - a start or stop (first/last) is transferred from the bit interface
      - a bit is transferred from the bit interface and 8 other bits have been received.
        In this case, the data and ack payload are valid, and both first and last will be cleared.
    - sink.ready = source.ready

    Inputs:
    - sink: stream.Endpoint(i2c_bit_operation_layout) - stream of bits, Start and Stop

    Outputs:
    - source: stream.Endpoint(i2c_byte_layout) - stream of Start/Stop/Byte+Ack
    - busy: set if a Start has been detected, cleared when a Stop is detected
    - wait_ack: set when waiting for the ACK bit. At this stage, source.data is already valid.
    """
    def __init__(self):
        # inputs
        self.sink = sink = stream.Endpoint(i2c_bit_operation_layout)

        # outputs
        self.source = source = stream.Endpoint(i2c_byte_layout)
        self.busy = busy = Signal(reset=1)
        self.wait_ack = wait_ack = Signal()

        # # #
        cnt = Signal(max=9)
        data = Signal(8)
        self.submodules.fsm = fsm = FSM("IDLE")
        self.comb += [
            sink.connect(source, keep=["ready", "first", "last"]),
            source.data.eq(data),
        ]
        fsm.act("IDLE",
            busy.eq(0),
            If(sink.valid & sink.first,
                source.valid.eq(1),
                NextState("DATA"),
            ),
        )
        fsm.act("DATA",
            If(sink.valid & sink.ready,
                If(sink.first,
                    NextValue(cnt, 0),
                    source.valid.eq(1),
                ).Elif(sink.last,
                    NextValue(cnt, 0),
                    NextState("IDLE"),
                    source.valid.eq(1),
                ).Else(
                    NextValue(data, Cat(sink.data, data)),
                    If(cnt == 7,
                        NextState("ACK"),
                    ).Else(
                        NextValue(cnt, cnt + 1),
                    ),
                ),
            )
        )
        fsm.act("ACK",
            wait_ack.eq(1),
            source.valid.eq(sink.valid),
            source.ack.eq(sink.data),
            source.valid.eq(sink.valid),
            If(sink.valid & sink.ready,
                NextValue(cnt, 0),
                If(sink.last,
                    NextState("IDLE"),
                ).Else(
                    NextState("DATA"),
                ),
            ),
        )


class I2cByteOperation(Module):
    """Perform byte-level operations on the I2C bus.

    If sink.first is set, then a start is generated before the byte is sent.
    If sink.last is set, no byte is send but a stop is generated.
    """
    def __init__(self, pads_tri, clk_div):
        # # #
        self.submodules.bit = I2cBitOperation(pads_tri, clk_div)
        self.submodules.tx = I2cByteOperationTx()
        self.submodules.rx = I2cByteOperationRx()
        self.sink = self.tx.sink
        self.source = self.rx.source
        self.busy = self.bit.busy
        self.speed = self.bit.speed

        self.comb += [
            self.tx.source.connect(self.bit.tx.sink),
            self.bit.source.connect(self.rx.sink),
        ]


class I2cOperation(Module):
    """Abstracts I2C address access operation into byte operation"""
    def __init__(self):
        # inputs
        self.sink = sink = stream.Endpoint(i2c_operation_layout)

        # outputs
        self.source = source = stream.Endpoint(i2c_byte_operation_layout)

        # # #
        data = Signal(8)
        address = Signal(10)
        stop = Signal()
        read = Signal()

        self.submodules.fsm = fsm = FSM("IDLE_CPY")
        fsm.act("IDLE_CPY",
            NextValue(data, sink.data),
            NextValue(stop, sink.last),
            NextValue(read, sink.read),
            NextValue(address, sink.address),
            If(sink.first,
                sink.connect(source, keep=["first", "ready", "valid"]),
                If(source.valid & source.ready,
                    # start is sent, write the address
                    If(sink.address10,
                        NextState("ADDRESS_MSB"),
                    ).Else(
                        NextState("ADDRESS"),
                    ),
                    If(sink.write,
                        NextValue(data, sink.data),
                    ).Else(
                        NextValue(data, 0xFF),
                    ),
                )
            ).Elif(sink.read | sink.write,
                sink.connect(source, keep=["valid", "ready", "write"]),
                If(sink.write,
                    source.data.eq(sink.data),
                ).Else(
                    source.data.eq(0xFF),
                ),
                If(sink.last,
                    NextState("STOP"),
                ),
            ).Elif(sink.last,
                sink.connect(source, keep=["valid", "ready", "last"]),
            )
        )
        fsm.act("ADDRESS",
            source.data.eq(Cat(read, address[0:7])),
            source.valid.eq(1),
            If(source.ready,
                NextState("DATA"),
            ),
        )
        fsm.act("ADDRESS_MSB",
            source.valid.eq(1),
            source.data.eq(Cat(read, address[8:10], C(0b11110))),
            If(source.ready,
                NextState("ADDRESS_LSB"),
            ),
        )
        fsm.act("ADDRESS_LSB",
            source.valid.eq(1),
            source.data.eq(address[0:8]),
            If(source.ready,
                NextState("DATA"),
            ),
        )
        fsm.act("DATA",
            source.valid.eq(1),
            source.data.eq(data),
            source.write.eq(~read),
            If(source.ready,
                If(stop,
                    NextState("STOP"),
                ).Else(
                    NextState("IDLE_CPY"),
                ),
            )
        )
        fsm.act("STOP",
            source.valid.eq(1),
            source.last.eq(1),
            If(source.ready,
                NextState("IDLE_CPY"),
            ),
        )
