from migen import *
from litex.soc.interconnect import stream

i2c_bit_layout = [
    ("data", 1),
]

i2c_bit_operation_layout = i2c_bit_layout + [
    # ("write", 1),
]

i2c_byte_layout = [
    ("data", 8),
    ("ack", 1),
]

i2c_byte_operation_layout = [
    ("data", 8),
    # ("write", 1),
]

i2c_operation_layout = [
    ("data", 8),
    ("address", 10),
    ("read", 1),
    ("address10", 1),
]


class I2cPads(Module):
    def __init__(self, pads):
        signals = ["sda", "scl"]
        for s in signals:
            self.t = t = TSTriple(1)
            setattr(self, s, Signal(reset=1))
            setattr(self, s+"_i", Signal())
            sig_out = getattr(self, s)
            sig_in = getattr(self, s+"_i")
            self.comb += [
                # sig_in.eq(t.i),
                t.o.eq(sig_out),
                t.oe.eq(~sig_out),
            ]
            self.specials += t.get_tristate(getattr(pads, s))
            self.specials += MultiReg(t.i, sig_in),


class I2CTimer(Module):
    def __init__(self, clk_div):
        # inputs
        self.speed = speed = Signal(max=len(clk_div) + 1)
        self.wait = wait = Signal()

        # outputs
        self.done = done = Signal()

        # # #
        cnt = Signal(max=max(clk_div))
        print(f"cnt={cnt}")
        self.comb += done.eq(cnt == 0)
        self.sync += [
            If(~wait,
                Case(speed, {k: [cnt.eq(div)] for k, div in enumerate(clk_div)}).makedefault(0)
            ).Elif(~done,
                cnt.eq(cnt - 1),
            ),
        ]


class I2cBitOperationTx(Module):
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
        pads.sda.reset = 1
        pads.scl.reset = 1

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
                    NextValue(pads.scl, 0),
                ).Elif(pads.sda_i & ~pads.scl_i,
                    NextValue(pads.scl, 1),
                ).Elif(~pads.sda_i & ~pads.scl_i,
                    NextValue(pads.sda, 1),
                ).Else(# both at 1
                    NextValue(pads.sda, 0),
                    NextState("IDLE"),
                ),
            ).Else(
                wt.wait.eq(1),
            ),
        )
        fsm.act("STOP",
# |     Actual          |           Next          |
# | SDA_i | SDA | SCL | SDA | SCL |    State    |
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
                If(~pads.scl_i & pads.scl,
                    # Clock stretching by a device, let's wait it's over
                    wt.wait.eq(1),
                ).Elif(~pads.sda_i & pads.scl_i,
                    NextValue(pads.sda, 1),
                    NextState("IDLE"),
                ).Elif(pads.sda_i & ~pads.scl_i,
                    NextValue(pads.sda, 0),
                ).Elif(~pads.sda_i & ~pads.scl_i,
                    NextValue(pads.scl, 1),
                ).Else(# both at 1
                    NextValue(pads.scl, 0),
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
                        NextValue(pads.sda, bit),
                    ).Elif(pads.scl_i & pads.scl,  # Falling edge
                        NextValue(pads.scl, 0),
                    ).Elif((bit == pads.sda) & ~pads.scl_i,  # rising edge
                        NextValue(pads.scl, 1),
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
                    If((bit ^ pads.sda) & ~pads.scl,  # Setup
                        NextValue(pads.sda, bit),
                    ).Elif(pads.scl,  # Falling edge
                        NextValue(pads.scl, 0),
                    ).Elif((bit == pads.sda) & ~pads.scl_i,  # rising edge
                        NextValue(pads.scl, 1),
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
    def __init__(self):
        # inputs
        self.sink = sink = stream.Endpoint(i2c_byte_operation_layout)
        self.busy = busy = Signal()

        # outputs
        self.source = source = stream.Endpoint(i2c_bit_operation_layout)

        # # #
        self.submodules.gb = gb = stream.Gearbox(8, 1)
        self.submodules.fsm = fsm = FSM("IDLE")
        fsm.act("IDLE",
            If(busy,
                NextState("SLAVE"),
            ).Else(
                sink.ready.eq(source.ready),
                If(sink.valid & sink.ready,
                    If(sink.first | sink.last,
                        sink.connect(source, keep=["ready", "valid", "first", "last"]),
                        source.valid.eq(1),
                        NextState("IDLE")
                    ).Else(
                        NextState("RDATA"),
                    )
                ),
            ),
        )
        fsm.act("SLAVE",
            If(~busy,
                NextState("IDLE"),
            ),
        )
        fsm.act("RDATA",
            sink.connect(gb.sink, keep=["ready", "valid", "data", "first", "last"]),
            If(sink.valid & sink.ready,
                If(sink.last,
                    NextState("IDLE"),
                ).Else(
                    NextState("TXDATA"),
                ),
            ),
        )

        fsm.act("TXDATA",
            gb.source.connect(source),
            If(~gb.source.valid,
                NextState("ACK"),
            ),
        )
        fsm.act("ACK",
            source.valid.eq(1),
            source.data.eq(1),  # reading ACK needs to send a recessive 1
            If(source.ready,
                NextState("IDLE"),
            ),
        )


class I2cByteOperationRx(Module):
    def __init__(self):
        # inputs
        self.sink = sink = stream.Endpoint(i2c_byte_operation_layout)

        # outputs
        self.source = source = stream.Endpoint(i2c_byte_layout)
        self.busy = busy = Signal(reset=1)

        # # #
        self.submodules.gb = gb = stream.Gearbox(1, 8)
        self.submodules.fsm = fsm = FSM("IDLE")
        fsm.act("IDLE",
            busy.eq(0),
            sink.connect(source, keep=["ready", "valid", "first", "last"]),
            If(sink.valid & sink.ready & sink.first,
                NextState("DATA"),
            ),
        )
        fsm.act("DATA",
            If((sink.last | sink.first) & sink.valid,
                sink.connect(source, keep=["ready", "valid", "first", "last"]),
                If(sink.last,
                    NextState("IDLE"),
                ),
            ).Else(
                sink.connect(gb.sink, keep=["ready", "valid", "data"]),
                gb.source.ready.eq(1),
                If(gb.source.valid,
                    NextValue(source.data, gb.source.data),
                    NextState("ACK"),
                ),
            ),
        )
        fsm.act("ACK",
            sink.ready.eq(1),
            source.valid.eq(sink.valid),
            source.ack.eq(sink.data),
            If(sink.valid,
                NextState("DATA"),
            ),
        )


class I2cByteOperation(Module):
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
    def __init__(self):
        # inputs
        self.sink = sink = stream.Endpoint(i2c_operation_layout)

        # outputs
        self.source = source = stream.Endpoint(i2c_byte_operation_layout)

        # # #
        data = Signal(8)
        address = Signal(10)
        stop = Signal()

        self.submodules.fsm = fsm = FSM("IDLE")
        fsm.act("IDLE",
            sink.ready.eq(source.ready),
            If(sink.valid,
                NextValue(stop, sink.last),
                NextValue(address, sink.address),
                If(sink.read,
                    NextValue(data, C(0xFF)),
                ).Else(
                    NextValue(data, sink.data),
                ),
                If(sink.first,
                    source.first.eq(1),
                    source.valid.eq(1),  # generate the START right away
                    If(sink.address10,
                        NextState("ADDRESS_MSB"),
                    ).Else(
                        NextState("ADDRESS"),
                    ),
                ).Else(
                    NextState("DATA"),
                ),
            ),
        )
        fsm.act("ADDRESS",
            source.data.eq(Cat(sink.read, address[0:7])),
            source.valid.eq(1),
            If(source.ready,
                NextState("DATA"),
            ),
        )
        fsm.act("ADDRESS_MSB",
            source.valid.eq(1),
            source.data.eq(Cat(sink.read, address[8:10], C(0b11110))),
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
            If(source.ready,
                If(stop,
                    NextState("STOP"),
                ).Else(
                    NextState("IDLE"),
                ),
            )
        )
        fsm.act("STOP",
            source.valid.eq(1),
            source.last.eq(data),
            If(source.ready,
                NextState("IDLE"),
            ),
        )
