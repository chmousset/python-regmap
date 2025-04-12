from math import ceil
from migen import Module, If, Signal, FSM, NextState, NextValue, Cat
from litex.soc.interconnect import stream

one_wire_bit_layout = [
    ("data", 1),
]

one_wire_bit_operation_layout = one_wire_bit_layout + [
    ("overdrive", 1),
]

one_wire_byte_layout = [
    ("data", 8),
]

one_wire_byte_operation_layout = one_wire_byte_layout + [
    ("overdrive", 1),
]

# https://www.analog.com/en/resources/technical-articles/1wire-communication-through-software.html
one_wire_timing = {
    "standard": {
        "reset": {
            "tSTART": 480E-6,
            "tSAMP": 70E-6,
            "tSTOP": 410E-6 / 2,
            "tREC": 410E-6 / 2,
        },
        "bit": {
            "tSTART": 6E-6,
            "tSAMP": 9E-6,
            "tSTOP": 55E-6 - 10E-6,
            "tREC": 10E-6,
        }
    },
    "overdrive": {
        "reset": {
            "tSTART": 70E-6,
            "tSAMP": 8.5E-6,
            "tSTOP": 40E-6 / 2,
            "tREC": 40E-6 / 2,
        },
        "bit": {
            "tSTART": 1E-6,
            "tSAMP": 1E-6,
            "tSTOP": 7E-6 - 2.5E-6,
            "tREC": 2.5E-6,
        }
    }
}


class OneWireTiming(Module):
    def __init__(self, fclk):
        max_timing = dict()
        for speed in one_wire_timing.values():
            for mode in speed.values():
                for segment, duration in mode.items():
                    max_timing[segment] = max(max_timing.get(segment, 0), ceil(fclk * duration))

        # inputs
        self.overdrive = Signal()
        self.reset_cmd = Signal()

        # outputs
        for segment, max_duration in max_timing.items():
            setattr(self, segment, Signal(max=max_duration))

        # # #
        self.comb += [
            If(self.overdrive,  # overdrive speed
                If(self.reset_cmd,  # reset command
                    self.tSTART.eq(ceil(fclk * one_wire_timing["overdrive"]["reset"]["tSTART"])),
                    self.tSAMP.eq(ceil(fclk * one_wire_timing["overdrive"]["reset"]["tSAMP"])),
                    self.tSTOP.eq(ceil(fclk * one_wire_timing["overdrive"]["reset"]["tSTOP"])),
                    self.tREC.eq(ceil(fclk * one_wire_timing["overdrive"]["reset"]["tREC"])),
                ).Else(  # bit command
                    self.tSTART.eq(ceil(fclk * one_wire_timing["overdrive"]["bit"]["tSTART"])),
                    self.tSAMP.eq(ceil(fclk * one_wire_timing["overdrive"]["bit"]["tSAMP"])),
                    self.tSTOP.eq(ceil(fclk * one_wire_timing["overdrive"]["bit"]["tSTOP"])),
                    self.tREC.eq(ceil(fclk * one_wire_timing["overdrive"]["bit"]["tREC"])),
                ),
            ).Else(  # standard speed
                If(self.reset_cmd,
                    self.tSTART.eq(ceil(fclk * one_wire_timing["standard"]["reset"]["tSTART"])),
                    self.tSAMP.eq(ceil(fclk * one_wire_timing["standard"]["reset"]["tSAMP"])),
                    self.tSTOP.eq(ceil(fclk * one_wire_timing["standard"]["reset"]["tSTOP"])),
                    self.tREC.eq(ceil(fclk * one_wire_timing["standard"]["reset"]["tREC"])),
                ).Else(
                    self.tSTART.eq(ceil(fclk * one_wire_timing["standard"]["bit"]["tSTART"])),
                    self.tSAMP.eq(ceil(fclk * one_wire_timing["standard"]["bit"]["tSAMP"])),
                    self.tSTOP.eq(ceil(fclk * one_wire_timing["standard"]["bit"]["tSTOP"])),
                    self.tREC.eq(ceil(fclk * one_wire_timing["standard"]["bit"]["tREC"])),
                ),
            ),
        ]


class InputFilter(Module):
    def __init__(self, count):
        self.i = i = Signal()
        self.o = o = Signal()
        bits = Signal(count)
        self.sync += [
            bits.eq(Cat(i, bits)),
            If(bits == 0,
                o.eq(0),
            ),
            If(bits == (1 << count) - 1,
                o.eq(1),
            ),
        ]


class WaitTimer(Module):
    def __init__(self, period):
        self.wait = wait = Signal()
        self.done = done = Signal()
        if isinstance(period, Signal):
            cnt = Signal(period.nbits)
        else:
            cnt = Signal(max=period)
        self.comb += done.eq(cnt == 0)
        self.sync += [
            If(wait & ~done,
                If(cnt,
                    cnt.eq(cnt - 1),
                ),
            ).Elif(~wait,
                cnt.eq(period),
            ),
        ]


class OneWireBitOperation(Module):
    def __init__(self, pad_tri, fclk):
        # inputs
        self.sink = sink = stream.Endpoint(one_wire_bit_operation_layout)

        # outputs
        self.source = source = stream.Endpoint(one_wire_bit_layout)
        self.busy = busy = Signal()

        # # #
        self.submodules.timing = timing = OneWireTiming(fclk)
        period = Signal(max(timing.tSTART.nbits, timing.tSAMP.nbits, timing.tREC.nbits))
        bit = Signal()
        self.submodules.wt = wt = WaitTimer(period)
        self.submodules.filt = filt = InputFilter(4)
        self.comb += filt.i.eq(pad_tri.i)
        self.sync += [
            If(source.valid & source.ready,
                source.valid.eq(0),
            ),
        ]

        self.submodules.fsm = fsm = FSM("IDLE")
        fsm.act("IDLE",
            sink.ready.eq(1),
            If(sink.valid,
                NextState("START_PREP"),
                NextValue(bit, sink.data),
                NextValue(timing.reset_cmd, sink.first),
                NextValue(timing.overdrive, sink.overdrive),
            ),
        )
        fsm.act("START_PREP",
            wt.wait.eq(~wt.done),
            period.eq(timing.tSTART),
            NextState("START"),
        )
        fsm.act("START",
            busy.eq(1),
            wt.wait.eq(~wt.done),
            pad_tri.oe.eq(1),
            pad_tri.o.eq(0),
            If(wt.done,
                period.eq(timing.tSAMP),
                NextState("SAMP"),
            ),
        )
        fsm.act("SAMP",
            busy.eq(1),
            wt.wait.eq(~wt.done),
            pad_tri.oe.eq(~bit),
            pad_tri.o.eq(0),
            If(wt.done,
                period.eq(timing.tSTOP),
                NextState("STOP"),
                NextValue(source.data, filt.o),
                NextValue(source.valid, 1),
            ),
        )
        fsm.act("STOP",
            busy.eq(1),
            wt.wait.eq(~wt.done),
            pad_tri.oe.eq(~bit),
            pad_tri.o.eq(0),
            If(wt.done,
                period.eq(timing.tREC),
                NextState("REC"),
            ),
        )
        fsm.act("REC",
            busy.eq(1),
            wt.wait.eq(1),
            If(wt.done,
                If(~source.valid,
                    NextState("IDLE"),
                ),
            ),
        )


class OneWireByteOperation(Module):
    def __init__(self):
        # inputs
        self.sink = sink = stream.Endpoint(one_wire_byte_operation_layout)
        self.sink_bit = sink_bit = stream.Endpoint(one_wire_bit_layout)

        # outputs
        self.source = source = stream.Endpoint(one_wire_byte_layout)
        self.source_bit = source_bit = stream.Endpoint(one_wire_bit_operation_layout)

        # # #
        overdrive = Signal()
        self.submodules.fsm = fsm = FSM("IDLE")
        self.submodules.gb_tx = gb_tx = stream.Gearbox(8, 1, msb_first=False)
        self.submodules.gb_rx = gb_rx = stream.Gearbox(1, 8, msb_first=False)
        self.comb += source_bit.overdrive.eq(overdrive)

        fsm.act("IDLE",
            If(sink.valid,
                If(sink.first,
                    source_bit.overdrive.eq(sink.overdrive),
                    NextValue(overdrive, sink.overdrive),
                    NextState("RESET"),
                ).Else(
                    NextState("DATA_PREP"),
                ),
            )
        )
        fsm.act("RESET",
            source_bit.valid.eq(1),
            source_bit.first.eq(1),
            source_bit.data.eq(1),
            sink_bit.ready.eq(1),
            If(sink_bit.valid,
                NextState("DATA_PREP"),
            ),
        )
        fsm.act("DATA_PREP",
            sink.connect(gb_tx.sink, omit=["first", "overdrive"]),
            If(sink.valid & sink.ready,
                NextState("DATA"),
            ),
        )
        fsm.act("DATA",
            gb_tx.source.connect(source_bit),
            sink_bit.connect(gb_rx.sink, omit=["overdrive"]),
            gb_rx.source.connect(source),
            If(source.valid & source.ready,
                NextState("IDLE"),
            ),
        )
