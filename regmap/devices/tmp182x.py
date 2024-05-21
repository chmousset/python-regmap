from migen import Module, C, Signal, If, FSM, Record, Case, NextValue, NextState, Cat
from migen.fhdl.specials import Memory
from regmap.core.onewire import one_wire_byte_layout, one_wire_byte_operation_layout
from litex.soc.interconnect import stream


class TMP182x(Module):
    def __init__(self):
        # inputs
        self.sink = sink = stream.Endpoint(one_wire_byte_layout)

        # outputs
        self.source = source = stream.Endpoint(one_wire_byte_operation_layout)
        self.temp = temp = stream.Endpoint([("temp", 16)])

        # # #
        self.comb += source.overdrive.eq(1)
        self.submodules.fsm = fsm = FSM("IDLE")
        from litex.gen.genlib.misc import WaitTimer
        self.submodules.wt = wt = WaitTimer(100000)
        fsm.act("IDLE",
            source.first.eq(1),
            source.data.eq(0xCC),  # skipaddress: single device on bus
            source.valid.eq(1),
            sink.ready.eq(1),
            If(sink.valid,
                NextState("CONVERT"),
            ),
        )
        fsm.act("CONVERT",
            source.data.eq(0x44),  # start conversion
            source.valid.eq(1),
            sink.ready.eq(1),  # ignore incoming data
            If(sink.valid,
                # NextState("READ_SCRATCHPAD-0"),
                NextState("WAIT_CONVERSION"),
            ),
        )
        fsm.act("WAIT_CONVERSION",
            wt.wait.eq(1),
            If(wt.done,
                NextState("READ_SCRATCHPAD-0"),
            ),
        )
        fsm.act("READ_SCRATCHPAD-0",
            source.first.eq(1),
            source.data.eq(0xCC),  # skipaddress: single device on bus
            source.valid.eq(1),
            sink.ready.eq(1),  # ignore incoming data
            If(sink.valid,
                NextState("READ_SCRATCHPAD-1"),
            ),
        )
        fsm.act("READ_SCRATCHPAD-1",
            source.data.eq(0xBE),  # Send READ SCRATCHPAD-1 command
            source.valid.eq(1),
            sink.ready.eq(1),
            If(sink.valid,
                NextState("READ_SCRATCHPAD-2"),
            ),
        )
        fsm.act("READ_SCRATCHPAD-2",
            source.data.eq(0xFF),  # read temp LSB
            source.valid.eq(1),
            sink.ready.eq(1),
            If(sink.valid,
                NextValue(temp.temp[0:8], sink.data),
                NextState("READ_SCRATCHPAD-3"),
            ),
        )
        fsm.act("READ_SCRATCHPAD-3",
            source.data.eq(0xFF),  # read temp MSB
            source.valid.eq(1),
            sink.ready.eq(1),
            If(sink.valid,
                NextValue(temp.temp[8:16], sink.data),
                NextState("READ_SCRATCHPAD-4"),
            ),
        )
        fsm.act("READ_SCRATCHPAD-4",
            source.data.eq(0xFF),  # read status
            source.valid.eq(1),
            sink.ready.eq(1),
            If(sink.valid,
                NextState("IDLE"),
                If(sink.data[3],
                    temp.valid.eq(1),
                ),
            ),
        )


class TMP1826(TMP182x):
    pass
