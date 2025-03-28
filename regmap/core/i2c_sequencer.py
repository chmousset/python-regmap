from math import ceil
from migen import Module, If, Signal, C, FSM, NextState, NextValue, Cat, Case
from migen.genlib.misc import WaitTimer
from migen.genlib.io import DDROutput
from migen.fhdl.specials import Memory, Tristate
from litex.soc.interconnect import stream
from litex.gen.fhdl.module import LiteXModule
from regmap.core.i2c import i2c_byte_operation_layout
from regmap.core.models import I2cTransaction

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
    Logic usage is minimum, but it does not support clock stretching or multi-master. It's intended
    for very simple use cases where readback isn't required.

    Example:

    bus = I2cBus()
    sensor = LM75(bus)
    transactions = [
        gpio.io.write(0xF0),
        gpio.io.read(),
    ]
    self.submodules.i2c = I2cSequencer(platform.request("i2c"), transactions, 12E6)
    
    Parameters:
    - pads: physical ios
    - transaction: `list(I2cTransaction)`
    - fclk: cd_sys clock frequency
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
                for i in range(tr.read):
                    sdascl.byte(0xFF)
                    sdascl.bit(1 if i == len(tr.read) - 1 else 0)  # ACK all but last
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



I2C_REGMAP_OP_DATA = (0b0 << 8)
I2C_REGMAP_OP_CMD = (0b1 << 8)

I2C_REGMAP_OP_CMD_DONE = 0b000
I2C_REGMAP_OP_CMD_SET_PARAM = 0b001
I2C_REGMAP_OP_CMD_READ = 0b100
I2C_REGMAP_OP_CMD_START_WRITE = 0b101
I2C_REGMAP_OP_CMD_STOP = 0b110
I2C_REGMAP_OP_CMD_SET_MUX = 0b111


def _OpData(data):
    if type(data) == C:
        data = data.value
    data &= 0xff
    return [data | I2C_REGMAP_OP_DATA]


def _OpSetParam(param):
    if type(param) == C:
        param = param.value
    param &= 0xff
    return [I2C_REGMAP_OP_CMD_SET_PARAM | I2C_REGMAP_OP_CMD, param]


def _OpCmd(cmd=0):
    if type(cmd) == C:
        cmd = cmd.value
    cmd &= 0xff
    return [cmd | I2C_REGMAP_OP_CMD]


def OpCmdStop():
    """Generate a Stop condition"""
    return _OpCmd(I2C_REGMAP_OP_CMD_STOP)


def OpCmdDone():
    """Set the `I2cRegmap` in 'IDLE' mode

    Also sets `I2cRegmap.done`. If `I2cRegmap.enable` is set, the sequence restarts from the
    beginning.
    """
    return _OpCmd(I2C_REGMAP_OP_CMD_DONE)


def OpCmdRead(count):
    """Read bytes

    Parameters:
    - count: [0, 255] - (number of bytes to read - 1)
    """
    assert count >= 0 and count < 256
    return _OpSetParam(count) + _OpCmd(I2C_REGMAP_OP_CMD_READ)


def OpCmdStartWrite(bytes_write):
    """Generate Start, setup write

    Parameters:
    - bytes_write: list of bytes to write
    """
    return _OpCmd(I2C_REGMAP_OP_CMD_START_WRITE) + [_OpData(b)[0] for b in bytes_write]


def OpCmdSetMux(mux_id):
    """Set stream Mux

    Parameters:
    - mux_id: id of the mux
    """
    assert mux_id >= 0 and mux_id < 256
    return _OpSetParam(mux_id) + _OpCmd(I2C_REGMAP_OP_CMD_SET_MUX)


class MemReader(Module):
    """Read a ROM to a stream.

    Parameters:
    - layout: layout of the output stream

    Inputs:
    - reset: when set, the read address is cleared.

    Outputs:
    - source: `Endpoint(layout)`. When `source.valid & source.ready`, auto-increment address.
    """
    def __init__(self, mem, layout):
        # Inputs
        self.reset = reset = Signal()

        # Outputs
        self.source = source = stream.Endpoint(layout)

        # # #
        self.specials.memops = mem
        self.specials.rp = rp = mem.get_port(has_re=True)
        self.comb += source.payload.raw_bits().eq(rp.dat_r)
        self.submodules.fsm = fsm = FSM("SETUP")
        fsm.act("SETUP",
            If(reset,
                NextValue(rp.adr, 0),
            ).Else(
                rp.re.eq(1),
                NextState("READ"),
            ),
        )
        fsm.act("READ",
            source.valid.eq(~reset),
            If(reset,
                NextValue(rp.adr, 0),
                NextState("SETUP"),
            ).Elif(source.valid & source.ready,
                NextValue(rp.adr, rp.adr + 1),
                NextState("SETUP"),
            ),
        )


class I2cRegmap(LiteXModule):
    """Map I2C registers to individual streams

    This module controls an `I2cBytesOperationRTx` instance to perform a sequence of actions on an
    I2C bus. The sequence of action can include fixed actions or can mux/demux I2C data stream into
    multiple `I2cReg.source` streams. This way, both fixed configuration and full communication with
    I2C devices can be acheived.

    Inputs:
    - sink: `Endpoint(i2c_byte_operation_layout)` - connect to an `I2cBytesOperationRTx.source`
    - enable: when set, the sequence is played
    - reset: when set, stop operation. If `enable` is then set, it restarts from the beginning.

    Outputs:
    - source: `Endpoint(i2c_byte_operation_layout)` - connect to an `I2cBytesOperationRTx.sink`
    - done: set when the sequnce has been entirely played. Cleared after `.enable` has been set.
    """

    _op_layout = [("data", 8), ("is_cmd", 1)]

    def __init__(self):
        # Inputs
        self.sink = sink = stream.Endpoint(i2c_byte_operation_layout)
        self.enable = enable = Signal()
        self.reset = Signal()

        # Outputs
        self.source = source = stream.Endpoint(i2c_byte_operation_layout)
        self.done = done = Signal()

        # # #
        self._tr = []
        self._streams = []
        self._op_sink = op_sink = stream.Endpoint(self._op_layout)
        self._read_source = read_source = stream.Endpoint([("data", 8)])
        self._write_sink = write_sink = stream.Endpoint([("data", 8)])
        self._mux_sel = mux_sel = Signal(8)

        cmd = Signal(3)
        data = Signal(8)
        is_cmd = Signal()
        mux_en = Signal()
        self.comb += [
            cmd.eq(op_sink.data[0:3]),
            data.eq(op_sink.data[0:8]),
            is_cmd.eq(op_sink.is_cmd),
        ]
        is_write = Signal()
        param = Signal(8)

        self.submodules.fsm = fsm = FSM("IDLE")
        fsm.act("IDLE",
            sink.ready.eq(1),  # purge
            If(enable,
                NextValue(done, 0),
                NextState("READ_OP"),
            ),
        )
        fsm.act("READ_OP",
            op_sink.ready.eq(1),
            sink.ready.eq(1),  # purge
            If(op_sink.ready & op_sink.valid,
                If(is_cmd & (cmd == I2C_REGMAP_OP_CMD_DONE),
                    NextValue(done, 1),
                    NextState("IDLE"),
                ).Elif(is_cmd & (cmd == I2C_REGMAP_OP_CMD_SET_PARAM),
                    NextState("SET_PARAM"),
                ).Elif(is_cmd & (cmd == I2C_REGMAP_OP_CMD_READ),
                    NextValue(is_write, 0),
                    NextState("DATA_READ"),
                ).Elif(is_cmd & (cmd == I2C_REGMAP_OP_CMD_START_WRITE),
                    NextValue(is_write, 1),
                    NextState("START_WRITE"),
                ).Elif(is_cmd & (cmd == I2C_REGMAP_OP_CMD_STOP),
                    NextState("STOP"),
                ).Elif(is_cmd & (cmd == I2C_REGMAP_OP_CMD_SET_MUX),
                    NextState("SET_MUX"),
                ).Else(
                    NextValue(param, data),
                ),
            ),
        )
        fsm.act("SET_PARAM",
            op_sink.ready.eq(1),
            If(op_sink.valid & op_sink.ready,
                NextValue(param, op_sink.data),
                NextState("READ_OP"),
            ),
        )
        fsm.act("START_WRITE",
            source.valid.eq(1),
            source.first.eq(1),
            sink.ready.eq(1),
            If(source.valid & source.ready,
                NextState("DATA_WRITE"),
            ),
        )
        fsm.act("DATA_WRITE",
            source.write.eq(1),
            If(mux_en,
                write_sink.connect(source, keep=["ready", "valid", "data"]),
                If(write_sink.valid & write_sink.ready & write_sink.last,
                    NextValue(mux_en, 0),  # when landing in WRITE_ACK, next op should be a cmd
                    NextState("WRITE_ACK"),
                ),
            ).Else(
                op_sink.connect(source, keep=["ready"]),
                source.data.eq(data),
                source.valid.eq(op_sink.valid & ~is_cmd),
            ),
            If(source.valid & source.ready,
                NextState("WRITE_ACK"),
            ),
        )
        fsm.act("WRITE_ACK",
            source.is_ack.eq(1),
            source.ack.eq(1),
            If(mux_en,
                source.valid.eq(1),
                If(sink.valid & sink.ready & sink.is_ack,
                    If(sink.ack,
                        NextState("READ_OP"),
                    ).Else(
                        NextState("DATA_WRITE"),
                    ),
                ),
            ).Else(
                If(op_sink.valid,
                    sink.ready.eq(1),
                    source.valid.eq(1),
                    If(sink.valid & sink.ready & sink.is_ack,
                        If(is_cmd | sink.ack,  # next: we send a command, or the write was NAK
                            NextState("READ_OP"),
                        ).Else(
                            NextState("DATA_WRITE"),
                        ),
                    ),
                ),
            ),
        )
        fsm.act("DATA_READ",
            If(mux_en,
                sink.connect(read_source, keep=["ready", "valid"]),
                read_source.data.eq(data),
                source.valid.eq(read_source.ready),
            ).Else(
                source.valid.eq(1),
            ),
            If(sink.valid & sink.ready,
                NextState("READ_ACK"),
            ),
        )
        fsm.act("READ_ACK",
            source.valid.eq(1),
            source.is_ack.eq(1),
            If(param == 0,  # last byte to read: NAK
                NextValue(mux_en, 0),
                source.ack.eq(1),
                If(source.valid & source.ready,
                    NextState("READ_OP"),
                ),
            ).Else(
                If(source.valid & source.ready,
                    NextValue(param, param - 1),
                    NextState("DATA_READ"),
                ),
            ),
        )
        fsm.act("STOP",
            source.valid.eq(1),
            source.last.eq(1),
            If(source.valid & source.ready,
                NextState("READ_OP"),
            ),
        )
        fsm.act("SET_MUX",
            NextValue(mux_sel, param),
            NextValue(mux_en, 1),
            NextState("READ_OP"),
        )
        fsm.act("MUX_ACK",
            If(is_write,
                write_sink.connect(source, keep=["data"]),
                source.write.eq(1),
                If(source.valid & source.ready,
                    NextState("MUX_ACK"),
                ),
            ).Else(
                source.valid.eq(1),
            ),
        )

    def add_transaction(self, transactions):
        if type(transactions) == I2cTransaction:
            transactions = [transactions]
        for tr in transactions:
            self._tr += OpCmdStartWrite(tr.device.raw_address_bytes(False) + tr.write)
            if tr.read:
                self._tr += OpCmdStartWrite(tr.device.raw_address_bytes(True))
                if tr.read_stream:
                    self._tr += OpCmdSetMux(len(self._streams))
                    self._streams += [tr.read_stream]
                self._tr += OpCmdRead(tr.read - 1)
            self._tr += OpCmdStop()

    def do_finalize(self):
        self._tr += OpCmdDone()
        mem_depth = len(self._tr)
        mem = Memory(init=self._tr, depth=mem_depth, width=10)
        self.submodules.reader = reader = MemReader(mem, self._op_layout)
        self.comb += [
            reader.source.connect(self._op_sink),
            reader.reset.eq(self.done),
            Case(self._mux_sel, {i: self._read_source.connect(s) for i, s in enumerate(self._streams)}),
        ]
        self.fsm.comb += If(self.reset, self.fsm.next_state.eq(self.fsm.state.reset))
        reader.finalize()

    def print(self):
        param = 0
        l = "I2C Regmap commands: [\n"
        for i, op in enumerate(self._tr):
            cmd = op >> 8
            op &= 0xFF
            if cmd:
                l += f"  {i:3}: "+ {
                    I2C_REGMAP_OP_CMD_DONE: "DONE",
                    I2C_REGMAP_OP_CMD_SET_PARAM: "SET_PARAM",
                    I2C_REGMAP_OP_CMD_READ: "READ",
                    I2C_REGMAP_OP_CMD_START_WRITE: "START_WRITE",
                    I2C_REGMAP_OP_CMD_STOP: "STOP",
                    I2C_REGMAP_OP_CMD_SET_MUX: "SET_MUX",
                }.get(op, f"?{op}") + ",\n"
            else:
                l += f"  {i:3}: DATA {op},\n"
                param = op
        print(f"{l}]")
