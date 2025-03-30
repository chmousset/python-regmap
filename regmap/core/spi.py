from litex.soc.interconnect.csr import *
from math import ceil, log2
from migen import Module, C, Signal, If, FSM, Record, Case, NextValue, NextState, Cat
from litex.soc.interconnect import stream
from litex.gen.genlib.misc import WaitTimer


spi_bit_csyn_layout = [
    ("update", 1),
    ("sample", 1),
    ("set_cs", 1),
    ("clear_cs", 1),
]

spi_bit_ctrl_layout = [
    ("cpol", 1),
    ("cpha", 1),
]

def spi_ctrl_layout(dw):
    return [
        ("cpol", 1),
        ("cpha", 1),
        ("width", ceil(log2(dw))),
    ]

def spi_layout(dw):
    return [
        ("data", dw),
    ]


class SpiClkSync(Module):
    def __init__(self, sys_fcy, min_fcy, cpol, cpha):
        self.csyn = csyn = Record(spi_bit_csyn_layout)
        self.sclk = sclk = Signal()

        # # #
        self.submodules.wt = wt = WaitTimer(max(int(sys_fcy // min_fcy // 2) - 1, 1))
        sclk_edge = wt.done
        self.comb += wt.wait.eq(~wt.done)
        self.sync += If(wt.done, sclk.eq(~sclk))
        set_cs = Signal()
        clear_cs = Signal()
        self.comb += [
            csyn.update.eq(sclk_edge & (sclk ^ cpol ^ cpha)),
            csyn.sample.eq(sclk_edge & ~(sclk ^ cpol ^ cpha)),
            If(cpha,
                set_cs.eq(csyn.sample),
                clear_cs.eq(csyn.sample),
            ).Else(
                set_cs.eq(csyn.update),
                clear_cs.eq(csyn.update),
            ),
        ]
        self.sync += [
            csyn.set_cs.eq(set_cs),
            csyn.clear_cs.eq(clear_cs),
        ]


class SpiMaster(Module):
    """SPI Master

    Full duplex SPI Master. Takes mosi commands on sink and outputs miso readback on source.

    parameters:
    - `sclk_output_idle`: if True, the master generates a continuous clock on sclk, but also returns
      to "IDLE" state if `sink.valid == 0` when `sink.ready == 1`.

    inputs:
    - `sink`: Control stream
      - `data[dw]`: the payload. `data[width]` is the first bit transmitted on MOSI
      - `width[log2n(dw)]`: the lenght of the transfer, minus 1 (ex: 15 => 16 bits transmitted)
      - `last`: if set, the master returns in "IDLE" after transmissing the payload
      - `cpol`: clock polarity
      - `cpha`: clock edge
      - `first`: passed to `source`, optional 

    outputs:
    - `source`:
      - `data`: the payload. `data[width]` is the first bit received on MISO
      - `first`: the value of `sink.first` transmitted
      - `last`: the value of `sink.last` transmitted
    - busy: 1 when the serialiser/deser are working. Can be used to drive the Chip Select lines
    """
    def __init__(self, sys_fcy, min_fcy, pads, dw, sclk_output_idle=0):
        # inputs
        self.sink = sink = stream.Endpoint(spi_layout(dw) + spi_ctrl_layout(dw))

        # outputs
        self.source = source = stream.Endpoint(spi_layout(dw))
        self.busy = busy = Signal()

        # # #
        cpol = Signal()
        cpha = Signal()
        start_with_cpha = Signal()
        self.submodules.csyn = csyn = SpiClkSync(sys_fcy, min_fcy, cpol, cpha)
        self.comb += [
            If(sclk_output_idle,
                pads.sclk.eq(csyn.sclk),
            ).Else(
                pads.sclk.eq((csyn.sclk & busy) | cpol),
            ),
        ]
        csyn = csyn.csyn
        if hasattr(pads, "cs"):
            self.sync += pads.cs.eq(~busy)

        # bufferred inputs
        tx = Signal(dw)
        first = Signal()
        last = Signal()

        # bit transmit / receive
        rx = Signal(dw)
        bit_sel = Signal().like(sink.width)
        self.comb += [
            Case(bit_sel, {
                i: pads.mosi.eq(tx[i]) for i in range(dw)
            }),
            source.data.eq(rx),
            source.first.eq(first),
            source.last.eq(last),
        ]

        self.submodules.fsm = fsm = FSM("FETCH")
        fsm.act("FETCH",
            sink.ready.eq(~busy | csyn.update),
            If(sink.valid & sink.ready,
                NextValue(tx, sink.data),
                NextValue(first, sink.first),
                NextValue(last, sink.last),
                NextValue(cpol, sink.cpol),
                NextValue(cpha, sink.cpha),
                NextValue(bit_sel, sink.width),
                NextValue(start_with_cpha, sink.cpha & ~busy),
                If(busy,
                    NextState("TRANSFER"),
                ).Else(
                    NextState("SET_CS"),
                ),
            ),
            If(source.ready,
                NextValue(source.valid, 0),
            ),
        )
        fsm.act("SET_CS",
            If(csyn.set_cs,
                NextState("TRANSFER"),
                NextValue(busy, 1),
            ),
        )
        fsm.act("TRANSFER",
            If(csyn.sample,
                Case(bit_sel, {
                    i: NextValue(rx[i], pads.miso) for i in range(dw)  # it's OK since MISO should be sync with SCLK
                }),
                If(bit_sel == 0,
                    NextValue(source.valid, 1),
                    If(last,
                        NextState("CLEAR_CS"),
                    ).Else(
                        NextState("FETCH"),
                    ),
                ).Else(
                    NextValue(source.valid, 0),
                ),
            ),
            If(csyn.update,
                If(start_with_cpha,
                    NextValue(start_with_cpha, 0),
                ).Else(
                    NextValue(bit_sel, bit_sel - 1),
                ),
            ),
        )
        fsm.act("CLEAR_CS",
            If(source.ready,
                NextValue(source.valid, 0),
            ),
            If(csyn.clear_cs,
                NextValue(busy, 0),
                NextState("FETCH"),
            )
        )


class SpiDevice(Module):
    min_clk = 0
    max_clk = None
    require_continuous_clk = False
    cpol = 0
    cpha = 0


# # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # #
#                                        LEGACY INTERFACE                                         #
# # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # #
class SPISlaveInterface(Module, AutoCSR):
    spi_speed = 0
    def __init__(self):
        pass

    def get_statements(self, spi, cs):
        return []


class SPISuperviser(Module, AutoCSR):
    """A simple SPI Master-master to sequentially interact with multiple slaves"""
    def __init__(self, sys_clk_freq, pads, slaves=[]):
        from litex.soc.cores.spi import SPIMaster
        self.submodules.spi = spi = SPIMaster(pads, 32, sys_clk_freq, 1e6, with_csr=False, mode="aligned")
        self.slaves = slaves

        old_done = Signal()
        self.sync += [
            old_done.eq(spi.done),
        ]
        self.comb += [
            spi.start.eq(old_done),
            spi.cs.eq(0),
            spi.length.eq(1),
        ]
        self.submodules.fsm = FSM('IDLE')

    def add_slave(self, slave):
        self.slaves.append(slave)

    def finalize(self):
        print(f"SPI Supervisor slaves:{self.slaves}")
        for name, slave in self.slaves.items():
            setattr(self.submodules, name, slave)
        fsm = self.fsm
        spi = self.spi
        last_state = ('IDLE', [spi.start.eq(0)])
        for slave, cs_index in zip(self.slaves.values(), range(len(self.slaves))):
            i = 0
            for statement in slave.get_statements(spi = spi, cs = spi.cs[cs_index]):
                next_state = f"{cs_index}_{i}"
                last_state[1].append(If(spi.done, NextState(next_state)))
                fsm.act(*last_state)
                last_state = (next_state, statement)
                i += 1
        last_state[1].append(If(spi.done, NextState('IDLE')))
        fsm.act(*last_state)
        super().finalize()


class VNI8200XP(SPISlaveInterface):
    """SPI controlled octal OpenDrain industrial IO controller.
    See https://www.st.com/resource/en/datasheet/vni8200xp.pdf"""
    spi_speed = 5e6
    def __init__(self, SEL1=1):
        self.outputs = out = Signal(8)
        self.fault = fault = Signal()
        self.length = length = 16 if SEL1 else 8
        self.status = status = Signal(length)
        self.mosi = mosi = Signal(length)

        # send
        if SEL1:
            parity = Signal(3)
            self.comb += [
                parity[0].eq(sum(out[i] for i in range(8))),
                parity[1].eq(out[1] + out[3] + out[5] + out[7]),
                parity[2].eq(out[0] + out[2] + out[4] + out[6]),
                mosi.eq(Cat(~parity[0], parity, Signal(4), out)),
            ]
        else:
            self.comb += [
                mosi.eq(out),
            ]

        # receive
        if SEL1:
            parity = Signal(3)
            self.comb += [
                parity[0].eq(sum(status[i] for i in range(8, 16))),

            ]

    def get_statements(self, spi, cs):
        return [
            [
                cs.eq(1),
                spi.mosi.eq(self.mosi),
                spi.length.eq(self.length),
                If(spi.done,
                    NextValue(self.status, spi.miso),
                ),
            ],
        ]

    def add_csr(self):
        self._outputs = CSRStorage("outputs", fields=[
            CSRField(f"OUT{i}", 1, description=f"OUT{i} output value") for i in range(1, 9)])
        self._status = CSRStatus('status', fields=[
            CSRField("fault", 1, description="Checksum fault detected"),])
        self.comb += [
            self._status.fields.fault.eq(self.fault),
            (self.outputs[i].eq(getattr(self._outputs.fields, f"OUT{i+1}")) for i in range(8)),
        ]


class CLT01(SPISlaveInterface):
    """SPI controlled Indistrial IO controller.
    See https://www.st.com/resource/en/datasheet/clt01-38sq7.pdf"""
    spi_speed = 4e6
    def __init__(self, SPM=0):
        self.inputs = i = Signal(8)
        self.fault = fault = Signal()
        self.length = length = 8 if SPM else 16
        self.status = status = Signal(8)

        # We only receive data
        inv_check = Signal(8)
        self.comb += [
            inv_check[0].eq(~C(1)),  # Constant
            inv_check[1].eq(~C(0)),  # Constant
            inv_check[2].eq(i[2] ^ i[3] ^ i[4] ^ i[5]),  # ~PC4
            inv_check[3].eq(i[0] ^ i[1] ^ i[2] ^ i[3]),  # ~PC3
            inv_check[4].eq(i[4] ^ i[5] ^ i[6] ^ i[7]),  # ~PC2
            inv_check[5].eq(inv_check[4] ^ inv_check[3]),  # ~PC1 = ~PC2 ^ ~PC3
            inv_check[6].eq(~C(1)),  # /OTA should be 1
            inv_check[7].eq(~C(1)),  # /UVA should be 1
            fault.eq((status ^ inv_check) != 0xFF),
        ]

    def get_statements(self, spi, cs):
        return [
            [
                cs.eq(1),
                spi.length.eq(self.length),
                If(spi.done,
                    NextValue(self.status, spi.miso[0:8]),
                    NextValue(self.inputs, spi.miso[8:16]),
                ),
            ],
        ]

    def add_csr(self):
        self._inputs = CSRStatus("inputs", fields=[
            CSRField(f"IN{i}", 1, description=f"IN{i} input value") for i in range(1, 9)] + [
            CSRField("fault", 1, description="Checksum fault detected"),])
        self._status = CSRStatus("status", fields=[
            CSRField("RSV1", 1, description="Constant. Should be 1"),
            CSRField("RSV0", 1, description="Constant. Should be 0"),
            CSRField("PC4", 1, description="Parity Check 4"),
            CSRField("PC3", 1, description="Parity Check 3"),
            CSRField("PC2", 1, description="Parity Check 2"),
            CSRField("PC1", 1, description="Parity Check 1"),
            CSRField("OTA", 1, description="Over Temperature Alarm", values=[("0", "Alarm active"), ("1", "No alarm")]),
            CSRField("UVA", 1, description="Under Voltage Alarm", values=[("0", "Alarm active"), ("1", "No alarm")]),
            CSRField("fault", 1, description="Fault detected in the status register"),])
        self.comb += [
            (getattr(self._inputs.fields, f"IN{i+1}").eq(self.inputs[i]) for i in range(8)),
            self._status.fields.fault.eq(self.fault),
            self._status.fields.RSV1.eq(self.status[0]),
            self._status.fields.RSV0.eq(self.status[1]),
            self._status.fields.PC4.eq(self.status[2]),
            self._status.fields.PC3.eq(self.status[3]),
            self._status.fields.PC2.eq(self.status[4]),
            self._status.fields.PC1.eq(self.status[5]),
            self._status.fields.OTA.eq(self.status[6]),
            self._status.fields.UVA.eq(self.status[7]),
        ]


class MAX31855(SPISlaveInterface):
    """SPI thermocouple interface"""
    spi_speed = 5e6
    def __init__(self, SPM=0):
        self.temp = Signal((14, True))
        self.internal_temp = Signal((12, True))
        self.length = 32
        self.fault = Signal()
        self.svc = Signal()
        self.scg = Signal()
        self.oc = Signal()

    def get_statements(self, spi, cs):
        return [
            [
                cs.eq(1),
                spi.length.eq(self.length),
                If(spi.done,
                    NextValue(self.temp, spi.miso[18:]),
                    NextValue(self.internal_temp, spi.miso[4:16]),
                    NextValue(self.fault, spi.miso[16]),
                    NextValue(self.svc, spi.miso[2]),
                    NextValue(self.scg, spi.miso[1]),
                    NextValue(self.oc, spi.miso[0]),
                ),
            ],
        ]

    def add_csr(self):
        self._temp = CSRStatus(name="temp", fields=[
            CSRField('temp', 16, description="Temperature in 0.25°C"),])
        self._internal_temp = CSRStatus(name="internal_temp", fields=[
            CSRField('internal_temp', 16, description="Internal Temperature in 0.0625°C"),])
        self._status = CSRStatus(name="status", fields=[
            CSRField('oc', 1, description="Open Connection Fault"),
            CSRField('scg', 1, description="Short-Circuit to GND Fault"),
            CSRField('svc', 1, description="Short-Circuit to VCC Fault"),
            CSRField('fault', 1, description="At least one Fault active"),])
        self.comb += [
            self._temp.fields.temp.eq(self.temp),
        ] + [
            self._temp.fields.temp[i].eq(self.temp[-1]) for i in range(self.temp.nbits, 16)
        ] + [
            self._internal_temp.fields.internal_temp.eq(self.internal_temp),
        ] + [
            self._internal_temp.fields.internal_temp[i].eq(self.internal_temp[-1]) for i in range(
                self.internal_temp.nbits, 16)
        ] + [
            self._status.fields.oc.eq(self.oc),
            self._status.fields.scg.eq(self.scg),
            self._status.fields.svc.eq(self.svc),
            self._status.fields.fault.eq(self.fault),
        ]
