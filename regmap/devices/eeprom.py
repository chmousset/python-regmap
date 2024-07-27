from regmap.core.models import I2cBus, I2cDevice, I2cReg
from regmap.core.i2c import i2c_byte_layout, i2c_operation_layout
from litex.soc.interconnect import stream
from migen import Signal, Module, FSM, If, NextValue, NextState, Case

eeprom_mac_layout = [ ("eui", 48) ]


class EEPROM_24(I2cDevice, Module):
    def __init__(self, bus, address=0b1010000, bytes_address=1):
        # inputs
        self.i2c_sink = i2c_sink = stream.Endpoint(i2c_byte_layout)

        # outputs
        self.i2c_source = i2c_source = stream.Endpoint(i2c_operation_layout)

        # # #
        super().__init__(bus, address=address)

        self.comb += [
            self.i2c_source.address.eq(self.address),
        ]


class EEPROM_MAC(EEPROM_24):
    """A specialized EEPROM interface for the 24AA02E48/24AA025E48 that hold a 48b EUI for use as
       Ethernet MAC addess.
       The stream interface `eui` will always be valid once the memory has been read.
       Re-reading the EUI is possible by setting read_eui to 1. During reading, `eui.valid` == 0
    """
    def __init__(self, bus, address=0b1010000):
        super().__init__(bus, address, bytes_address=1)
        i2c_source = self.i2c_source
        i2c_sink = self.i2c_sink

        # inputs
        self.read_eui = read_eui = Signal()

        # outputs
        self.eui = eui = stream.Endpoint(eeprom_mac_layout)

        # # #
        self.reg_eui = I2cReg(self, 0xFA, 6, ro=True)
        bytes_read = Signal(max=8)

        self.submodules.fsm = fsm = FSM("IDLE")
        fsm.act("IDLE",
            # setup: write I2C dev address, then memory address
            i2c_sink.ready.eq(1),
            i2c_source.valid.eq(read_eui),  # | ~eui.valid),
            i2c_source.first.eq(1),
            i2c_source.data.eq(self.reg_eui.reg_address),
            i2c_source.write.eq(1),
            If(i2c_source.ready & read_eui,
                NextState("CHECK_ACK"),
                NextValue(eui.valid, 0),
            ),
        )
        fsm.act("CHECK_ACK",
            # the EEPROM should ack its I2C address
            i2c_sink.ready.eq(1),
            If(i2c_sink.valid & ~i2c_sink.first,
                If(~i2c_sink.ack,
                    NextState("READ_SETUP")
                ).Else(
                    NextState("STOP"),
                ),
            ),
        )
        fsm.act("READ_SETUP",
            # generate a restart, write I2C dev address, then read 6 bytes.
            # on the last byte, generate a stop 
            i2c_sink.ready.eq(1),
            i2c_source.valid.eq(1),
            i2c_source.first.eq(bytes_read == 0),
            i2c_source.last.eq(bytes_read == 5),
            i2c_source.read.eq(1),
            If(i2c_source.ready,
                NextState("READ_BYTE"),
            ),
        )
        fsm.act("READ_BYTE",
            i2c_sink.ready.eq(1),
            If(i2c_sink.valid & ~i2c_sink.first,
                NextValue(bytes_read, bytes_read + 1),
                Case(bytes_read, {
                    i: [NextValue(eui.eui[i * 8 : i * 8 + 8], i2c_sink.data)]
                    for i in range(6)}),
                If(bytes_read == 5,
                    NextState("IDLE"),
                    NextValue(eui.valid, 1),
                ).Else(
                    NextState("READ_BYTE"),
                ),
            ),
        )
        fsm.act("STOP",
            # generate only the stop transition
            i2c_sink.ready.eq(1),
            i2c_source.valid.eq(1),
            i2c_source.last.eq(1),
            If(i2c_source.ready,
                NextState("IDLE"),
            ),
        )
