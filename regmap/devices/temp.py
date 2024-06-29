from regmap.core.models import I2cBus, I2cDevice, I2cReg
from regmap.core.i2c import i2c_byte_layout, i2c_operation_layout
from litex.soc.interconnect import stream
from migen import Module, FSM, If, NextValue, NextState

lm75_temp_layout = [("temp", 1)]


class LM75(I2cDevice, Module):
    def __init__(self, bus, address_pins=0, config_reg=None):
        # inputs
        self.i2c_sink = i2c_sink = stream.Endpoint(i2c_byte_layout)

        # outputs
        self.i2c_source = i2c_source = stream.Endpoint(i2c_operation_layout)
        self.temp = temp = stream.Endpoint(lm75_temp_layout)

        # # #
        super().__init__(bus, address=0b1001000 + address_pins)

        self.temperature = I2cReg(self, 0x00, 2, ro=True)
        self.configuration = I2cReg(self, 0x01, 1)
        self.thyst = I2cReg(self, 0x02, 2)
        self.tos = I2cReg(self, 0x03, 2)

        self.comb += [
            self.i2c_source.address.eq(self.address),
        ]

        self.submodules.fsm = fsm = FSM("CONFIG" if config_reg else "IDLE")
        if config_reg:
            fsm.act("CONFIG",
                i2c_source.first.eq(1),
                i2c_source.data.eq(self.configuration.reg_address),
                If(i2c_source.ready,
                    NextState("CONFIG_CHECK_ACK"),
                ),
            )
            fsm.act("CONFIG_CHECK_ACK",
                i2c_sink.ready.eq(1),
                If(i2c_sink.valid,
                    If(i2c_sink.ack,
                        NextState(""),
                    ).Else(
                        NextState("CONFIG_DATA"),
                    ),
                )
            )
            fsm.act("CONFIG_DATA",
                i2c_source.valid.eq(1),
                i2c_source.last.eq(1),
                i2c_source.data.eq(config_reg),
                If(i2c_source.ready,
                    NextState("IDLE"),
                ),
            )
        fsm.act("IDLE",
            i2c_source.valid.eq(1),
            i2c_source.first.eq(1),
            i2c_source.data.eq(self.temperature.reg_address),
            If(i2c_source.ready,
                NextState("READ_TEMP"),
            ),
        )
        fsm.act("READ_TEMP",
            i2c_source.valid.eq(1),
            i2c_source.first.eq(1),
            i2c_source.last.eq(1),
            i2c_source.read.eq(1),
            If(i2c_source.ready,
                NextState("READBACK_TEMP"),
            ),
        )
        fsm.act("READBACK_TEMP_ACK",
            i2c_sink.ready.eq(1),
            If(i2c_sink.valid,
                If(~i2c_sink.ack,
                    NextState("READBACK_TEMP")
                ).Else(
                    NextState("IDLE"),
                ),
            ),
        )
        fsm.act("READBACK_TEMP",
            i2c_sink.ready.eq(1),
            If(i2c_sink.valid,
                NextValue(self.temp.temp, i2c_sink.data),
                NextValue(self.temp.valid, 1),
                NextState("IDLE"),
            ),
        )
