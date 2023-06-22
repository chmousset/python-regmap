from regmap.core.models import I2cBus, I2cDevice, I2cReg


class PCF8574(I2cDevice):
    def __init__(self, bus, address_pins=0):
        assert address_pins >= 0
        assert address_pins < 8
        super().__init__(bus, address=0b0100000 + address_pins)

        self.io = I2cReg(self, None, 1)
