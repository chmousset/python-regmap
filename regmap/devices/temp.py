from regmap.core.models import I2cBus, I2cDevice, I2cReg


class LM75(I2cDevice):
    def __init__(self, bus, address_pins=0):
        super().__init__(bus, address=0b1001000 + address_pins)

        self.temperature = I2cReg(self, 0x00, 2, ro=True)
        self.configuration = I2cReg(self, 0x01, 1)
        self.thyst = I2cReg(self, 0x02, 2)
        self.tos = I2cReg(self, 0x03, 2)
