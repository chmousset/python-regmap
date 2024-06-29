class I2cBus:
    def __init__(self):
        pass


class I2cTransaction:
    def __init__(self, device, write, read):
        self.write = write
        self.read = read
        self.device = device

    def __repr__(self):
        write = f" W{self.write}" if len(self.write) else ""
        if self.read:
            return f"0x{self.device.address:02X} S{write} S R{self.read} P"
        else:
            return f"0x{self.device.address:02X} S{write} P"


class I2cDevice:
    def __init__(self, bus: I2cBus, address):
        self.address = address


class I2cReg:
    def __init__(self, device, reg_address, reg_size, big_endian=True, wo=False, ro=False):
        self.device = device
        self.reg_address = reg_address
        self.transaction_start = [reg_address] if reg_address is not None else []
        self.reg_size = reg_size
        self.big_endian = big_endian
        self.wo = wo
        self.ro = ro

    def write(self, value):
        assert not self.ro
        write = self.transaction_start.copy()
        if self.big_endian:
            for shift in range((self.reg_size - 1) * 8, -8, -8):
                write += [0xff & (value >> shift)]
        else:
            for shift in range(0, self.reg_size * 8, 8):
                write += [0xff & (value >> shift)]
        return I2cTransaction(self.device, write, None)

    def read(self):
        assert not self.wo
        return I2cTransaction(self.device, self.transaction_start, self.reg_size)
