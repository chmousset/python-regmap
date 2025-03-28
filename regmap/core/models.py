class I2cBus:
    def __init__(self):
        pass


class I2cTransaction:
    def __init__(self, device, write, read, read_stream=None):
        self.write = write
        self.read = read
        self.device = device
        self.read_stream = read_stream

    def __repr__(self):
        write = f" W{self.write}" if len(self.write) else ""
        if self.read:
            return f"0x{self.device.address:02X} S{write} S R{self.read} P"
        else:
            return f"0x{self.device.address:02X} S{write} P"


class I2cDevice:
    def __init__(self, bus: I2cBus, address, address_10b=False, be_data=True, be_reg_address=True, reg_address_bytes=1):
        self.address = address
        self.address_10b = address_10b
        self.v = be_data
        self.be_reg_address = be_reg_address
        self.reg_address_bytes = reg_address_bytes

    def raw_address_bytes(self, read=False):
        address = 1 if read else 0
        if self.address_10b:
            address |= ((self.address >> 8) | 0b1111000) << 1
            return [address, self.address & 0xFF]
        return [address | (self.address << 1)]

    def raw_reg_address(self, address):
        return list(address.to_bytes(self.reg_address_bytes,
            byteorder='big' if self.be_reg_address else 'small'))


class I2cReg:
    def __init__(self, device, reg_address, reg_size, big_endian=True, wo=False, ro=False):
        self.device = device
        self.reg_address = reg_address
        self.transaction_start = [reg_address] if reg_address is not None else []
        self.reg_size = reg_size
        self.big_endian = big_endian
        self.wo = wo
        self.ro = ro
        self.source = None

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
        return I2cTransaction(self.device, self.transaction_start, self.reg_size, self.source)

    def add_read_stream(self):
        from litex.soc.interconnect import stream
        from regmap.core.i2c import i2c_byte_layout
        self.source = stream.Endpoint(i2c_byte_layout)

    def raw_reg_address(self):
        return self.device.raw_reg_address(self.reg_address)
