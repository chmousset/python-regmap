from migen import C, Signal, If, FSM, Case, NextState, Cat, NextValue
from migen.genlib.cdc import MultiReg
from regmap.core.spi import SpiDevice, spi_layout, spi_ctrl_layout
from litex.soc.interconnect import stream


class ADS1120Config:
    # CR 1
    channels = { chan: i for (i, chan) in enumerate([
        "AIN0-AIN1",
        "AIN0-AIN2",
        "AIN0-AIN3",
        "AIN1-AIN2",
        "AIN1-AIN3",
        "AIN2-AIN3",
        "AIN1-AIN0",
        "AIN3-AIN2",
        "AIN0-AVSS",
        "AIN1-AVSS",
        "AIN2-AVSS",
        "AIN3-AVSS",
        "VREFP-VREFN",
        "AVDD-AVSS",
        "AVDD+AVSS",
    ])}
    gains = {gain: i for (i, gain) in enumerate([1, 2, 4, 8, 16, 32, 64, 128])}
    #  CR 1
    dr_mode = {
        # Normal mode
        20:    (0, 0),
        45:    (1, 0),
        90:    (2, 0),
        175:   (3, 0),
        330:   (4, 0),
        600:   (5, 0),
        1000:  (6, 0),
        # Duty Cycle mode
        5:     (0, 1),
        11.25: (1, 1),
        22.5:  (2, 1),
        44:    (3, 1),
        82.5:  (4, 1),
        150:   (5, 1),
        250:   (6, 1),
        # Turbo Mode
        40:    (0, 2),
        # 90:    (1, 2),  # 90 SPS available in normal mode
        180:   (2, 2),
        350:   (3, 2),
        660:   (4, 2),
        1200:  (5, 2),
        2000:  (6, 2),
    }
    # CR 2
    vrefs = {
        "internal": 0,
        "REF0": 1,
        "REF1": 2,
        "AVDD": 3,
    }
    filters = {"none": 0, "50Hz60Hz": 1, "50Hz": 2, "60Hz": 3}
    idac = {
        0:    0b000,
        50:   0b010,
        100:  0b011,
        250:  0b100,
        500:  0b101,
        1000: 0b110,
        1500: 0b111,
    }
    # CR 3
    iadc_mux = {
        "disabled":   0b000,
        "AIN0/REFP1": 0b001,
        "AIN1":       0b010,
        "AIN2":       0b011,
        "AIN3/REFN1": 0b100,
        "REFP0":      0b101,
        "REFN0":      0b110,
    }

    def __init__(self, channel="AIN0-AIN1", gain=1, pga_bypass=False, data_rate=20, single_shot=True,
        temp=False, bcs=False, vref="internal", fir="none", auto_power_switch=False, idac_ua=0,
        idac1_chan="disabled", idac2_chan="disabled", mux_drdy=False):
        self.channel = channel
        self.gain = gain
        self.pga_bypass = pga_bypass
        self.data_rate = data_rate
        self.single_shot = single_shot
        self.temp = temp
        self.bcs = bcs
        self.vref = vref
        self.fir = fir
        self.auto_power_switch = auto_power_switch
        self.idac_ua = idac_ua
        self.idac1_chan = idac1_chan
        self.idac2_chan = idac2_chan
        self.mux_drdy = mux_drdy

        # Config Register 0
        assert(channel in self.channels.keys())
        assert(gain in self.gains.keys())

        # Config Register 1
        assert(data_rate in self.dr_mode.keys())

        # Config Register 2
        assert(vref in self.vrefs.keys())
        assert(fir in self.filters.keys())
        assert(idac_ua in self.idac.keys())

        #  Config Register 3
        assert(idac1_chan in self.iadc_mux.keys())
        assert(idac2_chan in self.iadc_mux.keys())

    def to_bytes(self):
        cfg0 = (self.channels[self.channel] << 4) | (self.gains[self.gain] << 1) | \
            (1 if self.pga_bypass else 0)
        dr, mode = self.dr_mode[self.data_rate]
        cfg1 = (dr << 5) | (mode << 3) | (0 if self.single_shot else 1 << 2) | \
            (1 << 1 if self.temp else 0) | (1 if self.bcs else 0)
        cfg2 = (self.vrefs[self.vref] << 6)| (self.filters[self.fir] << 4) | \
            (1 << 3 if self.auto_power_switch else 0) | (self.idac[self.idac_ua])
        cfg3 = (self.iadc_mux[self.idac1_chan] << 5) | (self.iadc_mux[self.idac2_chan] << 2) | \
            (1 << 1 if self.mux_drdy else 0) | (0)
        return cfg0, cfg1, cfg2, cfg3

    def to_w32(self):
        cfg0, cfg1, cfg2, cfg3 = self.to_bytes()
        print(self.to_bytes())
        return (cfg0 << 24) | (cfg1 << 16) | (cfg2 << 8) | cfg3


def regs_to_mem_builder(regs):
    mem_init = []
    valid = 1 << 16
    address, values = regs
    cmd = (0b011 << 13) | (address << 7) | (len(values) - 1)
    mem_init += [valid | cmd]
    for value in values:
        mem_init += [valid | value]
        pass
    mem_init += [0]
    return mem_init


class ADS1120(SpiDevice):
    max_clk = int(1/150e-9)
    require_continuous_clk = False
    dw = 16
    data_layout = [
        ("channel", 4),
        ("data", dw),
    ]
    channel_count = 4

    """
    ADS1120 driver.

    The ADS1120 is a 16bits, 4 channels, 2kSPS ADC with SPI interface.
    [datasheet](https://www.ti.com/lit/ds/symlink/ads1120.pdf)


    parameters:

    inputs:
    - config: `Signal(32)` - the channel config is taken from these signals:
        - config[0:8] -> config register 3
        - config[8:16] -> config register 2
        - config[16:24] -> config register 1
        - config[24:32] -> config register 0
    - drdy_n: when cleared, a sample is ready to be read. This can either be controlled by a timer
      or the drdy pin

    outputs:
    - chip_select: when DRDY is muxed, the CS pin must be driven low before data is exchanged.
      This signal can be used to arbitrate between devices in the case of a shared bus.
      Set when the CS pin must be enabled (pulled to ground)
    - source: Stream that holds the channel index and data conversion result
    - spi_source: Stream that connects to a SPIMaster().sink
    - channels: `[Signal(16); 4]` - holds the most recent value converted.
      **NOTE**: the two least-significant bits of the MUX configuration correspond to the index of
      the channel. This limits logic usage, and in most cases is sufficient but doesn't support all
      mux configurations.
    - wait_conversion: set when waiting for DRDY

    ## SPI protocol
    When a conversion result is available, it is transmitted in the first two bytes the ADC sends.
    Availability of ADC sample is signaled through pin DRDY, which falls low to signal availability.
    This pin should be connected to drdy_n.

    ## Muxing MISO and DRDY
    It is possible to mux DRDY on MISO pin. In that case, connect drdy_n to miso input.
    This requires however that the ADC configuration sets bit DRDYM
    """

    def __init__(self):
        # inputs
        self.spi_sink = spi_sink = stream.Endpoint(spi_layout(self.dw))
        self.drdy_n = Signal()
        self.config = config = Signal(32)

        # outputs
        self.spi_source = spi_source = stream.Endpoint(spi_ctrl_layout(self.dw) + spi_layout(self.dw))
        self.source = source = stream.Endpoint(self.data_layout)
        self.channels = channels = [Signal(self.dw) for _ in range(self.channel_count)]
        self.chip_select = chip_select = Signal(reset=1)
        self.wait_conversion = wait_conversion = Signal()

        # # #
        drdy_n = Signal()
        self.specials += MultiReg(self.drdy_n, drdy_n)
        current_chan = Signal(max=self.channel_count)
        self.comb += [
            spi_source.cpha.eq(1),
            source.data.eq(spi_sink.data),
            source.channel.eq(current_chan),
        ]
        self.submodules.fsm = fsm = FSM("CMD+CFG0")
        fsm.act("WAIT_CONVERSION",
            wait_conversion.eq(1),
            chip_select.eq(0),
            spi_sink.ready.eq(1),
            If(~drdy_n,
                NextState("CMD+CFG0"),
            ),
        )

        fsm.act("CMD+CFG0",
            spi_source.valid.eq(1),
            spi_source.width.eq(15),
            spi_source.data.eq(Cat(config[24:32], C(0b01000011))), # write 3+1 reg from reg 0
            If(spi_source.ready,
                NextState("READ_DATA"),
            ),
        )
        fsm.act("READ_DATA",
            spi_sink.ready.eq(1),
            source.valid.eq(spi_sink.valid),
            If(spi_sink.valid,
                NextValue(current_chan, config[24+4:24+8]),
                Case(current_chan, {
                    i: NextValue(channels[i], spi_sink.data) for i in range(self.channel_count)
                }),
                NextState("CFG1-2"),
            ),
        )
        fsm.act("CFG1-2",
            spi_source.valid.eq(1),
            spi_source.width.eq(15),
            spi_source.data.eq(config[8:24]),
            If(spi_source.ready,
                NextState("CFG3-START"),
            ),
        )
        fsm.act("CFG3-START",
            spi_source.valid.eq(1),
            spi_source.width.eq(15),
            spi_source.last.eq(1),
            spi_source.data.eq(Cat(C(0b000_1001, 8), config[0:8])),
            If(spi_source.ready,
                NextState("FINISH"),
            ),
        )
        fsm.act("FINISH",
            chip_select.eq(0),
            If(spi_source.ready,
                NextState("WAIT_CONVERSION"),
            ),
        )
