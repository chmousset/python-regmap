from migen import Module, C, Signal, If, FSM, Record, Case, NextValue, NextState, Cat
from regmap.core.spi import SpiDevice, spi_layout, spi_ctrl_layout
from litex.soc.interconnect import stream


class ADS131Mxx(SpiDevice):
    max_clk = 8192*1024
    require_continuous_clk = True
    dw = 24
    data_layout = [
        ("channel", 3),
        ("data", 24),
    ]
    channel_count = None

    """
    parameters:
    - channel_count: 2, 4, 6 or 8 depending on the ADC model
    - config: list of tuple (reg_address, reg_value)
    """
    def __init__(self, config=None):
        # inputs
        self.spi_sink = spi_sink = stream.Endpoint(spi_layout(self.dw))
        self.launch_config = launch_config = Signal()

        # outputs
        self.spi_source = spi_source = stream.Endpoint(spi_ctrl_layout(self.dw) + spi_layout(self.dw))
        self.source = source = stream.Endpoint(self.data_layout)
        self.channels = [Signal(24) for _ in range(self.channel_count)]

        # # #
        chan_valid = Signal(8)
        current_chan = Signal(3)
        self.comb += [
            spi_source.valid.eq(1),
            spi_source.data.eq(0),  # NOP
            spi_source.width.eq(23),
            spi_source.cpha.eq(1),
            source.data.eq(spi_sink.data),
        ]
        self.submodules.fsm = fsm = FSM("IDLE")
        fsm.act("IDLE",
            If(spi_source.ready,
                NextState("READ_STATUS"),
            ),
        )
        fsm.act("READ_STATUS",
            If(spi_sink.valid,
                # NextValue(chan_valid, 0xF),
                If(spi_sink.data & (0xFF << 8),
                # If(1,
                    NextState("READ_DATA"),
                    NextValue(chan_valid, (spi_sink.data >> 8) & 0xFF),
                    NextValue(current_chan, 0),
                ).Else(
                    NextState("FINISH"),
                ),
            ),
        )
        fsm.act("READ_DATA",
            spi_sink.ready.eq(1),
            If(spi_sink.valid,
                If(chan_valid & 0b1,
                    source.valid.eq(1),
                    Case(current_chan, {
                        i: [
                            self.channels[i].eq(spi_sink.data),
                            source.channel.eq(i),
                        ] for i in range(self.channel_count)}),
                ),
                If(chan_valid,
                    NextValue(chan_valid, chan_valid >> 1),
                    NextValue(current_chan, current_chan + 1),
                ).Else(
                    NextState("FINISH"),
                ),
            ),
        )
        fsm.act("FINISH",
            spi_source.valid.eq(0),  # nothing to transfer anymore
            If(spi_source.ready,
                NextState("IDLE"),
            ),
        )

    def add_self_config(self, config):
        pass


class ADS131M04(ADS131Mxx):
    channel_count = 4


class ADS131M06(ADS131Mxx):
    channel_count = 6


class ADS131M08(ADS131Mxx):
    channel_count = 8
