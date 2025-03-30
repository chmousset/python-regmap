from migen import *

from litex.gen import *
from litex.build.generic_platform import *
from litex.soc.integration.soc_core import *
from litex.soc.integration.soc import SoCRegion
from litex.soc.integration.builder import *
from litex.soc.cores.led import LedChaser
from litex.soc.interconnect.csr import AutoCSR, CSRStorage, CSRStatus, CSRField
from litex.gen.fhdl.module import LiteXModule 

from litescope import LiteScopeAnalyzer

from litex_boards.platforms import icebreaker
from litex_boards.targets.icebreaker import _CRG, flash

from regmap.core.models import I2cReg, I2cDevice, I2cBus
from regmap.core.i2c import I2cPads, I2cBitOperationRTx, I2cByteOperationRTx
from regmap.core.i2c_sequencer import I2cRegmap
from regmap.devices.temp import LM75
from regmap.devices.eeprom import EEPROM_MAC


_ios = [
    ("i2c", 0,
        Subsignal("sda", Pins("PMOD1B:3")),
        Subsignal("scl", Pins("PMOD1B:2")),
        Subsignal("sda_o", Pins("PMOD1B:1")),
        Subsignal("scl_o", Pins("PMOD1B:0")),
    ),
    ("spi", 0,
        Subsignal("cs", Pins("PMOD1A:0")),
        Subsignal("sclk", Pins("PMOD1A:1")),
        Subsignal("mosi", Pins("PMOD1A:2")),
        Subsignal("miso", Pins("PMOD1A:3")),
        Subsignal("miso_cpy", Pins("PMOD1A:7")),
    ),
    ("onewire", 0,
        Subsignal("io", Pins("GPIO1:0")),
    ),
]


class I2cDemo(LiteXModule, AutoCSR):
    def __init__(self, platform, sys_clk_freq):
        from regmap.core.i2c import I2cPads, I2cOperation, I2cByteOperation
        bus = I2cBus()
        self.submodules.eui = eui = EEPROM_MAC(bus, address=0b1010000)
        self.submodules.pads = pads = I2cPads(platform.request("i2c"))
        self.submodules.op_b = op_b = I2cByteOperation(pads, 100)
        self.submodules.op = op = I2cOperation()
        enable = Signal()

        self.cr = CSRStorage("cr", fields = [
            CSRField("en", 1, offset=0),
            CSRField("speed", 1, offset=4),
            CSRField("sda_o", 1, offset=8),
            CSRField("scl_o", 1, offset=9),
            ])
        self.st = CSRStatus("st", fields = [
            CSRField("mac_valid", 1, offset=0),
            ])
        self.mac_msb = CSRStatus(name="mac_msb", size=48-32)
        self.mac_lsb = CSRStatus(name="mac_lsb", size=32)

        self.comb += [
            op.source.connect(op_b.sink),
            op_b.source.connect(eui.i2c_sink),
            If(self.cr.fields.en,
                eui.i2c_source.connect(op.sink),
            ),
            self.mac_lsb.status.eq(eui.eui.eui[0:32]),
            self.mac_msb.status.eq(eui.eui.eui[32:]),
            self.st.fields.mac_valid.eq(eui.eui.valid),
        ]

        self.analyzer_signals = [
            self.cr.fields.en,
            self.op.sink.first,
            self.op.sink.last,
            self.op.sink.read,
            self.op.sink.write,
            self.op_b.source.valid,
            self.op_b.source.ready,
            self.op_b.source.ack,
        ]


class I2cRegmapDemo(LiteXModule, AutoCSR):
    def __init__(self, platform, sys_clk_freq):
        pads = platform.request("i2c")
        self.submodules.i2c_pads = i2c_pads = I2cPads(pads)
        self.submodules.bit_rtx = bit_rtx = I2cBitOperationRTx(i2c_pads, sys_clk_freq)
        self.submodules.byte_rtx = byte_rtx = I2cByteOperationRTx()
        self.submodules.regmap = regmap = I2cRegmap()
        self.comb += [
            byte_rtx.connect_bit(bit_rtx),
            regmap.source.connect(byte_rtx.sink_master),
            byte_rtx.source_master.connect(regmap.sink),
            pads.sda_o.eq(i2c_pads.sda_o),
            pads.scl_o.eq(i2c_pads.scl_o),
            regmap.reset.eq(bit_rtx.timeout),
        ]

        bus = I2cBus()
        temp = LM75(bus=None, address_pins=7)
        mac48 = EEPROM_MAC(bus, address=0b1010000)
        temp.temperature.add_read_stream()
        mac48.reg_eui.add_read_stream()
        self.comb += [
            temp.temperature.source.ready.eq(1),
            mac48.reg_eui.source.ready.eq(1),
        ]
        mac = Signal(48)
        mac_byte = Signal(max=8)
        self.sync += [
            If(mac48.reg_eui.source.valid & (mac_byte != 6),
                Case(mac_byte, {i: mac[i*8:i*8+8].eq(mac48.reg_eui.source.data) for i in range(6) }),
                mac_byte.eq(mac_byte + 1),
            ),
        ]
        for _ in range(1):
            regmap.add_transaction(mac48.reg_eui.read())
            regmap.add_transaction(temp.temperature.read())
            # regmap.add_transaction(temp.tos.write(0xdead))

        self.analyzer_signals = [
        ]

        self.cr = CSRStorage("cr", fields = [
            CSRField("en", 1, offset=0),
            CSRField("speed", 1, offset=4),
            CSRField("sda_o", 1, offset=8),
            CSRField("scl_o", 1, offset=9),
            ])

        self.sr = CSRStatus("sr", fields = [
            CSRField("done", 1, offset=0),
            CSRField("sda_i", 1, offset=1),
            CSRField("scl_i", 1, offset=2),
            ])
        self.mac = CSRStatus("mac", fields = [
            CSRField("mac", 48, offset=0),
            CSRField("valid", 1, offset=48),
            ])
        self.mac2 = CSRStatus("mac2", fields = [
            CSRField("mac", 48, offset=0),
            CSRField("valid", 1, offset=48),
            ])

        self.comb += [
            self.sr.fields.done.eq(regmap.done),
            self.sr.fields.sda_i.eq(i2c_pads.sda_i),
            self.sr.fields.scl_i.eq(i2c_pads.scl_i),
            regmap.enable.eq(self.cr.fields.en & self.cr.re),
            bit_rtx.speed.eq(self.cr.fields.speed),
            # i2c_pads.sda_o.eq(self.cr.fields.sda_o),
            # i2c_pads.scl_o.eq(self.cr.fields.scl_o),
            self.mac.fields.mac.eq(mac48.eui.eui),
            self.mac.fields.valid.eq(mac48.eui.valid),
            self.mac2.fields.mac.eq(mac),
        ]
        self.regmap.finalize()
        self.regmap.print()


# BaseSoC ------------------------------------------------------------------------------------------

class BaseSoC(SoCCore):
    def __init__(self, sys_clk_freq=24e6,
        with_led_chaser     = True,
        with_i2c            = False,
        with_i2c_regmap     = False,
        **kwargs):
        platform = icebreaker.Platform()
        platform.add_extension(icebreaker.break_off_pmod)
        platform.add_extension(_ios)

        # CRG --------------------------------------------------------------------------------------
        self.crg = _CRG(platform, sys_clk_freq)

        # SoCCore ----------------------------------------------------------------------------------
        kwargs["integrated_sram_size"] = 0
        kwargs["integrated_rom_size"]  = 0
        kwargs["cpu_type"] = None
        kwargs["with_uart"] = False
        SoCCore.__init__(self, platform, sys_clk_freq, **kwargs)

        self.add_uartbone()

        # I2C
        if with_i2c:
            self.submodules.i2c = I2cDemo(platform, sys_clk_freq)
        if with_i2c_regmap:
            self.submodules.i2c = I2cRegmapDemo(platform, sys_clk_freq)

        analyzer_signals = [ ] + self.i2c.analyzer_signals

        if len(analyzer_signals):
            self.submodules.analyzer = LiteScopeAnalyzer(analyzer_signals,
                depth        = 4096,
                clock_domain = "sys",
                samplerate   = sys_clk_freq,
                csr_csv      = "analyzer.csv")


        # Leds -------------------------------------------------------------------------------------
        if with_led_chaser:
            self.leds = LedChaser(
                pads         = platform.request_all("user_led"),
                sys_clk_freq = sys_clk_freq)


def flash(build_dir, build_name):
    from litex.build.lattice.programmer import IceStormProgrammer
    prog = IceStormProgrammer()
    prog.flash(0x00000000,        f"{build_dir}/gateware/{build_name}.bin")


def main():
    from litex.build.parser import LiteXArgumentParser
    parser = LiteXArgumentParser(platform=icebreaker.Platform, description="regmap demo on iCEBreaker.")
    parser.add_target_argument("--flash",               action="store_true",      help="Flash Bitstream")
    parser.add_target_argument("--sys-clk-freq",        default=24e6, type=float, help="System clock frequency.")
    parser.add_target_argument("--i2c",                 action="store_true",      help="Enable the I2C demo")
    parser.add_target_argument("--i2c-regmap",          action="store_true",      help="Enable the I2C Regmap demo")
    args = parser.parse_args()

    if args.load:
        raise Exception("SRAM loading on icebreaker doesn't work")

    soc = BaseSoC(
        sys_clk_freq        = args.sys_clk_freq,
        with_i2c            = args.i2c,
        with_i2c_regmap     = args.i2c_regmap,
        **parser.soc_argdict
    )
    builder = Builder(soc, **parser.builder_argdict)
    if args.build:
        builder.build(**parser.toolchain_argdict)

    if args.flash:
        flash(builder.output_dir, soc.build_name)

if __name__ == "__main__":
    main()
