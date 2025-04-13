#!/usr/bin/env python3

#
# This file is part of LiteX-Boards.
#
# Copyright (c) 2015-2019 Florent Kermarrec <florent@enjoy-digital.fr>
# Copyright (c) 2020 Antmicro <www.antmicro.com>
# Copyright (c) 2022 Victor Suarez Rovere <suarezvictor@gmail.com>
# SPDX-License-Identifier: BSD-2-Clause

# Note: For now with --toolchain=yosys+nextpnr:
# - DDR3 should be disabled: ex --integrated-main-ram-size=8192
# - Clk Freq should be lowered: ex --sys-clk-freq=50e6

from types import FrameType
from migen import *

from litex.gen import *

from litex_boards.platforms import digilent_arty

from litex.soc.cores.clock import *
from litex.soc.integration.soc import SoCRegion
from litex.soc.integration.soc_core import *
from litex.soc.integration.builder import *
from litex.soc.cores.led import LedChaser
from litex.soc.cores.gpio import GPIOIn, GPIOTristate
from litex.soc.cores.xadc import XADC
from litex.soc.cores.dna  import DNA

from litedram.modules import MT41K128M16
from litedram.phy import s7ddrphy

from liteeth.phy.mii import LiteEthPHYMII

from litex.soc.interconnect import stream
from litex.soc.interconnect.csr import CSRStorage, CSRStatus, CSRField

# # #
from litex.build.generic_platform import *
from litescope import LiteScopeAnalyzer

from regmap.core.spi import SpiMaster
from regmap.devices.ads1120 import ADS1120, ADS1120Config
from regmap.core.onewire import WaitTimer
from liteeth.frontend.stream import LiteEthUDPStreamer
# from regmap.devices.eeprom import EEPROM_MAC
# from regmap.core.i2c import I2cPads, I2cBitOperationRTx, I2cByteOperationRTx

# CRG ----------------------------------------------------------------------------------------------

class _CRG(LiteXModule):
    def __init__(self, platform, sys_clk_freq, with_dram=True, with_rst=True):
        self.rst    = Signal()
        self.cd_sys = ClockDomain()
        self.cd_eth = ClockDomain()
        if with_dram:
            self.cd_sys4x     = ClockDomain()
            self.cd_sys4x_dqs = ClockDomain()
            self.cd_idelay    = ClockDomain()

        # # #

        # Clk/Rst.
        clk100 = platform.request("clk100")
        rst    = ~platform.request("cpu_reset") if with_rst else 0

        # PLL.
        self.pll = pll = S7PLL(speedgrade=-1)
        self.comb += pll.reset.eq(rst | self.rst)
        pll.register_clkin(clk100, 100e6)
        pll.create_clkout(self.cd_sys, sys_clk_freq)
        pll.create_clkout(self.cd_eth, 25e6)
        self.comb += platform.request("eth_ref_clk").eq(self.cd_eth.clk)
        platform.add_false_path_constraints(self.cd_sys.clk, pll.clkin) # Ignore sys_clk to pll.clkin path created by SoC's rst.
        if with_dram:
            pll.create_clkout(self.cd_sys4x,     4*sys_clk_freq)
            pll.create_clkout(self.cd_sys4x_dqs, 4*sys_clk_freq, phase=90)
            pll.create_clkout(self.cd_idelay,    200e6)

        # IdelayCtrl.
        if with_dram:
            self.idelayctrl = S7IDELAYCTRL(self.cd_idelay)


# IOs ----------------------------------------------------------------------------------------------

_ios = [
    ("spi_a", 0,
        Subsignal("cs", Pins("pmoda:0")),
        Subsignal("sclk", Pins("pmoda:1")),
        Subsignal("mosi", Pins("pmoda:2")),
        Subsignal("miso", Pins("pmoda:3")),
        IOStandard("LVCMOS33"),
    ),
]

# ADC Sequencer ------------------------------------------------------------------------------------

class Ads1120Sequencer(LiteXModule):
    layout = [
        ("data", 32),
    ]

    def __init__(self, sys_clk_freq, pads):
        """A simple 2-sequence ADS1120 driver.
        This module drives an ADS1120, performing conversions with alternating configuration, and
        streaming both values in a single stream. It assumes the configuration uses 2000SPS

        parameters:

        inputs:

        outputs:
        - source: stream.Endpoint(Ads1120Sequencer.layout)
        """

        # Inputs

        # Outputs
        self.source = source = stream.Endpoint(self.layout)

        # # #
        cfg_0 = ADS1120Config(temp=True, data_rate=2000).to_w32()
        cfg_1 = ADS1120Config(channel="AIN0-AIN1", gain=128, data_rate=2000).to_w32()

        # CSRs
        self.cr = CSRStorage("cr", fields=[
            CSRField("enable", 1, offset=0),
        ])
        self.cfg_0 = CSRStorage(32, name="cfg_0", reset=cfg_0)
        self.cfg_1 = CSRStorage(32, name="cfg_1", reset=cfg_1)
        self.seq_0 = CSRStatus(24, name="seq_0")
        self.seq_1 = CSRStatus(24, name="seq_1")

        # states
        sequence = Signal()
        seq_0_value = Signal(16)
        self.submodules.conv_wt = conv_wt = WaitTimer(int(sys_clk_freq // 1000))

        # SPI Master
        self.submodules.spi_master = master = SpiMaster(sys_clk_freq, 2E6, pads, 24)

        # SPI Slave: ADS1120
        self.submodules.ads1120 = ads = ADS1120()
        self.comb += [
            source.data.eq(Cat(seq_0_value, ads.source.data)),
            If(self.cr.fields.enable,
                master.source.connect(ads.spi_sink),
                ads.spi_source.connect(master.sink),
                conv_wt.wait.eq(ads.wait_conversion),
                ads.drdy_n.eq(~conv_wt.done),
            ),
            If(sequence,
                source.last.eq(1),
                ads.config.eq(self.cfg_1.storage),
                source.valid.eq(ads.source.valid),  # send both samples together
            ).Else(
                ads.config.eq(self.cfg_0.storage),
            ),
        ]
        self.sync += [
            If(ads.source.valid,
                sequence.eq(sequence + 1),
                If(sequence,
                    self.seq_1.status.eq(ads.source.data),
                ).Else(
                    self.seq_0.status.eq(ads.source.data),
                    seq_0_value.eq(ads.source.data),  # store the first sequence sample
                ),
            ),
        ]

        self.analyzer_signals = [
            ads.source.valid,
            ads.source.ready,
            source.valid,
            source.ready,
            source.last,
            sequence,
        ]

class TimeStampInserter(LiteXModule):
    def __init__(self, sys_clock_freq, ts_dw=32, dw=8, fifo_depth=0):
        """Inserts a timestamp and TSN timestamps to the end of a stream.
        This module adds
        - A timestamp, which value is sampled when the stream `last` is set
        - A timestamp, which is samples when the remote timestamp was written
        - A remote timestamp, which is produced by the clock of a remote  device

        Having these three values enable a remote host to re-synchronize a stream with its own
        clock, which can be useful in network applications

        parameters:
        - sys_clock_freq: system clock frequency, in Hz
        - ts_dw: timestamp data width
        - dw: input & output stream data width
        - fifo_depth: set to 0 to disable the FIFO

        inputs:
        - sink: steam.Endpoint([("data", 8)]). Its `.last` must be driven to trigger timestamp
          insertion
        - wait_full_packet: if set, wait for `sink.last == 1` before `source` is valid.

        outputs:
        - source: stream.Endpoint([("data", 8)])
        """
        # Parameters
        layout = [("data", dw)]

        # Inputs
        self.sink = sink = stream.Endpoint(layout)
        self.wait_full_packet = wait_full_packet = Signal()

        # Outputs
        self.source = source = stream.Endpoint(layout)

        # # #
        # State
        ts = Signal(ts_dw)
        ts_start = Signal(ts_dw)
        sync_local_ts = Signal(ts_dw)
        sync_local_ts_start = Signal(ts_dw)
        sync_remote_ts_start = Signal(ts_dw)
        self.fsm = fsm = FSM("STORE")

        # CSR
        self.ts = CSRStatus(ts_dw, name="ts")
        self.sync_remote_ts = CSRStorage(ts_dw, name="sync_remote_ts")

        # Timestamp timer
        self.sync += ts.eq(ts + 1)  # timestamp: freerunning clock
        self.comb += self.ts.status.eq(ts)

        # Remote timestamp sampling
        self.sync += If(self.sync_remote_ts.re, sync_local_ts.eq(ts))

        # FIFO
        self.submodules.fifo = fifo = stream.SyncFIFO([("data", 8)], fifo_depth)
        self.comb += sink.connect(fifo.sink),

        # Gearbox: transform ts_dw into 8 bits of data
        self.submodules.converter = converter = stream.Converter(ts_dw, dw)

        # Main logic
        fsm.act("STORE",
            If(~wait_full_packet,
                fifo.source.connect(source, omit=["last"]),
            ),
            If(sink.valid & sink.ready & sink.last,
                NextValue(ts_start, ts),
                NextValue(sync_local_ts_start, sync_local_ts),
                NextValue(sync_remote_ts_start, self.sync_remote_ts.storage),
                NextState("DATA"),
            ),
        )
        fsm.act("DATA",
            fifo.source.connect(source, omit=["last"]),
            If(fifo.source.valid & fifo.source.ready & fifo.source.last,
                NextState("TS"),
            ),
        )
        fsm.act("TS",
            converter.source.connect(source, omit=["last"]),
            converter.sink.data.eq(ts_start),
            converter.sink.valid.eq(1),
            If(converter.sink.ready,
                NextState("SYNC_REMOTE_TS"),
            ),
        )
        fsm.act("SYNC_REMOTE_TS",
            converter.source.connect(source, omit=["last"]),
            converter.sink.data.eq(sync_remote_ts_start),
            converter.sink.valid.eq(1),
            If(converter.sink.ready,
                NextState("SYNC_LOCAL_TS"),
            ),
        )
        fsm.act("SYNC_LOCAL_TS",
            converter.source.connect(source),
            converter.sink.data.eq(sync_local_ts_start),
            converter.sink.valid.eq(1),
            converter.sink.last.eq(1),
            If(converter.sink.ready,
                NextState("STORE")
            ),
        )
        fsm_STORE = Signal()
        fsm_DATA = Signal()
        fsm_TS = Signal()
        fsm_SYNC_REMOTE_TS = Signal()
        fsm_SYNC_LOCAL_TS = Signal()
        fsm_END = Signal()
        self.comb += [
            fsm_STORE.eq(fsm.ongoing("STORE")),
            fsm_DATA.eq(fsm.ongoing("DATA")),
            fsm_TS.eq(fsm.ongoing("TS")),
            fsm_SYNC_REMOTE_TS.eq(fsm.ongoing("SYNC_REMOTE_TS")),
            fsm_SYNC_LOCAL_TS.eq(fsm.ongoing("SYNC_LOCAL_TS")),
            fsm_END.eq(fsm.ongoing("END")),
        ]

        self.analyzer_signals = [
            sink.valid,
            sink.ready,
            sink.last,
            source.valid,
            source.ready,
            source.last,
            fsm_STORE,
            fsm_DATA,
            fsm_TS,
            fsm_SYNC_REMOTE_TS,
            fsm_SYNC_LOCAL_TS,
            fsm_END,
        ]

# BaseSoC ------------------------------------------------------------------------------------------

class BaseSoC(SoCCore):
    def __init__(self, variant="a7-35", toolchain="vivado", sys_clk_freq=100e6,
        with_xadc       = False,
        with_dna        = False,
        with_ethernet   = False,
        with_etherbone  = False,
        eth_ip          = "192.168.1.50",
        remote_ip       = None,
        eth_dynamic_ip  = False,
        with_usb        = False,
        with_led_chaser = True,
        with_spi_flash  = False,
        with_buttons    = False,
        with_pmod_gpio  = False,
        with_can        = False,
        **kwargs):
        platform = digilent_arty.Platform(variant=variant, toolchain=toolchain)
        platform.add_extension(_ios)

        analyzer_signals = []

        # CRG --------------------------------------------------------------------------------------
        with_dram = (kwargs.get("integrated_main_ram_size", 0) == 0)
        self.crg  = _CRG(platform, sys_clk_freq, with_dram)

        # SoCCore ----------------------------------------------------------------------------------
        SoCCore.__init__(self, platform, sys_clk_freq, ident="LiteX SoC on Arty A7", **kwargs)

        # XADC -------------------------------------------------------------------------------------
        if with_xadc:
            self.xadc = XADC()

        # DNA --------------------------------------------------------------------------------------
        if with_dna:
            self.dna = DNA()
            self.dna.add_timing_constraints(platform, sys_clk_freq, self.crg.cd_sys.clk)

        # DDR3 SDRAM -------------------------------------------------------------------------------
        if not self.integrated_main_ram_size:
            self.ddrphy = s7ddrphy.A7DDRPHY(platform.request("ddram"),
                memtype        = "DDR3",
                nphases        = 4,
                sys_clk_freq   = sys_clk_freq)
            self.add_sdram("sdram",
                phy           = self.ddrphy,
                module        = MT41K128M16(sys_clk_freq, "1:4"),
                l2_cache_size = kwargs.get("l2_size", 8192)
            )

        # Ethernet / Etherbone ---------------------------------------------------------------------
        if with_ethernet or with_etherbone:
            self.ethphy = LiteEthPHYMII(
                clock_pads = self.platform.request("eth_clocks"),
                pads       = self.platform.request("eth"))
            if with_etherbone:
                self.add_etherbone(phy=self.ethphy, ip_address=eth_ip, with_ethmac=with_ethernet)
            elif with_ethernet:
                self.add_ethernet(phy=self.ethphy, dynamic_ip=eth_dynamic_ip, local_ip=eth_ip, remote_ip=remote_ip)

            # ADC streamer -------------------------------------------------------------------------
            spi_pads = platform.request("spi_a", 0)
            self.submodules.udp_ads = udp = LiteEthUDPStreamer(
                udp = self.ethcore_etherbone.udp,
                ip_address="192.168.1.100",
                udp_port=0x6969,
                cd="etherbone",
                # tx_fifo_depth=1,
                # rx_fifo_depth=1,
                # with_broadcast=False,
            )
            # udp.tx.add_csr()
            self.submodules.ads1120 = ads1120 = Ads1120Sequencer(sys_clk_freq, spi_pads)
            self.submodules.ts_inserter = ts_inserter = TimeStampInserter(sys_clk_freq, ts_dw=64, fifo_depth=64)
            self.submodules.converter = converter = stream.Converter(ads1120.source.data.nbits, ts_inserter.sink.data.nbits)
            self.submodules.pipeline = stream.Pipeline(ads1120, converter, ts_inserter, udp)
            self.comb += [
                # ads1120.source.connect(converter.sink),
                # converter.source.connect(ts_inserter),
                # ts_inserter.source.connect(udp.sink),
                # ts_inserter.wait_full_packet.eq(1),
            ]
            analyzer_signals += ads1120.analyzer_signals + ts_inserter.analyzer_signals
            analyzer_signals += [
                udp.sink.valid,
                udp.sink.ready,
                udp.sink.last,
                udp.source.valid,
                udp.source.ready,
                udp.source.last,
            ]
            self.cr_test = CSRStorage("cr_test", fields=[
                CSRField("udp_en", 1, offset=0),
                CSRField("udp_last", 1, offset=1),
            ])
            frame_counter = Signal(8)
            self.submodules.wt_udp = wt = WaitTimer(int(sys_clk_freq / 100) )
            self.comb += [
                udp.sink.data.eq(frame_counter),
                udp.sink.last.eq(self.cr_test.fields.udp_last),
                wt.wait.eq(~wt.done),
                If(wt.done & self.cr_test.fields.udp_en,
                    udp.sink.valid.eq(1),
                ),
            ]
            self.sync += [
                If(wt.done & self.cr_test.fields.udp_en,
                    frame_counter.eq(frame_counter + 1),
                ),
            ]
            # analyzer_signals += [
            #     udp.sink,
            # ]



        # SPI Flash --------------------------------------------------------------------------------
        if with_spi_flash:
            from litespi.modules import S25FL128L
            from litespi.opcodes import SpiNorFlashOpCodes as Codes
            self.add_spi_flash(mode="4x", module=S25FL128L(Codes.READ_1_1_4), rate="1:2", with_master=True)

        # USB-OHCI ---------------------------------------------------------------------------------
        if with_usb:
            from litex.soc.cores.usb_ohci import USBOHCI
            from litex.build.generic_platform import Subsignal, Pins, IOStandard

            self.crg.cd_usb = ClockDomain()
            self.crg.pll.create_clkout(self.crg.cd_usb, 48e6, margin=0)

            # Machdyne PMOD (https://github.com/machdyne/usb_host_dual_socket_pmod)
            _usb_pmod_ios = [
                ("usb_pmoda", 0, # USB1 (top socket)
                    Subsignal("dp", Pins("pmoda:2")),
                    Subsignal("dm", Pins("pmoda:3")),
                    IOStandard("LVCMOS33"),
                ),
                ("usb_pmoda", 1, # USB2 (bottom socket)
                    Subsignal("dp", Pins("pmoda:0")),
                    Subsignal("dm", Pins("pmoda:1")),
                    IOStandard("LVCMOS33"),
                )
            ]
            self.platform.add_extension(_usb_pmod_ios)

            self.submodules.usb_ohci = USBOHCI(self.platform, self.platform.request("usb_pmoda", 0), usb_clk_freq=int(48e6))
            self.mem_map["usb_ohci"] = 0xc0000000
            self.bus.add_slave("usb_ohci_ctrl", self.usb_ohci.wb_ctrl, region=SoCRegion(origin=self.mem_map["usb_ohci"], size=0x100000, cached=False)) # FIXME: Mapping.
            self.dma_bus.add_master("usb_ohci_dma", master=self.usb_ohci.wb_dma)

            self.comb += self.cpu.interrupt[16].eq(self.usb_ohci.interrupt)

        # Leds -------------------------------------------------------------------------------------
        if with_led_chaser:
            self.leds = LedChaser(
                pads         = platform.request_all("user_led"),
                sys_clk_freq = sys_clk_freq,
            )

        # Buttons ----------------------------------------------------------------------------------
        if with_buttons:
            self.buttons = GPIOIn(
                pads     = platform.request_all("user_btn"),
                with_irq = self.irq.enabled
            )

        # GPIOs ------------------------------------------------------------------------------------
        if with_pmod_gpio:
            platform.add_extension(digilent_arty.raw_pmod_io("pmoda"))
            self.gpio = GPIOTristate(
                pads     = platform.request("pmoda"),
                with_irq = self.irq.enabled
            )

        # CAN --------------------------------------------------------------------------------------
        if with_can:
            from litex.soc.cores.can.ctu_can_fd import CTUCANFD
            self.platform.add_extension(digilent_arty.can_pmod_io("pmodc", 0))
            self.can0 = CTUCANFD(platform, platform.request("can", 0))
            self.bus.add_slave("can0", self.can0.bus, SoCRegion(origin=0xb0010000, size=0x10000, mode="rw", cached=False))
            self.irq.add("can0")

        # Analyzer ---------------------------------------------------------------------------------
        if len(analyzer_signals):
            self.submodules.analyzer = LiteScopeAnalyzer(analyzer_signals,
                depth        = 4096,
                clock_domain = "sys",
                samplerate   = sys_clk_freq,
                csr_csv      = "analyzer.csv")

# Build --------------------------------------------------------------------------------------------

def main():
    from litex.build.parser import LiteXArgumentParser
    parser = LiteXArgumentParser(platform=digilent_arty.Platform, description="LiteX SoC on Arty A7.")
    parser.add_target_argument("--flash",          action="store_true",       help="Flash bitstream.")
    parser.add_target_argument("--variant",        default="a7-35",           help="Board variant (a7-35 or a7-100).")
    parser.add_target_argument("--sys-clk-freq",   default=100e6, type=float, help="System clock frequency.")
    parser.add_target_argument("--with-xadc",      action="store_true",       help="Enable 7-Series XADC.")
    parser.add_target_argument("--with-dna",       action="store_true",       help="Enable 7-Series DNA.")
    parser.add_target_argument("--with-usb",       action="store_true",       help="Enable USB Host.")
    parser.add_target_argument("--with-ethernet",  action="store_true",       help="Enable Ethernet support.")
    parser.add_target_argument("--with-etherbone", action="store_true",       help="Enable Etherbone support.")
    parser.add_target_argument("--eth-ip",         default="192.168.1.50",    help="Ethernet/Etherbone IP address.")
    parser.add_target_argument("--remote-ip",      default="192.168.1.100",   help="Remote IP address of TFTP server.")
    parser.add_target_argument("--eth-dynamic-ip", action="store_true",       help="Enable dynamic Ethernet IP addresses setting.")
    sdopts = parser.target_group.add_mutually_exclusive_group()
    sdopts.add_argument("--with-spi-sdcard",       action="store_true",       help="Enable SPI-mode SDCard support.")
    sdopts.add_argument("--with-sdcard",           action="store_true",       help="Enable SDCard support.")
    parser.add_target_argument("--sdcard-adapter",                            help="SDCard PMOD adapter (digilent or numato).")
    parser.add_target_argument("--with-spi-flash", action="store_true",       help="Enable SPI Flash (MMAPed).")
    parser.add_target_argument("--with-pmod-gpio", action="store_true",       help="Enable GPIOs through PMOD.") # FIXME: Temporary test.
    parser.add_target_argument("--with-can",       action="store_true",       help="Enable CAN support (Through CTU-CAN-FD Core and SN65HVD230 'PMOD'.")
    args = parser.parse_args()

    assert not (args.with_etherbone and args.eth_dynamic_ip)

    soc = BaseSoC(
        variant        = args.variant,
        toolchain      = args.toolchain,
        sys_clk_freq   = args.sys_clk_freq,
        with_xadc      = args.with_xadc,
        with_dna       = args.with_dna,
        with_ethernet  = args.with_ethernet,
        with_etherbone = args.with_etherbone,
        eth_ip         = args.eth_ip,
        remote_ip      = args.remote_ip,
        eth_dynamic_ip = args.eth_dynamic_ip,
        with_usb       = args.with_usb,
        with_spi_flash = args.with_spi_flash,
        with_pmod_gpio = args.with_pmod_gpio,
        with_can       = args.with_can,
        **parser.soc_argdict
    )

    if args.sdcard_adapter == "numato":
        soc.platform.add_extension(digilent_arty._numato_sdcard_pmod_io)
    else:
        soc.platform.add_extension(digilent_arty._sdcard_pmod_io)
    if args.with_spi_sdcard:
        soc.add_spi_sdcard()
    if args.with_sdcard:
        soc.add_sdcard()

    builder = Builder(soc, **parser.builder_argdict)
    if args.build:
        builder.build(**parser.toolchain_argdict)

    if args.load:
        prog = soc.platform.create_programmer()
        prog.load_bitstream(builder.get_bitstream_filename(mode="sram"))

    if args.flash:
        prog = soc.platform.create_programmer()
        prog.flash(0, builder.get_bitstream_filename(mode="flash"))

if __name__ == "__main__":
    main()
