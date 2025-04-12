import unittest
from migen import passive, run_simulation, Record, C, Module
from regmap.core.spi import SpiMaster, SpiClkSync, SPISuperviser, VNI8200XP
from regmap.devices.ads131 import ADS131M04


def word2bits(word, dw=16):
    return [(word >> i) & 1 for i in range(dw - 1, -1, -1)]


def words2bits(words, dw=16):
    return [bit for word in words for bit in word2bits(word, dw)]


@passive
def spi_loopback(pads, inverted=True):
    while True:
        yield pads.miso.eq(pads.mosi if not inverted else ~pads.mosi)
        yield


@passive
def spi_check_cs_cpol(pads, cs_pad, cpol):
    old_cs = (yield cs_pad)
    yield
    while True:
        cs = (yield cs_pad)
        if cs != old_cs:
            if cpol:
                assert (yield pads.sclk) == 1
            else:
                assert (yield pads.sclk) == 0
        old_cs = cs
        yield


def spi_wait_cs_toggle(cs_pad, timeout=100):
    while (yield cs_pad) == 1:
        assert timeout > 0
        timeout -= 1
        yield
    while (yield cs_pad) == 0:
        assert timeout > 0
        timeout -= 1
        yield
    yield
    yield

@passive
def spi_check_data_stable_cpol_cpha(pads, cs, cpol, cpha):
    old_sclk = (yield pads.sclk)
    old_cs = (yield cs)
    old_mosi = (yield pads.mosi)
    yield
    while True:
        sclk = (yield pads.sclk)
        cs = (yield cs)
        mosi = (yield pads.mosi)
        if mosi != old_mosi:
            if (cs == 0) & (old_cs == 1):
                # MOSI can change at the same time as CS
                pass
            elif (cpol ^ cpha) == sclk and old_sclk == sclk:
                # or at the right clock flank
                pass
            else:
                raise Exception("MOSI changed at invalid time")
        old_sclk = sclk
        old_cs = cs
        old_mosi = mosi
        yield


def wait_min_sclk(cycles=10):
    for _ in range(cycles):
        yield


# @passive
def spi_miso_mock(pads, miso, cpol=0, cpha=0, cs=None):
    sclk = yield(pads.sclk)
    if cs is None:
        cs = C(0)

    for bit in miso:
        yield
        sclk = (yield pads.sclk)
        # wait for the correct edge
        while not ((yield pads.sclk) ^ sclk) & (sclk ^ cpol ^ cpha) & (yield ~cs):
            sclk = (yield pads.sclk)
            yield
        yield pads.miso.eq(bit)


def spi_mosi_check(pads, bits, cs_pad=None, cpol=0, cpha=0, timeout=100):
    if cs_pad is None:
        cs_pad = C(0)  # always active
    old_sclk = (yield pads.sclk)
    old_mosi = (yield pads.mosi)
    old_cs = (yield cs_pad)
    yield
    for bit in bits:
        next_bit = False
        for cycles in range(timeout):
            yield
            sclk = (yield pads.sclk)
            mosi = (yield pads.mosi)
            cs = (yield cs_pad)

            if cs == 0:
                if old_cs == 1:
                    # check cpha & cpol on sclk
                    assert sclk == cpol
                    pass
                elif old_sclk == sclk:
                    assert old_mosi == mosi
                else:
                    # clock flank
                    if sclk ^ cpol ^ cpha == 0:
                        # data update flank
                        if bit != 'x':
                            assert mosi == bit
                        next_bit = True
                    else:
                        # data sample flank
                        assert old_mosi == mosi
            elif old_cs == 0:
                assert old_sclk == cpol

            old_sclk = sclk
            old_mosi = mosi
            old_cs = cs
            if next_bit:
                break
        raise Exception("Timeout waiting for bit")


class TestSpiClkSync(unittest.TestCase):
    def test_cpol_cpha(self):
        def check(dut, cpol, cpha, cycles=10):
            if cpol:
                if cpha:
                    update_on_rising = False
                else:
                    update_on_rising = True
            else:
                if cpha:
                    update_on_rising = True
                else:
                    update_on_rising = False

            for cnt in range(cycles, -1):
                while True:
                    if (yield dut.update):
                        sclk = (yield dut.sclk)
                        yield
                        if update_on_rising:
                            assert sclk == 0
                            assert (yield dut.sclk) == 1
                        else:
                            assert sclk == 1
                            assert (yield dut.sclk) == 0
                    elif (yield dut.update):
                        sclk = (yield dut.sclk)
                        yield
                        if update_on_rising:
                            assert sclk == 1
                            assert (yield dut.sclk) == 0
                        else:
                            assert sclk == 0
                            assert (yield dut.sclk) == 1
                    else:
                        assert cnt

        for cpol in [0, 1]:
            for cpha in [0, 1]:
                print(f"cpol={cpol} cpha={cpha}")
                dut = SpiClkSync(12E6, 4E6, cpol, cpha)

                run_simulation(dut, [
                        check(dut, cpol, cpha),
                        wait_min_sclk(),
                    ], vcd_name=f"out/test_core_spi_clk_sync_{cpol}_{cpha}.vcd")


class TestSpiMaster(unittest.TestCase):
    def send_word(self, dut, width, word, cpol=0, cpha=0, last=0):
        yield dut.sink.valid.eq(1)
        yield dut.sink.data.eq(word)
        yield dut.sink.width.eq(width)
        yield dut.sink.cpol.eq(cpol)
        yield dut.sink.cpha.eq(cpha)
        yield dut.sink.last.eq(last)
        yield
        while (yield dut.sink.ready) == 0:
            yield
        yield dut.sink.valid.eq(0)

    def send_words(self, dut, width, words, cpol=0, cpha=0):
        yield
        for (i, word) in enumerate(words):
            yield from self.send_word(dut, width, word, cpol, cpha, last=1 if i==len(words)-1 else 0)

    def pop_word(self, dut, expected_word):
        yield dut.source.ready.eq(1)
        yield
        while (yield dut.source.valid) == 0:
            yield
        self.assertEqual((yield dut.source.data), expected_word)
        yield dut.source.ready.eq(0)

    def pop_words(self, dut, expected_words):
        for expected_word in expected_words:
            yield from self.pop_word(dut, expected_word)

    def test_master_single(self):
        pads = Record([
            ("sclk", 1),
            ("miso", 1),
            ("mosi", 1),
            ("cs", 1),
            ])
        dut = SpiMaster(
            sys_fcy=4E6, min_fcy=1E6, pads=pads, dw=16, sclk_output_idle=False)
        dut.comb += pads.cs.eq(~dut.busy)

        word = 0xde
        width = 8
        bits = [(word >> i) & 0b1 for i in range(width, -1)]
        read_word = (~word) & ((1<<width) - 1)
        def stim():
            for _ in range(100):
                yield
        run_simulation(dut, [
                # stim(),
                spi_mosi_check(pads, bits, ~dut.busy),
                spi_loopback(pads),
                self.send_words(dut, width-1, [word]),
                self.pop_words(dut, [read_word])
            ], vcd_name="out/test_core_spi_master_single.vcd")

    def test_master_multiple(self):
        pads = Record([
            ("sclk", 1),
            ("miso", 1),
            ("mosi", 1),
            ("cs", 1),
            ])

        for cpol in [0, 1]:
            for cpha in [0, 1]:
                dut = SpiMaster(
                    sys_fcy=4E6, min_fcy=1E6, pads=pads, dw=16, sclk_output_idle=False)
                dut.comb += pads.cs.eq(~dut.busy)
                words = [0xde, 0xad, 0xbe, 0xef]
                width = 8
                bits = [(word >> i) & 0b1 for i in range(width, -1) for word in words]
                read_words = [(~word) & ((1<<width) - 1) for word in words]
                print(f"cpol={cpol} cpha={cpha}")
                run_simulation(dut, [
                        spi_mosi_check(pads, bits, ~dut.busy, cpol, cpha),
                        spi_loopback(pads),
                        self.send_words(dut, width-1, words, cpol, cpha),
                        self.pop_words(dut, read_words),
                        spi_check_cs_cpol(pads, pads.cs, cpol),
                        spi_wait_cs_toggle(pads.cs, 300),
                        # spi_check_data_stable_cpol_cpha(pads, pads.cs, cpol, cpha),
                    ], vcd_name=f"out/test_core_spi_master_multiple_{cpol}_{cpha}.vcd")


class TestADS131(unittest.TestCase):
    def test_freerunning(self):
        class Dut(Module):
            def __init__(self, pads):
                self.submodules.spi = SpiMaster(
                    sys_fcy=4E6, min_fcy=1E6, pads=pads, dw=24, sclk_output_idle=False)
                self.submodules.ads = ADS131M04(config=(4, [
                    0x7777,  # reg=0x04; Gain=128 for all channels
                    0x0000,  # reg=0x05; reserved
                    (0b0011 << 9) | (0b1 << 8),  # reg=0x06; chop mode enable, delay=16
                ]))
                self.comb += [
                    self.spi.source.connect(self.ads.spi_sink),
                    self.ads.spi_source.connect(self.spi.sink),
                    pads.cs.eq(~self.spi.busy),
                ]

        pads = Record([
            ("sclk", 1),
            ("miso", 1),
            ("mosi", 1),
            ("cs", 1),
            ])

        dut = Dut(pads)
        run_simulation(dut, wait_min_sclk(1000), vcd_name="out/test_core_spi_ads131_freerunning.vcd")




class TestLegacy(unittest.TestCase):
    def _test_freerunning(self):
        # FIXME: delete or make it work
        def dut():
            pads = Record([("clk", 1), ("cs_n", 2), ("mosi", 1), ("miso", 1)])
            poller = SPISuperviser(100E6, pads, [VNI8200XP()])
            poller.add_slave(VNI8200XP())
            module_pads = set(pads.flatten())
            for slave in poller.slaves:
                module_pads |= {slave.outputs, slave.status}

            return poller, module_pads
        from migen.fhdl.verilog import convert
        print(convert(*dut()))

        from migen.sim import run_simulation
        def bench(dut):
            for _ in range(100):
                yield
            yield dut.slaves[0].outputs.eq(0x42)
            for _ in range(1000):
                yield

        top = dut()[0]
        run_simulation(top, bench(top), vcd_name="out/test_core_spi_SPISuperviser.vcd")
