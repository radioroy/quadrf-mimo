// Drives the Soapy mipi device's FPGA register interface and timed writes
// (JTAG stream) without going through the plugin loader:
//
//   g++ -O2 -std=c++17 -I<soapy src> soapy_jstream.cpp <soapy src>/MipiDevice.cpp \
//       <soapy src>/NEON.cpp $(pkg-config --cflags --libs SoapySDR) -o soapy_jstream
#include "MipiDevice.hpp"
#include <SoapySDR/Logger.hpp>
#include <chrono>
#include <cstdio>

static int fails;
#define CHECK(c, ...) do { if (!(c)) { fails++; std::printf("FAIL: " __VA_ARGS__); std::printf("\n"); } } while (0)

static double ms_since(std::chrono::steady_clock::time_point t)
{
    return std::chrono::duration<double, std::milli>(std::chrono::steady_clock::now() - t).count();
}

int main()
{
    SoapySDR::setLogLevel(SOAPY_SDR_WARNING);
    MipiDevice dev({});
    auto ifs = dev.listRegisterInterfaces();
    std::printf("register interfaces: %s %s\n", ifs[0].c_str(), ifs[1].c_str());

    dev.writeRegister("FPGA", 0x2C, 0x1111);
    CHECK(dev.readRegister("FPGA", 0x2C) == 0x1111, "immediate write/read");

    // two timed writes 50 ms apart; the read has to wait for both
    const long long t = dev.getHardwareTime("") + 20000000;
    auto tc = std::chrono::steady_clock::now();
    dev.setCommandTime(t, "");
    dev.writeRegisters("FPGA_WORDS", 0, {0x2CAAAAu, 0x2DBBBBu});
    dev.setCommandTime(t + 50000000, "");
    dev.writeRegister("FPGA", 0x2C, 0xCCCC);
    const double callms = ms_since(tc);
    std::printf("two timed write calls (stream start included): %.2f ms\n", callms);
    CHECK(callms < 5, "timed writes blocked %.1f ms", callms);
    std::printf("after timed writes: jtag_stream=%s\n  %s\n", dev.readSetting("jtag_stream").c_str(),
                dev.readSetting("jtag_stream_status").c_str());
    dev.setCommandTime(0, "");
    auto t0 = std::chrono::steady_clock::now();
    unsigned v2c = dev.readRegister("FPGA", 0x2C), v2d = dev.readRegister("FPGA", 0x2D);
    const double late = (dev.getHardwareTime("") - (t + 50000000)) / 1e6;
    std::printf("read waited %.1f ms, returned %.2f ms after the last timed write\n", ms_since(t0), late);
    CHECK(v2c == 0xCCCC && v2d == 0xBBBB, "timed values %04x %04x", v2c, v2d);
    CHECK(late >= 0, "read returned before the timed write");
    CHECK(dev.readSetting("jtag_stream") == "off", "on-demand stream still running");

    dev.setCommandTime(dev.getHardwareTime("") - 1000000, "");
    bool threw = false;
    try { dev.writeRegister("FPGA", 0x2C, 1); } catch (const std::exception &) { threw = true; }
    CHECK(threw, "late timed write did not throw");
    dev.setCommandTime(0, "");

    // held stream: reads refused, untimed writes ride the stream
    dev.writeSetting("jtag_stream_period_ns", "2005");
    dev.writeSetting("jtag_stream", "start");
    threw = false;
    try { dev.readRegister("FPGA", 0x2C); } catch (const std::exception &) { threw = true; }
    CHECK(threw, "read during held stream did not throw");
    std::vector<unsigned> words(20000);
    for (size_t i = 0; i < words.size(); i++)
        words[i] = ((i & 1 ? 0x2Du : 0x2Cu) << 16) | unsigned((i * 7907u + 0x5A5Au) & 0xFFFF);
    t0 = std::chrono::steady_clock::now();
    dev.writeRegisters("FPGA_WORDS", 0, words);
    const double qms = ms_since(t0);
    std::printf("held, 2005 ns: queued 20000 writes in %.1f ms\n  %s\n", qms, dev.readSetting("jtag_stream_status").c_str());
    dev.writeSetting("jtag_stream", "drain");
    std::printf("drained after %.1f ms (ideal %.1f)\n", ms_since(t0), 20000 * 2.005e-3);
    CHECK((dev.readRegister("FPGA", 0x2C) << 16 | dev.readRegister("FPGA", 0x2D)) ==
              ((words[19998] & 0xFFFF) << 16 | (words[19999] & 0xFFFF)), "held stream last values");

    // far-out timed write returns at once; quadrf-jtag then waits for it
    dev.setCommandTime(dev.getHardwareTime("") + 500000000, "");
    tc = std::chrono::steady_clock::now();
    dev.writeRegister("FPGA", 0x2C, 0x7777);
    std::printf("timed write 0.5 s out: call %.2f ms\n", ms_since(tc));
    CHECK(ms_since(tc) < 5, "far timed write blocked");
    dev.setCommandTime(0, "");
    t0 = std::chrono::steady_clock::now();
    const std::string pol = dev.readSetting("rx_pol");
    std::printf("rx_pol=%s via quadrf-jtag after %.1f ms\n", pol.c_str(), ms_since(t0));
    CHECK(ms_since(t0) > 450, "quadrf-jtag ran before the pending timed write");
    CHECK(dev.readRegister("FPGA", 0x2C) == 0x7777, "far timed write value");
    CHECK(!pol.empty(), "quadrf-jtag status after stream");
    dev.writeRegister("FPGA", 0x2D, 0);

    std::printf("%s (%d failures)\n", fails ? "FAILED" : "all ok", fails);
    return fails != 0;
}
