// bench_main.cpp
// Local performance harness for the per-packet hot path. The packet source is
// SIMULATED (no headset required): buffers shaped like the WH-1000XM5's Android
// Head Tracker input report feed the real library code (descriptor decode,
// orientation filter, JSON serialisation, UDP output). GUI per-sample text
// formatting and the paint back-buffer pattern are REPLICATED from gui.cpp,
// because the Window class is not callable headlessly.
//
// It also carries verbatim reference copies of the baseline implementations of
// decodePackedDescriptorValues and toJson, and checks the library against them
// on a randomized corpus, so any optimisation that changes results or output
// bytes fails loudly here.
#include "sony_head_tracker/windows_prelude.hpp"

#include "sony_head_tracker/hid_descriptor.hpp"
#include "sony_head_tracker/math.hpp"
#include "sony_head_tracker/orientation.hpp"
#include "sony_head_tracker/output_udp.hpp"
#include "sony_head_tracker/protocol.hpp"
#include "sony_head_tracker/types.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <cstdio>
#include <format>
#include <iomanip>
#include <random>
#include <sstream>
#include <string>
#include <vector>

using namespace sony;

namespace {

volatile std::uint64_t g_sink;   // defeats dead-code elimination

double bestOfSeconds(int repeats, auto&& body) {
    double best = 1e300;
    for (int r = 0; r < repeats; ++r) {
        const auto t0 = std::chrono::steady_clock::now();
        body();
        const auto t1 = std::chrono::steady_clock::now();
        best = std::min(best, std::chrono::duration<double>(t1 - t0).count());
    }
    return best;
}

void report(const char* name, double seconds, long long iterations) {
    const double ns = seconds * 1e9 / iterations;
    std::printf("%-28s %12.1f ns/op   %10.6f ms/op   (%lld iters, best run %.3f s)\n",
                name, ns, ns / 1e6, iterations, seconds);
}

// ---------------------------------------------------------------------------
// Reference copies of the BASELINE implementations (verbatim), used to verify
// that optimised library code produces identical results / identical bytes.
// ---------------------------------------------------------------------------

std::vector<double> referenceDecode(std::span<const std::uint8_t> packed, const DescriptorField& field) {
    std::vector<double> result;
    if (!field.bitSize || field.bitSize > 63) return result;
    result.reserve(field.reportCount);
    for (unsigned valueIndex=0; valueIndex<field.reportCount; ++valueIndex) {
        std::uint64_t raw{};
        const auto offset=static_cast<std::size_t>(valueIndex)*field.bitSize;
        for (unsigned bitIndex=0; bitIndex<field.bitSize; ++bitIndex) {
            const auto bit=offset+bitIndex;
            if (bit/8 < packed.size() && (packed[bit/8] & (1u << (bit%8)))) raw |= std::uint64_t{1} << bitIndex;
        }
        std::int64_t value=static_cast<std::int64_t>(raw);
        if (field.logicalMin < 0) {
            const auto sign=std::uint64_t{1} << (field.bitSize-1);
            const auto mask=(std::uint64_t{1} << field.bitSize)-1;
            value=static_cast<std::int64_t>(((raw&mask)^sign)-sign);
        }
        result.push_back(descriptorScale(value,field.logicalMin,field.logicalMax,field.physicalMin,field.physicalMax,field.unitExponent));
    }
    return result;
}

std::string referenceToJson(const MotionSample& s, std::string_view deviceJson) {
    const auto gyro = s.angularVelocity
        ? std::format("[{:.9g},{:.9g},{:.9g}]", s.angularVelocity->x, s.angularVelocity->y, s.angularVelocity->z)
        : std::string("null");
    const auto accel = s.acceleration
        ? std::format("[{:.9g},{:.9g},{:.9g}]", s.acceleration->x, s.acceleration->y, s.acceleration->z)
        : std::string("null");
    return std::format("{{\"version\":2,\"device\":{},\"rotationVector\":[{:.9g},{:.9g},{:.9g}],\"quaternion\":[{:.9g},{:.9g},{:.9g},{:.9g}],\"yprDegrees\":[{:.9g},{:.9g},{:.9g}],\"gyroscope\":{},\"accelerometer\":{},\"angularVelocity\":{},\"resetCounter\":{},\"packetsPerSecond\":{:.3f},\"receiveLatencyMs\":{:.3f}}}",
        deviceJson, s.rotationVector.x, s.rotationVector.y, s.rotationVector.z,
        s.orientation.w, s.orientation.x, s.orientation.y, s.orientation.z,
        s.euler.yaw, s.euler.pitch, s.euler.roll, gyro, accel, gyro, s.resetCounter, s.packetsPerSecond, s.receiveLatencyMs);
}

// Replicated from hid_backend.cpp (hexDump) so the bench links without the HID
// platform layer. Measures the per-packet raw-readout formatting cost.
std::wstring hexDumpReplica(const std::vector<std::uint8_t>& bytes) {
    std::wostringstream out;out<<std::hex<<std::uppercase<<std::setfill(L'0');for(std::size_t i=0;i<bytes.size();++i){if(i)out<<L' ';out<<std::setw(2)<<static_cast<unsigned>(bytes[i]);}return out.str();
}

// ---------------------------------------------------------------------------
// Simulated packet corpus (WH-1000XM5-like: 3x16-bit rotation + 3x16-bit gyro)
// ---------------------------------------------------------------------------

DescriptorField headTrackerField(std::int32_t physicalMax) {
    DescriptorField f;
    f.usagePage=0x20; f.reportCount=3; f.bitSize=16;
    f.logicalMin=-32767; f.logicalMax=32767;
    f.physicalMin=-physicalMax; f.physicalMax=physicalMax;
    f.unitExponent=-6;
    return f;
}

struct RawPacket { std::array<std::uint8_t,6> rotation; std::array<std::uint8_t,6> gyro; };

std::vector<RawPacket> makePackets(std::size_t count) {
    std::vector<RawPacket> packets(count);
    std::mt19937 rng(1234);
    for (auto& p : packets) {
        for (auto& b : p.rotation) b = static_cast<std::uint8_t>(rng());
        for (auto& b : p.gyro) b = static_cast<std::uint8_t>(rng());
    }
    return packets;
}

MotionSample typicalSample() {
    MotionSample s;
    s.rotationVector = {0.0123456789, -0.0456789012, 0.3123456789};
    s.orientation = {0.987654321, 0.006123456, -0.002345678, 0.155432109};
    s.euler = {17.8412345, -0.4612345, 1.3712345};
    s.angularVelocity = Vec3{0.0112345, -0.0009876, -0.0212345};
    s.resetCounter = 0;
    s.packetsPerSecond = 25.0;
    s.receiveLatencyMs = -1.0;
    return s;
}

bool verifyEquivalence() {
    std::mt19937 rng(42);
    bool ok = true;
    // decode equivalence over randomized fields and buffers (incl. truncation).
    for (int i = 0; i < 20000 && ok; ++i) {
        DescriptorField f;
        f.bitSize = static_cast<std::uint16_t>(rng() % 66);            // 0..65 incl. degenerate
        f.reportCount = static_cast<std::uint16_t>(rng() % 5);         // 0..4
        const bool sgn = rng() & 1;
        f.logicalMin = sgn ? -(1 << std::min<unsigned>(f.bitSize ? f.bitSize - 1 : 0, 30)) : 0;
        f.logicalMax = (1 << std::min<unsigned>(f.bitSize ? f.bitSize - 1 : 0, 30)) - 1;
        f.physicalMin = (rng() & 1) ? -3141592 : 0;
        f.physicalMax = (rng() & 1) ? 3141592 : 0;
        f.unitExponent = static_cast<std::int8_t>(static_cast<int>(rng() % 16) - 8);
        std::vector<std::uint8_t> buffer(rng() % 20);
        for (auto& b : buffer) b = static_cast<std::uint8_t>(rng());
        const auto a = referenceDecode(buffer, f);
        const auto b = decodePackedDescriptorValues(buffer, f);
        if (a.size() != b.size()) { ok = false; break; }
        for (std::size_t k = 0; k < a.size(); ++k)
            if (std::memcmp(&a[k], &b[k], sizeof(double)) != 0) { ok = false; break; }   // bit-identical
    }
    std::printf("decode equivalence vs baseline reference: %s\n", ok ? "OK (bit-identical)" : "MISMATCH");
    // toJson byte-equivalence over a randomized corpus.
    bool jsonOk = true;
    std::uniform_real_distribution<double> d(-200.0, 200.0);
    for (int i = 0; i < 20000 && jsonOk; ++i) {
        MotionSample s;
        s.rotationVector = {d(rng), d(rng), d(rng)};
        s.orientation = {d(rng), d(rng), d(rng), d(rng)};
        s.euler = {d(rng), d(rng), d(rng)};
        if (rng() & 1) s.angularVelocity = Vec3{d(rng), d(rng), d(rng)};
        if (rng() & 1) s.acceleration = Vec3{d(rng), d(rng), d(rng)};
        s.resetCounter = static_cast<std::uint8_t>(rng());
        s.packetsPerSecond = d(rng);
        s.receiveLatencyMs = (rng() & 1) ? d(rng) : -1.0;
        const char* device = (rng() & 1) ? "\"WH-1000XM5\"" : "null";
        if (referenceToJson(s, device) != toJson(s, device)) jsonOk = false;
    }
    std::printf("toJson byte-equivalence vs baseline reference: %s\n", jsonOk ? "OK (identical bytes)" : "MISMATCH");
    return ok && jsonOk;
}

} // namespace

int main() {
    std::printf("sony-head-tracker local benchmark (SIMULATED packet source; no headset)\n");
    std::printf("=========================================================================\n");
    const bool equivalent = verifyEquivalence();
    std::printf("\n");

    const auto packets = makePackets(1024);
    const auto rotationField = headTrackerField(3141592);   // +-pi rad, 10^-6
    const auto gyroField = headTrackerField(31415926);      // +-10*pi rad/s, 10^-6

    // --- packet parsing: descriptor decode of both vector fields ------------
    {
        constexpr long long N = 2'000'000;
        const auto s = bestOfSeconds(5, [&] {
            std::uint64_t sink = 0;
            for (long long i = 0; i < N; ++i) {
                const auto& p = packets[static_cast<std::size_t>(i) & 1023];
                const auto r = decodePackedDescriptorValues(p.rotation, rotationField);
                const auto g = decodePackedDescriptorValues(p.gyro, gyroField);
                sink += r.size() + g.size();
            }
            g_sink += sink;
        });
        report("parse (2 vector fields)", s, N);
    }

    // --- orientation filter --------------------------------------------------
    {
        constexpr long long N = 2'000'000;
        OrientationFilter filter{FilterConfig{}};
        auto base = std::chrono::steady_clock::now();
        const auto s = bestOfSeconds(5, [&] {
            std::uint64_t sink = 0;
            for (long long i = 0; i < N; ++i) {
                MotionSample in;
                const double t = static_cast<double>(i) * 0.04;
                in.rotationVector = {0.3 * std::sin(t * 0.1), 0.2 * std::cos(t * 0.13), 0.4 * std::sin(t * 0.07)};
                in.angularVelocity = Vec3{0.02 * std::sin(t), 0.01, 0.015 * std::cos(t)};
                in.receivedAt = base + std::chrono::milliseconds(40 * i);
                const auto out = filter.process(in);
                sink += static_cast<std::uint64_t>(out.euler.yaw * 0.0 + 1.0);
            }
            g_sink += sink;
        });
        report("filter.process", s, N);
    }

    // --- JSON serialisation --------------------------------------------------
    {
        constexpr long long N = 500'000;
        const auto sample = typicalSample();
        const auto device = jsonEscapeString("WH-1000XM5");
        const auto s = bestOfSeconds(5, [&] {
            std::uint64_t sink = 0;
            for (long long i = 0; i < N; ++i) sink += toJson(sample, device).size();
            g_sink += sink;
        });
        report("toJson (gyro present)", s, N);
    }
    {
        constexpr long long N = 500'000;
        MotionSample sample = typicalSample();
        sample.angularVelocity.reset();
        const auto s = bestOfSeconds(5, [&] {
            std::uint64_t sink = 0;
            for (long long i = 0; i < N; ++i) sink += toJson(sample, "null").size();
            g_sink += sink;
        });
        report("toJson (gyro null)", s, N);
    }

    // --- OpenTrack pose (trivial, for completeness) --------------------------
    {
        constexpr long long N = 10'000'000;
        const auto sample = typicalSample();
        const auto s = bestOfSeconds(5, [&] {
            double sink = 0;
            for (long long i = 0; i < N; ++i) sink += toOpenTrackPose(sample)[3];
            g_sink += static_cast<std::uint64_t>(sink);
        });
        report("toOpenTrackPose", s, N);
    }

    // --- UDP output: full send path (toJson + 2x sendto to loopback) --------
    {
        constexpr long long N = 100'000;
        UdpOutput udp;
        if (!udp.open("127.0.0.1", 45242)) { std::printf("udp open FAILED\n"); return 1; }
        udp.setDeviceLabel(L"WH-1000XM5");
        const auto sample = typicalSample();
        const auto s = bestOfSeconds(5, [&] {
            for (long long i = 0; i < N; ++i) udp.send(sample);
        });
        report("udp.send (both datagrams)", s, N);
    }

    // --- GUI per-sample formatting (REPLICATED from gui.cpp onSample) --------
    {
        constexpr long long N = 500'000;
        const auto f = typicalSample();
        const auto s = bestOfSeconds(5, [&] {
            std::uint64_t sink = 0;
            for (long long i = 0; i < N; ++i) {
                const auto stats = std::format(L"Yaw {:+8.2f}°   Pitch {:+8.2f}°   Roll {:+8.2f}°   Samples/s {:6.1f}   Latency {}",
                    f.euler.yaw, f.euler.pitch, f.euler.roll, f.packetsPerSecond,
                    f.receiveLatencyMs < 0 ? L"     N/A" : std::format(L"{:7.2f} ms", f.receiveLatencyMs));
                const auto simple = std::format(L"Yaw {:+8.1f}°       Pitch {:+8.1f}°       Roll {:+8.1f}°", f.euler.yaw, f.euler.pitch, f.euler.roll);
                const auto gyro = f.angularVelocity ? std::format(L"X {:+7.2f}  Y {:+7.2f}  Z {:+7.2f}", f.angularVelocity->x, f.angularVelocity->y, f.angularVelocity->z) : std::wstring(L"unavailable");
                const auto accel = f.acceleration ? std::format(L"X {:+7.2f}  Y {:+7.2f}  Z {:+7.2f}", f.acceleration->x, f.acceleration->y, f.acceleration->z) : std::wstring(L"not reported by this device");
                const auto motion = std::format(L"Gyroscope (rad/s)  {}     Accelerometer (m/s²)  {}", gyro, accel);
                sink += stats.size() + simple.size() + motion.size();
            }
            g_sink += sink;
        });
        report("gui per-sample text fmt", s, N);
    }
    {
        constexpr long long N = 500'000;
        std::vector<std::uint8_t> raw(13);
        for (std::size_t i = 0; i < raw.size(); ++i) raw[i] = static_cast<std::uint8_t>(i * 37);
        const auto s = bestOfSeconds(5, [&] {
            std::uint64_t sink = 0;
            for (long long i = 0; i < N; ++i) sink += (L"Raw packet: " + hexDumpReplica(raw)).size();
            g_sink += sink;
        });
        report("gui raw hexDump fmt", s, N);
    }

    // --- GDI back-buffer patterns (REPLICATED paint cost, no real window) ----
    {
        constexpr long long FRAMES = 2'000;
        const int W = 1160, H = 860;
        HDC screen = GetDC(nullptr);
        HDC target = CreateCompatibleDC(screen);
        HBITMAP targetBmp = CreateCompatibleBitmap(screen, W, H);
        auto oldTarget = SelectObject(target, targetBmp);
        HBRUSH brush = CreateSolidBrush(RGB(20, 23, 30));
        RECT full{0, 0, W, H};

        const auto sAlloc = bestOfSeconds(3, [&] {
            for (long long i = 0; i < FRAMES; ++i) {
                HDC mem = CreateCompatibleDC(screen);
                HBITMAP bmp = CreateCompatibleBitmap(screen, W, H);
                auto old = SelectObject(mem, bmp);
                FillRect(mem, &full, brush);
                BitBlt(target, 0, 0, W, H, mem, 0, 0, SRCCOPY);
                SelectObject(mem, old); DeleteObject(bmp); DeleteDC(mem);
            }
        });
        report("paint: fresh buffer/frame", sAlloc, FRAMES);

        HDC cached = CreateCompatibleDC(screen);
        HBITMAP cachedBmp = CreateCompatibleBitmap(screen, W, H);
        auto oldCached = SelectObject(cached, cachedBmp);
        const auto sCached = bestOfSeconds(3, [&] {
            for (long long i = 0; i < FRAMES; ++i) {
                FillRect(cached, &full, brush);
                BitBlt(target, 0, 0, W, H, cached, 0, 0, SRCCOPY);
            }
        });
        report("paint: cached buffer/frame", sCached, FRAMES);

        RECT graph{16, 414, W - 16, H - 14};   // simple-mode graph band
        const auto sClip = bestOfSeconds(3, [&] {
            for (long long i = 0; i < FRAMES; ++i) {
                FillRect(cached, &full, brush);
                BitBlt(target, graph.left, graph.top, graph.right - graph.left, graph.bottom - graph.top,
                       cached, graph.left, graph.top, SRCCOPY);
            }
        });
        report("paint: cached + clip blit", sClip, FRAMES);

        SelectObject(cached, oldCached); DeleteObject(cachedBmp); DeleteDC(cached);
        SelectObject(target, oldTarget); DeleteObject(targetBmp); DeleteDC(target);
        DeleteObject(brush); ReleaseDC(nullptr, screen);
    }

    // --- simulated end-to-end hot path: parse -> filter -> UDP ---------------
    {
        constexpr long long N = 100'000;
        OrientationFilter filter{FilterConfig{}};
        UdpOutput udp;
        udp.open("127.0.0.1", 45244);
        udp.setDeviceLabel(L"WH-1000XM5");
        auto base = std::chrono::steady_clock::now();
        const auto s = bestOfSeconds(5, [&] {
            for (long long i = 0; i < N; ++i) {
                const auto& p = packets[static_cast<std::size_t>(i) & 1023];
                const auto r = decodePackedDescriptorValues(p.rotation, rotationField);
                const auto g = decodePackedDescriptorValues(p.gyro, gyroField);
                MotionSample in;
                if (r.size() == 3) in.rotationVector = {r[0], r[1], r[2]};
                if (g.size() == 3) in.angularVelocity = Vec3{g[0], g[1], g[2]};
                in.receivedAt = base + std::chrono::milliseconds(40 * i);
                in.packetsPerSecond = 25.0;
                udp.send(filter.process(in));
            }
        });
        report("e2e parse+filter+udp", s, N);
    }

    std::printf("\nsink=%llu\n", static_cast<unsigned long long>(g_sink));
    return equivalent ? 0 : 1;
}
