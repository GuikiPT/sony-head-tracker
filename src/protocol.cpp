// protocol.cpp
// Pure serialisation of a MotionSample. No sockets, no Windows.
#include "sony_head_tracker/protocol.hpp"

#include <charconv>

namespace sony {

namespace {

// std::format's {:.9g} and {:.3f} are specified in terms of std::to_chars
// ([format.string.std]), so appending through to_chars produces byte-identical
// output while skipping the per-call format-string parsing. Verified against
// the previous std::format implementation by the randomized byte-equivalence
// check in bench/bench_main.cpp.
void appendDouble(std::string& out, double value, std::chars_format form, int precision) {
    char buffer[64];
    const auto r = std::to_chars(buffer, buffer + sizeof buffer, value, form, precision);
    out.append(buffer, r.ptr);
}

void appendVector9g(std::string& out, const Vec3& v) {
    out.push_back('[');
    appendDouble(out, v.x, std::chars_format::general, 9); out.push_back(',');
    appendDouble(out, v.y, std::chars_format::general, 9); out.push_back(',');
    appendDouble(out, v.z, std::chars_format::general, 9); out.push_back(']');
}

// Formats "[x,y,z]" at 9 significant digits into a caller buffer (>= 96 bytes)
// so the gyroscope text can be appended twice (angularVelocity is a deprecated
// alias) without re-formatting or a heap allocation.
std::size_t formatVector9g(char* buffer, char* end, const Vec3& v) {
    char* p = buffer;
    *p++ = '[';
    p = std::to_chars(p, end, v.x, std::chars_format::general, 9).ptr; *p++ = ',';
    p = std::to_chars(p, end, v.y, std::chars_format::general, 9).ptr; *p++ = ',';
    p = std::to_chars(p, end, v.z, std::chars_format::general, 9).ptr; *p++ = ']';
    return static_cast<std::size_t>(p - buffer);
}

} // namespace

std::array<double, 6> toOpenTrackPose(const MotionSample& s) {
    // Translation is always zero -- this protocol reports orientation only -- so
    // the head angles occupy the last three slots.
    return {0.0, 0.0, 0.0, s.euler.yaw, s.euler.pitch, s.euler.roll};
}

std::string jsonEscapeString(std::string_view utf8) {
    std::string escaped;
    escaped.reserve(utf8.size() + 2);
    escaped.push_back('"');
    for (const char c : utf8) {
        if (c == '"' || c == '\\') escaped.push_back('\\');
        if (static_cast<unsigned char>(c) >= 0x20) escaped.push_back(c);   // control characters are dropped
    }
    escaped.push_back('"');
    return escaped;
}

void toJsonTo(std::string& out, const MotionSample& s, std::string_view deviceJson) {
    // gyroscope is radians/second; accelerometer is m/s^2 and is null unless the device actually reports it.
    char gyroBuffer[96];
    const auto gyro = s.angularVelocity
        ? std::string_view(gyroBuffer, formatVector9g(gyroBuffer, gyroBuffer + sizeof gyroBuffer, *s.angularVelocity))
        : std::string_view("null");
    out.clear();
    out.append("{\"version\":2,\"device\":").append(deviceJson);
    out.append(",\"rotationVector\":"); appendVector9g(out, s.rotationVector);
    out.append(",\"quaternion\":[");
    appendDouble(out, s.orientation.w, std::chars_format::general, 9); out.push_back(',');
    appendDouble(out, s.orientation.x, std::chars_format::general, 9); out.push_back(',');
    appendDouble(out, s.orientation.y, std::chars_format::general, 9); out.push_back(',');
    appendDouble(out, s.orientation.z, std::chars_format::general, 9); out.push_back(']');
    out.append(",\"yprDegrees\":[");
    appendDouble(out, s.euler.yaw, std::chars_format::general, 9); out.push_back(',');
    appendDouble(out, s.euler.pitch, std::chars_format::general, 9); out.push_back(',');
    appendDouble(out, s.euler.roll, std::chars_format::general, 9); out.push_back(']');
    out.append(",\"gyroscope\":").append(gyro);
    out.append(",\"accelerometer\":");
    if (s.acceleration) appendVector9g(out, *s.acceleration); else out.append("null");
    out.append(",\"angularVelocity\":").append(gyro);
    out.append(",\"resetCounter\":");
    {
        char buffer[8];
        const auto r = std::to_chars(buffer, buffer + sizeof buffer, static_cast<unsigned>(s.resetCounter));
        out.append(buffer, r.ptr);
    }
    out.append(",\"packetsPerSecond\":"); appendDouble(out, s.packetsPerSecond, std::chars_format::fixed, 3);
    out.append(",\"receiveLatencyMs\":"); appendDouble(out, s.receiveLatencyMs, std::chars_format::fixed, 3);
    out.push_back('}');
}

std::string toJson(const MotionSample& s, std::string_view deviceJson) {
    std::string out;
    out.reserve(384);
    toJsonTo(out, s, deviceJson);
    return out;
}

} // namespace sony
