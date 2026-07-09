// hid_descriptor.cpp
// Pure HID report-descriptor value decoding. No Windows.
#include "sony_head_tracker/hid_descriptor.hpp"

#include <array>
#include <cmath>
#include <cstring>

namespace sony {

namespace {
// HID unit exponents are a nibble, decoded to [-8, 7]. Cache the ten-powers so
// the per-packet scale path skips std::pow; the table is filled by the same
// std::pow call the code used previously, so results stay bit-identical.
double powerOfTen(std::int8_t exponent) {
    static const auto table = [] {
        std::array<double, 16> t{};
        for (int i = 0; i < 16; ++i) t[static_cast<std::size_t>(i)] = std::pow(10.0, i - 8);
        return t;
    }();
    return exponent >= -8 && exponent <= 7 ? table[static_cast<std::size_t>(exponent + 8)]
                                           : std::pow(10.0, exponent);
}
} // namespace

double descriptorScale(std::int64_t raw, std::int32_t lmin, std::int32_t lmax,
                       std::int32_t pmin, std::int32_t pmax, std::int8_t exponent) {
    if (lmax == lmin || (pmax == 0 && pmin == 0)) return static_cast<double>(raw);
    const auto fraction = (static_cast<double>(raw) - lmin) / (static_cast<double>(lmax) - lmin);
    return (pmin + fraction * (static_cast<double>(pmax) - pmin)) * powerOfTen(exponent);
}

std::int8_t decodeHidUnitExponent(std::uint32_t exponent) {
    auto nibble = static_cast<std::int8_t>(exponent & 0x0f);
    return nibble >= 8 ? static_cast<std::int8_t>(nibble - 16) : nibble;
}

void decodePackedDescriptorValuesInto(std::vector<double>& result, std::span<const std::uint8_t> packed, const DescriptorField& field) {
    result.clear();
    if (!field.bitSize || field.bitSize > 63) return;
    result.reserve(field.reportCount);
    const auto mask=(std::uint64_t{1} << field.bitSize)-1;
    for (unsigned valueIndex=0; valueIndex<field.reportCount; ++valueIndex) {
        const auto offset=static_cast<std::size_t>(valueIndex)*field.bitSize;
        const auto shift=static_cast<unsigned>(offset%8);
        const auto byteIndex=offset/8;
        // Little-endian word assembly instead of a per-bit loop: gather the up
        // to nine bytes covering [offset, offset+bitSize), shift, and mask.
        // Bytes beyond the buffer read as zero, exactly as the bit loop did.
        std::uint64_t chunk{};
        if (byteIndex+8 <= packed.size()) std::memcpy(&chunk, packed.data()+byteIndex, 8);   // Windows x64/ARM64 are little-endian
        else if (byteIndex < packed.size()) std::memcpy(&chunk, packed.data()+byteIndex, packed.size()-byteIndex);
        auto raw = (chunk >> shift);
        if (shift && byteIndex+8 < packed.size()) raw |= std::uint64_t{packed[byteIndex+8]} << (64-shift);
        raw &= mask;
        std::int64_t value=static_cast<std::int64_t>(raw);
        if (field.logicalMin < 0) {
            const auto sign=std::uint64_t{1} << (field.bitSize-1);
            value=static_cast<std::int64_t>((raw^sign)-sign);
        }
        result.push_back(descriptorScale(value,field.logicalMin,field.logicalMax,field.physicalMin,field.physicalMax,field.unitExponent));
    }
}

std::vector<double> decodePackedDescriptorValues(std::span<const std::uint8_t> packed, const DescriptorField& field) {
    std::vector<double> result;
    decodePackedDescriptorValuesInto(result, packed, field);
    return result;
}

std::int64_t hidSigned(std::uint32_t value, unsigned bytes) {
    if (bytes==0) return 0;
    const auto bits=bytes*8u;
    if (bits>=32) return static_cast<std::int32_t>(value);
    const auto sign=std::uint32_t{1}<<(bits-1);
    const auto mask=(std::uint32_t{1}<<bits)-1;
    value&=mask;
    return static_cast<std::int32_t>((value^sign)-sign);
}

} // namespace sony
