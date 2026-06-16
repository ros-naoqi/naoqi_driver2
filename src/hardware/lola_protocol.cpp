/*
 * Copyright 2025 Aldebaran
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 *
 */

#include "naoqi_driver/hardware/lola_protocol.hpp"

#include <cstring>

// Minimal MessagePack codec for the subset LoLA uses: maps of string keys to
// arrays of float32 (and a few integer / string fields we tolerate but ignore).
// Hand-rolled to avoid a msgpack system dependency. Wire format follows the
// MessagePack spec (big-endian). Correctness against real LoLA is to be
// verified on hardware (see ASSUMPTIONS.md); the fake LoLA server round-trips
// the same codec in CI.

namespace naoqi
{
namespace hardware
{
namespace lola
{

const std::vector<std::string>& jointOrder()
{
  static const std::vector<std::string> kJoints = {
      "HeadYaw",    "HeadPitch",   "LShoulderPitch", "LShoulderRoll",  "LElbowYaw",
      "LElbowRoll", "LWristYaw",   "LHipYawPitch",   "LHipRoll",       "LHipPitch",
      "LKneePitch", "LAnklePitch", "LAnkleRoll",     "RHipRoll",       "RHipPitch",
      "RKneePitch", "RAnklePitch", "RAnkleRoll",     "RShoulderPitch", "RShoulderRoll",
      "RElbowYaw",  "RElbowRoll",  "RWristYaw",      "LHand",          "RHand"};
  return kJoints;
}

namespace
{
void writeByte(std::vector<uint8_t>& b, uint8_t v)
{
  b.push_back(v);
}

void writeStr(std::vector<uint8_t>& b, const std::string& s)
{
  if (s.size() < 32)
  {
    writeByte(b, static_cast<uint8_t>(0xa0 | s.size()));
  }
  else
  {
    writeByte(b, 0xd9);
    writeByte(b, static_cast<uint8_t>(s.size()));
  }
  b.insert(b.end(), s.begin(), s.end());
}

void writeArrayHeader(std::vector<uint8_t>& b, size_t n)
{
  if (n < 16)
  {
    writeByte(b, static_cast<uint8_t>(0x90 | n));
  }
  else
  {
    writeByte(b, 0xdc);
    writeByte(b, static_cast<uint8_t>((n >> 8) & 0xff));
    writeByte(b, static_cast<uint8_t>(n & 0xff));
  }
}

void writeFloat32(std::vector<uint8_t>& b, float f)
{
  uint32_t u;
  std::memcpy(&u, &f, 4);
  writeByte(b, 0xca);
  writeByte(b, static_cast<uint8_t>((u >> 24) & 0xff));
  writeByte(b, static_cast<uint8_t>((u >> 16) & 0xff));
  writeByte(b, static_cast<uint8_t>((u >> 8) & 0xff));
  writeByte(b, static_cast<uint8_t>(u & 0xff));
}

void writeFloatArray(std::vector<uint8_t>& b,
                     const std::string& key,
                     const std::vector<float>& values)
{
  writeStr(b, key);
  writeArrayHeader(b, values.size());
  for (float v : values)
    writeFloat32(b, v);
}

/// Cursor over a byte buffer with bounds-checked reads.
struct Reader
{
  const uint8_t* p;
  const uint8_t* end;

  bool byte(uint8_t& out)
  {
    if (p >= end)
      return false;
    out = *p++;
    return true;
  }
  bool bytes(uint64_t& out, int n)
  {
    out = 0;
    for (int i = 0; i < n; ++i)
    {
      uint8_t b;
      if (!byte(b))
        return false;
      out = (out << 8) | b;
    }
    return true;
  }
  bool skip(size_t n)
  {
    if (static_cast<size_t>(end - p) < n)
      return false;
    p += n;
    return true;
  }
};

bool readContainerCount(Reader& r,
                        uint8_t first,
                        uint8_t fix_mask,
                        uint8_t fix_prefix,
                        uint8_t tag16,
                        uint8_t tag32,
                        size_t& count)
{
  if ((first & 0xf0) == fix_prefix && fix_mask == 0xf0)
  {
    count = first & 0x0f;
    return true;
  }
  uint64_t v;
  if (first == tag16)
  {
    if (!r.bytes(v, 2))
      return false;
    count = static_cast<size_t>(v);
    return true;
  }
  if (first == tag32)
  {
    if (!r.bytes(v, 4))
      return false;
    count = static_cast<size_t>(v);
    return true;
  }
  return false;
}

// Reads one value, appending any numeric leaves to out. Strings/nil/bool are
// consumed without contributing. Returns false on malformed input.
bool readValueFloats(Reader& r, std::vector<float>& out)
{
  uint8_t b;
  if (!r.byte(b))
    return false;

  if (b <= 0x7f)  // positive fixint
  {
    out.push_back(static_cast<float>(b));
    return true;
  }
  if (b >= 0xe0)  // negative fixint
  {
    out.push_back(static_cast<float>(static_cast<int8_t>(b)));
    return true;
  }
  if ((b & 0xe0) == 0xa0)  // fixstr
    return r.skip(b & 0x1f);
  if ((b & 0xf0) == 0x90)  // fixarray
  {
    const size_t n = b & 0x0f;
    for (size_t i = 0; i < n; ++i)
      if (!readValueFloats(r, out))
        return false;
    return true;
  }
  if ((b & 0xf0) == 0x80)  // fixmap
  {
    const size_t n = b & 0x0f;
    for (size_t i = 0; i < 2 * n; ++i)
      if (!readValueFloats(r, out))
        return false;
    return true;
  }

  uint64_t v;
  switch (b)
  {
    case 0xc0:  // nil
    case 0xc2:  // false
    case 0xc3:  // true
      return true;
    case 0xcc:
      return r.bytes(v, 1) && (out.push_back(static_cast<float>(v)), true);
    case 0xcd:
      return r.bytes(v, 2) && (out.push_back(static_cast<float>(v)), true);
    case 0xce:
      return r.bytes(v, 4) && (out.push_back(static_cast<float>(v)), true);
    case 0xcf:
      return r.bytes(v, 8) && (out.push_back(static_cast<float>(v)), true);
    case 0xd0:
      return r.bytes(v, 1) && (out.push_back(static_cast<float>(static_cast<int8_t>(v))), true);
    case 0xd1:
      return r.bytes(v, 2) && (out.push_back(static_cast<float>(static_cast<int16_t>(v))), true);
    case 0xd2:
      return r.bytes(v, 4) && (out.push_back(static_cast<float>(static_cast<int32_t>(v))), true);
    case 0xd3:
      return r.bytes(v, 8) && (out.push_back(static_cast<float>(static_cast<int64_t>(v))), true);
    case 0xca:  // float32
    {
      if (!r.bytes(v, 4))
        return false;
      const uint32_t u = static_cast<uint32_t>(v);
      float f;
      std::memcpy(&f, &u, 4);
      out.push_back(f);
      return true;
    }
    case 0xcb:  // float64
    {
      if (!r.bytes(v, 8))
        return false;
      double d;
      std::memcpy(&d, &v, 8);
      out.push_back(static_cast<float>(d));
      return true;
    }
    case 0xd9:  // str8
      return r.bytes(v, 1) && r.skip(static_cast<size_t>(v));
    case 0xda:  // str16
      return r.bytes(v, 2) && r.skip(static_cast<size_t>(v));
    case 0xdb:  // str32
      return r.bytes(v, 4) && r.skip(static_cast<size_t>(v));
    case 0xdc:  // array16
    case 0xdd:  // array32
    {
      const int nbytes = (b == 0xdc) ? 2 : 4;
      if (!r.bytes(v, nbytes))
        return false;
      for (uint64_t i = 0; i < v; ++i)
        if (!readValueFloats(r, out))
          return false;
      return true;
    }
    default:
      return false;
  }
}

bool readString(Reader& r, std::string& out)
{
  uint8_t b;
  if (!r.byte(b))
    return false;
  size_t len;
  if ((b & 0xe0) == 0xa0)
  {
    len = b & 0x1f;
  }
  else
  {
    uint64_t v;
    if (b == 0xd9 && r.bytes(v, 1))
      len = static_cast<size_t>(v);
    else if (b == 0xda && r.bytes(v, 2))
      len = static_cast<size_t>(v);
    else
      return false;
  }
  if (static_cast<size_t>(r.end - r.p) < len)
    return false;
  out.assign(reinterpret_cast<const char*>(r.p), len);
  r.p += len;
  return true;
}
}  // namespace

std::vector<uint8_t> encodeActuators(const std::vector<float>& position,
                                     const std::vector<float>& stiffness)
{
  std::vector<uint8_t> b;
  writeByte(b, static_cast<uint8_t>(0x80 | 2));  // fixmap, 2 entries
  writeFloatArray(b, "Position", position);
  writeFloatArray(b, "Stiffness", stiffness);
  return b;
}

bool decodeSensors(const uint8_t* data, size_t size, std::map<std::string, std::vector<float>>& out)
{
  Reader r{data, data + size};
  uint8_t first;
  if (!r.byte(first))
    return false;
  size_t count;
  if (!readContainerCount(r, first, 0xf0, 0x80, 0xde, 0xdf, count))
    return false;

  for (size_t i = 0; i < count; ++i)
  {
    std::string key;
    if (!readString(r, key))
      return false;
    std::vector<float> values;
    if (!readValueFloats(r, values))
      return false;
    out[key] = std::move(values);
  }
  return true;
}

}  // namespace lola
}  // namespace hardware
}  // namespace naoqi
