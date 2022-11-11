#pragma once
#include <fstream>
#include <cstdint>  // int8_t, int16_t, ...
// #include <libserialport.h>  // libserial-dev
#include <SerialStream.h>  // libserial-dev

template <class Integer, std::enable_if_t<std::is_integral<Integer>::value, bool> = true>
void write(std::fstream& file, Integer value)
{
  const std::size_t bytes{ sizeof(Integer) };

  std::size_t offset{ 0 };
  int8_t buffer[bytes];
  for (int i = 0; i < bytes; ++i)
  {
    buffer[i] = (int8_t)(value >> (i << offset) & 0xff);
    offset += 8;
  }
  file.write((char*)buffer, bytes * sizeof(int8_t));
}

template <class Integer, std::enable_if_t<std::is_integral<Integer>::value, bool> = true>
void read(SerialPort& serial_port)
{
  const std::size_t bytes{ sizeof(Integer) };

  SerialPort::DataBuffer buffer;
  serial_port.Read(buffer, 4);

  Integer value{ 0 };
  Integer mask{ 0xff };
  std::size_t offset{ 0 };
  for (int i = 0; i < bytes; ++i)
  {
    value |= (((int32_t)buffer[i]) << (i << offset) & mask);
    offset += 8;
    mask = mask << offset;
  }

  return value;
}

template <class Integer, std::enable_if_t<std::is_integral<Integer>, bool> = true>
void read(fstream& file)
{
  const std::size_t bytes{ sizeof(Integer) };
  char buffer[bytes];

  file.read(buffer, bytes);

  Integer value{ 0 };
  Integer mask{ 0xff };
  std::size_t offset{ 0 };
  for (int i = 0; i < bytes; ++i)
  {
    value |= (((Integer)buffer[i]) << (i << offset) & mask);
    offset += 8;
    mask = mask << offset;
  }

  return value;
}
