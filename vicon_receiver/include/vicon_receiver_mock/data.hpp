#pragma once
#include <array>
#include "publisher.hpp"

class DataImport {
private:
  std::array<std::array<std::array<float, 3>, 6>, 422> positions;
  std::array<std::array<std::array<float, 4>, 6>, 422> rot;

public:
  DataImport();

  void load();

  void fetch_data(std::size_t, PositionStruct&);
}
