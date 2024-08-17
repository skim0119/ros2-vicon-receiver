#pragma once
#include "publisher.hpp"
#include <array>

class DataImport {
private:
  std::array<std::array<std::array<float, 3>, 6>, 422> positions;
  std::array<std::array<std::array<float, 4>, 6>, 422> rot;

public:
  DataImport();

  void load();

  void fetch_data(unsigned int, PositionStruct &);
};
