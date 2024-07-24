#include "tests_runner.h"

#include <fstream>
#include <iostream>

int main(int argc, char* argv[]) {
  if (argc < 2) {
    std::cerr << "Usage: " << argv[0] << " <path to test file>" << std::endl;
    return 1;
  }

  std::string filePath = argv[1];

  auto pairsToCheck = GetPairsToCheck(filePath);
  RunTaskCases(pairsToCheck);

  return 0;
}
