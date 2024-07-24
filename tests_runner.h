#pragma once

#include "geometry_utils.h"
#include "object3D.h"

#include <fstream>
#include <iostream>
#include <memory>
#include <sstream>
#include <string>
#include <vector>

struct TTaskFormatCase {
  std::pair<std::shared_ptr<IObject>, std::shared_ptr<IObject>> objects;
  bool expectedIntersection;
  int line;
};

std::shared_ptr<IObject> BuildObject(const TPoint3D& s,
                                     const TPoint3D& q,
                                     const TPoint3D& p);

void RunTaskCases(const std::vector<TTaskFormatCase>& cases);
std::shared_ptr<IObject> ReadIObject(std::string& line);
std::vector<TTaskFormatCase> GetPairsToCheck(const std::string& pathToFile);
