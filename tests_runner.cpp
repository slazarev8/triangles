#include "tests_runner.h"
#include "point3D.h"
#include "segment3D.h"
#include "triangle3D.h"

std::shared_ptr<IObject> BuildObject(const TPoint3D& s,
                                     const TPoint3D& q,
                                     const TPoint3D& p) {
  if (s == q && s == p) {
    return std::make_shared<TPointObject3D>(s);
  } else if (CheckCollinear(s, q, p)) {
    const auto sq = Distance(s, q);
    const auto sp = Distance(s, p);
    const auto pq = Distance(p, q);

    TPoint3D start, end;
    if (sq >= sp && sq >= pq) {
      start = s;
      end = q;
    } else if (sp >= sq && sp >= pq) {
      start = s;
      end = p;
    } else {
      start = p;
      end = q;
    }

    auto segment = std::make_shared<TSegment3D>(start, end);

    return segment;
  } else {
    auto triangle = std::make_shared<TTriangle3D>(s, q, p);
    return triangle;
  }
}

void RunTaskCases(const std::vector<TTaskFormatCase>& cases) {
  int failed = 0;

  for (const auto& testCase : cases) {
    auto result = testCase.objects.first->Intersects(testCase.objects.second);
    if (result != testCase.expectedIntersection) {
      failed++;
      const std::string expectedResult =
          testCase.expectedIntersection ? "true" : "false";
      const std::string testResult = result ? "true" : "false";
      std::cout << "line " + std::to_string(testCase.line) + " expected " +
                       expectedResult + ", got " + testResult
                << std::endl;
    }
  }
  std::cout << "Tests done " + std::to_string(cases.size()) + "/" +
                   std::to_string(failed)
            << " failed" << std::endl;
}

std::shared_ptr<IObject> ReadIObject(std::string& line) {
  std::stringstream in(line);
  TPoint3D s, q, p;

  in >> s.x >> s.y >> s.z;
  in >> q.x >> q.y >> q.z;
  in >> p.x >> p.y >> p.z;

  return BuildObject(s, q, p);
}

std::vector<TTaskFormatCase> GetPairsToCheck(const std::string& pathToFile) {
  std::vector<TTaskFormatCase> result;

  std::ifstream in(pathToFile);
  if (!in.is_open()) {
    throw std::runtime_error("incorrect file!");
  }

  std::string line;
  std::vector<std::string> testCaseLines;
  int lineCnt = 0;
  while (std::getline(in, line)) {
    lineCnt++;
    if (line.find('#') != std::string::npos || line.empty()) {
      continue;
    }

    testCaseLines.emplace_back(line);
    if (testCaseLines.size() == 3) {
      TTaskFormatCase current;
      current.objects.first = ReadIObject(testCaseLines[0]);
      current.objects.second = ReadIObject(testCaseLines[1]);
      if (testCaseLines[2] == "true") {
        current.expectedIntersection = true;
      } else if (testCaseLines[2] == "false") {
        current.expectedIntersection = false;
      } else {
        throw std::runtime_error("incorrect format!");
      }

      current.line = lineCnt;
      result.push_back(current);
      testCaseLines.clear();
    }
  }
  if (!testCaseLines.empty()) {
    throw std::runtime_error("incorrect format!");
  }

  return result;
}
