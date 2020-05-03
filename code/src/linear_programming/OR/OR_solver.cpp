#include <mutex>

#include "goc/collection/collection_utils.h"
#include "goc/string/string_utils.h"
#include "goc/exception/exception_utils.h"
#include "goc/math/number_utils.h"
#include "goc/time/stopwatch.h"
#include "goc/linear_programming/cuts/separation_algorithm.h"
#include "goc/linear_programming/OR/OR_solver.h
#include "goc/linear_programming/OR/OR_formulation.h"

using namespace std;
using namespace nlohmann;

namespace goc
{
void solveOR(goc::ORFormulation* formulation){
  formulation.solve();
}
void Print(){
  formulation.Print();
}
}
