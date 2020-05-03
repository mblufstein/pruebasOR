#ifndef GOC_LINEAR_PROGRAMMING_OR_OR_SOLVER_H
#define GOC_LINEAR_PROGRAMMING_OR_OR_SOLVER_H

#include <iostream>
#include <string>
#include <unordered_set>

#include "goc/linear_programming/OR/OR_formulation.h"
#include "goc/linear_programming/cuts/separation_strategy.h"
#include "goc/linear_programming/model/branch_priority.h"
#include "goc/linear_programming/solver/lp_solver.h"
#include "goc/linear_programming/solver/bc_solver.h"
#include "goc/log/bc_execution_log.h"
#include "goc/log/lp_execution_log.h"

namespace goc
{
void solveOR(ORFormulation* formulation);
void Print();
}

#endif //GOC_LINEAR_PROGRAMMING_OR_OR_SOLVER_H
