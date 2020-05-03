//
// Created by Gonzalo Lera Romero.
// Grupo de Optimizacion Combinatoria (GOC).
// Departamento de Computacion - Universidad de Buenos Aires.
//

#include "goc/linear_programming/solver/lp_solver.h"

#include "goc/linear_programming/cplex/cplex_formulation.h"
#include "goc/linear_programming/OR/OR_formulation.h"
#include "goc/linear_programming/cplex/cplex_solver.h"
#include "goc/linear_programming/OR/OR_solver.h"

using namespace std;
using namespace nlohmann;

namespace goc
{
LPSolver::LPSolver()
{
	// Set default values.
	time_limit = Duration::Max();
	config = {};
	screen_output = nullptr;
}

LPExecutionLog LPSolver::Solve(Formulation* formulation, const unordered_set<LPOption>& options) const
{
	return cplex::solve_lp((CplexFormulation*)formulation, screen_output, time_limit, config, options);
}

void LPSolver::ORSolve(Formulation* formulation, const unordered_set<LPOption>& options) const
{
	((ORFormulation*)formulation)->solve();
}

Formulation* LPSolver::NewFormulation()
{
	return new CplexFormulation();
}

Formulation* LPSolver::NewOrFormulation()
{
	return new ORFormulation();
}

void LPSolver::Print(Formulation* formulation)
{
	formulation->Print(std::cout);
}
} // namespace goc
