#ifndef GOC_LINEAR_PROGRAMMING_OR_OR_FORMULATION_H
#define GOC_LINEAR_PROGRAMMING_OR_OR_FORMULATION_H
#define USE_GLOP

#include <vector>
#include <string>
#include <unordered_map>
#include <memory>

#include "goc/linear_programming/model/constraint.h"
#include "goc/linear_programming/model/expression.h"
#include "goc/linear_programming/model/formulation.h"
#include "ortools/linear_solver/linear_solver.h"
#include "ortools/linear_solver/linear_solver.pb.h"
#include "ortools/base/logging.h"

using namespace operations_research;

namespace goc
{
//namespace operations_research
//{
class SeparationRoutine;

class ORFormulation : public Formulation
{
public:
	ORFormulation();

	virtual ~ORFormulation();

	// Adds the constraint to the formulation.
	// Observation: Constraints are normalized before added to the formulation, this is, the coefficients of each
	// variables are added up on the left side, and the constants are added up on the right side.
	// Returns: the index of the constraint added.
	virtual int AddConstraint(const Constraint& constraint);

	// Removes the constraint with index 'constraint_index' from the formulation.
	// If such constraint does not exists, it does nothing.
	virtual void RemoveConstraint(int constraint_index);

	// Adds a lazy constraint to the model. When a solution is found by the solver, it tries to add some lazy
	// constraint that separates that solution from the actual solution space.
	// Observation: if lazy_constraint == nullptr, the call to the function is ignored.
	virtual void AddLazyConstraint(SeparationRoutine* lazy_constraint);

	// Removes a lazy constraint from the model.
	// Observation: if lazy_constraint == nullptr, the call to the function is ignored.
	virtual void RemoveLazyConstraint(SeparationRoutine* lazy_constraint);

	// Adds a variable to the model.
	// Returns: a Variable with its correspondent name and index in the formulation.
	virtual Variable AddVariable(const std::string& name, VariableDomain domain, double lower_bound, double upper_bound);

	// Removes the variable with the same index than the one given as a parameter.
	// Observation: The name is ignored.
	virtual void RemoveVariable(const Variable& variable);

	// Sets the domain of the variable in the model. Domain might be Real, Integer, or Binary.
	virtual void SetVariableDomain(const Variable& variable, VariableDomain domain);

	// Sets the lower and upper bounds of the variable in the formulation.
	// Observation: use -INFTY or INFTY constants to specify no bounds.
	virtual void SetVariableBound(const Variable& v, double lower_bound, double upper_bound);

	// Sets the lower bound of the variable in the formulation.
	// Observation: use -INFTY or INFTY constants to specify no bounds.
	virtual void SetVariableLowerBound(const Variable& v, double lower_bound);

	// Sets the upper bound of the variable in the formulation.
	// Observation: use -INFTY or INFTY constants to specify no bounds.
	virtual void SetVariableUpperBound(const Variable& v, double upper_bound);

	// Sets the objective function as a minimization function.
	virtual void Minimize(const Expression& objective_function);

	// Sets the objective function as a maximization function.
	virtual void Maximize(const Expression& objective_function);

	// Gets the constraint right hand constant.
	virtual void SetConstraintRightHandSide(int constraint_index, double value);

	// Sets the constraint coefficient of a certain variable.
	virtual void SetConstraintCoefficient(int constraint_index, const Variable& variable, double coefficient);

	// Sets the coefficient of a variable in the objective function.
	virtual void SetObjectiveCoefficient(const Variable& variable, double coefficient);

	// Returns: the objective function sense.
	virtual ObjectiveSense GetObjectiveSense() const;

	// Returns: the coefficient of a variable in the objective function.
	virtual double GetObjectiveCoefficient(const Variable& variable) const;

	// Returns: the right hand constant of constraint with index 'constraint_index'.
	virtual double GetConstraintRightHandSide(int constraint_index) const;

	// Returns: the coefficient of a variable in the constraint with index 'constraint_index'.
	virtual double GetConstraintCoefficient(int constraint_index, const Variable& variable);

	// Returns: the domain of the variable with the specified index.
	virtual VariableDomain GetVariableDomain(const Variable& variable) const;

	// Returns: a pair (lower, upper) with the bounds of the variables.
	// Observation: -INFTY and INFTY specifies no bounds are contemplated.
	virtual std::pair<double, double> GetVariableBound(const Variable& variable) const;

	// Returns: the objective function expression.
	virtual Expression ObjectiveFunction() const;

	// Returns: a sequence with all the variables.
	virtual std::vector<Variable> Variables() const;

	// Returns: a sequence with all the constraints.
	virtual std::vector<Constraint> Constraints() const;

	// Returns: a sequence with all the lazy constraints.
	virtual const std::vector<SeparationRoutine*>& LazyConstraints() const;

	// Returns: the number of variables in the model.
	virtual int VariableCount() const;

	// Returns: the number of constraints in the model.
	virtual int ConstraintCount() const;

	// Returns: the variable at a specified index.
	virtual Variable VariableAtIndex(int variable_index) const;

	// Returns: the objective function evaluated at valuation 'v'.
	virtual double EvaluateValuation(const Valuation& v) const;

	// Returns: if the constraints of the model are satisfied by 'v'.
	//	- verbose: if true, then the first violated constraint is printed in clog.
	virtual bool IsFeasibleValuation(const Valuation& v, bool verbose=false) const;

	// Returns: a copy in the heap of the current formulation.
	// Observation: memory should be managed by the receiver and the pointer should be freed.
	virtual Formulation* Copy() const;

	// Prints the formulation.
	virtual void Print(std::ostream& os) const;

	void solve();

private:
	//GLOP solver
	MPSolver* solver;
	//objective function
	MPObjective* objective_;
	//map from index to MPvariable
	std::unordered_map<int, MPVariable*> variable_data_;
	//map from index to MPconstraint
	std::unordered_map<int, MPConstraint*> constraint_data_;
	//map from constraint index to it's sense
	std::unordered_map<int, enum Constraint::Sense> constraint_sense_;
	//objective function sense
	enum ObjectiveSense objectiveSense_;

};
//} // namespace operations_research
} // namespace goc

#endif //GOC_LINEAR_PROGRAMMING_OR_OR_FORMULATION_H
