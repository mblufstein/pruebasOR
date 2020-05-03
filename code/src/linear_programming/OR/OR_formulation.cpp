#include "goc/linear_programming/OR/OR_formulation.h"

#include <map>

#include "goc/collection/collection_utils.h"
#include "goc/exception/exception_utils.h"
#include "goc/math/number_utils.h"

using namespace std;
using namespace operations_research;

namespace goc
{
//namespace operations_research
//{
ORFormulation::ORFormulation()
{
	solver = new MPSolver("LinearExample", MPSolver::GLOP_LINEAR_PROGRAMMING);
	objective_ = solver->MutableObjective();
}

ORFormulation::~ORFormulation()
{

}

int ORFormulation::AddConstraint(const Constraint& constraint)
{
	const double infinity = solver->infinity();
	MPConstraint* const c0 = solver->MakeRowConstraint(-infinity, infinity);

	if(constraint.Sense() == Constraint::LessEqual)
		c0->SetUB(constraint.RightSide());
	else if(constraint.Sense() == Constraint::Equality)
		c0->SetBounds(constraint.RightSide(), constraint.RightSide());
	else
		c0->SetLB(constraint.RightSide());

	for (auto& term: constraint.LeftSide().Terms()){
		c0->SetCoefficient(variable_data_[term.first.Index()], term.second);
	}

	constraint_data_[c0->index()] = c0;
	constraint_sense_[c0->index()] = constraint.Sense();

	return c0->index();
}

void ORFormulation::RemoveConstraint(int constraint_index)
{
	const double infinity = solver->infinity();
	constraint_data_[constraint_index]->SetBounds(-infinity, infinity);
}

void ORFormulation::AddLazyConstraint(SeparationRoutine* lazy_constraint)
{
	// ?? preguntar si el de cplex hace lo mismo que el de OR
}

void ORFormulation::RemoveLazyConstraint(SeparationRoutine* lazy_constraint)
{
	// poner todo en 0 ?
}

Variable ORFormulation::AddVariable(const string& name, VariableDomain domain, double lower_bound, double upper_bound)
{
	// Add variable to OR.
	MPVariable* const x = solver->MakeNumVar(lower_bound, upper_bound, name);

	Variable var(name, new int(x->index()));

	variable_data_[var.Index()] = x;

	return var;
}

void ORFormulation::RemoveVariable(const Variable& variable)
{
	// PENSAR
}

void ORFormulation::SetVariableDomain(const Variable& variable, VariableDomain domain)
{
	//PENSAR
}

void ORFormulation::SetVariableBound(const Variable& v, double lower_bound, double upper_bound)
{
	const double infinity = solver->infinity();
	if (upper_bound == INFTY) upper_bound = infinity;
	if (lower_bound == -INFTY) lower_bound = -infinity;
	variable_data_[v.Index()]->SetBounds(lower_bound, upper_bound);
}

void ORFormulation::SetVariableLowerBound(const Variable& v, double lower_bound)
{
	const double infinity = solver->infinity();
	if (lower_bound == -INFTY) lower_bound = -infinity;
	variable_data_[v.Index()]->SetLB(lower_bound);
}

void ORFormulation::SetVariableUpperBound(const Variable& v, double upper_bound)
{
	const double infinity = solver->infinity();
	if (upper_bound == INFTY) upper_bound = infinity;
	variable_data_[v.Index()]->SetUB(upper_bound);
}

void ORFormulation::Minimize(const Expression& objective_function)
{
	for (auto& term: objective_function.Terms())
		objective_->SetCoefficient(variable_data_[term.first.Index()], term.second);
  objective_->SetMinimization();
	objectiveSense_ = Minimization;
}

void ORFormulation::Maximize(const Expression& objective_function)
{
	for (auto& term: objective_function.Terms())
		objective_->SetCoefficient(variable_data_[term.first.Index()], term.second);
	objective_->SetMaximization();
	objectiveSense_ = Maximization;
}

void ORFormulation::SetConstraintRightHandSide(int constraint_index, double value)
{
	if(constraint_sense_[constraint_index] == Constraint::LessEqual)
		constraint_data_[constraint_index]->SetUB(value);
	else if(constraint_sense_[constraint_index] == Constraint::Equality)
		constraint_data_[constraint_index]->SetBounds(value, value);
	else
		constraint_data_[constraint_index]->SetLB(value);
}

void ORFormulation::SetConstraintCoefficient(int constraint_index, const Variable& variable, double coefficient)
{
	int id = variable.Index();
	constraint_data_[constraint_index]->SetCoefficient(variable_data_[id], coefficient);
}

void ORFormulation::SetObjectiveCoefficient(const Variable& variable, double coefficient)
{
	int id = variable.Index();
	objective_->SetCoefficient(variable_data_[id], coefficient);
}

Formulation::ObjectiveSense ORFormulation::GetObjectiveSense() const
{
	return objectiveSense_;
}

double ORFormulation::GetObjectiveCoefficient(const Variable& variable) const
{
	int id = variable.Index();
	return objective_->GetCoefficient(variable_data_.at(id));
}

double ORFormulation::GetConstraintRightHandSide(int constraint_index) const
{
	if(constraint_sense_.at(constraint_index) == Constraint::LessEqual)
		return constraint_data_.at(constraint_index)->ub();
	else if(constraint_sense_.at(constraint_index) == Constraint::Equality)
		return constraint_data_.at(constraint_index)->ub();
	else
		return constraint_data_.at(constraint_index)->lb();
}

double ORFormulation::GetConstraintCoefficient(int constraint_index, const Variable& variable)
{
	int id = variable.Index();
	return constraint_data_[constraint_index]->GetCoefficient(variable_data_[id]);
}

VariableDomain ORFormulation::GetVariableDomain(const Variable& variable) const
{
	//PENSAR
}

pair<double, double> ORFormulation::GetVariableBound(const Variable& variable) const
{
	double lb, ub;
	const double infinity = solver->infinity();
	int id = variable.Index();
	lb = variable_data_.at(id)->lb();
	ub = variable_data_.at(id)->ub();
	if (lb == -infinity) lb = -INFTY;
	if (ub == infinity) ub = INFTY;
	return {lb, ub};
}

Expression ORFormulation::ObjectiveFunction() const
{
	auto coefs = objective_->terms();
	Expression obj;
	for(auto it = coefs.begin(); it != coefs.end(); it++){
		obj += (*it).second * Variable((*it).first->name(), new int((*it).first->index()));
	}
	return obj;
}

vector<Variable> ORFormulation::Variables() const
{
	//variables() devuelve arreglo variables mp
	vector<MPVariable*> vars = solver->variables();
	vector<Variable> variables;
	for (int i = 0; i < vars.size(); ++i) variables.push_back(Variable(vars[i]->name(), new int(vars[i]->index())));
	return variables;
}

vector<Constraint> ORFormulation::Constraints() const
{
	//Pensar sense
}

const vector<SeparationRoutine*>& ORFormulation::LazyConstraints() const
{
	//pensar
}

int ORFormulation::VariableCount() const
{
	return solver->NumVariables();
}

int ORFormulation::ConstraintCount() const
{
	return solver->NumConstraints();
}

Variable ORFormulation::VariableAtIndex(int variable_index) const
{
	return Variable(variable_data_.at(variable_index)->name(), new int(variable_data_.at(variable_index)->index()));
}

double ORFormulation::EvaluateValuation(const Valuation& valuation) const
{
	//pensar
}

bool ORFormulation::IsFeasibleValuation(const Valuation& v, bool verbose) const
{
	// pensar
}

Formulation* ORFormulation::Copy() const
{
	//pensar
	//CplexFormulation* copy = new CplexFormulation(env_memory_handler_, cplex::cloneprob(env_, problem_));
	//copy->variable_names_ = variable_names_;
	//for (int i = 0; i < VariableCount(); ++i) copy->variable_indices_.push_back(new int(i));
	//for (int i = 0; i < ConstraintCount(); ++i) copy->constraint_indices_.push_back(new int(i));
	//copy->lazy_constraints_ = lazy_constraints_;
	//return copy;
}

void ORFormulation::Print(ostream& os) const
{
	os << "Solution:" << '\n';
  os << "Objective value = " << objective_->Value() << '\n';
	vector<Variable> vars = Variables();
  for(int i = 0; i < vars.size(); i++){
		string name = vars[i].name;
		os << name<< ": " << solver->LookupVariableOrNull(name)->solution_value() << '\n';
	}

/*
	vector<MPConstraint*> constr = solver->constraints();
	vector<MPVariable*> vars = solver->variables();
	for(int i = 0; i < vars.size(); i++){
		cout<<vars[i]->solution_value()<<' ';
	}
	cout<<'\n';
	for(int i = 0; i < constr.size(); i++){
		cout<<constr[i]->lb()<<' '<<constr[i]->ub()<<'\n';
		for(int j = 0; j < vars.size(); j++){
			cout<<constr[i]->GetCoefficient(vars[j])<<' ';
		}
		cout<<'\n';
	}
	const MPObjective& obj = solver->Objective();
	for(int j = 0; j < vars.size(); j++){
		cout<<obj.GetCoefficient(vars[j])<<' ';
	}
	cout<<'\n';
	*/
}

void ORFormulation::solve()
{
	solver->Solve();
}

//} // namespace operations_research
} // namespace goc
