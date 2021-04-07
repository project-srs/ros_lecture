#include <ifopt/problem.h>
#include <ifopt/ipopt_solver.h>
#include <ifopt/constraint_set.h>
#include <ifopt/variable_set.h>
#include <ifopt/cost_term.h>
#include <cppad/cppad.hpp>
#include <cmath>
#include <iostream>

// Define Variables
class TestVariables : public ifopt::VariableSet
{
public:
  TestVariables() : VariableSet(2, "var_set1")
  {
    x0_ = 0.0;
    x1_ = 0.0;
  }

  void SetVariables(const VectorXd& x) override
  {
    x0_ = x(0);
    x1_ = x(1);
  };

  VectorXd GetValues() const override
  {
    return Eigen::Vector2d(x0_, x1_);
  };

  VecBound GetBounds() const override
  {
    VecBound bounds(GetRows());
    bounds.at(0) = ifopt::Bounds(1.0, 2.0);
    bounds.at(1) = ifopt::NoBound;
    return bounds;
  }

private:
  double x0_, x1_;
};

// Define Constraint
class TestConstraint : public ifopt::ConstraintSet
{
public:
  TestConstraint() : ConstraintSet(1, "constraint1")
  {
  }
  VectorXd GetValues() const override
  {
    Eigen::Vector2d x = GetVariables()->GetComponent("var_set1")->GetValues();
    std::vector<double> tx(2);
    tx[0] = x(0);
    tx[1] = x(1);
    std::vector<double> fw = getAdFun().Forward(0, tx);
    VectorXd g(GetRows());
    g(0) = fw[0];
    return g;
  };
  VecBound GetBounds() const override
  {
    VecBound b(GetRows());
    b.at(0) = ifopt::Bounds(2.0, 4.0); // bound of g(0)
    return b;
  }
  void FillJacobianBlock(std::string var_set, Jacobian& jac_block) const override
  {
    if (var_set == "var_set1")
    {
      Eigen::Vector2d x = GetVariables()->GetComponent("var_set1")->GetValues();
      std::vector<double> tx(2);
      tx[0] = x(0);
      tx[1] = x(1);
      std::vector<double> ad_jac = getAdFun().Jacobian(tx);
      jac_block.coeffRef(0, 0) = ad_jac[0]; // d(c)/d(x(0))
      jac_block.coeffRef(0, 1) = ad_jac[1]; // d(c)/d(x(1))
    }
  }
private:
  CppAD::ADFun<double> getAdFun(void) const {
    std::vector<CppAD::AD<double>> ax(2);
    CppAD::Independent(ax);
    std::vector<CppAD::AD<double>> ay(1);
    ay[0] = ax[0] + ax[1];
    CppAD::ADFun<double> f(ax, ay);
    return f;
  }
};

// Define Cost
class TestCost : public ifopt::CostTerm
{
public:
  TestCost() : CostTerm("cost_term1")
  {
  }

  double GetCost() const override
  {
    Eigen::Vector2d x = GetVariables()->GetComponent("var_set1")->GetValues();
    std::vector<double> tx(2);
    tx[0] = x(0);
    tx[1] = x(1);
    std::vector<double> fw0;
    fw0 = getAdFun().Forward(0, tx);
    return fw0[0];
  };

  void FillJacobianBlock(std::string var_set, Jacobian& jac) const override
  {
    if (var_set == "var_set1")
    {
      Eigen::Vector2d x = GetVariables()->GetComponent("var_set1")->GetValues();
      std::vector<double> tx(2);
      tx[0] = x(0);
      tx[1] = x(1);
      std::vector<double> ad_jac = getAdFun().Jacobian(tx);
      jac.coeffRef(0, 0) = ad_jac[0]; // d(c)/d(x(0))
      jac.coeffRef(0, 1) = ad_jac[1]; // d(c)/d(x(1))
    }
  }

private:
  CppAD::ADFun<double> getAdFun(void) const {
    std::vector<CppAD::AD<double>> ax(2);
    CppAD::Independent(ax);
    std::vector<CppAD::AD<double>> ay(1);
    ay[0] = CppAD::pow(ax[0], 2) + CppAD::pow(ax[1], 2);
    CppAD::ADFun<double> f(ax, ay);
    return f;
  }
};

int main()
{
  // Define problem
  ifopt::Problem nlp;
  nlp.AddVariableSet(std::make_shared<TestVariables>());
  nlp.AddConstraintSet(std::make_shared<TestConstraint>());
  nlp.AddCostSet(std::make_shared<TestCost>());

  // Initialize solver
  ifopt::IpoptSolver ipopt;
  ipopt.SetOption("linear_solver", "mumps");
  ipopt.SetOption("jacobian_approximation", "exact");
  ipopt.SetOption("print_level", 0); // supress log
  ipopt.SetOption("sb", "yes"); // supress startup message

  ipopt.Solve(nlp);
  std::cout << nlp.GetOptVariables()->GetValues().transpose() << std::endl;
}
