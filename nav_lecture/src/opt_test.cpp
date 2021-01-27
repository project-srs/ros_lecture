#include "ros/ros.h"
#include "geometry_msgs/Pose.h"
#include "nav_msgs/Odometry.h"
#include "gazebo_msgs/ModelStates.h"
#include "tf/transform_broadcaster.h"

#include <ifopt/problem.h>
#include <ifopt/ipopt_solver.h>
#include <ifopt/constraint_set.h>
#include <ifopt/variable_set.h>
#include <ifopt/cost_term.h>

#include "math.h"
#include <string>
#include <random>

using Eigen::Vector2d;

class ExVariables : public ifopt::VariableSet {
public:
  ExVariables() : ExVariables("var_set1") {};
  ExVariables(const std::string& name) : VariableSet(2, name)
  {
    x0_ = 3.5;
    x1_ = 1.5;
  }

  void SetVariables(const VectorXd& x) override
  {
    x0_ = x(0);
    x1_ = x(1);
  };

  VectorXd GetValues() const override
  {
    return Vector2d(x0_, x1_);
  };
  VecBound GetBounds() const override
  {
    VecBound bounds(GetRows());
    bounds.at(0) = ifopt::Bounds(-5.0, 5.0);
    bounds.at(1) = ifopt::NoBound;
    return bounds;
  }

private:
  double x0_, x1_;
};


class ExConstraint : public ifopt::ConstraintSet {
public:
  ExConstraint() : ExConstraint("constraint1") {}
  ExConstraint(const std::string& name) : ConstraintSet(1, name) {}
  VectorXd GetValues() const override
  {
    VectorXd g(GetRows());
    Vector2d x = GetVariables()->GetComponent("var_set1")->GetValues();
    // g(0) = std::pow(x(0),2) + x(1);
    g(0) = std::pow(x(0),2) + std::pow(x(1),2);
    return g;
  };
  VecBound GetBounds() const override
  {
    VecBound b(GetRows());
    b.at(0) = ifopt::Bounds(1.0, 1.0);
    return b;
  }
  void FillJacobianBlock (std::string var_set, Jacobian& jac_block) const override
  {
    if (var_set == "var_set1") {
      Vector2d x = GetVariables()->GetComponent("var_set1")->GetValues();

      jac_block.coeffRef(0, 0) = 2.0*x(0); // derivative of first constraint w.r.t x0
      jac_block.coeffRef(0, 1) = 2.0*x(1); // derivative of first constraint w.r.t x1
    }
  }
};


class ExCost: public ifopt::CostTerm {
public:
  ExCost() : ExCost("cost_term1") {}
  ExCost(const std::string& name) : CostTerm(name) {}

  double GetCost() const override
  {
    Vector2d x = GetVariables()->GetComponent("var_set1")->GetValues();
    // return -std::pow(x(1)-2,2);
    return x(0) + x(1);
  };

  void FillJacobianBlock (std::string var_set, Jacobian& jac) const override
  {
    if (var_set == "var_set1") {
      Vector2d x = GetVariables()->GetComponent("var_set1")->GetValues();

      jac.coeffRef(0, 0) = 1; // derivative of cost w.r.t x0
      jac.coeffRef(0, 1) = 1; // derivative of cost w.r.t x1
    }
  }
};

int main() {

  // Define the solver independent problem
  ifopt::Problem nlp;
  nlp.AddVariableSet  (std::make_shared<ExVariables>());
  nlp.AddConstraintSet(std::make_shared<ExConstraint>());
  nlp.AddCostSet      (std::make_shared<ExCost>());

  // Initialize solver and options
  ifopt::IpoptSolver ipopt;
  ipopt.SetOption("linear_solver", "mumps");
  ipopt.SetOption("jacobian_approximation", "exact");

  // Solve
  ipopt.Solve(nlp);

  std::cout << nlp.GetOptVariables()->GetValues().transpose() << std::endl;
}
