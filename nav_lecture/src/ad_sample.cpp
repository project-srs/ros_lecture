#include <cppad/cppad.hpp>
#include <iostream>

int main()
{
  std::vector<CppAD::AD<double>> ax(2);
  CppAD::Independent(ax);
  std::vector<CppAD::AD<double>> ay(1);
  ay[0] = 2 * CppAD::pow(ax[0], 3) + CppAD::pow(ax[1], 2);
  CppAD::ADFun<double> f(ax, ay);

  std::vector<double> x(2);
  x[0] = 2;
  x[1] = 3;
  std::vector<double> fw0 = f.Forward(0, x);
  std::cout << fw0[0] << "=f(" << x[0] << "," << x[1] << ")" << std::endl;

  std::vector<double> jac = f.Jacobian(x);
  std::cout << jac[0] << "=dy/dx0(" << x[0] << "," << x[1] << ")" << std::endl;
  std::cout << jac[1] << "=dy/dx1(" << x[0] << "," << x[1] << ")" << std::endl;
}
