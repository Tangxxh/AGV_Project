#include <iostream>
#include <cppad/ipopt/solve.hpp>
#include <cmath>
#include <chrono>

#include <model_equa_solver.hpp>

//#include <robot_bringup/joint.h>

using namespace std;
// #define X 0.4
// #define Y 0.0

namespace {
using CppAD::AD;
    class FG_eval {
    public:
        FG_eval(double target_x, double target_y)
        {
            X = target_x; Y = target_y;
        }
        typedef CPPAD_TESTVECTOR(AD<double>) ADvector;
        // 
        void operator()(ADvector& fg, const ADvector& x)
        {
            assert(fg.size() == 2);
            assert(x.size() == 2);
            // variables 优化变量
            AD<double> x1 = x[0];
            AD<double> x2 = x[1];
            // f(x) objective function 目标函数
            fg[0] = (L1 * cos(x1) + L2 * cos(x2) - X) * (L1 * cos(x1) + L2 * cos(x2) - this->X);//x = L1 cosa + L2 cosb -> minJ = (L1 cosa + L2 cosb)^2
            // constraints 约束条件
            fg[1] = L1 * sin(x1) + L2 * sin(x2) - L3;//y = L1sina + L2sinb - L3 作为在约束 在下面不等式上下界处
            return;
        }

        double X, Y;
    };
} //end namespace
 

bool get_started(double _x, double _y, robot_bringup::joint &target_joint)
{
    bool ok = true;
    size_t i;
    typedef CPPAD_TESTVECTOR(double) Dvector;
 
    // object that computes objective and constraints
    FG_eval fg_eval(_x, _y);

    size_t nx = 2; // number of varibles 
    size_t ng = 1; // number of constraints 
    Dvector x0(nx); // initial condition of varibles 初始？
 
    x0[0] = pi/2;
    x0[1] = 0.0;
 
    // lower and upper bounds for varibles
    Dvector xl(nx), xu(nx);
    xl[0] = 0.0;    xu[0] = 1.710423;
    xl[1] = -1.745329;    xu[1] = 0.733038;
	
    //不等式上下界
    Dvector gl(ng), gu(ng);
    gl[0] = fg_eval.Y;    gu[0] = fg_eval.Y;
 
    // options
    string options;
    // turn off any printing
    options += "Integer print_level  0\n";
    options += "String sb            yes\n";
    // maximum iterations
    options += "Integer max_iter     10\n";
    //approximate accuracy in first order necessary conditions;
    // see Mathematical Programming, Volume 106, Number 1,
    // Pages 25-57, Equation (6)
    options += "Numeric tol          1e-6\n";
    //derivative tesing
    options += "String derivative_test   second-order\n";
    // maximum amount of random pertubation; e.g.,
    // when evaluation finite diff
    options += "Numeric point_perturbation_radius   0.\n";
 
    //定义solution并求解
    CppAD::ipopt::solve_result<Dvector> solution; // solution
    CppAD::ipopt::solve<Dvector, FG_eval>(options, x0, xl, xu, gl, gu, fg_eval, solution); // solve the problem

    target_joint.bigJoint_deg = solution.x[0];
    target_joint.smallJoint_deg = solution.x[1];

    cout << "solution: " << solution.x << endl;
    cout << "fx min value:" << solution.obj_value << endl;
 
    //check some of the solution values 这个check有什么用？
    ok &= solution.status == CppAD::ipopt::solve_result<Dvector>::success;
 
    double check_x[]  = {1.29106, -0.264582};
    double rel_tol    = 1e-6; // relative tolerance
    double abs_tol    = 1e-6; // absolute tolerance
    for(i = 0; i < nx; i++)
    {
        ok &= CppAD::NearEqual(check_x[i], solution.x[i], rel_tol, abs_tol);
    }
 
    return ok;
}
