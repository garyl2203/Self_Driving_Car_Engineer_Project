#include <iostream>
#include "tools.h"

using Eigen::VectorXd;
using Eigen::MatrixXd;
using std::vector;

Tools::Tools() {}

Tools::~Tools() {}

VectorXd Tools::CalculateRMSE(const vector<VectorXd> &estimations,
                              const vector<VectorXd> &ground_truth) {
  /**
  TODO:
    * Calculate the RMSE here.
  */
	if (estimations.size() == 0 || estimations.size() != ground_truth.size()) {
        return VectorXd();
    }

    VectorXd rmse(estimations[0].size());

    for (unsigned int i = 0; i < estimations.size(); ++i) {
        VectorXd residual = estimations[i] - ground_truth[i];
        residual = residual.array() * residual.array();
        rmse += residual;
    }

    rmse = rmse / estimations.size();
    rmse = rmse.array().sqrt();
	
    return rmse;
}

void Tools::CalculateJacobian(const VectorXd& x_state, MatrixXd &Hj) {
  /**
  TODO:
    * Calculate a Jacobian here.
  */
	float px = x_state(0);
    float py = x_state(1);

    if (px == 0 && py == 0) {
        std::cout << "CalculateJacobian () - Error - Division by Zero" << std::endl;
        return;
    }

    float vx = x_state(2);
    float vy = x_state(3);
    float px2py2 = pow(px, 2) + pow(py, 2);
    float sqrtpx2py2 = sqrt(px2py2);

    Hj << px / sqrtpx2py2, py / sqrtpx2py2, 0, 0,
            -(py / px2py2), px / px2py2, 0, 0,
            py * (vx * py - vy * px) / pow(px2py2, 1.5), px * (vy * px - vx * py) / pow(px2py2, 1.5), px / sqrtpx2py2,
            py / sqrtpx2py2;
}
//multivariate taylor expansions
VectorXd Tools::h(const VectorXd &x) {
    VectorXd Hx(3);

    float first = sqrt(pow(x(0), 2) + pow(x(1), 2));
    float second = atan2(x(1), x(0));
    float third = (x(0) * x(2) + x(1) * x(3)) / first;

    Hx << first, second, third;

    return Hx;
}
