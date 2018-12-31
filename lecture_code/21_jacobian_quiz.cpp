#include <iostream>
#include "Dense"
#include <vector>

using namespace std;
using Eigen::MatrixXd;
using Eigen::VectorXd;

MatrixXd CalculateJacobian(const VectorXd& x_state);

int main() {

	/*
	 * Compute the Jacobian Matrix
	 */

	//predicted state  example
	//px = 1, py = 2, vx = 0.2, vy = 0.4
	VectorXd x_predicted(4);
	x_predicted << 1, 2, 0.2, 0.4;

	MatrixXd Hj = CalculateJacobian(x_predicted);

	cout << "Hj:" << endl << Hj << endl;

	return 0;
}

MatrixXd CalculateJacobian(const VectorXd& x_state) {

	MatrixXd Hj(3,4);
	//recover state parameters
	float px = x_state(0);
	float py = x_state(1);
	float vx = x_state(2);
	float vy = x_state(3);

    if ((px*px+py*py) < 0.0001) {
        cout << "Error - Division by Zero" << endl;
        return Hj;
    }
    else {
        Hj << (px/sqrt(px*px+py*py)), (py/sqrt(px*px+py*py)), 0, 0,
              -(py/(px*px+py*py)), (px/(px*px+py*py)), 0, 0,
              py*(vx*py - vy*px)/(sqrt(px*px+py*py)*(px*px+py*py)), px*(px*vy - py*vx)/(sqrt(px*px+py*py)*(px*px+py*py)), px/sqrt(px*px+py*py), py/sqrt(px*px+py*py);
    }
    
	return Hj;
}
