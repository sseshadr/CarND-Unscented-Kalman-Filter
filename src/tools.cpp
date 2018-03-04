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

	VectorXd rmse(4);
	rmse << 0, 0, 0, 0;
	VectorXd residual;
	VectorXd mean;

	// check the validity of the following inputs:
	//  * the estimation vector size should not be zero
	//  * the estimation vector size should equal ground truth vector size
	// ... your code here
	if (estimations.size() == 0)
	{
		cout << "Estimation vector has zero size!";
		return rmse;
	}
	else if (estimations.size() != ground_truth.size())
	{
		cout << "Estimation and ground truth vectors are of varying sizes!";
		return rmse;
	}

	//accumulate squared residuals
	for (int i = 0; i < estimations.size(); ++i) {
		// ... your code here
		residual = estimations[i] - ground_truth[i];
		residual = residual.array()*residual.array();
		rmse += residual;
	}

	//calculate the mean
	// ... your code here
	mean = rmse / estimations.size();

	//calculate the squared root
	// ... your code here
	rmse = mean.array().sqrt();

	//return the result
	return rmse;
}