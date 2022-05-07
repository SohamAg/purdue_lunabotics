#include <lunabot_filtering/kalman.h>

Vec calcState(Mat F, Vec x, Mat G, Vec u) //State extrapolation equation
{
	Vec s = (F * x) + (G * u);
#ifdef DEBUG
	std::cout << "Calc state: \n" << s << std::endl;
#endif
	return s;
}

Mat calcUncertainty(Mat F, Mat POld, Mat Q) //Covariance extrapolation equation
{
	Mat U = F * POld * F.transpose() + Q;
#ifdef DEBUG
	std::cout << "Calc uncertainty: \n" << U << std::endl;
#endif
	return U;
}

Mat calcGain(Mat POld, Mat H, Mat R) //Kalman gain equation
{   Mat K = POld * (H.transpose()) * ((H * POld * (H.transpose())) + R).inverse();
#ifdef DEBUG
	std::cout << "Calc gain: \n" << K << std::endl;
#endif
	return K;
}

Vec updateState(Vec x, Mat K, Vec z, Mat H) //State update equation
{
	Vec s =  x + K * (z - H*x);
#ifdef DEBUG
	std::cout << "Update state: \n" << s << std::endl;
#endif
	return s;
}

Mat updateUncertainty(Mat K, Mat H, Mat POld, Mat R) //Covariance update equation
{
	Mat U = (Mat::Identity() - K * H) * POld * (Mat::Identity() - K * H).transpose() + K * R * K.transpose();
#ifdef DEBUG
	std::cout << "Update uncertainty: \n" << U << std::endl;
#endif
	return U;
}
Kalman::Kalman(Vec x0_, Mat Q_, Mat F_, Mat G_, Mat *P_, Mat *H_, Mat *R_) //Constructor for Kalman filter
{
	this->x = x0_;
	this->Q = Q_;
	this->F = F_;
	this->G = G_;
	for (int i = 0; i < sensors; i++)
	{
		this->P[i] = P_[i];
		this->POld[i] = P_[i];
		this->H[i] = H_[i];
		this->R[i] = R_[i];
	}
}

void Kalman::predict(int sensorID, Vec u) //Predict the future
{
#ifdef DEBUG
	std::cout << "Predicting sensor #" << sensorID << std::endl;
#endif
	//Are the parameters sensible?
	if (sensorID < 0 || sensorID >= sensors)
	{
		throw std::invalid_argument("Sensor ID out of range.");
	}

	this->x = calcState(this->F, this->x, this->G, u);
	Mat phold = this->P[sensorID];
	this->P[sensorID] = calcUncertainty(this->F, this->P[sensorID], this->Q);
	this->POld[sensorID] = phold;
}

void Kalman::correct(int sensorID, Vec z)
{
#ifdef DEBUG
	std::cout << "Correcting sensor #" << sensorID << std::endl;
#endif
	//Also known as measurement update
	//This should be run with different parameters for each sensor
	//Then predict should be run

	//Are the parameters sensible?
	if (sensorID < 0 || sensorID >= sensors)
	{
		throw std::invalid_argument("Sensor ID out of range.");
	}

	//Compute the Kalman gain
	Mat K = calcGain(this->P[sensorID], this->H[sensorID], this->R[sensorID]);
	//Update estimate with measurement
	this->x = updateState(this->x, K, z, this->H[sensorID]);
	//Update estimate uncertainty
	Mat Phold = updateUncertainty(K, this->H[sensorID], this->P[sensorID], this->R[sensorID]);
	this->POld[sensorID] = this->P[sensorID];
	this->P[sensorID] = Phold;
}

Vec Kalman::getX()
{
	return this->x;
}