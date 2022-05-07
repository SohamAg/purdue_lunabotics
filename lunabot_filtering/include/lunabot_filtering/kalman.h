#ifndef KALMAN_H
#define KALMAN_H

#include <bits/stdc++.h>
#include <Eigen/Dense>
#include <Eigen/Core>
#include <Eigen/LU>

#include <exception>
#include <iostream>

//#define DEBUG 1

//Explained here: https://www.kalmanfilter.net/multiSummary.html

//Consts and typedefs
const int sz = 3; //Number of variables tracked- x, x', x'', y, y', y'', z, z', z'', theta, theta', theta''
typedef Eigen::Matrix<double, sz, sz> Mat;
typedef Eigen::Matrix<double, sz, 1> Vec;
const int sensors = 3; //Number of sensors

//Prediction Functions
Vec calcState(Mat F, Vec x, Mat G, Vec u);
Mat calcUncertainty(Mat F, Mat POld, Mat Q);

//Correction Functions
Mat calcGain(Mat POld, Mat H, Mat R);
Vec updateState(Vec x, Mat K, Vec z, Mat H);
Mat updateUncertainty(Mat K, Mat H, Mat POld, Mat R);

//Kalman Simulation Class
class Kalman 
{
private:
	Mat Q, K, F, G;
	Mat P[sensors], POld[sensors], H[sensors], R[sensors];
	Vec x;

public:
	Kalman(Vec x0_, Mat Q_, Mat F_, Mat G_, Mat *P_, Mat *H_, Mat *R_);
	void predict(int sensorID, Vec u); //Predict the future
	void correct(int sensorID, Vec z); //Measurement Update
	Vec getX();
};

#endif

/* Sensor descriptions
Apriltag:
H diagonal =
[1, 0, 0, 1, 0, 0, 1, 0, 0, 0, 0, 0]

Odometry Camera:
H diagonal =
[1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1]

Bluetooth:
H diagonal =
[1, 0, 0, 1, 0, 0, 1, 0, 0, 0, 0, 0]
*/