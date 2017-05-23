/*
 * particle_filter.cpp
 *
 *  Created on: Dec 12, 2016
 *      Author: Tiffany Huang
 */

#include <random>
#include <algorithm>
#include <iostream>
#include <numeric>

#include "particle_filter.h"

void ParticleFilter::init(double x, double y, double theta, double std[]) {
	// TODO: Set the number of particles. Initialize all particles to first position (based on estimates of 
	//   x, y, theta and their uncertainties from GPS) and all weights to 1. 
	// Add random Gaussian noise to each particle.
	// NOTE: Consult particle_filter.h for more information about this method (and others in this file).
	num_particles = 2;
	weights.resize(num_particles);
	std::default_random_engine gen;
	//double std_x, std_y, std_psi; // Standard deviations for x, y, and psi

	// TODO: Set standard deviations for x, y, and psi
	double std_x = std[0];
	double std_y = std[1];
	double std_psi = std[2];
	 

	// This line creates a normal (Gaussian) distribution for x
	std::normal_distribution<double> dist_x(x, std_x);
	std::normal_distribution<double> dist_y(y, std_y);
	std::normal_distribution<double> dist_psi(theta, std_psi);

	
	for (int i = 0; i < num_particles ; ++i) {
		double sample_x, sample_y, sample_psi;
		
		// TODO: Sample  and from these normal distrubtions like this: 
		//	 sample_x = dist_x(gen);
		//	 where "gen" is the random engine initialized earlier.
		
		 sample_x = dist_x(gen);
		 sample_y = dist_y(gen);
		 sample_psi = dist_psi(gen);	
		 Particle p; 
		 p.id = i;
		 p.x = sample_x;
		 p.y = sample_y;
		 p.theta = sample_psi;
		 p.weight = 1.0;

		 particles.push_back(p);
		 // Print your samples to the terminal.
		 //cout << "Sample " << i + 1 << " " << sample_x << " " << sample_y << " " << sample_psi << endl;
	}

	is_initialized = true;

}

void ParticleFilter::prediction(double delta_t, double std_pos[], double velocity, double yaw_rate) {
	// TODO: Add measurements to each particle and add random Gaussian noise.
	// NOTE: When adding noise you may find std::normal_distribution and std::default_random_engine useful.
	//  http://en.cppreference.com/w/cpp/numeric/random/normal_distribution
	//  http://www.cplusplus.com/reference/random/default_random_engine/
	if (fabs(yaw_rate) < 0.0001) {
		yaw_rate = 0.0001;
	}

	for (int i = 0; i < particles.size() ; ++i) {
		double x = particles[i].x;
		double y = particles[i].y;
		double theta = particles[i].theta;
		double x_pred, y_pred, theta_pred;
		x_pred = x + velocity/yaw_rate*(sin(theta+yaw_rate*delta_t)-sin(theta));
		y_pred = y + velocity/yaw_rate*(cos(theta)-cos(theta+yaw_rate*delta_t));
		theta_pred = theta + yaw_rate*delta_t;
		particles[i].x = x_pred;
		particles[i].y = y_pred;
		particles[i].theta = theta_pred;

	}
	//std::cout << "particles[0].x: " << particles[0].x << std::endl;

}

void ParticleFilter::dataAssociation(std::vector<LandmarkObs> predicted, std::vector<LandmarkObs>& observations) {
	// TODO: Find the predicted measurement that is closest to each observed measurement and assign the 
	//   observed measurement to this particular landmark.
	// NOTE: this method will NOT be called by the grading code. But you will probably find it useful to 
	//   implement this method and use it as a helper during the updateWeights phase.

	// double obs_x, obs_y, pred_x, pred_y, distance;
	// std::vector<LandmarkObs> predicted_;

	// for (int i = 0; i < observations.size() ; ++i) {

	// 	obs_x = observations[i].x;
	// 	obs_y = observations[i].x;
	// 	double dist_min = 100000.0;
	// 	int ID_closest;

	// 	for (int j = 0; j < predicted.size() ; ++j) {
	// 		pred_x = predicted[j].x;
	// 		pred_y = predicted[j].y;
	// 		distance = dist(pred_x,pred_y,obs_x,obs_y);
	// 		if (distance<dist_min){
	// 			dist_min = distance;
	// 			ID_closest = j;
	// 		}

	// 	}

	// 	predicted_.push_back(predicted[ID_closest]);

	// }

	// predicted = predicted_;

}

void ParticleFilter::updateWeights(double sensor_range, double std_landmark[], 
		std::vector<LandmarkObs> observations, Map map_landmarks) {
	// TODO: Update the weights of each particle using a mult-variate Gaussian distribution. You can read
	//   more about this distribution here: https://en.wikipedia.org/wiki/Multivariate_normal_distribution
	// NOTE: The observations are given in the VEHICLE'S coordinate system. Your particles are located
	//   according to the MAP'S coordinate system. You will need to transform between the two systems.
	//   Keep in mind that this transformation requires both rotation AND translation (but no scaling).
	//   The following is a good resource for the theory:
	//   https://www.willamette.edu/~gorr/classes/GeneralGraphics/Transforms/transforms2d.htm
	//   and the following is a good resource for the actual equation to implement (look at equation 
	//   3.33. Note that you'll need to switch the minus sign in that equation to a plus to account 
	//   for the fact that the map's y-axis actually points downwards.)
	//   http://planning.cs.uiuc.edu/node99.html
	double s_x = std_landmark[0];
	double s_y = std_landmark[1];
	double weight_sum = 0;

	for (int i = 0; i < particles.size() ; ++i) {
		double x = particles[i].x;
		double y = particles[i].y;
		double theta = particles[i].theta;

		std::vector<LandmarkObs> observations_;

		for (int j = 0; j < observations.size() ; ++j){
			LandmarkObs LO;
			double xt = observations[j].x;
			double yt = observations[j].y;
			LO.x = xt*cos(theta) - yt*sin(theta) + x;
			LO.y = xt*sin(theta) + yt*cos(theta) + y;
			LO.id = j;
			observations_.push_back(LO);
		}

		std::vector<LandmarkObs> predicted;	

		for (int j = 0; j < map_landmarks.landmark_list.size() ; ++j){
			LandmarkObs lmo;			
			double xt = map_landmarks.landmark_list[j].x_f;
			double yt = map_landmarks.landmark_list[j].y_f;
			if(dist(xt,yt,x,y)<=1.2*sensor_range){
				lmo.x = xt;
				lmo.y = yt;
				lmo.id = j;
				predicted.push_back(lmo);
			}
			// lmo.x = xt;
			// lmo.y = yt;
			// lmo.id = j;
			// predicted.push_back(lmo);
		}

		long double w=1.0;

		//std::cout << "observations_.size(): " << observations_.size() << std::endl;

		for (int k = 0; k < observations_.size() ; ++k) {
			double obs_x = observations_[k].x;
			double obs_y = observations_[k].x;
			double dist_min = 100000.0;

			int ID_closest = 0;

			for (int j = 0; j < predicted.size() ; ++j) {
				double pred_x = predicted[j].x;
				double pred_y = predicted[j].y;
				double distance = dist(pred_x,pred_y,obs_x,obs_y);
				if (distance<dist_min){
					dist_min = distance;
					ID_closest = j;
				}

			}

			double P_xy = ( 1.0 / ( s_y*s_x*2.0*M_PI) ) * exp( -(0.5*pow( (observations_[k].x-predicted[ID_closest].x)/s_x, 2.0 )+0.5*pow( (observations_[k].y-predicted[ID_closest].y)/s_y, 2.0 ) ));
			
			w = w*P_xy;

			//std::cout << "observations_[k].x: " << observations_[k].x << std::endl;
			//std::cout << "predicted[ID_closest].x: " << predicted[ID_closest].x << std::endl;
			//std::cout << "observations_[k].y: " << observations_[k].y << std::endl;
			//std::cout << "predicted[ID_closest].y: " << predicted[ID_closest].y << std::endl;
			//std::cout << "pdf_gaussian_y: " << pdf_gaussian_y << std::endl;
			//std::cout << "P_xy: " << P_xy << std::endl;
		}

		particles[i].weight = w;
		weights[i] = w;
		weight_sum = weight_sum + w;

		//dataAssociation(predicted,observations_);

	}
	std::cout << "weight_sum: " << weight_sum << std::endl;


	weights.resize(particles.size(),0);
	//weights.fill(1.0);

	for (int i = 0; i < particles.size() ; ++i) {
		particles[i].weight = particles[i].weight/weight_sum;
		weights[i] = particles[i].weight;
	}


}

void ParticleFilter::resample() {
	// TODO: Resample particles with replacement with probability proportional to their weight. 
	// NOTE: You may find std::discrete_distribution helpful here.
	//   http://en.cppreference.com/w/cpp/numeric/random/discrete_distribution
	std::default_random_engine gen;

	//std::initializer_list<double> weights

	std::discrete_distribution<> dist_weighted(weights.begin(),weights.end());
	std::vector<Particle> particles_sampled;

	for (int i = 0; i < particles.size() ; ++i) {
		int sample_index = dist_weighted(gen);
		particles_sampled.push_back(particles[sample_index]);
	}

	particles = particles_sampled;
	
}

void ParticleFilter::write(std::string filename) {
	// You don't need to modify this file.
	std::ofstream dataFile;
	dataFile.open(filename, std::ios::app);
	for (int i = 0; i < num_particles; ++i) {
		dataFile << particles[i].x << " " << particles[i].y << " " << particles[i].theta << "\n";
	}
	dataFile.close();
}
