/*
 * Copyright 2020 Casey Sanchez
 */

#pragma once

#include <eigen3/Eigen/Dense>
#include <eigen3/Eigen/LU>

template<int dim>
class Measurement 
{
	Eigen::Matrix<double, dim, 1> m_state;
	Eigen::Matrix<double, dim, dim> m_covariance;
	
public:
	Measurement(Eigen::Matrix<double, dim, 1> const &state, Eigen::Matrix<double, dim, dim> const &covariance) : m_state(state), m_covariance(covariance)
	{
	}

	Eigen::Matrix<double, dim, 1> state() const
	{
		return m_state;
	}

	Eigen::Matrix<double, dim, dim> covariance() const
	{
		return m_covariance;
	}
};

template<int dim>
class KalmanFilter 
{
	Eigen::Matrix<double, dim, 1> m_sigma;
	Eigen::Matrix<double, dim, 1> m_state;
	Eigen::Matrix<double, dim, dim> m_covariance;

public:
	KalmanFilter(Eigen::Matrix<double, dim, 1> const &sigma) : m_sigma(sigma), m_state(Eigen::Matrix<double, dim, 1>::Zero()), m_covariance(Eigen::Matrix<double, dim, dim>::Identity())
	{
	}

	void predict(double const &delta_time)
	{
		m_covariance += (m_sigma * delta_time).asDiagonal();
	}

	void update(Measurement<dim> const &measurement) 
	{
		Eigen::Matrix<double, dim, 1> const innovation_state = measurement.state() - m_state;
		Eigen::Matrix<double, dim, dim> const innovation_covariance = measurement.covariance() + m_covariance;

		Eigen::FullPivLU<Eigen::Matrix<double, dim, dim>> const lower_upper = innovation_covariance.fullPivLu();
		Eigen::Matrix<double, dim, dim> const kalman_gain = m_covariance * lower_upper.inverse();
		
		m_state += kalman_gain * innovation_state;

		m_covariance = (Eigen::Matrix<double, dim, dim>::Identity() - kalman_gain) * m_covariance;
	}

	Eigen::Matrix<double, dim, 1> state() const
	{
		return m_state;
	}
};
