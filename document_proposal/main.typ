#import "@preview/cleanified-hpi-research-proposal:0.1.0": *

#show: project.with(
  title: "Parking Algorithm based on Multiple Paths (pending to update)",
  author: "Alejandro D.J Gomez Florez",
  date: "November 26, 2025",
  remove-hpi-logo: true,
  additional-logos: (image("pictures/eafit.png"),),
  chair: "Ms Applied Mathematics | Universidad EAFIT",
)

= Introduction


== Problem Definition

In this work, we develop a probabilistic algorithm to address the autonomous parking problem. We model the vehicle as a point mass in a 2D plane with position and orientation, following the unicycle kinematic model. The vehicle's state is represented by the vector $X = (x, y, theta)^T$, where $(x, y)$ denotes the position and $theta$ represents the orientation angle. The continuous-time dynamics are governed by:

$ dot(X) = mat(dot(x); dot(y); dot(theta)) = mat(v cos(theta); v sin(theta); omega) $

where $v$ is the linear velocity and $omega$ is the angular velocity, which constitute the control inputs $u = (v, omega)^T$ to the vehicle. For implementation purposes, we discretize this model and formulate it as a discrete-time state-space system:

$ X_(k+1) = f(X_k, u_k) + w_k $

where $X_k$ represents the state at time step $k$, $f(dot, dot)$ is the nonlinear state transition function obtained by discretizing the unicycle dynamics, and $w_k tilde cal(N)(0, Q)$ represents the process noise accounting for model uncertainties and disturbances.

In the autonomous parking problem, a mobile vehicle must move from a set of initial positions towards a target pose known as the *docking pose*. However, the actual location of the vehicle is not perfectly known; it is affected by various sources of uncertainty such as sensor noise, odometry errors, and changes in environmental conditions.

To represent this uncertainty, the vehicle's state is no longer modeled as a deterministic pose $X = (x, y, theta)^T$, but rather as a probability distribution over that state. A common approach is to use a Gaussian model, where the vehicle maintains a _belief_ defined as $b = cal(N)(mu, P)$, where $mu$ is the mean pose and $P$ is the associated covariance matrix.


=== Really Small Stuff


= Related Work

