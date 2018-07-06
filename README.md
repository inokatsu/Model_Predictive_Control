# Model Predictive Controls(MPC) project)

[//]: # (Image References)

[image1]: ./images/MMPC_result_movie.gif "result gif"
[image2]: ./images/model_predictive_control_formula.png "MPC formula"

![result gif][image1]
#### Displaying the MPC trajectory path in green, and the polynomial fitted reference path in yellow.

## Introduction
The project goal is to make a simulated car drive around a track by using model predictive control. 


## Model

Kinematic models are simplification of dynamic models that ignore tire forces, gravity, and mass. This simplication reduces the accuracy of the models, but it also makes them more tractable.
The kinematic model of this project includes 6 vehicle states: x, y, orientation angle(psi), velocity, cross tracking error and psi error(epsi). Actuator outputs are throttle value and steering angle.

![model formula][image2]
###### Sited from Udacity SDND program

## Timestep Length and Elapsed Duration

Timestep length slould be not only large enough to predict further status ahead, but also need to be small enough to compute efficiently. 

Timestep frequency need to be small to accurately approximate a continuous reference trajectory, but small frequency needs a lot of computational cost.

Timestep length (N) is chosen to be `10`, and timestep frequency (dt) `0.1`, which means that 1s ahead (= N * dt) state is predicted.
WIth 20 Timestep length the controller starts to run slower and it went out of the track. 




## Polynominal Fitting and MPC Preprocessing

To estimate waypoints I transformed the waypoints from gloval coordinate coordinate system to the vehicle's coordinate system.

```cpp
// Convert waypoints to car's coordinate
Eigen::VectorXd ptsxWaypoint(ptsx.size());
Eigen::VectorXd ptsyWaypoint(ptsy.size());

for(int i = 1; i < ptsx.size(); i++){
  ptsxWaypoint[i] = (ptsx[i] - px) * cos(0-psi) - (ptsy[i] - py) * sin(0-psi);
  ptsyWaypoint[i] = (ptsx[i] - px) * sin(0-psi) + (ptsy[i] - py) * cos(0-psi);
}

```

The reference trajectory is typically passed to the control block as a polynomial. The polynomial is usually 3rd order, since third order polynomials will fit trajectories for most roads. 

To fit a polynomial, I used a function which is adupted from here:
https://github.com/JuliaMath/Polynomials.jl/blob/master/src/Polynomials.jl#L676-L716



## Model Predictive Control with Latency
there's a 100 millisecond latency between actuations commands on top of the connection latency. To deal with this latency, it is artificially included before sending atuations to the simulator to mimic typical response times for real world cars.

The latency factor was included in the state vector before sending in to the model calculation.

```cpp
// Add Latency
const double delay_t = 0.1;

double delay_px = v * delay_t;
double delay_py = 0;
double delay_psi = v * (-steer_value) * delay_t / Lf;
double delay_v = v + throttle_value * delay_t;
double delay_cte = cte + v * sin(epsi) * delay_t;
double delay_epsi = epsi + delay_psi;

Eigen::VectorXd state(6);
state << delay_px, delay_py, delay_psi, delay_v, delay_cte, delay_epsi;
auto mpc_result = mpc.Solve(state, coeffs);

```





## Result video

https://youtu.be/IoL0CUUihWM




---

## Dependencies

* cmake >= 3.5
 * All OSes: [click here for installation instructions](https://cmake.org/install/)
* make >= 4.1(mac, linux), 3.81(Windows)
  * Linux: make is installed by default on most Linux distros
  * Mac: [install Xcode command line tools to get make](https://developer.apple.com/xcode/features/)
  * Windows: [Click here for installation instructions](http://gnuwin32.sourceforge.net/packages/make.htm)
* gcc/g++ >= 5.4
  * Linux: gcc / g++ is installed by default on most Linux distros
  * Mac: same deal as make - [install Xcode command line tools]((https://developer.apple.com/xcode/features/)
  * Windows: recommend using [MinGW](http://www.mingw.org/)
* [uWebSockets](https://github.com/uWebSockets/uWebSockets)
  * Run either `install-mac.sh` or `install-ubuntu.sh`.
  * If you install from source, checkout to commit `e94b6e1`, i.e.
    ```
    git clone https://github.com/uWebSockets/uWebSockets
    cd uWebSockets
    git checkout e94b6e1
    ```
    Some function signatures have changed in v0.14.x. See [this PR](https://github.com/udacity/CarND-MPC-Project/pull/3) for more details.

* **Ipopt and CppAD:** Please refer to [this document](https://github.com/udacity/CarND-MPC-Project/blob/master/install_Ipopt_CppAD.md) for installation instructions.
* [Eigen](http://eigen.tuxfamily.org/index.php?title=Main_Page). This is already part of the repo so you shouldn't have to worry about it.
* Simulator. You can download these from the [releases tab](https://github.com/udacity/self-driving-car-sim/releases).
* Not a dependency but read the [DATA.md](./DATA.md) for a description of the data sent back from the simulator.


## Basic Build Instructions

1. Clone this repo.
2. Make a build directory: `mkdir build && cd build`
3. Compile: `cmake .. && make`
4. Run it: `./mpc`.

