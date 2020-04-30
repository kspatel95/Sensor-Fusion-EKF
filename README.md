# Extended Kalman Filter Project

For this project the outline was to implement a sensor fusion of Radar and LiDAR to track a cyclist, using an Extended Kalman filter.
The key objectives for this project were to:

1) Be able to track a cyclist in real time and making sure to be agnostic to if it was a radar measurment or LiDAR measurement.
2) Be able to output a tracked position in cartesian coordinates.
3) Minimise the Root Mean Squared Error (RMSE) based off ground truth data from the dataset.

![Kalman_filter](./Images/Screenshot%20from%202020-03-04%2016-04-33.png)

## Theory of Operation

The theory behind this Kalman Filter implementation is that we will receive raw values from our sensors, therefore in the 
case of LiDAR this will be a positional vector but with no velocity reading:

<img src="http://chart.apis.google.com/chart?cht=tx&chl=z = \begin{bmatrix} p^x \\ p^y \end{bmatrix}" />

And in the case of Radar this will be a non linear function:

<img src="http://chart.apis.google.com/chart?cht=tx&chl=z = \begin{bmatrix} \rho \\ \varphi \\ \dot{\rho} \end{bmatrix} " />

In the case of our LiDAR measurement a basic implementation of a Kalman filter will suffice as the data we are receiving
is a linear function and therefore fits a Gaussian distribution by which the Kalman filter can derive predictions and measurements:

**Prediction Step LiDAR:**

<img src="http://chart.apis.google.com/chart?cht=tx&chl=x\prime = Fx + u \text{ %20Note%20 here%20 we%20 assume%20 u%20 for%20 the%20 motion%20 vector%20 is%20 negligable%20 therefore%20 ignored%20}" /> 
<br />

<img src="http://chart.apis.google.com/chart?cht=tx&chl=P\prime = FPF^T + Q" />
<br />
<br />

**Measurement Step LiDAR**

<img src="http://chart.apis.google.com/chart?cht=tx&chl=y = z - Hx " />
<br />

<img src="http://chart.apis.google.com/chart?cht=tx&chl=S = HPH^T + R " />
<br />

<img src="http://chart.apis.google.com/chart?cht=tx&chl=K = Ph^TS^i " />
<br />
<br />

After the final measurment step we update the returned values of our Kalman Filter:
<br />
<br />

<img src="http://chart.apis.google.com/chart?cht=tx&chl=x\prime = x + Ky " />
<br />

<img src="http://chart.apis.google.com/chart?cht=tx&chl=P\prime = (I - KH) * P " />
<br />
<br />

Here we can define:

<img src="http://chart.apis.google.com/chart?cht=tx&chl=x = \text{ Position and Velocity Estimate} " />
<br />

<img src="http://chart.apis.google.com/chart?cht=tx&chl=P = \text{ The Uncertainty Covariance Matrix} " />
<br />

<img src="http://chart.apis.google.com/chart?cht=tx&chl=Q = \text{ The Process Covariance Matrix} " />
<br />

<img src="http://chart.apis.google.com/chart?cht=tx&chl=F = \text{ The State Transition matrix} " />
<br />

<img src="http://chart.apis.google.com/chart?cht=tx&chl=u = \text{ The Motion Vector} " />
<br />

<img src="http://chart.apis.google.com/chart?cht=tx&chl=z = \text{ The Measurement Vector} " />
<br />

<img src="http://chart.apis.google.com/chart?cht=tx&chl=H = \text{ The Measurement Function} " />
<br />

<img src="http://chart.apis.google.com/chart?cht=tx&chl=R = \text{ The Measurement Noise} " />
<br />

<img src="http://chart.apis.google.com/chart?cht=tx&chl=I = \text{ Identity Matrix} " />
<br />

Here we can see that based on the prediction on measurement update the velocity for LiDAR will initially start in a high 
state of uncertainty. However as the filter progresses the prediction can start to build a more accurate idea of the 
defined position and a later time t therefore by knowing t and the projected position, velocity can be inferred.

**Radar Measurement**

In the case of radar we have to take a slightly different approach due to the fact that radar will need to be converted
from the cartesian prediction to the polar coordinates received by the radar, this leaves us an issue as we are required to use the arctangent which produces a non linear function:

<img src="http://chart.apis.google.com/chart?cht=tx&chl=h(x\prime) = \begin{pmatrix} \rho \\ \phi \\ \dot{\rho} \end{pmatrix}" />
<img src="http://chart.apis.google.com/chart?cht=tx&chl=h(x\prime)= \begin{pmatrix} \sqrt{p\prime_x^2 + p\prime_y^2} \\ \arctan{p\prime_y / p\prime_x} \\ \frac{(p\prime_x v\prime_x + p\prime_y v\prime_y)/}{(\sqrt{p\prime_x^2 + p\prime_y^2)}} \end{pmatrix}" />
<br />
<br />

So the adaption here is in the measurement step as dude to the fact we are using a linear model there is no need to change the prediction step.
So when it comes to measuring for Radar we convert firstly the H function to become a jacobian matrix based on the partial derivatives of the radar measurements with the state vector:

<img src="http://chart.apis.google.com/chart?cht=tx&chl=H = H_\text{jacobian} "/>
<br />

We then also adapt out y calculation to take into account out h(x) function to calculate the polar coordiantes:

<img src="http://chart.apis.google.com/chart?cht=tx&chl=y = x - h(x\prime) "/>
<br />

Finally the data defines the measurement that is received and the filter performs the update and the filter performs the measurement step accordingly.

## Data Input/Output

**INPUT**: values provided by the simulator to the c++ program

["sensor_measurement"] => the measurement that the simulator observed (either lidar or radar)


**OUTPUT**: values provided by the c++ program to the simulator

["estimate_x"] <= kalman filter estimated position x

["estimate_y"] <= kalman filter estimated position y

["rmse_x"]

["rmse_y"]

["rmse_vx"]

["rmse_vy"]

## Links

- For more info on the specific data structure see [here](./Docs/Input_Output%20File%20Format.txt). 
- For a sample dataset see [here](./data/obj_pose-laser-radar-synthetic-input.txt).
- To download the simulator to run this tool see [here](https://github.com/udacity/self-driving-car-sim/releases).

## Other Important Dependencies

* cmake >= 3.5
  * All OSes: [click here for installation instructions](https://cmake.org/install/)
* make >= 4.1 (Linux, Mac), 3.81 (Windows)
  * Linux: make is installed by default on most Linux distros
  * Mac: [install Xcode command line tools to get make](https://developer.apple.com/xcode/features/)
  * Windows: [Click here for installation instructions](http://gnuwin32.sourceforge.net/packages/make.htm)
* gcc/g++ >= 5.4
  * Linux: gcc / g++ is installed by default on most Linux distros
  * Mac: same deal as make - [install Xcode command line tools](https://developer.apple.com/xcode/features/)
  * Windows: recommend using [MinGW](http://www.mingw.org/)

## Basic Build Instructions

1. Clone this repo.
2. Make a build directory: `mkdir build && cd build`
3. Compile: `cmake .. && make` 
   * On windows, you may need to run: `cmake .. -G "Unix Makefiles" && make`
4. Run it: `./ExtendedKF `
