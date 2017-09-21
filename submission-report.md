# Extended Kalman Filter Project

[video1]: ./video1.mp4
[video2]: ./video2.mp4
[laser]: ./laser.mp4
[radar]: ./radar.mp4

## Content of the Submission and Usage
This submission includes the following c++ files:
* main.cpp: the main function that communicate with the simulator and drive the estimation process using EKF.
* filter/FusionEKF.h, filter/FusionEKF.cpp: contains EKF implementation
* filter/KalmanFilter.h, filter/KalmanFilter.cpp]: contains the base Kalman filter implementation
* filter/utils.h, filter/utils.cpp]: contains some utility functions used by FusionEKF
* performance/RMSEEvaluator.h: A template class for memory-efficient evaluation of RMSE
* sensor/SensorType.h: defines SensorType enum
* sensor/MeasurementPackage.h defines sensor measurement package structure

### Usage
To start the program:

    ./ExtendedKF

The program will listen on port 4567 for an incoming simulator connection. Only one simulator should be connected at anytime, though the program does not prohibit it. To start a new simulator, terminate the existing one first, then start a new one.

To start the simulator:

#### On Windows

    term2_sim9.exe path_to_obj_pose-laser-radar-synthetic-input.txt

#### Other platforms:

    ./term2_sim9 path_to_obj_pose-laser-radar-synthetic-input.txt

#### Run multiple simulations
*ExtendedKF* allows one to run multiple simulations, whether from the same simulator or different simulators, one at a time without having to restart it. This allows the RMSE to be evaluated for a much bigger set of estimates.

#### Build ExtendedKF
Follow the *Basic Build Instructions* described in [README.md](README.md) to build ExtendedKF.
For Windows, Bash on Ubuntu on Windows should be used. Please follow the *Tips for Setting Up Your Environment* on how to setup and configure the build environment.

The program can be built to output more information for disgnosis purpose by defining **VERBOSE_OUT** macro.

#### Build API Documentation
The documentation for functions, classes and methods are included in the header files in Doxygen format. To generate Api documentation with the included doxygen.cfg:

1. Make sure doxygen is installed
2. cd to the root folder of this project
2. Issue the following command:

    doxygen doxygen.cfg

## The Implementation

### The source code structure
The src folder contains three folders for the source files:
. filter: contains KalmanFilter.[h,cpp], FusionEKF.[h,cpp], and utils.[h,cpp] files which implement the Extended Kalman Filter.
. performance: contains RMSEEvaluator.h which provides RMSE evaluation functionality.
. sensor: contains source code for defining sensor measurement.

##### File names
In this project, I made a class to have its own .h, and .cpp files. More specifically, a class source file contains exactly one class, and has the same name as the class it contains. On the other hand, 

#### The libs folder
The libs folder contains Eigen and json.hpp required by the program.

### KalmanFilter class
This class implement the baseline Kalman filter functionalities including the predict, update functions.

It's data members include the estimation state, state transition matrix, the measurement matrix, and the covariance matrices. Getter and setter functions for some of them are provided. Refer to the API documentation for [KalmanFilter](api/html/classKalmanFilter.html).

### FusionEKF class
This class implement the extended Kalman filter functionalities. It extends KalmanFilter class and implements *ProcessMeasurement()* methods.

The extended Kalman filter update for Radar measurement is implemented in method *UpdateEKF()*.

The Jacobian matrix for the Radar measurement matrix is computed in method *CalculateJacobian()*.

The API documentation for FusionEKF can be found [here](api/html/classKalmanFilter.html).

#### Angular correction for radar estimates

The radar measurement's bearing angle is expected to lie between [ &#960;, -&#960; ). This has to be enforced for radar measurements even though they are expected to fall into this range, but some exceptions are seen in the supplied data file for this project.

In addition to the bearing angle, the angular difference between predictions and measurements needs to aligned to be within this range which is also the minimum angle to move from one angle to the other.

Failing to align the angles correctly will severely degrade the accuracy of the estimation.

#### The Extended Kalman Filter Process
The process is implemented in method *ProcessMeasurement()*.

##### Initialization
The FusionEKF is initialized upon receiving the first measurement. 

If the first measurement is from radar. The state is computed from the radar's measurement using utils::RadarToPV() function.

If the first measurement is from lidar, the measured position is used to populate the state with acceleration 0.

The *ProcessMeasurement()* method returns after the initialized is completed.

##### Prediction
After initialization, a prediction is performed for a new measurement if the time elapsed since the previoud measurement is above a small threshold (defined in constant MINIMAL_TIME_THRESHOLD) in the implementation.

The prediction process is identical for radar and lidar measurements. It starts by computing the state transition matrix and the process covariance matrix.

Then the Predict() method (implemented in KalmanFilter class) is invoked.

Finally, the update process is launched.

##### Update
The update process is always performed for every measurement after initialization.

Lidar measurement is carried out by *KalmanFilter::Update()* method.

Radar measurement is updated with *FusionEKF::UpdateEKF()* method. It calls *CalculateJacobian()* to compute the Jacobian matrix, and *utils::PVToRadar()* to convert state values to radar values. Angular correction described above is performed on:

1. The measurement's bearing angle
2. The difference between the measurement's bearing angle and the prediction's bearing angle.

### RMSEEvaluator class
The RMSE evaluation implemented in the starter code is not efficient as it keeps all past measurements and estimates in a list, and evaluate RMSE for every new measurement on the entire list.

The purpose of this class is to implement a more efficient RMSE mechanism that does not need to store past measurements and estimates. It maintains current sum of the square errors, and total number of past estimates. RMSE for a new estimate can be simply obtained by taking the square root of the mean of square error.

## Result
The RMSE performance of the implementation is averaged as follows after running dataset 1 followed by dataset 2:

|            |   RMSE   |
|:----------:|:--------:|
| X    		 |   0.0859 |
| Y          |   0.0911 |
| Vx    	 |   0.4367 |
| Vy         |   0.4673 |

Refer to [this video](video1.mp4).

### Discussion and Extra Experiments
The Kalman filter produces estimates effectively by dealing with uncertainity due to sensor noise and random external factors using a weighted average between current state estimation and the new measurement. The weight are derived from the covariance matrices. Intuitively, when the state covariance is smaller than the measurement covariance, the predicted state is trusted more, and will have a bigger weight, and when the state covariance is larger than the measurement covariance, the predicted is trusted less, and will have a smaller weight.

Despite the assumptions made for EKF, will changing condifence on prediction improves the RMSE performance? To answer this question, I run through a set of experiments that use different level of acceleration noise.  

|            |   8      |   9      |   10     |   12     |   20     |   30     |   40     |   60     |
|:----------:|:--------:|:--------:|:--------:|:--------:|:--------:|:--------:|---------:|---------:|
| X    		 |   0.0869 |   0.0859 |   0.085  |   0.0839 |   0.0822 (-3.93%) |   0.0821 |   0.0824 |   0.0832 | 
| Y          |   0.0930 |   0.0911 |   0.0897 |   0.0878 |   0.0854 (-6.25%) |   0.0859 |   0.0871 |   0.0896 | 
| Vx    	 |   0.4407 |   0.4367 |   0.4337 |   0.4298 |   0.4265 (-2.33%) |   0.4310 |   0.4337 |   0.4519 |
| Vy         |   0.4756 |   0.4673 |   0.4605 |   0.4502 |   0.4316 (-7.64%) |   0.4283 |   0.4324 |   0.4481 |

Acceleration noise between 20 and 30 yields the best RMSE perfoamance (RMSE improvement for noise 20 with respect to noise 9 is shown in the table). [This video](video2.mp4) shows the simulation with noise 20.

Regarding to whether using lidar alone or radar alone changes the performance, I did not observe any difference.
The following videos demonstrate this:

[for Laser alone](laser.mp4)

[for Radar alone](radar.mp4)

