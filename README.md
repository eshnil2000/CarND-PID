# CarND-Controls-PID
Self-Driving Car Engineer Nanodegree Program

## Results
![PID Video](https://youtu.be/5M15KDOCLRI)

The CTE error stabilized after several laps around the track:
```
CTE: 0.8503 Steering Value: -0.07513
42["steer",{"steering_angle":-0.07513,"throttle":0.3}]
CTE: 0.8394 Steering Value: -0.07304
42["steer",{"steering_angle":-0.0730400000000001,"throttle":0.3}]
CTE: 0.8275 Steering Value: -0.07085
42["steer",{"steering_angle":-0.07085,"throttle":0.3}]
```

## Background
The PID controller is designed to minimize the distance to this reference trajectory. The primary control output of the PID controller here is the steering angle.

## Details
CTE - Cross Track Error

the PID controller consists of 3 components:

P - proportional gain
The proportional term computes an output proportional to the cross-track error. The proportional gain contributes a control output to the steering angle of the form -K*cte with a positive constant K. By itself, the P control will cause oscillations

D - differential gain
The oscillations caused by purely D control can be mitigated by a term proportional to the derivative of the cross-track error. The derivative gain contributes a control output of the form -K*d/dt cte. 

I - integral gain
A third contribution is given by the integral gain which simply sums up the cross-track error over time. The corresponding contribution to the steering angle is given by -K*sum(cte). This works well if zero steering angle does not correspond to a straight trajectory. At high speeds this term can also be useful to accumulate a large error signal quickly. 

---

## The Udacity simulator provides the Cross Track Error. The PID controller takes in this value, calculates the PID errors/ gains, and the total error, and then responds back to the simulator with a steering angle proportional to the total error [ Total error = sum of P, I, D errors along with the gain factors]

## Code explanation

The key components that have been modified are main.cpp & PID.cpp.

## main.cpp: Initialize the gain variables. this is by far the most important step.
The gain factor determines the ultimate behavior of the PID controller.
The values were selected by trial & error. 
Initially, the Kp was set to 1, all other values set to 0. This led to stable steering in the beginning, but as soon as the first sharp corner was made, wild oscillations started as the CTE started growing and the PID controller could not adapt well given the linear nature of the Proportional correction.

With Kp set to 0.1[reduced by a factor of 10X], it took much longer for the oscillations to set it, but they did start eventually and throw the vehicle off track. So I kept the value of 0.1.

Now it was time to play with the other 2 variables. Next I changed the Integral factor to 0.1, leaving the P factor at 0.1 and the D factor set to 0. In this case, the vehicle quickly went out of control due to the accumulated errors. i reset the I gain factor to 0.0001, to reduce this effect.

Finally, I started off the D gain factor with 0.1, the vehicle performed much better making it half way through the track but then eventually went off track. I increased this factor by 10X, and then the vehicle had no problem taking multiple laps over the tracks. It seems this was the crucial factor in stabilising the steering angle to reasonably small values enough to stay on track, while not getting completely thrown off track at the sharp turns as experienced with Just the P gain factor set.



```
int main()
{
  uWS::Hub h;

  PID pid;
  // Initialize the pid variable.
  double init_Kp=0.1;
  double init_Ki=0.0001;
  double init_Kd=1;
  pid.Init(init_Kp,init_Ki,init_Kd);
  ...

```
## The next important code in main.cpp is computing the steering angle based on the cte provided by the simulator, and returning it back to the simulator.
```
pid.UpdateError(cte);
steer_value=-pid.TotalError();
```
## PID.cpp
The key code was: 
UpdateError, based on the incoming CTE values.
TotalError: addition of all the error parameters.
Note: for the D error, it's important to have a starting point CTE error.

```
void PID::UpdateError(double cte) {
  //Initial value of p_error to set u differential error d_error
  cte_prev_ = cte_;
  cte_ = cte;
  p_error = cte_;
  d_error = cte_-cte_prev_;
    i_error += cte;
    

}

double PID::TotalError() {
  return Kp_* p_error+ Ki_* i_error + Kd_* d_error;
  //return 0.5;
}
```
## Future work
1. Automate parameter settings using Twiddle, find lowest error PID gains similar to Udacity lessons
```
def twiddle(tol=0.2): 
    p = [0, 0, 0]
    dp = [1, 1, 1]
    robot = make_robot()
    x_trajectory, y_trajectory, best_err = run(robot, p)

    it = 0
    while sum(dp) > tol:
        print("Iteration {}, best error = {}".format(it, best_err))
        for i in range(len(p)):
            p[i] += dp[i]
            robot = make_robot()
            x_trajectory, y_trajectory, err = run(robot, p)

            if err < best_err:
                best_err = err
                dp[i] *= 1.1
            else:
                p[i] -= 2 * dp[i]
                robot = make_robot()
                x_trajectory, y_trajectory, err = run(robot, p)

                if err < best_err:
                    best_err = err
                    dp[i] *= 1.1
                else:
                    p[i] += dp[i]
                    dp[i] *= 0.9
        it += 1
    return p
```
2. Implement PID for throttle mechanism along the lines of:

```
throttle_value = Bias - pid.Kp * pid.p_error
                        - pid.Kd * pid.d_error 
                        - pid.Ki * pid.i_error;
```

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
  * Run either `./install-mac.sh` or `./install-ubuntu.sh`.
  * If you install from source, checkout to commit `e94b6e1`, i.e.
    ```
    git clone https://github.com/uWebSockets/uWebSockets 
    cd uWebSockets
    git checkout e94b6e1
    ```
    Some function signatures have changed in v0.14.x. See [this PR](https://github.com/udacity/CarND-MPC-Project/pull/3) for more details.
* Simulator. You can download these from the [project intro page](https://github.com/udacity/self-driving-car-sim/releases) in the classroom.

There's an experimental patch for windows in this [PR](https://github.com/udacity/CarND-PID-Control-Project/pull/3)

## Basic Build Instructions

1. Clone this repo.
2. Make a build directory: `mkdir build && cd build`
3. Compile: `cmake .. && make`
4. Run it: `./pid`. 

Tips for setting up your environment can be found [here](https://classroom.udacity.com/nanodegrees/nd013/parts/40f38239-66b6-46ec-ae68-03afd8a601c8/modules/0949fca6-b379-42af-a919-ee50aa304e6a/lessons/f758c44c-5e40-4e01-93b5-1a82aa4e044f/concepts/23d376c7-0195-4276-bdf0-e02f1f3c665d)

## Editor Settings

We've purposefully kept editor configuration files out of this repo in order to
keep it as simple and environment agnostic as possible. However, we recommend
using the following settings:

* indent using spaces
* set tab width to 2 spaces (keeps the matrices in source code aligned)

## Code Style

Please (do your best to) stick to [Google's C++ style guide](https://google.github.io/styleguide/cppguide.html).

## Project Instructions and Rubric

Note: regardless of the changes you make, your project must be buildable using
cmake and make!

More information is only accessible by people who are already enrolled in Term 2
of CarND. If you are enrolled, see [the project page](https://classroom.udacity.com/nanodegrees/nd013/parts/40f38239-66b6-46ec-ae68-03afd8a601c8/modules/f1820894-8322-4bb3-81aa-b26b3c6dcbaf/lessons/e8235395-22dd-4b87-88e0-d108c5e5bbf4/concepts/6a4d8d42-6a04-4aa6-b284-1697c0fd6562)
for instructions and the project rubric.

## Hints!

* You don't have to follow this directory structure, but if you do, your work
  will span all of the .cpp files here. Keep an eye out for TODOs.

## Call for IDE Profiles Pull Requests

Help your fellow students!

We decided to create Makefiles with cmake to keep this project as platform
agnostic as possible. Similarly, we omitted IDE profiles in order to we ensure
that students don't feel pressured to use one IDE or another.

However! I'd love to help people get up and running with their IDEs of choice.
If you've created a profile for an IDE that you think other students would
appreciate, we'd love to have you add the requisite profile files and
instructions to ide_profiles/. For example if you wanted to add a VS Code
profile, you'd add:

* /ide_profiles/vscode/.vscode
* /ide_profiles/vscode/README.md

The README should explain what the profile does, how to take advantage of it,
and how to install it.

Frankly, I've never been involved in a project with multiple IDE profiles
before. I believe the best way to handle this would be to keep them out of the
repo root to avoid clutter. My expectation is that most profiles will include
instructions to copy files to a new location to get picked up by the IDE, but
that's just a guess.

One last note here: regardless of the IDE used, every submitted project must
still be compilable with cmake and make./

## How to write a README
A well written README file can enhance your project and portfolio.  Develop your abilities to create professional README files by completing [this free course](https://www.udacity.com/course/writing-readmes--ud777).

