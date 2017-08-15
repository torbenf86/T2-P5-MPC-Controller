The advantage of an MPC controller over an PID controller is that the MPC controller uses a model of the system to predict the system's behavior. An optimizer is used to find the optimal inputs to minimize a defined objective function. Thus, it can react already in advance on changes. In this application, the objective function is of least squares type and consists of... 
  - trajectory variables (cross track error, epsi and velocity) and their target values (0,0, reference velocity)
  - input variables (acceleration and steering angle, punishment to minimize the use) and their target values (0,0)
  - input variables at node i (acceleration and steering angle, punishment to minimize the gap between sequential actuations) and their target values (  input variables at node i+1)

The waypoints are provided by the simulator in a global corordinate system. These are transformed to the car's coordinate system to simplify calculations, then a 3rd order polynom is then fitted through these waypoints (line 127 in main.cpp). 
The difference between the calculated trajectory and the waypoint polynom at each node leads to the calculation of the cross track error. 

The time horizon is N * dt = 0.75s and is discritized into N=15 intervals. The output of the optimization is a control vector with 15 values for the discretized time horizon for each input. Due to an actuation delay of 100ms = 2 * dt, only the third element of the vector is used as control input. Due to mismatches between model and reality (in this case, a simulator) the optimization is repeated with new measurement data of all state variables. These measurements are provided by the simulator.

The state variables are free variables and part of the optimization, but are constrained by the model equations (see Multiple Shooting vs. Single Shooting, where the model equations are part of the objective function). At each node there are matching conditions for the state variables to ensure a continuous trajectory. The used optimizer is IPOPT in combination with cppAD, which is able to supply analytic derivatives.


An alternative approach would be to model the system as an differential-algebraic system (DAE) for example in the modelling language Modelica. The model can be exported as Functional Mockup Unit (FMU) and then interfaced by the C++-code. The advantage is, that complex models can be developed in an modelling environment and variable step solvers can be included to solve the system.

Example:
```modelica
model kinematicModel
  Real x(start=-1),y(start=10),psi(start=0),v(start=10),cte,epsi;
  constant Real Lf = 2.67;
  input Real a, delta;
initial equation 
  cte = -1 -y;
  epsi = psi - atan(0);
equation 

      der(x) =  v * cos(psi);
      der(y) = v * sin(psi);
      der(psi) =  v / Lf * delta;
      der(v) = a;
      der(cte) = - ((-1 - y) + (v * sin(epsi)));
      der(epsi) = - ((psi - 0) + v * delta / Lf);

end kinematicModel;
```
