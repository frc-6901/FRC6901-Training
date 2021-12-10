# Project 5 - Closed Loop Control

## Context
Now that you've gotten familiar with programming robots, we'll look into programming them to move accurately and autonomously based off of sensor input (closed loop control). We'll learn about a very foundational and commonly used technique called [PID or Proportional, Integral, Derivative](https://en.wikipedia.org/wiki/PID_controller). To learn more and understand PID and the logic behind it look at the resource videos below.

## Videos
- [6901 Basic Controls Video](https://youtu.be/cJW8OJNd2R4?t=644) (start at 10:44). This starts off at the Bang Bang controller (a more basic type of controller) to contextualize what PID control improves over it. This explains it in an FRC context.
- [PID Control Video](https://youtu.be/UR0hOmjaHp0). This provides a broader and more general definition of PID.
- [PID Examples Video](https://www.youtube.com/watch?v=XfAt6hNV8XM&ab_channel=BrianDouglas). This explains PID with the example of a car.


## Useful Hints
You'll be taking advantage of the [PID Command](https://docs.wpilib.org/en/stable/docs/software/commandbased/pid-subsystems-commands.html#pidcommand) to implement this. Do note the following terminology used for the PIDCommand parameters:

- DoubleSupplier: A function that returns a double
- DoubleConsumer: A function that takes in a double
Hint: You can use `::` to return a object's function. For example `m_romiDrivetrain::arcadeDrive` returns the `arcadeDrive` function of the romi drivetrain.

### Tuning The Loop
You'll have to find PID, PD, or P constants that will create accurate control (can set terms you're not using to zero). You will have to use trial and error to find the ideal constants. For example for a PD controller you can:
1. Keep increasing the P constant until there's no steady state error.
2. Start increasing the D constant until there's no more overshoot.  

## Links
- [PID Control in WPILib](https://docs.wpilib.org/en/stable/docs/software/commandbased/pid-subsystems-commands.html): Contains information on the PID API that is available.
- [Gyro PID Example](https://github.com/wpilibsuite/allwpilib/tree/main/wpilibjExamples/src/main/java/edu/wpi/first/wpilibj/examples/gyrodrivecommands): Setups up a Gyro based PID controller.
