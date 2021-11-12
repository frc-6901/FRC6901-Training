# Project 2

In this project, you will build upon the first project by making the romi have different speeds to its drive. Pressing different buttons on the Xbox Controller can switch between normal, fast, and slow speeds. This is important in regular robotics as the driver often needs to switch between different speeds to be optimized for various conditions. For example, if the robot is in slow mode, its forward speed and turn speed from the controller could be multiplied by 0.5 so that the driver has more control over the robot. 

Ideally, you'll accomplish this through adding a basic state machine (see below) to the RomiDrivetrain class and add code in the teleop periodic method of Robot.java to bind the Xbox Buttons to different states.

### State Machine
A state machine effectively models a program with a finite number of states and it manages the actions at each state and how the program transitions between states. For example, a rocket has a finite number of states such as Lift Off, stage separation, etc. and it generally uses a state machine that manages what the rocket does at each of those states and what conditions are necessary to go to the next state.

The way you can implement a basic state machine is through the use of [Java Enums](https://www.w3schools.com/java/java_enums.asp) and the [switch statements](https://docs.oracle.com/javase/tutorial/java/nutsandbolts/switch.html). Here's a demo example:
```java
    public enum RocketState {
        IGNITION,
        LIFTOFF,
        MECO
    }
    
    public void processState(RocketState state) {
        switch (state) {
            case IGNITION:
                System.out.println("The Rocket has reached Ignition");
                break;
            case LIFTOFF:
                System.out.println("The Rocket has Lifted Off");
                break;
            case MECO:
                System.out.println("The Rocket has reached MECO");
                break;
        }
    }
```

The enum RocketState stores the different states the rocket could be in. Then, `processState` determines which code should be run based off of the state.

Similarly, you'll have to implement something similar for the three different states of the robot: Fast, Normal, and Slow.

### Possible Approach
The following is a possible approach to solving this.
1. Modify the `RomiDrivetrain` class to have an enum representing the three states of the robot. 
2. Store the current state of the Drivetrain as an instance variable
3. Add a [setter method](https://www.w3schools.com/java/java_encapsulation.asp) to set the state of the drivetrain.
4. Modify the `arcadeDrive` method to run at different speeds based off of the current stored state
5. Modify `Robot.java` to set the drivetrain state when different buttons are pressed.
