# Using Motors

**Note: Anything that looks like ~~this~~ is deprecated.**

Motors on the robot can be referenced with the `Motor` class. They can be controlled in autonomous OpModes by themselves ~~or with the SimpleMotorController and ComplexMotorController control planes. Keep in mind the SimpleMotorController and ComplexMotorControllers are both prototypes and will probably change later.~~

**Another Note: Everything below this is deprecated but i cant be asked to wrap every line with tildes**

The simple control plane is best used for moving grouped motors at once as the simple control plane can only run one motor or motor group at a time, and must let the current group stop before running the next group.

The complex control plane is best used for moving motors dynamically on the same thread, as each motor is controlled fully by the programmer rather than the simple control plane.

### Simple Control Plane
The simple control plane is the easiest and highest-level control plane to use. It runs a motor to a specific distance at a specific speed.

To use motors with the simple control plane, first make a new `Motor`:
```java
Motor motor = new Motor(...);
```
Then, make a new `SimpleMotorController`:
```java
SimpleMotorController simpleMotorController = new SimpleMotorController(telemetry);
```
You can control a single motor with the control plane:
```java
simpleMotorController.moveSingleMotor(...);
```
And multiple motors in the same direction:
```java
simpleMotorController.moveGroupedMotors(...);
```
And multiple motors in different directions:
```java
simpleMotorController.moveMultipleMotors();
```
Once time is up, you can kill the motor(s) currently running:
```java
simpleMotorController.killMotor(motor);
simpleMotorController.killMotors(motors);
```

### Complex Control Plane
The complex control plane is more complicated to use but allows for lower-level control over motors. You need to prepare, run, and stop motors manually.

To use motors with the complex control plane, first make a new `Motor`:
```java
Motor motor = new Motor(...);
```
Then make a new `ComplexMotorController`:
```java
ComplexMotorController complexMotorController = new ComplexMotorController(telemetry);
```
You can prepare a motor:
```java
complexMotorController.prepMotor(motor, distance);
```
And then start it:
```java
complexMotorController.startMotor(motor, speed);
```
And finally stop it:
```java
complexMotorController.stopMotor(motor);
```

**Note: Reverse movement requires a negative direction, not speed**

Other note: The original TeamCode readme can be found [here](https://github.com/FIRST-Tech-Challenge/FtcRobotController/blob/master/TeamCode/src/main/java/org/firstinspires/ftc/teamcode/readme.md)
