package org.firstinspires.ftc.teamcode.Classes;

//abstract class because every drivetrain has these fixed methods
abstract class DrivetrainClass {

    // defines all of the standard methods for drivetrains

    abstract void initialize();

    // parameters include how long to move and the speed at which to move
    abstract void moveForward(int time, double speed) throws InterruptedException;

    abstract void moveBackward(int time, double speed) throws InterruptedException;

    abstract void moveRight(int time, double speed) throws InterruptedException;

    abstract void moveLeft(int time, double speed) throws InterruptedException;

    abstract void turnLeft(int time, double speed) throws InterruptedException;

    abstract void turnRight(int time, double speed) throws InterruptedException;
}
