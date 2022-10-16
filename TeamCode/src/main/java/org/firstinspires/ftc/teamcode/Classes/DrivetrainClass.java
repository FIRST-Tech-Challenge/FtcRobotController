package org.firstinspires.ftc.teamcode.Classes;

abstract class DrivetrainClass {

    abstract void initialize();
    abstract void moveForward(double speed, int time);
    abstract void moveBackward(double speed, int time);
    abstract void moveRight(double speed, int time);
    abstract void moveLeft(double speed, int time);

    abstract void turnLeft(double speed, int time);
    abstract void turnRight(double speed, int time);


}
