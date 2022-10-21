package org.firstinspires.ftc.teamcode.Classes;

abstract class DrivetrainClass {

    abstract void initialize();

    abstract void moveForward(int time, double speed) throws InterruptedException;

    abstract void moveBackward(int time, double speed) throws InterruptedException;

    abstract void moveRight(int time, double speed) throws InterruptedException;

    abstract void moveLeft(int time, double speed) throws InterruptedException;

    abstract void turnLeft(int time, double speed) throws InterruptedException;

    abstract void turnRight(int time, double speed) throws InterruptedException;
}
