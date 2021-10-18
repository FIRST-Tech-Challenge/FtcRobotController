package org.firstinspires.ftc.teamcode.driver;


import org.firstinspires.ftc.teamcode.robot.Robot;

/**
 * Represents a type of driver for a specific drive train
 * @author 22jmiller
 */
public abstract class Driver {
    public Robot robot;

    public Driver(Robot robot) {
        this.robot = robot;
    }

    public void start() {
        robot.start();
    }
    public void stop() {
        robot.stop();
    }
    public abstract void loop();
}
