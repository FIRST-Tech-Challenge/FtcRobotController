package org.firstinspires.ftc.teamcode.toolkit.background;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.UpliftRobot;


public abstract class Background implements Runnable{
    Thread t;
    UpliftRobot robot;
    LinearOpMode opMode;

    public Background(UpliftRobot robot) {
        this.robot = robot;
        this.opMode = robot.opMode;
    }

    public void enable() {
        t.start();
    }

    public abstract void loop();

    public void stop() {
        t = null;
    }

    @Override
    public void run() {
        opMode.waitForStart();
        while(t != null && opMode.opModeIsActive() && !opMode.isStopRequested()) {
            loop();
        }
        stop();
    }
}
