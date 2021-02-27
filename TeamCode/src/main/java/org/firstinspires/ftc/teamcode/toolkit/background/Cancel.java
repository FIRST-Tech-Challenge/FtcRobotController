package org.firstinspires.ftc.teamcode.toolkit.background;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.UpliftRobot;

public class Cancel implements Runnable{

    Thread t;
    UpliftRobot robot;
    LinearOpMode opMode;

    public Cancel(UpliftRobot robot) {
        t = new Thread(this);
        this.robot = robot;
        this.opMode = robot.opMode;
        t.start();
    }

    @Override
    public void run() {
        opMode.waitForStart();
        while(t != null && opMode.opModeIsActive() && !opMode.isStopRequested()) {
            robot.driverCancel = robot.opMode.gamepad1.dpad_left;
            robot.operatorCancel = robot.opMode.gamepad2.dpad_left;
        }
        robot.driverCancel = true;
        robot.operatorCancel = true;
    }

    public void stopUpdatingCancel() {
        t = null;
    }
}
