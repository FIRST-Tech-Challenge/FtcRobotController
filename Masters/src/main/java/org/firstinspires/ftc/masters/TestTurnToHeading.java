package org.firstinspires.ftc.masters;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp
public class TestTurnToHeading extends LinearOpMode {
    RobotClass robot;

    @Override
    public void runOpMode() throws InterruptedException {
        telemetry.addData("Turning to ", 90);
        robot.turnToHeading(.5,90,2);
        robot.pauseButInSecondsForThePlebeians(1);
        telemetry.addData("Turning to ", 180);
        robot.turnToHeading(.5,180,1);
        robot.pauseButInSecondsForThePlebeians(1);
        telemetry.addData("Turning to ", 90);
        robot.turnToHeading(.5,90,3);
        robot.pauseButInSecondsForThePlebeians(1);
        telemetry.addData("Turning to ", 270);
        robot.turnToHeading(.5,270,4);
        robot.pauseButInSecondsForThePlebeians(10);
    }
}
