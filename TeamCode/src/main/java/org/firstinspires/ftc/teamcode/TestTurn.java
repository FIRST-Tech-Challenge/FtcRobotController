package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

@Autonomous(name="testTurn")
public class TestTurn extends LinearOpMode {

    RobotClass robot;


    @Override
    public void runOpMode() throws InterruptedException {

        robot= new RobotClass(hardwareMap, telemetry, this);

        waitForStart();

        robot.pivotLeftSloppy(0.4,45);
        Thread.sleep(1000);
        robot.pivotRightSloppy(0.4, 45);
        Thread.sleep(1000);
        robot.pivotLeft(0.4,90);
        Thread.sleep(1000);
        robot.pivotRight(0.4, 90);

    }
}
