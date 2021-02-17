package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

@Autonomous(name="testGyro")
public class TestGyro extends LinearOpMode {

    RobotClass robot;


    @Override
    public void runOpMode() throws InterruptedException {

        robot= new RobotClass(hardwareMap, telemetry, this);

        waitForStart();

        robot.testGyro();
//        Thread.sleep(1000);
//        robot.pivotRight(0.2, 45);
//        Thread.sleep(1000);
//        robot.pivotLeft(0.2,90);
//        Thread.sleep(1000);
//        robot.pivotRight(0.2, 90);

    }
}
