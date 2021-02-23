package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
@Autonomous(name="strafeTest")
public class TestStrafe extends LinearOpMode {

    RobotClass robot;


    @Override
    public void runOpMode() throws InterruptedException {

        robot= new RobotClass(hardwareMap, telemetry, this);

        waitForStart();

        robot.strafeLeft(0.5,2);
        robot.strafeRight(0.5, 2);

    }
}
