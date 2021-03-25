package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;


@Autonomous(name="testShooterLoading")
public class TestShooterLoading extends LinearOpMode{
    RobotClass robot;

    @Override
    public void runOpMode() throws InterruptedException {
        robot= new RobotClass(hardwareMap, telemetry, this);

        waitForStart();

//        robot.shooterServo1(.7);
//        robot.shooterServo1(.7);

    }
}

