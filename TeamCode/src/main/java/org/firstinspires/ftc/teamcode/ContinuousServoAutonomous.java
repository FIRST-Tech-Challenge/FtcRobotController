package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

@Autonomous(name="testAuto")
public class ContinuousServoAutonomous extends LinearOpMode {

    RobotClass robot;

    @Override
    public void runOpMode() {
        robot= new RobotClass(hardwareMap, telemetry, this);

        waitForStart();
        robot.testServo1(.9,500);

    }
}