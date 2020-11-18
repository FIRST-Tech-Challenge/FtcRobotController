package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

@Autonomous(name="testAuto")
public class OlliesFirstAutonomous extends LinearOpMode {

    RobotClass robot;

    @Override
    public void runOpMode(){

        robot= new RobotClass(hardwareMap, telemetry, this);

        waitForStart();

        robot.forward(0.1,5);
//        robot.turnLeft(.5,90);
//        robot.strafeRight(.7,4);
    }
}