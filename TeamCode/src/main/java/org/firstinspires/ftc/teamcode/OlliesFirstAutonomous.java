package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

@Autonomous(name="testAuto")
public class OlliesFirstAutonomous extends LinearOpMode {

    RobotClass robot;

    @Override
    public void runOpMode() throws InterruptedException {

        robot= new RobotClass(hardwareMap, telemetry);

        waitForStart();

        robot.forward(0.5,2);
        robot.mecanumWitchcraftColor(45,2,1);
        robot.turnleft(.5,90);
        robot.strafeRight(.7,4);
    }
}