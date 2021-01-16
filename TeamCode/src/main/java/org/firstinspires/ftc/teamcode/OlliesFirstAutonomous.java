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

        robot.pivotLeft(.7,90);
        telemetry.addData("pivot left complete",0);
        telemetry.update();
        robot.pivotRight(.7,90);
        telemetry.addData("pivot right complete",0);
        telemetry.update();
        robot.strafeRight(.7,5);
        telemetry.addData("strafe right complete",0);
        telemetry.update();
        robot.mecanumWitchcraft(76,2);
        telemetry.addData("mecanum stuff complete",0);
        telemetry.update();


    }
}