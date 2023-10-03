package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

@Autonomous(name = "AutoRedFar", group = "linear autoMode")
public class AutoRedFar extends LinearOpMode {
    private ElapsedTime runtime = new ElapsedTime();
    private DcMotor leftFrontDrive = null;
    private DcMotor leftBackDrive = null;
    private DcMotor rightFrontDrive = null;
    private DcMotor rightBackDrive = null;

    @Override
    public void runOpMode(){
        leftFrontDrive  = hardwareMap.get(DcMotor.class, "frontleft");
        leftBackDrive  = hardwareMap.get(DcMotor.class, "backleft");
        rightFrontDrive = hardwareMap.get(DcMotor.class, "frontright");
        rightBackDrive = hardwareMap.get(DcMotor.class, "backright");

        leftFrontDrive.setDirection(DcMotor.Direction.FORWARD);
        leftBackDrive.setDirection(DcMotor.Direction.REVERSE);
        rightFrontDrive.setDirection(DcMotor.Direction.FORWARD);
        rightBackDrive.setDirection(DcMotor.Direction.FORWARD);

        telemetry.addData("Status", "Initialized");
        telemetry.update();

        waitForStart();
        runtime.reset();

        while (opModeIsActive() && (runtime.seconds() <= .5)){
            leftFrontDrive.setPower(0.25);
            leftBackDrive.setPower(0.25);
            rightFrontDrive.setPower(0.25);
            rightBackDrive.setPower(0.25);
        }
        sleep(500);
        runtime.reset();
        while (opModeIsActive() && (runtime.seconds() <= 3)){
            leftFrontDrive.setPower(0.5);
            leftBackDrive.setPower(-0.5);
            rightFrontDrive.setPower(-0.5);
            rightBackDrive.setPower(0.5);
        }
        runtime.reset();
        while (opModeIsActive() && (runtime.seconds() <= .75)){
            leftFrontDrive.setPower(-0.25);
            leftBackDrive.setPower(-0.25);
            rightFrontDrive.setPower(-0.25);
            rightBackDrive.setPower(-0.25);
        }
        runtime.reset();
        while (opModeIsActive() && (runtime.seconds() <= 1)){
            leftFrontDrive.setPower(0.5);
            leftBackDrive.setPower(-0.5);
            rightFrontDrive.setPower(-0.5);
            rightBackDrive.setPower(0.5);
        }
        leftFrontDrive.setPower(0);
        leftBackDrive.setPower(0);
        rightFrontDrive.setPower(0);
        rightBackDrive.setPower(0);
    }
}
