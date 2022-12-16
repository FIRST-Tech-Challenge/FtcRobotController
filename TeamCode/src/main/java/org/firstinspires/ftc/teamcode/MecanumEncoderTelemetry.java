package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

// The purpose of this file is to show a mechanum drive train's motor encoders.

@Autonomous(name="MecanumEncoderTelemetry", group="Linear Opmode")
public class MecanumEncoderTelemetry extends LinearOpMode {

    DcMotor leftFrontMotor = null;
    DcMotor rightFrontMotor = null;
    DcMotor leftBackMotor = null;
    DcMotor rightBackMotor = null;

    @Override
    public void runOpMode() {

        // Configure hardware maps:
        leftFrontMotor = hardwareMap.dcMotor.get("leftFront");
        rightFrontMotor = hardwareMap.dcMotor.get("rightFront");
        leftBackMotor = hardwareMap.dcMotor.get("leftBack");
        rightBackMotor = hardwareMap.dcMotor.get("rightBack");

        // Set motor directions:
        rightFrontMotor.setDirection(DcMotor.Direction.REVERSE);
        leftFrontMotor.setDirection(DcMotor.Direction.FORWARD);
        rightBackMotor.setDirection(DcMotor.Direction.REVERSE);
        leftBackMotor.setDirection(DcMotor.Direction.FORWARD);

        // Set the drive motor run modes:
        leftFrontMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightFrontMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftBackMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightBackMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        
        waitForStart();
        // Repeats telemetry:
        while (opModeIsActive()) {
            telemetry.addData("Encoder Position", "%7d :%7d", leftFrontMotor.getCurrentPosition(),
                    rightFrontMotor.getCurrentPosition(), leftBackMotor.getCurrentPosition(),
                    rightBackMotor.getCurrentPosition());
            telemetry.update();
        }
    }
}
