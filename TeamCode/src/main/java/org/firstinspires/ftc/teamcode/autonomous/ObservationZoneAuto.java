package org.firstinspires.ftc.teamcode.autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

@Autonomous
public class ObservationZoneAuto extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        // Declare our motors
        // Make sure your ID's match your configuration
        DcMotor frontLeftMotor = hardwareMap.dcMotor.get("frontLeftMotor");
        DcMotor backLeftMotor = hardwareMap.dcMotor.get("backLeftMotor");
        DcMotor frontRightMotor = hardwareMap.dcMotor.get("frontRightMotor");
        DcMotor backRightMotor = hardwareMap.dcMotor.get("backRightMotor");
        DcMotor rightSlideMotor = hardwareMap.get(DcMotor.class, "rightSlideMotor");
        Servo rightWristServo = hardwareMap.get(Servo.class, "rightWristServo");
        Servo specServo = hardwareMap.get(Servo.class, "specServo");
        CRServo activeIntake = hardwareMap.get(CRServo.class, "activeIntake");
        DcMotor intakeArmMotor = hardwareMap.get(DcMotor.class, "intakeArmMotor");
        Servo bucketServo = hardwareMap.get(Servo.class, "bucketServo");
        intakeArmMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        intakeArmMotor.setTargetPosition(0);
        intakeArmMotor.setPower(0.3);
        telemetry.addLine("Start");
        intakeArmMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        backRightMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        waitForStart();
        // Run the code for otos
        frontLeftMotor.setPower(-0.5);
        frontRightMotor.setPower(-0.5);
        backRightMotor.setPower(-0.5);
        backLeftMotor.setPower(-0.5);
        sleep(4000);
        frontLeftMotor.setPower(0);
        frontRightMotor.setPower(0);
        backRightMotor.setPower(0);
        backLeftMotor.setPower(0);
    }
}