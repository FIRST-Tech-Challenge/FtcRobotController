package org.firstinspires.ftc.teamcode;

//import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.acmerobotics.roadrunner.Vector2d;
import org.firstinspires.ftc.teamcode.RRLocalizationRead;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.RoadRunner.MecanumDrive;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.acmerobotics.roadrunner.Vector2d;
import org.firstinspires.ftc.teamcode.RRLocalizationRead;

@Autonomous(name = "park auto", group = "autos")
public class parkAuto extends LinearOpMode {

    public RRLocalizationRead localizationRead;
    public DcMotor frontL;
    public DcMotor backL;
    public DcMotor frontR;
    public DcMotor backR;

    Servo inTakeClaw;
    Servo inTakeFlipExpansion;
    ElapsedTime totalTime = new ElapsedTime();



    @Override
    public void runOpMode() {
        localizationRead = new RRLocalizationRead();
        localizationRead.initLocalization(hardwareMap);
        backL = hardwareMap.get(DcMotor.class, "backL");
        frontL = hardwareMap.get(DcMotor.class, "frontL");
        frontR = hardwareMap.get(DcMotor.class, "frontR");
        backR = hardwareMap.get(DcMotor.class, "backR");

        frontL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        backL.setDirection(DcMotorSimple.Direction.REVERSE);

        inTakeFlipExpansion = hardwareMap.servo.get("itfe"); // port 1
        inTakeClaw = hardwareMap.servo.get(("itc")); // port 3

        waitForStart();

        if (opModeIsActive()) {

            moveForwardByInches(60.0, 10);  // Moves the robot forward by 10 inches

        }
    }

    public void moveForwardByInches(double inches, double timeOut) {
        // Record the initial position
        Vector2d initialPosition = localizationRead.returnPose().position;
        double targetY = initialPosition.y + inches;  // Assuming "forward" is along the y-axis
        timeOut = totalTime.seconds() + timeOut;

        // Move forward until we reach the target distance
        while (opModeIsActive() && (localizationRead.returnPose().position.y < targetY) && totalTime.seconds() <= timeOut) {
            backL.setPower(.3);// Apply forward power
            backR.setPower(.3);
            frontL.setPower(.3);// Apply forward power
            frontR.setPower(.3);
            localizationRead.returnPose();  // Update position each loop
        }

        // Stop the robot once the target position is reached
        backL.setPower(0);// Apply forward power
        backR.setPower(0);
        frontL.setPower(0);// Apply forward power
        frontR.setPower(0);
    }

    public void moveBackwardByInches(double inches, double timeOut) {
        // Record the initial position
        Vector2d initialPosition = localizationRead.returnPose().position;
        double targetY = initialPosition.y + inches;  // Assuming "forward" is along the y-axis
        timeOut = totalTime.seconds() + timeOut;

        // Move forward until we reach the target distance
        while (opModeIsActive() && (localizationRead.returnPose().position.y > targetY && totalTime.seconds() >= timeOut)) {
            backL.setPower(-.3);// Apply forward power
            backR.setPower(-.3);
            frontL.setPower(-.3);// Apply forward power
            frontR.setPower(-.3);
            localizationRead.returnPose();  // Update position each loop
        }

        // Stop the robot once the target position is reached
        backL.setPower(0);// Apply forward power
        backR.setPower(0);
        frontL.setPower(0);// Apply forward power
        frontR.setPower(0);
    }
}