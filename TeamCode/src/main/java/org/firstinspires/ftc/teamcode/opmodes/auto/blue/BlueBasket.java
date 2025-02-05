package org.firstinspires.ftc.teamcode.opmodes.auto.blue;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

// RR-specific imports
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.Servo;

// Non-RR imports
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.roadrunner.teamcode.MecanumDrive;

@Autonomous(name = "BlueBasket", group = "Blue Alliance",preselectTeleOp = "RobotCentricDrive")
@Disabled
public class BlueBasket extends LinearOpMode {
    @Override
    public void runOpMode() {
        // Initialize the MecanumDrive with the hardware map
        MecanumDrive drive = new MecanumDrive(hardwareMap, new Pose2d(10, 62, Math.toRadians(-90)));

        DcMotorEx linearSlideMotor = (DcMotorEx)hardwareMap.get(DcMotor.class, "armMotor");

        DcMotor armMotor = hardwareMap.dcMotor.get("liftMotor");
        CRServo clawServo = hardwareMap.get(CRServo.class, "claw");
        Servo wristServo = hardwareMap.get(Servo.class, "wrist");
        DcMotor HangMotor1 = hardwareMap.dcMotor.get("HM1");
        DcMotor HangMotor2 = hardwareMap.dcMotor.get("HM2");


        DcMotor frontLeftMotor = hardwareMap.dcMotor.get("frontLeftMotor");
        DcMotor backLeftMotor = hardwareMap.dcMotor.get("backLeftMotor");
        DcMotor frontRightMotor = hardwareMap.dcMotor.get("frontRightMotor");
        DcMotor backRightMotor = hardwareMap.dcMotor.get("backRightMotor");

        // Reverse the right side motors. This may be wrong for your setup.
        // If your robot moves backwards when commanded to go forwards,
        // reverse the left side instead.
        // See the note about this earlier on this page.
        frontLeftMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        backLeftMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        //backRightMotor.setDirection(DcMotorSimple.Direction.REVERSE);
//        frontRightMotor.setDirection(DcMotorSimple.Direction.REVERSE);
//        backRightMotor.setDirection(DcMotorSimple.Direction.REVERSE);


        if (isStopRequested()) return;

        // Initialize and configure IMU
        IMU imu = hardwareMap.get(IMU.class, "imu");
        IMU.Parameters imuParameters = new IMU.Parameters(new RevHubOrientationOnRobot(
                RevHubOrientationOnRobot.LogoFacingDirection.UP,
                RevHubOrientationOnRobot.UsbFacingDirection.LEFT));
        imu.initialize(imuParameters);

        // Set zero power behavior
        linearSlideMotor.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        linearSlideMotor.setPositionPIDFCoefficients(10.0);
        armMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        HangMotor1.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        HangMotor2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        // Wait for the start signal
        waitForStart();



        if (opModeIsActive()) {
            // Define the trajectory for the Blue Basket sequence with waits
            Actions.runBlocking(
                    drive.actionBuilder(new Pose2d(10, 62, Math.toRadians(-90)))


                             .splineTo(new Vector2d(52, 57), Math.toRadians(45))  // First drop point
                            .waitSeconds(0.5)
                            .splineTo(new Vector2d(47, 37), Math.toRadians(-90)) // Return to collect
                            .waitSeconds(0.5)
                            .splineTo(new Vector2d(52, 57), Math.toRadians(45))  // Second drop point
                            .waitSeconds(0.5)
                            .splineTo(new Vector2d(57, 37), Math.toRadians(-90)) // Return to collect
                            .waitSeconds(0.5)
                            .splineTo(new Vector2d(52, 57), Math.toRadians(45))  // Third drop point
                            .waitSeconds(0.5)
                            .splineTo(new Vector2d(57, 25), Math.toRadians(0)) // Return to collect
                            .waitSeconds(0.5)
                            .splineTo(new Vector2d(52, 57), Math.toRadians(45))  // Fourth drop point
                            .waitSeconds(0.5)
                            .splineTo(new Vector2d(35, 15), Math.toRadians(180)) // Park
                            .waitSeconds(0.5)
                            .build()
            );
        }
    }
}