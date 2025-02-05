package org.firstinspires.ftc.teamcode.opmodes.test.auto;

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.roadrunner.teamcode.MecanumDrive;

@Autonomous(name = "BluePickSampleDropInHighBasketAutoPath", group = "Blue Alliance" , preselectTeleOp = "RobotCentricDrive")
@Disabled
public class BluePickSampleDropInHighBasketAutoPath extends LinearOpMode {
    @Override
    public void runOpMode() {
        // Initialize the MecanumDrive with the hardware map
        MecanumDrive drive = new MecanumDrive(hardwareMap, new Pose2d(-24, 60, Math.toRadians(-90)));

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
                    drive.actionBuilder(new Pose2d(24, 60, Math.toRadians(-90)))
                            .waitSeconds(1)
                            .splineToLinearHeading(new Pose2d(36, 24, Math.toRadians(0)), -90)
                            .waitSeconds(1)
                            .strafeTo(new Vector2d(36, 24))
                            .turnTo(Math.toRadians(45))
                            .waitSeconds(1)
                            .strafeTo(new Vector2d(48, 48))
                            .turnTo(Math.toRadians(0))
                            .waitSeconds(1)
                            .strafeTo(new Vector2d(48, 24))
                            .turnTo(Math.toRadians(45))
                            .strafeTo(new Vector2d(48, 48))
                            .turnTo(Math.toRadians(0))
                            .waitSeconds(1)
                            .strafeTo(new Vector2d(56, 24))
                            .waitSeconds(1)
                            .strafeTo(new Vector2d(48, 48))
                            .turnTo(Math.toRadians(45))
                            .waitSeconds(1)
                            .build()


//                    new Pose2d(-24, 58, Math.toRadians(-90)))
//                            .strafeTo(new Vector2d(-36,10))
//                            .strafeTo(new Vector2d(-48,57))
////                            .strafeTo(new Vector2d(-47,10))
////                            .strafeTo(new Vector2d(-60,57))
////                            .strafeTo(new Vector2d(-55,10))
////                            .strafeTo(new Vector2d(-61,57))
//                            .build()
            );
        }
    }
}