package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.hardware.sparkfun.SparkFunOTOS;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.RobotLog;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

@Autonomous(name="Auto Drive", group="Robot")
public class OTOSAutoDrive extends LinearOpMode {
    // Initialize all variables for the program
    // Hardware variables
    SparkFunOTOS myOtos;
    double xLoc = 0;
    double yLoc = 0;
    double hLoc = 0;
    private DcMotor leftFrontDrive = null;
    private DcMotor rightFrontDrive = null;
    private DcMotor leftBackDrive = null;
    private DcMotor rightBackDrive = null;
    private double leftFrontPower = 0;
    private double rightFrontPower = 0;
    private double leftBackPower = 0;
    private double rightBackPower = 0;
    double max = 0;

    // This chunk controls our vertical
    DcMotor vertical = null;
    final int VERTICAL_MIN = 0;
    final int VERTICAL_MAX = 1700;
    final int VERTICAL_DEFAULT_SPEED = 2000;

    // This chunk controls our viper slide
    DcMotor viperSlide = null;
    final int VIPER_MAX = 3100;
    final int VIPER_MIN = 0;

    // This chunk controls our claw
    Servo claw = null;
    final double CLAW_MIN = 0.9;    // Claw is closed
    final double CLAW_MAX = 0.7;    // Claw is open

    @Override
    public void runOpMode() {
        // Define all the hardware
        myOtos = hardwareMap.get(SparkFunOTOS.class, "OTOS");
        SparkFunOTOS.Pose2D pos = myOtos.getPosition();
        leftFrontDrive = hardwareMap.get(DcMotor.class, "left_front_drive");
        leftBackDrive = hardwareMap.get(DcMotor.class, "left_back_drive");
        rightFrontDrive = hardwareMap.get(DcMotor.class, "right_front_drive");
        rightBackDrive = hardwareMap.get(DcMotor.class, "right_back_drive");
        leftFrontDrive.setDirection(DcMotor.Direction.REVERSE);
        leftBackDrive.setDirection(DcMotor.Direction.REVERSE);
        rightFrontDrive.setDirection(DcMotor.Direction.FORWARD);
        rightBackDrive.setDirection(DcMotor.Direction.FORWARD);

        vertical = hardwareMap.get(DcMotor.class, "vertical");
        vertical.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        vertical.setTargetPosition(0);
        vertical.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        viperSlide = hardwareMap.get(DcMotor.class, "viper_slide");
        viperSlide.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        viperSlide.setDirection(DcMotor.Direction.REVERSE);
        viperSlide.setTargetPosition(0);
        viperSlide.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        claw = hardwareMap.get(Servo.class, "claw");
        claw.setPosition(CLAW_MIN);

        RevHubOrientationOnRobot.LogoFacingDirection logoDirection = RevHubOrientationOnRobot.LogoFacingDirection.UP;
        RevHubOrientationOnRobot.UsbFacingDirection  usbDirection  = RevHubOrientationOnRobot.UsbFacingDirection.FORWARD;
        RevHubOrientationOnRobot orientationOnRobot = new RevHubOrientationOnRobot(logoDirection, usbDirection);

        // Send telemetry message to signify robot waiting;
        telemetry.addData("Ready", "You can press start"); // todo: Add "Autonomous" before ready so it is super clear to the driver
        telemetry.update();

        configureOtos();

        // Wait for the game to start (driver presses PLAY)
        waitForStart();

        // Code here
        driveToLoc(0, 0, -90);
        driveToLoc(25, 25, -90);

        // End of autonomous program
        telemetry.addData("Autonomous", "Complete");
        telemetry.update();
    }

    public void setViper(int length){
        viperSlide.setTargetPosition(length);
        ((DcMotorEx) viperSlide).setVelocity(2000);
        viperSlide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        RobotLog.vv("Rockin' Robots", "Viper set to %d", viperSlide.getCurrentPosition());
    }

    public void setVertical(int height){
        setVertical(height, VERTICAL_DEFAULT_SPEED);
    }

    public void setVertical(int height, int speed){
        vertical.setTargetPosition(height);
        ((DcMotorEx) vertical).setVelocity(speed);
        vertical.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    }

    public void setClaw(double position){
        claw.setPosition(position);
        sleep(300);
    }

    private void stopMoving() {
        leftFrontDrive.setPower(0);
        rightFrontDrive.setPower(0);
        leftBackDrive.setPower(0);
        rightBackDrive.setPower(0);
    }

    private void configureOtos() {
        myOtos.setLinearUnit(DistanceUnit.INCH);
        myOtos.setAngularUnit(AngleUnit.DEGREES);
        SparkFunOTOS.Pose2D offset = new SparkFunOTOS.Pose2D(-3.9, 1.61, 90); // todo: do a test to improve these numbers
        // -3.4, 1.5: xLoc: 1.13, yLoc: -0.34, hLoc: 179.45
        // -3.6, 1.6: xLoc: 0.43, yLoc: 0.05, hLoc: 179.37
        // -3.9, 1.61: xLoc: -0.12, yLoc: 0.04, hLoc: 179.19
        myOtos.setOffset(offset);
        myOtos.setLinearScalar(1.0);
        myOtos.setAngularScalar(1.0);
        myOtos.calibrateImu();
        myOtos.resetTracking();
        SparkFunOTOS.Pose2D currentPosition = new SparkFunOTOS.Pose2D(0, 0, 0);
        myOtos.setPosition(currentPosition);
        SparkFunOTOS.Version hwVersion = new SparkFunOTOS.Version();
        SparkFunOTOS.Version fwVersion = new SparkFunOTOS.Version();
        myOtos.getVersionInfo(hwVersion, fwVersion);
    }
    private void getPosition() {
        SparkFunOTOS.Pose2D pos = myOtos.getPosition();
        xLoc = pos.x;
        yLoc = pos.y;
        hLoc = pos.h;
    }
    private void driveToLoc(double xTarget, double yTarget, double hTarget) {
        driveToLoc(xTarget, yTarget, hTarget, 2);
    }
    private void driveToLoc(double xTarget, double yTarget, double hTarget, double accuracy) {
        getPosition();
        double xDistance = xTarget - xLoc;
        double yDistance = yTarget - yLoc;
        double hDistance = hTarget - hLoc;
        if(hDistance < -180) hDistance += 360;
        if(hDistance > 180) hDistance -= 360;

        RobotLog.vv("Rockin' Robots", "xDistance: %.2f, yDistance: %.2f, hDistance: %.2f, " +
                        "leftFrontPower: %.2f, rightFrontPower: %.2f, leftBackPower: %.2f, rightBackPower: %.2f",
                xDistance, yDistance, hDistance, leftFrontPower, rightFrontPower, leftBackPower, rightBackPower);

        while (opModeIsActive()
                && (Math.abs(xDistance) > accuracy
                || Math.abs(yDistance) > accuracy
                || Math.abs(hDistance) > accuracy)) {

            double headingAdjustment = (180 - (180 - hLoc))*.1;
            // Combine the joystick requests for each axis-motion to determine each wheel's power.
            // Set up a variable for each drive wheel to save the power.
            leftFrontPower = (yDistance + xDistance - hDistance) / 8;
            rightFrontPower = (yDistance - xDistance + hDistance) / 8;
            leftBackPower = (yDistance - xDistance - hDistance) / 8;
            rightBackPower = (yDistance + xDistance + hDistance) / 8;

            // Normalize the values so no wheel power exceeds 100%
            max = Math.max(Math.abs(leftFrontPower), Math.abs(rightFrontPower));
            max = Math.max(max, Math.abs(leftBackPower));
            max = Math.max(max, Math.abs(rightBackPower));
            if (max > 1.0) {
                leftFrontPower /= max;
                rightFrontPower /= max;
                leftBackPower /= max;
                rightBackPower /= max;
            }
            leftFrontPower *= 0.5;
            rightFrontPower *= 0.5;
            leftBackPower *= 0.5;
            rightBackPower *= 0.5;

            leftFrontDrive.setPower(leftFrontPower);
            rightFrontDrive.setPower(rightFrontPower);
            leftBackDrive.setPower(leftBackPower);
            rightBackDrive.setPower(rightBackPower);
            RobotLog.vv("Rockin' Robots", "xDist: %.2f, yDist: %.2f, hDist: %.2f, " +
                    "leftFrontPower: %.2f, rightFrontPower: %.2f, leftBackPower: %.2f, rightBackPower: %.2f",
                    xDistance, yDistance, hDistance, leftFrontPower, rightFrontPower, leftBackPower, rightBackPower);

            getPosition();
            xDistance = xTarget - xLoc;
            yDistance = yTarget - yLoc;
            hDistance = hTarget - hLoc;
        }
        stopMoving();
        RobotLog.vv("Rockin' Robots", "Done Moving: xDist: %.2f, yDist: %.2f, hDist: %.2f",
                xDistance, yDistance, hDistance);
    }
}