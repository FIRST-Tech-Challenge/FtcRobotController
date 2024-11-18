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
// todo: you have 5 warnings and 11 typos. Please fix.
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
    final int VIPER_MIN = 0; // todo: MIN goes before MAX

    // This chunk controls our claw
    Servo claw = null;
    final double CLAW_MIN = 0.00;    // Claw is closed
    final double CLAW_MAX = 0.16;    // Claw is open

    Servo ascentStick = null;
    final double ASCENT_MIN = 0.2;          // Stick is down
    final double ASCENT_MAX = 0.49;         // Stick is up
    @Override // todo: new line before this
    public void runOpMode() {
        // todo: put all of the hardware definitions in a separate method
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
        claw.setDirection(Servo.Direction.REVERSE);
        claw.setPosition(CLAW_MIN);

        ascentStick = hardwareMap.get(Servo.class, "ascentStick");
        ascentStick.setDirection(Servo.Direction.REVERSE);
        ascentStick.setPosition(ASCENT_MIN);

        RevHubOrientationOnRobot.LogoFacingDirection logoDirection = RevHubOrientationOnRobot.LogoFacingDirection.UP;
        RevHubOrientationOnRobot.UsbFacingDirection  usbDirection  = RevHubOrientationOnRobot.UsbFacingDirection.FORWARD;
        RevHubOrientationOnRobot orientationOnRobot = new RevHubOrientationOnRobot(logoDirection, usbDirection);

        // Send telemetry message to signify robot waiting;
        telemetry.addData("Ready", "You can press start"); // todo: Add "Autonomous" before ready so it is super clear to the driver
        telemetry.update();

        configureOtos();

        // Wait for the game to start (driver presses PLAY)
        waitForStart();

        // First Sample ///////////////////////////////////////////////////////////////
        setVertical(VERTICAL_MAX);                        // Raising Arm
        sleep(300);
        setViper(VIPER_MAX);                              // Extending Viper
        driveToLoc(2, 13, 20);     // Go to basket
        sleep(600);
        setClaw(CLAW_MAX);                                // Drop the block

        // Second Sample ///////////////////////////////////////////////////////////////
        driveToLoc(34, 2, 0, 1);
        setViper(1900);
        setVertical(100, 1000);
        sleep(1500);
        setClaw(CLAW_MIN);                              // Grab second block
        sleep(100);
        setVertical(VERTICAL_MAX);
        setViper(VIPER_MAX);
        driveToLoc(10, 13, 45);  // Go to basket
        sleep(600);
        setClaw(CLAW_MAX);                              // Drop second block

        // Third Sample ///////////////////////////////////////////////////////////////
        driveToLoc(36, -2, 0, 1);
        setViper(1100);
        sleep(700);
        setVertical(0, 1500);
        sleep(1700);
        setClaw(CLAW_MIN);                               // Grab third block
        sleep(100);
        setVertical(VERTICAL_MAX);
        setViper(VIPER_MAX);
        driveToLoc(9, 14, 45, 1.5);   // Go to basket
        sleep(600);
        setClaw(CLAW_MAX);                               // Drop third block

        // Park ///////////////////////////////////////////////////////////////
        driveToLoc(25, 5, 0, 3);
        setViper(VIPER_MIN);
        sleep(700);
        setVertical(VERTICAL_MIN);
        driveToLoc(55, 5, 0);
        RobotLog.vv("Rockin'", "Set to max");
        setAscentStick(ASCENT_MAX);
        turnRight(0.5, 170);
        moveForward(0.5, 1000);
        RobotLog.vv("Rockin'", "End program");
        claw.close();                                    // Release tension on the claw
        // End of autonomous program
        telemetry.addData("Autonomous", "Complete");
        telemetry.update();
    }

    public void setAscentStick(double target) {
        RobotLog.vv("Rockin' Robots", "Set Ascent Stick to: %4.2f, Current: %4.2f", target, ascentStick.getPosition()); // todo: we don't need this comment anymore
        ascentStick.setPosition(target);
        sleep(1000);
        RobotLog.vv("Rockin' Robots", "Target: %4.2f, Current: %4.2f", target, ascentStick.getPosition());
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

    public void setClaw(double position) {
        claw.setPosition(position);
        sleep(300);
    }

    private void moveForward(double speed, long msDuration) {
        leftFrontDrive.setPower(speed);
        rightFrontDrive.setPower(speed);
        leftBackDrive.setPower(speed);
        rightBackDrive.setPower(speed);
        sleep(msDuration);
        stopMoving();
    }

    private void turnRight(double speed, double target) {
        leftFrontDrive.setPower(speed);
        rightFrontDrive.setPower(-speed);
        leftBackDrive.setPower(speed);
        rightBackDrive.setPower(-speed);
        getPosition();
        while(hLoc < target -2 || hLoc > target + 2) {
            sleep(10);
            getPosition();
            if(hLoc > 180) hLoc -= 360;
            if(hLoc < -180) hLoc += 360;
        }
        stopMoving();
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
        SparkFunOTOS.Pose2D offset = new SparkFunOTOS.Pose2D(-3.5, 0.5, 90);
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
        if(hDistance > 180) hDistance -= 360;
        if(hDistance < -180) hDistance += 360;

        RobotLog.vv("Rockin' Robots", "driveToLoc: xTarget: %.2f, yTarget: %.2f, hTarget: %.2f, accuracy: %.2f", xTarget, yTarget, hTarget, accuracy);
        RobotLog.vv("Rockin' Robots", "xDistance: %.2f, yDistance: %.2f, hDistance: %.2f, " +
                        "leftFrontPower: %.2f, rightFrontPower: %.2f, leftBackPower: %.2f, rightBackPower: %.2f",
                xDistance, yDistance, hDistance, leftFrontPower, rightFrontPower, leftBackPower, rightBackPower);

        while (opModeIsActive()
                && (Math.abs(xDistance) > accuracy
                || Math.abs(yDistance) > accuracy
                || Math.abs(hDistance) > accuracy)) {

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
            leftFrontPower *= 0.6;
            rightFrontPower *= 0.6;
            leftBackPower *= 0.6;
            rightBackPower *= 0.6;

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
            if(hDistance > 180) {
                hDistance -= 360;
            }
            else if(hDistance < -180) {
                hDistance += 360;
            }
        }
        stopMoving();
        RobotLog.vv("Rockin' Robots", "Done Moving: xDist: %.2f, yDist: %.2f, hDist: %.2f",
                xDistance, yDistance, hDistance);
    }
}