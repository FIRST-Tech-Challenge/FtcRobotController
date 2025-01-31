package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.hardware.sparkfun.SparkFunOTOS;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.RobotLog;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

@Autonomous(name="Auto Drive", group="Robot")
public class WristOTOSAutoDrive extends LinearOpMode {
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
    final int VERTICAL_MAX = 2150;
    final int VERTICAL_DEFAULT_SPEED = 2000;

    // This chunk controls our viper slide
    DcMotor viperSlide = null;
    final int VIPER_MIN = 0;
    final int VIPER_MAX = 2540;
    final int VIPER_GROUND = 1000;
    final int VIPER_DEFAULT_SPEED = 3000;

    // This chunk controls our claw
    //Callie
    Servo claw = null;
    final double CLAW_MIN = 0.17;           // Claw is closed
    final double CLAW_MAX = 0.5;          // Claw is open - Og non-wrist value was 0.8

    // This chunk controls our wrist
    Servo wrist = null;
    final double WRIST_PICKUP = 0.23;           // Wrist is in intake position (picking up)
    final double WRIST_MID = 0.4;              // Wrist is out of the way
    final double WRIST_DROPOFF = 0.89;          // Wrist is in outtake position (dropping in basket)

    Servo ascentStick = null;
    final double ASCENT_MIN = 0.2;          // Stick is down
    final double ASCENT_MAX = 0.43;         // Stick is up

    final ElapsedTime runtime = new ElapsedTime();

    @Override
    public void runOpMode() {
        initializeHardwareVariables();

        // Send telemetry message to signify robot waiting;
        telemetry.addData("Autonomous Ready", "You can press start");
        telemetry.addData("This code was last updated", "01/31/2025, 11:55 am"); // Todo: Update this date when the code is updated
        telemetry.update();

        // Wait for the game to start (driver presses PLAY)
        waitForStart();
        configureOtos();

        // First Sample ///////////////////////////////////////////////////////////////
        RobotLog.vv("Rockin' Robots", "Drop off Sample 1");
        driveToLoc(-4, -11, 0, 1);  // Go to basket
        setWrist(WRIST_MID);
        setVertical(VERTICAL_MAX, 3000);                      // Raising Arm
        sleep(500);
        setViper(VIPER_MAX);                                        // Extending Viper
        sleep(900);
        setWrist(WRIST_DROPOFF);
        sleep(800);
        setClaw(CLAW_MAX);                                          // Drop the block
        sleep(300);

        // Second Sample ///////////////////////////////////////////////////////////////
        RobotLog.vv("Rockin' Robots", "Get Sample 2");
        setWrist(WRIST_PICKUP);
        turnLeft(0.4, 80);
        setViper(VIPER_GROUND);
        driveToLoc(-20, -14, 85, .5); // Get in position
        setVertical(110);
        sleep(1400);
        setClaw(CLAW_MIN);                                          // Grab second block
        sleep(200);
        setWrist(WRIST_MID);                                        // Protect claw by tucking wrist away
        setVertical(VERTICAL_MAX, 2000);
        sleep(1100);
        setViper(VIPER_MAX);
        setWrist(WRIST_DROPOFF);
        driveToLoc(-3, -17, 0, 1);  // Go to basket
        sleep(100);
        setClaw(CLAW_MAX);                                          // Drop second block

        // Third Sample ///////////////////////////////////////////////////////////////
        RobotLog.vv("Rockin' Robots", "Get Sample 3");
        setWrist(WRIST_PICKUP);
        turnLeft(0.4, 100);
        setViper(VIPER_GROUND);
        driveToLoc(-20, -23, 95, .5); // Get in position
        setVertical(110);
        sleep(1300);
        setClaw(CLAW_MIN);                                          // Grab second block
        sleep(200);
        setWrist(WRIST_MID);                                        // Protect claw by tucking wrist away
        setVertical(VERTICAL_MAX, 2000);
        sleep(1100);
        setViper(VIPER_MAX);
        setWrist(WRIST_DROPOFF);
        driveToLoc(-3, -17, 0, 1);  // Go to basket
        sleep(100);
        setClaw(CLAW_MAX);

        // Park ///////////////////////////////////////////////////////////////
        RobotLog.vv("Rockin' Robots", "Parking");
        driveToLoc(-55, -10, 0, 3);
        setViper(VIPER_MIN);
        sleep(700);
        setVertical(VERTICAL_MIN);
        setAscentStick(ASCENT_MAX);                                 // Ascent stick up
        moveForward(.4, 1500);
        setAscentStick(ASCENT_MAX);                                 // Ascent stick up
        sleep(100);
        RobotLog.vv("Rockin' Robots", "Route is done");
        //sleep(500);
        //ascentStick.close();
        claw.close();                                               // Release tension on the claw
        sleep(200);
        RobotLog.vv("Rockin' Robots", "Ending the program, current ascent stick: %4.2f", ascentStick.getPosition());
        telemetry.addData("Autonomous", "Complete");
        telemetry.update();
    }

    public void setAscentStick(double target) {
        RobotLog.vv("Rockin' Robots", "setAscentStick() target: %4.2f, current: %4.2f", target, ascentStick.getPosition());
        ascentStick.setPosition(target);
    }

    public void setViper(int length){ setViper(length, VIPER_DEFAULT_SPEED); }

    public void setViper(int length, int velocity){
        RobotLog.vv("Rockin' Robots", "setViper() length: %d, current: %d", length, viperSlide.getCurrentPosition());
        viperSlide.setTargetPosition(length);
        ((DcMotorEx) viperSlide).setVelocity(velocity);
        viperSlide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    }

    public void setVertical(int height){
        setVertical(height, VERTICAL_DEFAULT_SPEED);
    }

    public void setVertical(int height, int speed){
        RobotLog.vv("Rockin' Robots", "setVertical() height: %d, speed: %d, current: %d", height, speed, vertical.getCurrentPosition());
        vertical.setTargetPosition(height);
        ((DcMotorEx) vertical).setVelocity(speed);
        vertical.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        if (height == VERTICAL_MAX) { sleep(500); }
    }

    public void setClaw(double position) {
        RobotLog.vv("Rockin' Robots", "setClaw() position: %4.2f, current: %4.2f", position, claw.getPosition());
        claw.setPosition(position);
        sleep(400);
    }

    public void setWrist(double position) {
        RobotLog.vv("Rockin' Robots", "setWrist() position: %4.2f, current: %4.2f", position, wrist.getPosition());
        wrist.setPosition(position);
        RobotLog.vv("Rockin' Robots", "setWrist() done: current: %4.2f", wrist.getPosition());
    }

    private void moveForward(double speed, long msDuration) {
        RobotLog.vv("Rockin' Robots", "moveForward() speed: %4.2f, msDuration:: %d", speed, msDuration);
        leftFrontDrive.setPower(speed);
        rightFrontDrive.setPower(speed);
        leftBackDrive.setPower(speed);
        rightBackDrive.setPower(speed);
        sleep(msDuration);
        stopMoving();
    }

    private void turnRight(double speed, double target) {
        RobotLog.vv("Rockin' Robots", "turnRight() speed: %4.2f, target: %4.2f, current: %4.2f", speed, target, hLoc);
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

    private void turnLeft(double speed, double target) {
        RobotLog.vv("Rockin' Robots", "turnLeft() speed: %4.2f, target: %4.2f, current: %4.2f", speed, target, hLoc);
        rightFrontDrive.setPower(speed);
        rightBackDrive.setPower(speed);
        leftFrontDrive.setPower(-speed/2);
        leftBackDrive.setPower(-speed/2);
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
        RobotLog.vv("Rockin' Robots", "stopMoving()");
        leftFrontDrive.setPower(0);
        rightFrontDrive.setPower(0);
        leftBackDrive.setPower(0);
        rightBackDrive.setPower(0);
    }

    private void configureOtos() {
        myOtos.setLinearUnit(DistanceUnit.INCH);
        myOtos.setAngularUnit(AngleUnit.DEGREES);
        SparkFunOTOS.Pose2D offset = new SparkFunOTOS.Pose2D(-3.5, 1.1, 90);
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
        double angleRadians = Math.toRadians(hLoc);
        double xRotatedDistance = xDistance * Math.cos(angleRadians) + yDistance * Math.sin(angleRadians);
        double yRotatedDistance = - xDistance * Math.sin(angleRadians) + yDistance * Math.cos(angleRadians);

        RobotLog.vv("Rockin' Robots", "driveToLoc() xTarget: %.2f, yTarget: %.2f, hTarget: %.2f, accuracy: %.2f",
                xTarget, yTarget, hTarget, accuracy);

        runtime.reset();
        while (opModeIsActive()
                &&(runtime.seconds() < 5)
                && (Math.abs(xDistance) > accuracy
                || Math.abs(yDistance) > accuracy
                || Math.abs(hDistance) > accuracy)) {
            if(runtime.seconds() > 4.5)
                RobotLog.vv("Rockin' Robots", "Hit the timeout");

            leftFrontPower = (yRotatedDistance + xRotatedDistance - hDistance) / 8;
            rightFrontPower = (yRotatedDistance - xRotatedDistance + hDistance) / 8;
            leftBackPower = (yRotatedDistance - xRotatedDistance - hDistance) / 8;
            rightBackPower = (yRotatedDistance + xRotatedDistance + hDistance) / 8;

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
            max = Math.max(Math.abs(leftFrontPower), Math.abs(rightFrontPower));
            max = Math.max(max, Math.abs(leftBackPower));
            max = Math.max(max, Math.abs(rightBackPower));
            if (max < 0.2) {
                leftFrontPower *= 1.5;
                rightFrontPower *= 1.5;
                leftBackPower *= 1.5;
                rightBackPower *= 1.5;
            }

            leftFrontDrive.setPower(leftFrontPower);
            rightFrontDrive.setPower(rightFrontPower);
            leftBackDrive.setPower(leftBackPower);
            rightBackDrive.setPower(rightBackPower);
            //RobotLog.vv("Rockin' Robots", "xDist: %.2f, yDist: %.2f, hDist: %.2f, " +
            //        "leftFrontPower: %.2f, rightFrontPower: %.2f, leftBackPower: %.2f, rightBackPower: %.2f",
            //        xDistance, yDistance, hDistance, leftFrontPower, rightFrontPower, leftBackPower, rightBackPower);

            getPosition();
            xDistance = xTarget - xLoc;
            yDistance = yTarget - yLoc;
            hDistance = hTarget - hLoc;
            if(hDistance > 180) hDistance -= 360;
            if(hDistance < -180) hDistance += 360;

            angleRadians = Math.toRadians(hLoc);
            xRotatedDistance = xDistance * Math.cos(angleRadians) + yDistance * Math.sin(angleRadians);
            yRotatedDistance = - xDistance * Math.sin(angleRadians) + yDistance * Math.cos(angleRadians);
        }
        stopMoving();
        RobotLog.vv("Rockin' Robots", "Done Moving: xDist: %.2f, yDist: %.2f, hDist: %.2f",
                xDistance, yDistance, hDistance);
    }

    public void initializeHardwareVariables() {
        // Define all the hardware
        myOtos = hardwareMap.get(SparkFunOTOS.class, "OTOS");
        SparkFunOTOS.Pose2D pos = myOtos.getPosition();
        leftFrontDrive = hardwareMap.get(DcMotor.class, "left_front_drive");
        leftBackDrive = hardwareMap.get(DcMotor.class, "left_back_drive");
        rightFrontDrive = hardwareMap.get(DcMotor.class, "right_front_drive");
        rightBackDrive = hardwareMap.get(DcMotor.class, "right_back_drive");
        leftFrontDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftBackDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightFrontDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightBackDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftFrontDrive.setDirection(DcMotor.Direction.REVERSE);
        leftBackDrive.setDirection(DcMotor.Direction.REVERSE);
        rightFrontDrive.setDirection(DcMotor.Direction.FORWARD);
        rightBackDrive.setDirection(DcMotor.Direction.FORWARD);

        vertical = hardwareMap.get(DcMotor.class, "vertical");
        vertical.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        vertical.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        vertical.setTargetPosition(0);
        vertical.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        viperSlide = hardwareMap.get(DcMotor.class, "viper_slide");
        viperSlide.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        viperSlide.setDirection(DcMotor.Direction.REVERSE);
        viperSlide.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        viperSlide.setTargetPosition(0);
        viperSlide.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        claw = hardwareMap.get(Servo.class, "claw");
        claw.setPosition(CLAW_MIN);

        wrist = hardwareMap.get(Servo.class, "wrist");
        wrist.setDirection(Servo.Direction.REVERSE);
        wrist.setPosition(1.0);

        ascentStick = hardwareMap.get(Servo.class, "ascentStick");
        ascentStick.setDirection(Servo.Direction.REVERSE);
        ascentStick.setPosition(ASCENT_MIN);

        RevHubOrientationOnRobot.LogoFacingDirection logoDirection = RevHubOrientationOnRobot.LogoFacingDirection.UP;
        RevHubOrientationOnRobot.UsbFacingDirection  usbDirection  = RevHubOrientationOnRobot.UsbFacingDirection.FORWARD;
        RevHubOrientationOnRobot orientationOnRobot = new RevHubOrientationOnRobot(logoDirection, usbDirection);
    }

}