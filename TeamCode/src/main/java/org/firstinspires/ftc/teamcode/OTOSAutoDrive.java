package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.hardware.sparkfun.SparkFunOTOS;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;
import com.qualcomm.robotcore.util.RobotLog;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;

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
    int verticalAdjustedMin = 0;
    int verticalPosition = VERTICAL_MIN;
    // This chunk controls our viper slide
    DcMotor viperSlide = null;
    final int VIPER_MAX = 3100;
    final int VIPER_MIN = 0;
    int viperSlidePosition = 0;

    // This chunk controls our claw
    Servo claw = null;
    final double CLAW_MIN = 0.9;    // Claw is closed
    final double CLAW_MAX = 0.7;    // Claw is open

    final ElapsedTime runtime = new ElapsedTime();

    // Software variables
    static final double     DEFAULT_SPEED = 0.6;

    private IMU imu = null;
    static final double TURN_SPEED_ADJUSTMENT = 0.015;     // Larger is more responsive, but also less stable
    static final double HEADING_ERROR_TOLERANCE = 1.0;    // How close must the heading get to the target before moving to next step.
    static final double MAX_TURN_SPEED = 1.0;     // Max Turn speed to limit turn rate
    static final double MIN_TURN_SPEED = 0.15;     // Min Turn speed to limit turn rate
    private double turnSpeed = 0;
    private double degreesToTurn = 0;

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

        imu = hardwareMap.get(IMU.class, "imu");
        imu.initialize(new IMU.Parameters(orientationOnRobot));
        imu.resetYaw();

        // Send telemetry message to signify robot waiting;
        telemetry.addData("Status", "Ready");
        telemetry.update();

        telemetry.addData("Current Yaw", "%.0f", getHeading());
        telemetry.update();

        configureOtos();

        // Wait for the game to start (driver presses PLAY)
        waitForStart();

        // Code here

        setVertical(VERTICAL_MAX);                             // Raising Arm
        setViper(VIPER_MAX);                                   // Extending Viper
        driveToLoc(3, 14, 20);
        claw.setPosition(CLAW_MAX);
        sleep(500);
        driveToLoc(34, 0, 0);
        setViper(1800);
        setVertical(100);
        sleep(10000);
        /*
        strafeRight(1.5, 0.4);       // Positioning to basket
        moveForward(0.4, 0.4);       // Positioning to basket
        turnToHeading(45, 0.4);           // Turn to basket, claw in position
        moveForward(0.35, 0.3);     // Move forward: Originally 0.3
        setClaw(CLAW_MAX);                                    // Drop the block
        moveBackward(0.3, 0.3);
        turnToHeading(-2, 0.4);            // Turning away from basket
        setViper(VIPER_MIN);                                  // Retracting Viper
        setVertical(VERTICAL_MIN);                            // Lowering Arm
        strafeRight(0.5, 0.4);
        turnToHeading(0, 0.4);
        moveBackward(4, 0.5);      // Todo: Change to 4 seconds before competition
        turnToHeading(0, 0.4);
        strafeLeft(2.7, 0.4);       // Positioning to park
        */
        claw.close();                                         // Release tension on the claw

        // End of autonomous program
        telemetry.addData("Path", "Complete");
        telemetry.addData("Current Yaw", "%.0f", getHeading());
        telemetry.update();
    }

    public void setViper(int length){
        viperSlide.setTargetPosition(length);
        ((DcMotorEx) viperSlide).setVelocity(2000);
        viperSlide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        sleep(2000);
    }

    public void setVertical(int height){
        vertical.setTargetPosition(height);
        ((DcMotorEx) vertical).setVelocity(2000);
        vertical.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        sleep(1500);
    }

    public void setVertical(int height, int speed){
        vertical.setTargetPosition(height);
        ((DcMotorEx) vertical).setVelocity(speed);
        vertical.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        sleep(1500);
    }

    public void setClaw(double position){
        claw.setPosition(position);
        sleep(300);
    }

    private void acceleration(double secondsToDrive, double targetSpeed,
                              double leftFrontDriveDirection, double rightFrontDriveDirection,
                              double leftBackDriveDirection, double rightBackDriveDirection){
        double currentSpeed = 0.0;

        runtime.reset();
        while (opModeIsActive() && (runtime.seconds() < secondsToDrive)) {
            double elapsedTime = runtime.seconds();

            // Driving less than a second
            if(secondsToDrive < 1){
                currentSpeed = targetSpeed;
            }
            // Acceleration phase
            if (elapsedTime < 1 && currentSpeed < targetSpeed) {
                currentSpeed = currentSpeed + 0.01; // Increase the speed by 0.01 per second
            }
            // Deceleration phase
            else if (elapsedTime > secondsToDrive - 1 && elapsedTime < secondsToDrive && currentSpeed > 0) {
                currentSpeed = currentSpeed - 0.01; // Decrease the speed by 0.01 per second
            }

            leftFrontDrive.setPower(currentSpeed*leftFrontDriveDirection);
            rightFrontDrive.setPower(currentSpeed*rightFrontDriveDirection);
            leftBackDrive.setPower(currentSpeed*leftBackDriveDirection);
            rightBackDrive.setPower(currentSpeed*rightBackDriveDirection);

            telemetry.addData("Move forward: %4.1f S Elapsed", runtime.seconds());
            telemetry.update();
        }
        stopMoving();
    }

    private void stopMoving() {
        leftFrontDrive.setPower(0);
        rightFrontDrive.setPower(0);
        leftBackDrive.setPower(0);
        rightBackDrive.setPower(0);
    }

    private void moveForward(double secondsToDrive) {
        moveForward(secondsToDrive, DEFAULT_SPEED);
    }

    private void moveForward(double secondsToDrive, double speedToDrive) {
        acceleration(secondsToDrive, speedToDrive, 1, 1, 1, 1);
    }

    private void moveBackward(double secondsToDrive) {
        moveBackward(secondsToDrive, DEFAULT_SPEED);
    }

    private void moveBackward(double secondsToDrive, double speedToDrive) {
        acceleration(secondsToDrive, speedToDrive, -1, -1,-1,-1);
    }

    private void turnLeft(double secondsToDrive) {
        turnLeft(secondsToDrive, DEFAULT_SPEED);
    }

    private void turnLeft(double secondsToDrive, double speedToDrive) {
        acceleration(secondsToDrive, speedToDrive, -1, 1, -1, 1);
    }

    private void turnRight(double secondsToDrive) {
        turnRight(secondsToDrive, DEFAULT_SPEED);
    }

    private void turnRight(double secondsToDrive, double speedToDrive) {
        acceleration(secondsToDrive, speedToDrive, 1, -1, 1, -1);
    }

    private void strafeLeft(double secondsToDrive) {
        strafeLeft(secondsToDrive, DEFAULT_SPEED);
    }

    private void strafeLeft(double secondsToDrive, double speedToDrive) {
        acceleration(secondsToDrive, speedToDrive, -1, 1, 1, -1);
    }
    private void strafeRight(double secondsToDrive) {
        strafeRight(secondsToDrive, DEFAULT_SPEED);
    }

    private void strafeRight(double secondsToDrive, double speedToDrive) {
        acceleration(secondsToDrive, speedToDrive, 1, -1, -1, 1);
    }

    public double getHeading() {
        YawPitchRollAngles orientation = imu.getRobotYawPitchRollAngles();
        return orientation.getYaw(AngleUnit.DEGREES);
    }

    private void turnToHeading(double targetYaw) {
        turnToHeading(targetYaw, DEFAULT_SPEED);
    }

    private void turnToHeading(double heading, double speedToDrive) {
        degreesToTurn = heading - getHeading();

        while (opModeIsActive() && (Math.abs(degreesToTurn) > HEADING_ERROR_TOLERANCE)) {
            degreesToTurn = heading - getHeading();
            if (degreesToTurn < -180) degreesToTurn += 360;
            if (degreesToTurn > 180) degreesToTurn -= 360;

            // Clip the speed to the maximum permitted value
            turnSpeed = Range.clip(degreesToTurn * TURN_SPEED_ADJUSTMENT, -speedToDrive, speedToDrive);
            if (turnSpeed < MIN_TURN_SPEED && turnSpeed >= 0) turnSpeed = MIN_TURN_SPEED;
            if (turnSpeed > -MIN_TURN_SPEED && turnSpeed < 0) turnSpeed = -MIN_TURN_SPEED;

            leftFrontDrive.setPower(-turnSpeed);
            rightFrontDrive.setPower(turnSpeed);
            leftBackDrive.setPower(-turnSpeed);
            rightBackDrive.setPower(turnSpeed);
        }
        stopMoving();
    }
    private void configureOtos() {
        telemetry.addLine("Configuring OTOS...");
        telemetry.update();

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

        telemetry.addLine("OTOS configured! Press start to get position data!");
        telemetry.addLine();
        telemetry.addLine(String.format("OTOS Hardware Version: v%d.%d", hwVersion.major, hwVersion.minor));
        telemetry.addLine(String.format("OTOS Firmware Version: v%d.%d", fwVersion.major, fwVersion.minor));
        telemetry.update();
    }
    private void getPosition() {
        SparkFunOTOS.Pose2D pos = myOtos.getPosition();
        xLoc = pos.x;
        yLoc = pos.y;
        hLoc = pos.h;
    }
    private void driveToLoc(double xTarget, double yTarget, double hTarget) {
        getPosition();
        double xDistance = xTarget - xLoc;
        double yDistance = yTarget - yLoc;
        double hDistance = hTarget - hLoc;

        RobotLog.vv("Rockin' Robots", "xDistance: %.2f, yDistance: %.2f, hDistance: %.2f, " +
                        "leftFrontPower: %.2f, rightFrontPower: %.2f, leftBackPower: %.2f, rightBackPower: %.2f",
                xDistance, yDistance, hDistance, leftFrontPower, rightFrontPower, leftBackPower, rightBackPower);

        while (opModeIsActive()
                && (Math.abs(xDistance) > 2      // todo: After adding accuracy, change to 1.
                || Math.abs(yDistance) > 2
                || Math.abs(hDistance) > 2)) {

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