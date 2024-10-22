package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;

@Autonomous(name="Auto Drive", group="Robot")
public class RobotAutoDriveByTime_Linear extends LinearOpMode {
    // Initialize all variables for the program
    // Hardware variables
    private DcMotor leftFrontDrive = null;
    private DcMotor rightFrontDrive = null;
    private DcMotor leftBackDrive = null;
    private DcMotor rightBackDrive = null;

    DcMotor vertical = null;
    final int VERTICAL_MIN = 0;
    final int VERTICAL_MAX = 1800;
    int verticalAdjustedMin = 0;
    int verticalPosition = VERTICAL_MIN;

    // This chunk controls our viper slide
    DcMotor viperSlide = null;
    final int VIPER_MAX = 2759;
    final int VIPER_MIN = 0;
    int viperSlidePosition = 0;

    // This chunk controls our claw
    Servo claw = null;
    final double CLAW_MIN = 0.6;    // Claw is closed
    final double CLAW_MAX = 0.4;    // Claw is open

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

        // Wait for the game to start (driver presses PLAY)
        waitForStart();

        /*
        moveForward(2.3);
        strafeLeft(0.5);                        //strafing to 1st block
        moveBackward(2.1 );                     //moving first block backward
        turnRightToHeading(-20);                //turning to the red line to angle the block
        moveBackward(1.1);                      //moving the block backward into zone
        turnLeftToHeading(20);                  //turning to 0 degrees
        strafeRight(1.4);                       //strafing before going forward
        moveForward(2.3);                       //moving to 2nd block
        strafeLeft(1.2);                        //strafing left to 2nd block
        moveBackward(2, 0.2);                   //moving to red zone
        turnRightToHeading(20);                 //turning to face the red zone
        moveBackward(1.5, 0.2);                 //placing block in red zone
       */

        // Neha: You should not start at an angle because getting the exact same angle every time is difficult.
        // You should count on the robot starting facing the basket.
        // You don't have to be directly in front of the basket to score. You can approach it from the side.
        // Update this sequence and we will test it out.
        //moveForward(1);
        //turnLeftToHeading(130);
        //moveForward(0.3);
        setVertical(VERTICAL_MAX);
        setViper(VIPER_MAX);
        moveForward(0.1); // I recommend making the robot drive slowly here. Maybe .2 or .3?
        setClaw(CLAW_MAX);
        setViper(VIPER_MIN);
        setVertical(VERTICAL_MIN);

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

    private void turnLeftToHeading(double targetYaw) {
        turnLeftToHeading(targetYaw, DEFAULT_SPEED);
    }

    // Turn left to desired heading.
    private void turnLeftToHeading(double heading, double speedToDrive) {
        degreesToTurn = heading - getHeading();

        while (opModeIsActive()
                && (Math.abs(degreesToTurn) > HEADING_ERROR_TOLERANCE)
                && (gamepad1.left_stick_y == 0) && (gamepad1.left_stick_x == 0) && (gamepad1.right_stick_x == 0)) {

            degreesToTurn = heading - getHeading();
            if (degreesToTurn < -180) degreesToTurn += 360;
            if (degreesToTurn > 180) degreesToTurn -= 360;

            // Clip the speed to the maximum permitted value
            turnSpeed = Range.clip(degreesToTurn * TURN_SPEED_ADJUSTMENT, -MAX_TURN_SPEED, MAX_TURN_SPEED);
            if (turnSpeed < MIN_TURN_SPEED && turnSpeed >= 0) turnSpeed = MIN_TURN_SPEED;
            if (turnSpeed > -MIN_TURN_SPEED && turnSpeed < 0) turnSpeed = -MIN_TURN_SPEED;

            leftFrontDrive.setPower(-turnSpeed);
            rightFrontDrive.setPower(turnSpeed);
            leftBackDrive.setPower(-turnSpeed);
            rightBackDrive.setPower(turnSpeed);
        }
    }

    private void turnRightToHeading(double targetYaw) {
        turnRightToHeading(targetYaw, DEFAULT_SPEED);
    }

    // Turn right to desired heading.
    private void turnRightToHeading(double heading, double speedToDrive) {
        degreesToTurn = heading - getHeading();

        while (opModeIsActive()
                && (Math.abs(degreesToTurn) > HEADING_ERROR_TOLERANCE)
                && (gamepad1.left_stick_y == 0) && (gamepad1.left_stick_x == 0) && (gamepad1.right_stick_x == 0)) {

            degreesToTurn = heading - getHeading();
            if(degreesToTurn < -180) degreesToTurn += 360;
            if(degreesToTurn > 180) degreesToTurn -= 360;

            // Clip the speed to the maximum permitted value
            turnSpeed = Range.clip(degreesToTurn*TURN_SPEED_ADJUSTMENT, -MAX_TURN_SPEED, MAX_TURN_SPEED);
            if(turnSpeed < MIN_TURN_SPEED && turnSpeed >= 0) turnSpeed = MIN_TURN_SPEED;
            if(turnSpeed > -MIN_TURN_SPEED && turnSpeed < 0) turnSpeed = -MIN_TURN_SPEED;

            leftFrontDrive.setPower(turnSpeed);
            rightFrontDrive.setPower(-turnSpeed);
            leftBackDrive.setPower(turnSpeed);
            rightBackDrive.setPower(-turnSpeed);

        }
    }


}

//NC TELEMETRY TRANSFER