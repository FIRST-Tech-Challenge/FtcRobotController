package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;


@Config
@Autonomous(name="AutonomousMoveCode", group="Examples")
public class AutonomousMoveCode extends LinearOpMode {
    private FtcDashboard dashboard;
    private RobotHardware robot = new RobotHardware();

    // Constants for distance calculations
    private static final double COUNTS_PER_MOTOR_GOBILDA_435 = 384.5;
    private static final double COUNTS_PER_MOTOR_GOBILDA_312 = 537.7;
    private static final double DRIVE_GEAR_REDUCTION = 0.66; //24:16 Motor:Wheel
    private static final double WHEEL_DIAMETER_MM = 96; // Wheel diameter mm
    private static final double COUNTS_PER_MM_Drive = (COUNTS_PER_MOTOR_GOBILDA_435 * DRIVE_GEAR_REDUCTION) /
            (WHEEL_DIAMETER_MM * Math.PI);
    private static final double COUNTS_PER_CM_Slides = COUNTS_PER_MOTOR_GOBILDA_312 / 38.2; //Ticks Per Rotation * Pulley Circumference

    //Timer
    private static ElapsedTime hook_Time = new ElapsedTime();
    public static double intake_Wait_Time = 0.5;
    public static double deposit_Wait_Time = 0.5;

    //Segment 1 Distance
    public static double first_forward = -100;
    public static double speed = 0.2;

    //Action 1:

    public static int first_strafe = 686;

    @Override
    public void runOpMode() {
        // Initialize hardware
        robot = new RobotHardware();
        robot.init(hardwareMap);
        robot.initIMU();

        telemetry = new MultipleTelemetry(telemetry,FtcDashboard.getInstance().getTelemetry());

        robot.frontLeftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        robot.frontRightMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        robot.backLeftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        robot.backRightMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        robot.depositClawServo.setPosition(0);

        // Wait for the game to start
        waitForStart();

        //Segment 1: Score first Specimen
        driveToPosition(first_forward, speed);
        
        //Action
        Action_One();

        // Segment 2: Move forward 24 inches

        // Segment 3: Move to park step 1

        // Segment 4: Move to park step 2

        //Segment 5: Raise Slides

        //Segment 6: Move to bar
    }

    /**
     * Moves the robot forward a specified number of inches at a given speed.
     */
    private void driveToPosition(double dist_mm, double speed) {
        int targetPosition = (int)(dist_mm * COUNTS_PER_MM_Drive);

        // Set target position for both motors
        robot.frontLeftMotor.setTargetPosition(robot.frontLeftMotor.getCurrentPosition() + targetPosition);
        robot.frontRightMotor.setTargetPosition(robot.frontRightMotor.getCurrentPosition() + targetPosition);
        robot.backLeftMotor.setTargetPosition(robot.backLeftMotor.getCurrentPosition() + targetPosition);
        robot.backRightMotor.setTargetPosition(robot. backRightMotor.getCurrentPosition() + targetPosition);

        // Set to RUN_TO_POSITION mode
        robot.frontLeftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.frontRightMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.backLeftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.backRightMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        // Set motor power
        robot.frontLeftMotor.setPower(speed);
        robot.frontRightMotor.setPower(speed);
        robot.backLeftMotor.setPower(speed);
        robot.backRightMotor.setPower(speed);

        // Wait until the robot reaches the target position
        while (opModeIsActive() && (robot.frontLeftMotor.isBusy() && robot.frontRightMotor.isBusy())) {
            telemetry.addData("Motor Position", "Left: %d, Right: %d",
                    robot.frontLeftMotor.getCurrentPosition(), robot.frontRightMotor.getCurrentPosition());
            telemetry.update();
        }

        // Stop the motors
        robot.frontLeftMotor.setPower(0);
        robot.frontRightMotor.setPower(0);
        robot.backLeftMotor.setPower(0);
        robot.backRightMotor.setPower(0);

        // Reset to RUN_USING_ENCODER mode
        robot.frontLeftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.frontRightMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.backLeftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.backRightMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

    }
    /**
     strafing
     **/
    private void strafeToPosition(double dist_mm, double speed) {
        int targetPosition = (int)(dist_mm * COUNTS_PER_MM_Drive);

        // Reset to RUN_USING_ENCODER mode
        robot.frontLeftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.frontRightMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.backLeftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.backRightMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        // Set target position for both motors
        robot.frontLeftMotor.setTargetPosition(robot.frontLeftMotor.getCurrentPosition() + targetPosition);
        robot.frontRightMotor.setTargetPosition(robot.frontRightMotor.getCurrentPosition() - targetPosition);
        robot.backLeftMotor.setTargetPosition(robot.backLeftMotor.getCurrentPosition() - targetPosition);
        robot.backRightMotor.setTargetPosition(robot.backRightMotor.getCurrentPosition() + targetPosition);

        // Set to RUN_TO_POSITION mode
        robot.frontLeftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.frontRightMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.backLeftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.backRightMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        // Set motor power
        robot.frontLeftMotor.setPower(speed);
        robot.frontRightMotor.setPower(speed);
        robot.backLeftMotor.setPower(speed);
        robot.backRightMotor.setPower(speed);

        // Wait until the robot reaches the target position
        while (opModeIsActive() && (robot.frontLeftMotor.isBusy() && robot.frontRightMotor.isBusy())) {
            telemetry.addData("Motor Position", "Left: %d, Right: %d",
                    robot.frontLeftMotor.getCurrentPosition(), robot.frontRightMotor.getCurrentPosition());
            telemetry.update();
        }

        // Stop the motors
        robot.frontLeftMotor.setPower(0);
        robot.frontRightMotor.setPower(0);
        robot.backLeftMotor.setPower(0);
        robot.backRightMotor.setPower(0);
    }
    /**
     * Turns the robot by a specific angle (in degrees) at a given speed.
     */
    private void turnToAngle(double targetAngle, double speed) {
        // Reset the IMU angle
        double currentAngle = getHeading();

        while (opModeIsActive() && Math.abs(targetAngle - currentAngle) > 1) { // Tolerance of 1 degree
            double turnDirection = Math.signum(targetAngle - currentAngle); // Positive for clockwise, negative for counter-clockwise

            // Apply power for turning
            robot.frontLeftMotor.setPower(turnDirection * speed);
            robot.backLeftMotor.setPower(turnDirection * speed);
            robot.frontRightMotor.setPower(-turnDirection * speed);
            robot.backRightMotor.setPower(-turnDirection * speed);

            // Update the current angle
            currentAngle = getHeading();

            telemetry.addData("Current Angle", currentAngle);
            telemetry.update();
        }

        // Stop all motors
        robot.frontLeftMotor.setPower(0);
        robot.backLeftMotor.setPower(0);
        robot.frontRightMotor.setPower(0);
        robot.backRightMotor.setPower(0);
    }

    /**
     * Returns the current heading angle from the IMU.
     *
     * @return The heading angle in degrees
     */
    private double getHeading() {
        return robot.imu.getRobotYawPitchRollAngles().getYaw();
    }
    /**
     * Perform an action at the target position.
     */
    private void Slides_Move(double dist_cm, double speed) {
        int target_Position = (int)(dist_cm * COUNTS_PER_CM_Slides);

        robot.liftMotorLeft.setTargetPosition(target_Position);
        robot.liftMotorRight.setTargetPosition(target_Position);
        robot.liftMotorLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.liftMotorRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.liftMotorRight.setPower(speed);
        robot.liftMotorLeft.setPower(speed);
    }

    private void Action_One (){
        robot.intakeRightArmServo.setPosition(0.9);
        robot.intakeLeftArmServo.setPosition(0.9);

        robot.depositRightArmServo.setPosition(0.83);
        robot.depositLeftArmServo.setPosition(0.83);
        robot.depositWristServo.setPosition(0.5);

        Slides_Move(15, 0.3);
    }
}
