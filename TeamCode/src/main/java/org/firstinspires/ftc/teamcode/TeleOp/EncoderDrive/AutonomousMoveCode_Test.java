package org.firstinspires.ftc.teamcode.AutoTest.EncoderDrive;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.AutoTest.RobotHardware;

@Autonomous(name="AutonomousMoveCode_Test", group="org.firstinspires.ftc.teamcode")
public class AutonomousMoveCode_Test extends LinearOpMode {
    private RobotHardware robot = new RobotHardware();

    private ElapsedTime         runtime = new ElapsedTime();

    // Constants for distance calculations
    static final double COUNTS_PER_MOTOR_GOBILDA_435    = 384.5;
    static final double COUNTS_PER_MOTOR_GOBILDA_312    = 537.7;
    static final double DRIVE_GEAR_REDUCTION            = 1.5; //24:16 Motor:Wheel
    static final double WHEEL_DIAMETER_MM               = 96; // Wheel diameter mm
    static final double COUNTS_PER_MM_Drive             = (COUNTS_PER_MOTOR_GOBILDA_435 * DRIVE_GEAR_REDUCTION) / (WHEEL_DIAMETER_MM * Math.PI);
    static final double COUNTS_PER_CM_Slides = COUNTS_PER_MOTOR_GOBILDA_312 / 38.2; //Ticks Per Rotation * Pulley Circumference

    //
    public static double intake_slide_Extension = 0.6;// range(0.3 - 0.65)
    public static double intake_slide_Retract   = 0.3;

    public static double intake_Rotation        = 0.49;

    public static double intake_Arm_initial     = 0.35;//0-0.56
    public static double intake_Arm_down        = 0.05;
    public static double intake_Arm_retract     = 0.53;

    public static double intake_Claw_Open       = 0.55;
    public static double intake_Claw_Close      = 0.3;

    //Deposit Config
    public static int deposit_Slide_down_Pos         = 50;   //slides Position Configure
    public static int deposit_Slide_Highbar_Pos      = 795;  //slides Position Configure
    public static int deposit_Slide_Highbasket_Pos   = 2800; //slides Position Configure

    public static double deposit_Wrist_dump_Pos         = 0.3;
    public static double deposit_Wrist_retract_Pos      = 0.1;

    public static double deposit_Arm_dump_Pos           = 0.8;
    public static double deposit_Arm_retract_Pos        = 0.0;

    public static double deposit_Arm_hook_Pos           = 0.8;
    public static double deposit_Claw_Open              = 0.11;
    public static double deposit_Claw_Close             = 0.0;

    public static double dumpTime                       = 1.8;
    public static double retractTime                    = 3.2;

    public static double deposit_Slide_UpLiftPower      = 0.9;  //slides power
    public static double downLiftPower                  = 0.3;  //slides power

    private ElapsedTime intakeTimer = new ElapsedTime();


    //Timer
    static ElapsedTime hook_Time = new ElapsedTime();
    static double intake_Wait_Time = 0.5;
    static double deposit_Wait_Time = 0.5;

    //Segment 1 Distance
    static double first_forward = -500;
    static double speed = 0.2;

    //Action 1:


    //Segment 2 Distance
    static  double second_forward = -150;

    //Segment 3 Distance -  strafe distance
    static int first_strafe = 1280;

    @Override
    public void runOpMode() {
        // Initialize hardware
        robot = new RobotHardware();
        robot.init(hardwareMap);
        robot.initIMU();

        //
        robot.frontLeftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        robot.frontRightMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        robot.backLeftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        robot.backRightMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        //
        robot.depositClawServo.setPosition(0.0);
        robot.depositWristServo.setPosition(0.1);
        //robot.depositLeftArmServo.setPosition(0.1);
        //robot.depositRightArmServo.setPosition(0.1);
        robot.intakeSlideServo.setPosition(0.3);// range 0.3 to 0.6
        robot.intakeRightArmServo.setPosition(0.4); // range 0.55 - 0
        robot.intakeLeftArmServo.setPosition(0.4); // range 0.55 - 0
        robot.intakeClawServo.setPosition(0.55);
        robot.intakeRotationServo.setPosition(0.49);

        //
        telemetry.addData("Starting at ", "%7d:%7d",
                robot.frontLeftMotor.getCurrentPosition(),
                robot.frontRightMotor.getCurrentPosition());
        telemetry.addData("Heading",robot.imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES));
        telemetry.update();

        // Wait for the game to start
        waitForStart();

        //Segment 1 movement
        driveToPosition(-420, 0.4,15);

        // Action 1 - move the intake away
        robot.intakeLeftArmServo.setPosition(0.1);
        robot.intakeRightArmServo.setPosition(0.1);
        sleep(500);

        //Action 2.1 - rise up the verticla slide
        Slides_Move(56.5,0.5); //Ris up the vertical slide
        //Action 2.2 - set the position for deposit arm for hung
        robot.depositLeftArmServo.setPosition(0.8);
        robot.depositRightArmServo.setPosition(0.8);
        robot.depositWristServo.setPosition(0.3);
        sleep(1700);

        //Segment 2 movement
        driveToPosition(-140,0.6,15);

        //segment 3 movement
        driveToPosition(300,0.6,15);

        //Action 3 - retract deposit
        robot.depositLeftArmServo.setPosition(0);
        robot.depositRightArmServo.setPosition(0);
        robot.depositWristServo.setPosition(deposit_Wrist_retract_Pos);
        sleep(100);

        //segment 4 movement
        strafeToPosition(1070,0.6,15);

        telemetry.addData("first turn", "ready");
        telemetry.update();
        turnToAngle(82,0.2);
        double heading = robot.imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES);
        telemetry.addData("heading", -1*heading);
        telemetry.update();
        sleep(100);

        telemetry.addData("second turn", "ready");
        telemetry.update();
        turnToAngle(168 ,0.2);

        //Action 4 intake extend out
        robot.intakeClawServo.setPosition(intake_Claw_Open);
        robot.intakeLeftArmServo.setPosition(0);
        robot.intakeRightArmServo.setPosition(0);
        sleep(2000);

        telemetry.addData("Path", "Complete");
        telemetry.update();
    }

    /**
     * Moves the robot forward a specified number of inches at a given speed.
     */
    private void driveToPosition(double dist_mm, double speed,double timeoutS) {
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
        runtime.reset();
        robot.frontLeftMotor.setPower(speed);
        robot.frontRightMotor.setPower(speed);
        robot.backLeftMotor.setPower(speed);
        robot.backRightMotor.setPower(speed);

        // Wait until the robot reaches the target position
        while (opModeIsActive() &&
                (runtime.seconds() < timeoutS) &&
                (robot.frontLeftMotor.isBusy() &&
                        robot.frontRightMotor.isBusy() &&
                        robot.backLeftMotor.isBusy() &&
                        robot.backRightMotor.isBusy())) {
            // display
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
        robot.frontLeftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.frontRightMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.backLeftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.backRightMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        sleep(1000);
    }
    /**
     strafing
     **/
    private void strafeToPosition(double dist_mm, double speed, double timeoutS) {
        int targetPosition = (int)(dist_mm * COUNTS_PER_MM_Drive);

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
        runtime.reset();
        robot.frontLeftMotor.setPower(speed);
        robot.frontRightMotor.setPower(speed);
        robot.backLeftMotor.setPower(speed);
        robot.backRightMotor.setPower(speed);

        // Wait until the robot reaches the target position
        while (opModeIsActive() && (runtime.seconds() < timeoutS) &&
                (robot.frontLeftMotor.isBusy()
                        && robot.frontRightMotor.isBusy()
                        && robot.backLeftMotor.isBusy()
                        && robot.backRightMotor.isBusy())) {
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
        robot.frontLeftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.frontRightMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.backLeftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.backRightMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        sleep(1000);
    }
    /**
     * Turns the robot by a specific angle (in degrees) at a given speed.
     */
    private void turnToAngle(double targetAngle, double speed) {
        // Reset the IMU angle

        //Get the current hading from the IMU
        double currentAngle = -getHeading();
        /**
        while (opModeIsActive() && intakeTimer.seconds() < 10){
            telemetry.addData("Heading",currentAngle);
        }
         **/

        while (opModeIsActive() && Math.abs(targetAngle - currentAngle) > 1.5) { // Tolerance of 1 degree
            double turnDirection = Math.signum(targetAngle - currentAngle); // Positive for clockwise, negative for counter-clockwise

            // Apply power for turning
            robot.frontLeftMotor.setPower(turnDirection * speed);
            robot.backLeftMotor.setPower(turnDirection * speed);
            robot.frontRightMotor.setPower(-turnDirection * speed);
            robot.backRightMotor.setPower(-turnDirection * speed);

            // Update the current angle
            currentAngle = -getHeading();

            telemetry.addData("Current Angle", currentAngle);
            telemetry.addData("Turn Direction", turnDirection);
            telemetry.addData("target - current", targetAngle - currentAngle);
            telemetry.addData("LFMotor", robot.frontLeftMotor.getPower());
            telemetry.addData("LBMotor", robot.backLeftMotor.getPower());
            telemetry.addData("RFMotor", robot.frontRightMotor.getPower());
            telemetry.addData("RBMotor", robot.backRightMotor.getPower());
            telemetry.update();
        }

        // Stop all motors
        robot.frontLeftMotor.setPower(0);
        robot.backLeftMotor.setPower(0);
        robot.frontRightMotor.setPower(0);
        robot.backRightMotor.setPower(0);

        // Reset to RUN_USING_ENCODER mode
        robot.frontLeftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.frontRightMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.backLeftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.backRightMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        sleep(1000);
    }

    /**
     * Returns the current heading angle from the IMU.
     *
     * @return The heading angle in degrees
     */
    private double getHeading() {
        double adjustedHeading =  robot.imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES);
        if (adjustedHeading > 180) {
            adjustedHeading -= 360;
        } else if (adjustedHeading < -180) {
            adjustedHeading += 360;
        }
        return adjustedHeading;
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
        robot.liftMotorLeft.setPower(speed);
        robot.liftMotorRight.setPower(speed);
        while (opModeIsActive() && (robot.liftMotorLeft.isBusy() && robot.liftMotorRight.isBusy())) {

        }
        sleep(1000);
    }

}
