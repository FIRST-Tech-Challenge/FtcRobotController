package org.firstinspires.ftc.McQueen;

import static org.firstinspires.ftc.onbotjava.OnBotJavaManager.initialize;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;


@Autonomous
public class calibrateMeca extends LinearOpMode {

    /* Declare OpMode members. */
    private DcMotor frontLeft = null;
    private DcMotor frontRight = null;
    private DcMotor backLeft = null;
    private DcMotor backRight = null;
    public DcMotor armLift = null;
    public DcMotor armExtend = null;
    public Servo claw = null;
    public Servo rotate = null;
    public Servo wrist = null;
    public Servo elbow = null;
    public Servo upperWrist = null;
    private ElapsedTime runtime = new ElapsedTime();

    // Calculate the COUNTS_PER_INCH for your specific drive train.
    // Go to your motor vendor website to determine your motor's COUNTS_PER_MOTOR_REV
    // For external drive gearing, set DRIVE_GEAR_REDUCTION as needed.
    // For example, use a value of 2.0 for a 12-tooth spur gear driving a 24-tooth spur gear.
    // This is gearing DOWN for less speed and more torque.
    // For gearing UP, use a gear ratio less than 1.0. Note this will affect the direction of wheel rotation.
    static final double     COUNTS_PER_MOTOR_REV    = 1440 ;    // eg: TETRIX Motor Encoder
    static final double     DRIVE_GEAR_REDUCTION    = 1.0 ;     // No External Gearing.
    static final double     WHEEL_DIAMETER_INCHES   = 4.0 ;     // For figuring circumference
    static final double     COUNTS_PER_INCH         = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) /
            (WHEEL_DIAMETER_INCHES * 3.1415);
    static final double     DRIVE_SPEED             = 0.6;
    static final double     TURN_SPEED              = 0.5;

    static final double     STRAFE_SPEED            = 0.5;

    static final double     INCHES_PER_NINETY_DEGREES = 14.0;

    boolean anyMotorRunning;

    boolean netSide = false;

    boolean redAlliance = false;

    public int turnDirection = +1;

    @Override
    public void runOpMode(){
        initialize();

        // Initialize the drive system variables.
        frontLeft = hardwareMap.get(DcMotor.class, "frontLeft");
        frontRight = hardwareMap.get(DcMotor.class, "frontRight");
        backLeft = hardwareMap.get(DcMotor.class, "backLeft");
        backRight = hardwareMap.get(DcMotor.class, "backRight");

        // To drive forward, most robots need the motor on one side to be reversed, because the axles point in opposite directions.
        // When run, this OpMode should start both motors driving forward. So adjust these two lines based on your first test drive.
        // Note: The settings here assume direct drive on left and right wheels.  Gear Reduction or 90 Deg drives may require direction flips
        frontLeft.setDirection(DcMotor.Direction.REVERSE);
        frontRight.setDirection(DcMotor.Direction.FORWARD);
        backLeft.setDirection(DcMotor.Direction.REVERSE);
        backRight.setDirection(DcMotor.Direction.FORWARD);

        // Put everything into the basic "yes, I have encoders" mode
        frontLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        frontRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        backLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        backRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        // Do anything else that's autonomous-specific here
        telemetry.addData("Status", "Initialized");
        telemetry.update();

        //User interface for start of match
        while(!isStopRequested() && !gamepad1.right_bumper)
        {
            if (gamepad1.b) redAlliance=true;
            else if (gamepad1.x) redAlliance=false;
            if (gamepad1.y) netSide=true;
            else if (gamepad1.a) netSide=false;

            telemetry.addLine ("Match setup!");
            telemetry.addLine ("Set alliance using B or X");
            telemetry.addLine ("Set netside using Y or A");
            telemetry.addLine ("Press right bumper to exit setup");
            //Signal that we're ready
            telemetry.addLine ("Here goes nothing...");
            telemetry.addLine ("");

            if (redAlliance)
            {
                telemetry.addData("Alliance", "Red");
            }

            else
            {
                telemetry.addData
                        ("Alliance", "Blue");
            }


            if (netSide)
            {
                telemetry.addData
                        ("Side", "Net");
            }
            else
            {
                telemetry.addData
                        ("Side", "Observer pit");
            }

            telemetry.addLine("");
            telemetry.addLine("DO NOT PRESS START!");
            telemetry.update ();
        }

        //Signal that we're ready
        telemetry.addLine ("Auto ready!");
        telemetry.update();

        if (netSide) turnDirection = -1;

        waitForStart();

        // Checks if the drive train is moving at all. This allows us to run 2 things at once!**
        // ** hypothetically.
        if (frontLeft.isBusy()); {
            anyMotorRunning = true;
        }

        if (frontRight.isBusy()); {
            anyMotorRunning = true;
        }

        if (backLeft.isBusy()); {
            anyMotorRunning = true;
        }

        if (backRight.isBusy()); {
            anyMotorRunning = true;
        }

        // Auto start
        // Calibration tests!
        driveForward(12);
        turnRight(360);
        turnRight(-360);
        driveForward(-12);
        strafeLeft(12);
    }

    public void encoderDrive(double speed,
                             double frontLeftInches, double frontRightInches,
                             double backLeftInches, double backRightInches)
    {



        int newFrontLeftTarget;
        int newFrontRightTarget;
        int newBackRightTarget;
        int newBackLeftTarget;


        // Ensure that the OpMode is still active
        if (opModeIsActive()) {

            // Determine new target position, and pass to motor controller
            newFrontLeftTarget = frontLeft.getCurrentPosition() + (int)(frontLeftInches * COUNTS_PER_INCH);
            newFrontRightTarget = frontRight.getCurrentPosition() + (int)(frontRightInches * COUNTS_PER_INCH);
            newBackLeftTarget = backLeft.getCurrentPosition() + (int)(backLeftInches * COUNTS_PER_INCH);
            newBackRightTarget = backRight.getCurrentPosition() + (int)(backRightInches * COUNTS_PER_INCH);

            frontLeft.setTargetPosition(newFrontLeftTarget);
            frontRight.setTargetPosition(newFrontRightTarget);
            backLeft.setTargetPosition(newBackLeftTarget);
            backRight.setTargetPosition(newBackRightTarget);

            // Turn On RUN_TO_POSITION
            frontLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            frontRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            backLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            backRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            // start motion.
            frontLeft.setPower(Math.abs(speed));
            frontRight.setPower(Math.abs(speed));
            backLeft.setPower(Math.abs(speed));
            backRight.setPower(Math.abs(speed));

            // keep looping while we are still active, and there is time left, and both motors are running.
            // Note: We use (isBusy() && isBusy()) in the loop test, which means that when EITHER motor hits
            // its target position, the motion will stop.  This is "safer" in the event that the robot will
            // always end the motion as soon as possible.
            // However, if you require that BOTH motors have finished their moves before the robot continues
            // onto the next step, use (isBusy() || isBusy()) in the loop test.
            while (opModeIsActive() &&
                    (frontLeft.isBusy() && frontRight.isBusy() && backLeft.isBusy() && backRight.isBusy())) {

                // Display it for the driver.
                telemetry.addData("Running to",  " %7d :%7d", newFrontLeftTarget,  newFrontRightTarget);
                telemetry.addData("Currently at",  " at %7d :%7d",
                        frontLeft.getCurrentPosition(), frontRight.getCurrentPosition());
                telemetry.update();
            }

            // Stop all motion;
            frontLeft.setPower(0);
            frontRight.setPower(0);
            backLeft.setPower(0);
            backRight.setPower(0);

            // Turn off RUN_TO_POSITION
            frontLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            frontRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            backLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            backRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

            sleep(250);   // optional pause after each move.
        }
    }

    public void driveForward(double distance, double speed){

        encoderDrive(speed, distance, distance, distance, distance);
    }

    // Purely a convenience function - with this, we can just call driveForward(12)
    // without bothering to state a speed
    public void driveForward(double distance) {
        driveForward(distance, DRIVE_SPEED);
    }

    public void strafeLeft(double distance, double speed){
        encoderDrive(speed, -distance, distance,
                distance, -distance);
    }

    public void strafeLeft(double distance){
        strafeLeft(distance, STRAFE_SPEED); //
    }

    public void turnRight(double degrees, double speed){
        double inches = (degrees / 90.0) * INCHES_PER_NINETY_DEGREES * turnDirection;
        encoderDrive (speed, inches, -inches, inches, -inches);
    }

    public void turnRight(double degrees){
        turnRight (degrees, TURN_SPEED);
    }

    public void driveForwardDiagonalRight(double inches, double speed){
        encoderDrive (speed, inches, 0,
                0, inches);
    }
    public void driveForwardDiagonalRight(double inches){
        driveForwardDiagonalRight(inches, DRIVE_SPEED);
    }

    public void driveForwardDiagonalLeft(double inches, double speed){
        encoderDrive (speed, 0, inches,
                inches, 0);
    }

    public void driveForwardDiagonalLeft(double inches){
        driveForwardDiagonalLeft(inches, DRIVE_SPEED);
    }

}
