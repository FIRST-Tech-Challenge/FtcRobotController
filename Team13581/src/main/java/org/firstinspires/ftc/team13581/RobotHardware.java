package org.firstinspires.ftc.team13581;

/*
This file defines a Java Class that performs all the setup and configuration for our robot's hardware (motors
and sensors). It has five motors (left_front_drive, left_back_drive, right_front_drive, right_back_drive, and arm) and
two servos (left_hand and right_hand)

This on file/class is used by ALL of our OpModes without having to cut and paste the code each time.

Where possible, the actual hardware objects are "abstracted" (or hidden) so the OpMode code just makes calls into the
class rather than accessing the internal hardware directly. This is why the objects are declared "private".
 */

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.*;
import com.qualcomm.robotcore.util.ElapsedTime;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;


public class   RobotHardware {

    // Declare OpMode members.
    private LinearOpMode myOpMode = null; // gains access to methods in the calling OpMode.

    // Define Motor and Servo objects (Make them private so they can't be accessed externally)
    public ElapsedTime runtime = new ElapsedTime();
    public DcMotor leftFrontDrive = null;
    public DcMotor leftBackDrive = null;
    public DcMotor rightFrontDrive = null;
    public DcMotor rightBackDrive = null;
    public DcMotor vSlideDrive = null;
    public CRServo intake = null;
    public Servo leftSlide = null;
    public Servo rightSlide = null;
    public Servo leftWrist = null;
    public Servo rightWrist = null;
    public Servo intake2 = null;
    public IMU imu = null;

    // Calculate the COUNTS_PER_INCH for your specific drive train.
    // Go to your motor vendor website to determine your motor's COUNTS_PER_MOTOR_REV
    // For external drive gearing, set DRIVE_GEAR_REDUCTION as needed.
    // For example, use a value of 2.0 for a 12-tooth spur gear driving a 24-tooth spur gear.
    // This is gearing DOWN for less speed and more torque.
    // For gearing UP, use a gear ratio less than 1.0. Note this will affect the direction of wheel rotation.
    public int newLeftFrontTarget;
    public int newLeftBackTarget;
    public int newRightFrontTarget;
    public int newRightBackTarget;

    private final double ROBOT_DIAG_RADIUS = Math.sqrt(Math.pow(17.5/2f, 2) + Math.pow(14.75/2f, 2));;
    public final double     COUNTS_PER_MOTOR_REV    = 537.6 ;    // AndyMark 20 Motor Encoder
    public final double     DRIVE_GEAR_REDUCTION    = 1.0 ;     // No External Gearing.
    public final double     WHEEL_DIAMETER_INCHES   = 3.77953 ;     // For figuring circumference
    public final double     COUNTS_PER_INCH         = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) /
            (WHEEL_DIAMETER_INCHES * Math.PI);

    public final double DRIVE_SPEED = 0.8;
    public final double TURN_SPEED = 0.8;

    /* Variables to store the speed the intake servo should be set at to intake, and deposit game elements. */
    public final double INTAKE_COLLECT    = -1.0;
    public final double INTAKE_OFF        =  0.0;
    public final double INTAKE_DEPOSIT    =  1.0;

    public final double LEFT_SLIDE_EXTEND =  0.2;
    public final double RIGHT_SLIDE_EXTEND= -0.2;

    public final double LEFT_WRIST_SCORE  =  0.0055;
    public final double RIGHT_WRIST_SCORE =  0;
    public final double LEFT_WRIST_INTAKE =  0.57;
    public final double RIGHT_WRIST_INTAKE=  0;

    public final double VSLIDE_TICKS_PER_DEGREE= 28*19.2/360;
    public final double VSLIDE_START_POSITION= 0;
    public final double VSLIDE_SCORE_SAMPLE_LOW= 3*360*VSLIDE_TICKS_PER_DEGREE;
    public final double VSLIDE_SCORE_SAMPLE_HIGH= 6*360*VSLIDE_TICKS_PER_DEGREE;

    public double vSlidePosition= (int)VSLIDE_START_POSITION;

    // Define a constructor that allows the OpMode to pass a reference to itself.
    public RobotHardware (LinearOpMode opmode) {
        myOpMode = opmode;
    }

    /**
     * Initialize all the robot's hardware. This method must be called ONCE when the OpMode is initialized.
     * <p>
     * All the hardware devices are accessed via the hardware map, and initialized
     */
    public void init() {
        // Define and Initialize Motors. (need to use reference to actual OpMode
        leftFrontDrive = myOpMode.hardwareMap.get(DcMotor.class, "left_front_drive");
        leftBackDrive = myOpMode.hardwareMap.get(DcMotor.class, "left_back_drive");
        rightFrontDrive = myOpMode.hardwareMap.get(DcMotor.class, "right_front_drive");
        rightBackDrive = myOpMode.hardwareMap.get(DcMotor.class, "right_back_drive");
        vSlideDrive = myOpMode.hardwareMap.get(DcMotor.class, "slide_drive");

        // To drive forward, most robot need the motor on one side to be reversed, because the axles point in opposite
        // directions.
        // Pushing the left stick forward MUST make robot go forward. So adjust these two lines based on the first test
        // drive.
        // Note: The settings here assume direct drive on left and right wheels. Gear Reduction or 90 Deg drives may
        // require direction flips
        leftFrontDrive.setDirection(DcMotor.Direction.FORWARD);
        leftBackDrive.setDirection(DcMotor.Direction.FORWARD);
        rightFrontDrive.setDirection(DcMotor.Direction.REVERSE);
        rightBackDrive.setDirection(DcMotor.Direction.REVERSE);

        leftFrontDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftBackDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightFrontDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightBackDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        leftFrontDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        leftBackDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightFrontDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightBackDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        /* Setting zeroPowerBehavior to BRAKE enables a "brake mode". This causes the motor to slow down
        much faster when it is coasting. This creates a much more controllable drivetrain. As the robot
        stops much quicker. */
        leftFrontDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftBackDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightFrontDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightBackDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        vSlideDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        ((DcMotorEx) vSlideDrive).setCurrentAlert(5, CurrentUnit.AMPS);

        vSlideDrive.setTargetPosition(0);
        vSlideDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        vSlideDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        /* Define and initialize servos.*/
        intake = myOpMode.hardwareMap.get(CRServo.class, "intake");
        leftSlide = myOpMode.hardwareMap.get(Servo.class, "left_slide");
        rightSlide = myOpMode.hardwareMap.get(Servo.class, "right_slide");
        leftWrist = myOpMode.hardwareMap.get(Servo.class, "left_wrist");
        rightWrist = myOpMode.hardwareMap.get(Servo.class, "right_wrist");
        intake2 = myOpMode.hardwareMap.get(Servo.class, "intake2");

        /* Make sure that the intake is off, and the wrist is folded in. */
        intake.setPower(INTAKE_OFF);
        leftWrist.setPosition(LEFT_WRIST_SCORE);
        intake2.setPosition(0);
        //rightWrist.setPosition(RIGHT_WRIST_SCORE);

        imu = myOpMode.hardwareMap.get(IMU.class, "imu");

        //If there are encoders connected, switch to RUN_USING_ENCODER mode for greater accuracy
        //leftFrontDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        //leftBackDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        //rightFrontDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        //rightBackDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        //Define and initialize ALL installed servos.
        // leftHand = myOpMode.hardwareMap.get(Servo.class, "left_hand");
        // rightHand = myOpMode.hardwareMap.get(Servo.class, "right_hand");
        // leftHand.setPosition(MID_SERVO);
        // rightHand.setPosition(MID_SERVO);

        myOpMode.telemetry.addData(">", "Hardware Initialized");
        myOpMode.telemetry.update();
    }

    /**
     * Calculates the leftFront/leftBack/rightFront/rightBack motor powers required to achieve the requested
     * robot motions: Drive (Axial motion) Strafe (Lateral motion) and Turn (Yaw motion).
     * Then sends these power levels to the motors.
     *
     * @param axial    Fwd/Rev driving power (-1.0 to 1.0) +ve is forward
     * @param lateral  Right/Left Driving power (-1.0 to 1.0) +ve is right
     * @param yaw      Right/Left turning power (-1.0 to 1.0) +ve is CW
     */
    public void driveRobot(double axial, double lateral, double yaw) {
        // Combine the joystick requests for each axis-motion to determine each wheel's power.
        //Set up a variable for each drive wheel to save the power level for telemetry.
        double leftFrontPower = axial + lateral + yaw;
        double rightFrontPower = axial - lateral - yaw;
        double leftBackPower = axial - lateral + yaw;
        double rightBackPower = axial + lateral - yaw;

        // Normalize the values so no wheel power exceeds 100%. This ensures that the robot maintains the desired
        // motion.
        double max = Math.max(Math.abs(leftFrontPower), Math.abs(leftBackPower));
        max = Math.max(max, Math.abs(rightFrontPower));
        max = Math.max(max, Math.abs(rightBackPower));

        if (max > 1.0) {
            leftFrontPower /= max;
            leftBackPower /= max;
            rightFrontPower /= max;
            rightBackPower /= max;
        }

        // Use the existing function to drive all wheels.
        setDrivePower(leftFrontPower, leftBackPower, rightFrontPower, rightBackPower);

    }

    /**
     * Pass the requested wheel motor powers to the appropriate hardware drive motors.
     *
     * @param leftFrontWheel    Fwd/Rev driving power (-1.0 to 1.0) + ve is forward
     * @param leftBackWheel    Fwd/Rev driving power (-1.0 to 1.0) + ve is forward
     * @param rightFrontWheel    Fwd/Rev driving power (-1.0 to 1.0) + ve is forward
     * @param rightBackWheel    Fwd/Rev driving power (-1.0 to 1.0) + ve is forward
     */
    public void setDrivePower(double leftFrontWheel, double leftBackWheel, double rightFrontWheel, double rightBackWheel) {
        // Output the values to the motor drives.
        leftFrontDrive.setPower(leftFrontWheel);
        leftBackDrive.setPower(leftBackWheel);
        rightBackDrive.setPower(rightBackWheel);
        rightFrontDrive.setPower(rightFrontWheel);

        /* This is test code: Uncomment the following code to test your motor directions. Each button should make
        the corresponding motor run FORWARD. 1) First get all the motors to take to correct position on the robot
        by adjusting your Robot Configuration if necessary. 2) Then make sure they run in the correct direction by
        modifying the setDirection() calls above. Once the correct motors move in the correct direction re-comment
        this code
         */

            /*
            leftFrontWheel = myOpMode.gamepad1.x ? 1.0 : 0.0; // X gamepad
            leftBackWheel = myOpMode.gamepad1.a ? 1.0 : 0.0; // A gamepad
            rightFrontWheel = myOpMode.gamepad1.y ? 1.0 : 0.0; // Y gamepad
            rightBackWheel = myOpMode.gamepad1.b ? 1.0 : 0.0; // B gampad
            */

        // Show the elapsed game time and wheel power.
        myOpMode.telemetry.addData("Status", "Run Time: " + runtime.toString());
        myOpMode.telemetry.addData("Front left/Right", "%4.2f, %4.2f", leftFrontWheel, rightFrontWheel);
        myOpMode.telemetry.addData("Back  left/Right", "%4.2f, %4.2f", leftBackWheel, rightBackWheel);
        myOpMode.telemetry.addData("armTarget: ", vSlideDrive.getTargetPosition());
        myOpMode.telemetry.addData("arm Encoder: ", vSlideDrive.getCurrentPosition());
        myOpMode.telemetry.update();
    }

    public void encoderDrive(double speed, double leftFrontInches, double leftBackInches,
                             double rightFrontInches, double rightBackInches, double timeoutS) {



        // Ensure that the OpMode is still active
        if (myOpMode.opModeIsActive()) {

            // Determine new target position, and pass to motor controller
            newLeftFrontTarget = leftFrontDrive.getCurrentPosition() + (int)(leftFrontInches * COUNTS_PER_INCH);
            newLeftBackTarget = leftBackDrive.getCurrentPosition() + (int)(leftBackInches * COUNTS_PER_INCH);
            newRightFrontTarget = rightFrontDrive.getCurrentPosition() + (int)(rightFrontInches * COUNTS_PER_INCH);
            newRightBackTarget = rightBackDrive.getCurrentPosition() + (int)(rightBackInches * COUNTS_PER_INCH);

            leftFrontDrive.setTargetPosition(newLeftFrontTarget);
            leftBackDrive.setTargetPosition(newLeftBackTarget);
            rightFrontDrive.setTargetPosition(newRightFrontTarget);
            rightBackDrive.setTargetPosition(newRightBackTarget);

            // Turn On RUN_TO_POSITION
            leftFrontDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            leftBackDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            rightFrontDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            rightBackDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            // reset the timeout time and start motion.
            runtime.reset();
            leftFrontDrive.setPower(Math.abs(speed));
            leftBackDrive.setPower(Math.abs(speed));
            rightFrontDrive.setPower(Math.abs(speed));
            rightBackDrive.setPower(Math.abs(speed));

            // keep looping while we are still active, and there is time left, and both motors are running.
            // Note: We use (isBusy() && isBusy()) in the loop test, which means that when EITHER motor hits
            // its target position, the motion will stop.  This is "safer" in the event that the robot will
            // always end the motion as soon as possible.
            // However, if you require that BOTH motors have finished their moves before the robot continues
            // onto the next step, use (isBusy() || isBusy()) in the loop test.
            while (myOpMode.opModeIsActive() &&
                    (runtime.seconds() < timeoutS) &&
                    (leftFrontDrive.isBusy() && leftBackDrive.isBusy() &&
                            rightFrontDrive.isBusy() && rightBackDrive.isBusy())) {

                // Display it for the driver.
                myOpMode.telemetry.addData("Running to",  " %7d :%7d :%7d :%7d",
                        newLeftFrontTarget, newLeftBackTarget, newRightFrontTarget, newRightBackTarget);
                myOpMode.telemetry.addData("Currently at",  " at %7d :%7d :%7d :%7d",
                        leftFrontDrive.getCurrentPosition(), leftBackDrive.getCurrentPosition(),
                        rightFrontDrive.getCurrentPosition(), rightBackDrive.getCurrentPosition());
                myOpMode.telemetry.update();
            }

            // Stop all motion;
            setDrivePower(0, 0, 0, 0);

            // Turn off RUN_TO_POSITION
            leftFrontDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            leftBackDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            rightFrontDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            rightBackDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

            myOpMode.sleep(250);   // optional pause after each move.
        }
    }

    public void setvSlideDrivePosition(){
        vSlideDrive.setTargetPosition((int) (vSlidePosition));
        ((DcMotorEx) vSlideDrive).setVelocity(2100);
        vSlideDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    }

    public void setHorizontalSlidePosition(double position) {
        // limit servo range to min and max of the slide mechanism
        position /= 4.5;

        leftSlide.setPosition(position);
        rightSlide.setPosition(1.0 - position);
    }

    public void encoderFieldCentric(double driveIN, double strafeIN, double turnDEG){
        double botHeading = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);

        // Rotate the movement direction counter to the bot rotation
        double strafeRotation = (strafeIN*COUNTS_PER_INCH) * Math.cos(-botHeading) - (driveIN*COUNTS_PER_INCH) * Math.sin(-botHeading);
        double driveRotation = (strafeIN*COUNTS_PER_INCH) * Math.sin(-botHeading) + (driveIN*COUNTS_PER_INCH) * Math.cos(-botHeading);

        // sector length formula (rad*radius)
        double turnTICKS = (Math.toRadians(turnDEG) * ROBOT_DIAG_RADIUS) *COUNTS_PER_INCH;

        newLeftFrontTarget = (int) (driveRotation + strafeRotation + turnTICKS);
        newLeftBackTarget = (int) (driveRotation - strafeRotation + turnTICKS);
        newRightFrontTarget = (int) (driveRotation - strafeRotation - turnTICKS);
        newRightBackTarget = (int) (driveRotation +  strafeRotation - turnTICKS);

        leftFrontDrive.setTargetPosition(leftFrontDrive.getCurrentPosition() + newLeftFrontTarget);
        leftBackDrive.setTargetPosition(leftBackDrive.getCurrentPosition() + newLeftBackTarget);
        rightFrontDrive.setTargetPosition(rightFrontDrive.getCurrentPosition() + newRightFrontTarget);
        rightBackDrive.setTargetPosition(rightBackDrive.getCurrentPosition() + newRightBackTarget);
    }
}

