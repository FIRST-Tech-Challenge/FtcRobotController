package org.firstinspires.ftc.team13588;

/*
 *This file defines a Java class that performs the configuration and setup for our robot's
 *  hardware (motors and sensors).
 * It assumes 5 motors (left_front_drive, left_back_drive, right_front_drive, right_back_drive, and arm) and
 * 2 servos (intake and wrist)
 *
 * This one class can be used by ALL of our OpMode without having to cut and paste the code each time.
 *
 * Where possible, the actual hardware objects are "abstracted" (or hidden) so the OpMode code just makes calls
 * into the class, rather than accessing the internet hardware directly. This is why the objects are declared "private".
 */

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;
import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;

public class RobotHardware {
    // Declare OpModes members.
    private LinearOpMode myOpMode = null;    // gain access to the methods in the calling OpMode.

    // Define Motor and Servo objects (Make them private so they can't ve accessed externally)
    private DcMotor leftFrontDrive = null;
    private DcMotor leftBackDrive = null;
    private DcMotor rightFrontDrive = null;
    private DcMotor rightBackDrive = null;
    public DcMotor armDrive = null;
    public DcMotor liftDrive = null;
    public CRServo intake = null;
    public Servo wrist = null;

    /* This constant is the number of encoder ticks for each degree of rotation of the arm.
   To find this, we first need to consider the total gear reduction powering our arm.
   First, we have an external 20t:100t (5:1) reduction created by two spur gears.
   But we also have an internal gear reduction in our motor.
   The motor we use for this arm is a 60RPM Yellow Jacket. Which has an internal gear
   reduction of ~99.5:1. (more precisely it is 13904/261:1)
   We can multiply these two ratios together to get our final reduction.
   The motor's encoder counts 28 times per rotation. So in total you should see about 7458.08
   counts per rotation of the arm. We divide that by 360 to get the counts per degree. */
    public static final double ARM_TICKS_PER_DEGREE =
            28 // number of encoder ticks per rotation of the bare motor
                    * 13904.0 / 261.0 //This is the exact gear ratio of the 99.5:1 Yellow Jacket gearbox
                    * 100.0 / 20.0 // This is the external gear reduction, a 20T pinion gear that drives a 100T hub-mount gear
                    * 1/360.0; // we want ticks per degree, not per rotation

    public static final double LIFT_TICKS_PER_DEGREE =
            28 // number of encoder ticks per rotation of the bare motor
                    * 6000.0/313.0 // This is the exact gear ratio of the 312 rpm Yellow Jacket gearbox
                    * 1/360; // we want ticks per degree, not per rotation.


    /* These constants hold the position that the arm is commanded to run to.
    These are relative to where the arm was located when you start the OpMode. So make sure the
    arm is reset to collapsed inside the robot before you start the program.

    In these variables you'll see a number in degrees, multiplied by the ticks per degree of the arm.
    This results in the number of encoder ticks the arm needs to move in order to achieve the ideal
    set position of the arm. For example, the ARM_SCORE_SAMPLE_IN_LOW is set to
    160 * ARM_TICKS_PER_DEGREE. This asks the arm to move 160° from the starting position.
    If you'd like it to move further, increase that number. If you'd like it to not move
    as far from the starting position, decrease it. */

    public static final double ARM_COLLAPSED_INTO_ROBOT  = 0;
    public static final double ARM_COLLECT               = 200 * ARM_TICKS_PER_DEGREE;
    public static final double ARM_CLEAR_BARRIER         = 270 * ARM_TICKS_PER_DEGREE;
    public static final double ARM_SCORE_SPECIMEN        = 130 * ARM_TICKS_PER_DEGREE;
    public static final double ARM_SCORE_SAMPLE_IN_LOW   = 160 * ARM_TICKS_PER_DEGREE;
    public static final double ARM_SCORE_SAMPLE_IN_HIGH  = 160 * ARM_TICKS_PER_DEGREE;
    public static final double ARM_ATTACH_HANGING_HOOK   = 120 * ARM_TICKS_PER_DEGREE;
    public static final double ARM_WINCH_ROBOT           = 18  * ARM_TICKS_PER_DEGREE;

    /* Variables to store the speed the intake servo should be set at to intake, and deposit game elements. */
    public static final double INTAKE_COLLECT    = -0.85;
    public static final double INTAKE_OFF        =  0.0;
    public static final double INTAKE_DEPOSIT    =  0.5;

    /* Variables to store the positions that the wrist should be set to when folding in, or folding out. */
    public static final double WRIST_FOLDED_IN   = 1.15;
    public static final double WRIST_FOLDED_OUT  = -0.5;

    /* Variables to store the position that the arm extends and retracts. */
    public static final double EXTEND  = 5 * LIFT_TICKS_PER_DEGREE;
    public static final double  RETRACT = 0 * LIFT_TICKS_PER_DEGREE;

    /* A number in degrees that the triggers can adjust the arm position by */
    public static final double FUDGE_FACTOR = 15 * ARM_TICKS_PER_DEGREE;

    /* Variables that are used to set the arm to a specific position */
    double armPosition = (int)ARM_COLLAPSED_INTO_ROBOT;
    double liftPosition = (int)RETRACT;
    double armPositionFudgeFactor;

    // Define a constructor that allows the OpMode to pass a reference to itself.
    public RobotHardware (LinearOpMode opmode)  {myOpMode = opmode;}

    /**
     * Initialise all the robot;s hardware. This method must be called ONCE when the OpMode is Initialised.
     * </p>
     * All the hardware devices are accessed via the hardware map, and Initialised.
     */

    public void init() {
        leftFrontDrive  = myOpMode.hardwareMap.get(DcMotor.class, "left_front_drive");
        leftBackDrive = myOpMode.hardwareMap.get(DcMotor.class, "left_back_drive");
        rightFrontDrive = myOpMode.hardwareMap.get(DcMotor.class, "right_front_drive");
        rightBackDrive = myOpMode.hardwareMap.get(DcMotor.class, "right_back_drive");
        armDrive = myOpMode.hardwareMap.get(DcMotor.class, "arm_drive");
        liftDrive = myOpMode.hardwareMap.get(DcMotor.class, "arm_extension");


        /* To drive forward, our robot need the motors on one side to be reversed, because the axles point in opposite
        direction
        Pushing the left stick forward MUST make robot go forward. So we will adjust these line if necessary, after
        the first test drive,
         */

        leftFrontDrive.setDirection(DcMotor.Direction.FORWARD);
        leftBackDrive.setDirection(DcMotor.Direction.FORWARD);
        rightFrontDrive.setDirection(DcMotor.Direction.REVERSE);
        rightBackDrive.setDirection(DcMotor.Direction.REVERSE);

        leftFrontDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftBackDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightFrontDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightBackDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        armDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        liftDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        /*This sets the maximum current that the control hub will apply to the arm before throwing a flag */
        ((DcMotorEx) armDrive).setCurrentAlert(5, CurrentUnit.AMPS);
        ((DcMotorEx) liftDrive).setCurrentAlert(5, CurrentUnit.AMPS);

        /* Before starting the armMotor. We'll make sure the TargetPosition is set to 0.
        Then we'll set the RunMode to RUN_TO_POSITION. And we'll ask it to stop and reset encoder.
        If you do not have the encoder plugged into this motor, it will not run in this code. */
        armDrive.setTargetPosition(0);
        armDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        armDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        /* Define and initialize servos.*/
        intake = myOpMode.hardwareMap.get(CRServo.class, "intake");
        wrist  = myOpMode.hardwareMap.get(Servo.class, "wrist");

        /* Make sure that the intake is off, and the wrist is folded in. */
        intake.setPower(INTAKE_OFF);
        wrist.setPosition(WRIST_FOLDED_IN);

        /*Define and Initialize new arm extension motor*/
        liftDrive.setTargetPosition(0);
        liftDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        liftDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        myOpMode.telemetry.addData(">", "Hardware Initialized");
        myOpMode.telemetry.update();
    }

    /**
     *
     *  Calculates the left/right motor powers required to achieve the requested
     *  robot motions: Drive (Axial motion) and Turn (Yaw motion).
     * Then sends these power levels to the motors.
     *
     * @param axial     Fwd/Rev driving power (-1.0 to 1.0) +ve is forward
     * @param lateral   Left?Right driving power (-1.0 to 1.0) +ve is right
     * @param yaw      Right/Left turning power (-1.0 to 1.0) +ve is CW
     */

    public void driveRobot(double axial, double lateral, double yaw) {
        double max;

        // Combine drive and turn for blended motion.
        double leftFrontPower  = -axial + lateral + yaw;
        double leftBackPower = axial - lateral - yaw;
        double rightFrontPower = axial + lateral - yaw;
        double rightBackPower = axial - lateral + yaw;


        //Scale the values so neither exceed +/- 1.0
        max = Math.max(Math.abs(leftFrontPower) ,  Math.abs(rightFrontPower));
        max = Math.max(max, Math.abs(leftBackPower));
        max = Math.max(max, Math.abs(rightBackPower));

        if (max > 1.0)  {
            leftFrontPower  /= max;
            rightFrontPower /= max;
            leftBackPower   /= max;
            rightBackPower  /= max;
        }

        // Use existing function to drive wheels.
        setDrivePower (leftFrontPower, rightFrontPower, leftBackPower, rightBackPower );

    }

    /**
     * Pass the requested wheel motor powers to the appropriate hardware drive motors.
     *
     * @param leftFrontWheel     Fed/Rev driving power {-1.0 to 1.0}  +ve is forward
     * @param leftBackWheel
     * @param rightFrontWheel
     * @param rightBackWheel    Fed/Rev driving power (-1.0 to 1.0)  +ve is forward
     */

    public void setDrivePower(double leftFrontWheel, double leftBackWheel,  double rightFrontWheel, double rightBackWheel) {
        leftFrontDrive.setPower(leftFrontWheel);
        leftBackDrive.setPower(leftBackWheel);
        rightFrontDrive.setPower((rightFrontWheel));
        rightBackDrive.setPower(rightBackWheel);
    }

}
