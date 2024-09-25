package org.firstinspires.ftc.teamcode;

/*
This file defines a Java class that performs all the setup and configuration for our robot's hardware (motors and
sensors). It has 5 motors (left_front_drive, left_back_drive, right_front_drive, right_back_drive) and to servos
(left_hand and right_hand).

This one file/class is used by ALL of your OpModes without having to cut & paste the code each time.

Where possible, the actual hardware objects are "abstracted" (or hidden) so the OpMode code just makes calls into the
class, rather than accessing the internal hardware directly. This is why the objects are declared "private."
 */

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.hardwareMap;

public class RobotHardware {

    // Declare the OpMode members.
    private LinearOpMode myOpMode = null; // gains access to the methods in the calling OpMode.

    //Define motors and servo objects (make them private so they can't be accessed externally)
    private DcMotor leftFrontDrive = null;
    private DcMotor leftBackDrive = null;
    private DcMotor rightFrontDrive = null;
    private DcMotor rightBackDrive = null;
    //private DcMotor arm = null;
    //private Servo leftHand = null;
    //private Servo rightHand = null;

    public IMU imu = null; // Universal IMU interface

    // Define the drive constants. Make them public so they CAN be used by the calling OpMOde
    public static final double MID_SERVO = 0.5;
    public static final double HAND_SPEED = 0.02;
    public static final double ARM_UP_POWER = 0.45;
    public static final double ARM_DOWN_POWER = -0.45;

    //Define encoder constants
    static final double COUNTS_PER_MOTOR_REV = 1440;
    static final double DRIVE_GEAR_REDUCTION = 1.0;
    static final double WHEEL_DIAMETER_INCHES=4.0;
    static final double COUNTS_PER_INCH = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION)/(WHEEL_DIAMETER_INCHES*3.1415);
    static final double DRIVE_SPEED =  0.6;
    static final double TURN_SPEED= 0.5;

    //Define a constructor that allows the OpMode to pass a reference to itself.
    public RobotHardware (LinearOpMode opmode) {myOpMode = opmode;}

    /**
     * initialize all the robot's hardware. This method must be called ONCE when the OpMOde is Initialized.
     * <p>
     * All the hardware devices are accessed via the hardware map, and initialized.
     */

    public void init (){
        // Define and initialize Motors (note: we need to use reference to actual OpMode)
        leftFrontDrive = myOpMode.hardwareMap.get(DcMotor.class, "left_front_drive");
        leftBackDrive = myOpMode.hardwareMap.get(DcMotor.class, "left_back_drive");
        rightFrontDrive = myOpMode.hardwareMap.get(DcMotor.class, "right_front_drive");
        rightBackDrive = myOpMode.hardwareMap.get(DcMotor.class, "right_back_drive");
        //arm= myOpMode.hardwareMap.get(DcMotor.class, "arm");

        /* To drive forward, our robot need the motors on one side to be reversed, because the axles point in opposite
         direction.
         Pushing the left stick forward MUST make robot go forward. So we will adjust these lines if necessary, after
         the first test drive.
         */
        leftFrontDrive.setDirection(DcMotor.Direction.FORWARD);
        leftBackDrive.setDirection(DcMotor.Direction.FORWARD);
        rightFrontDrive.setDirection(DcMotor.Direction.REVERSE);
        rightBackDrive.setDirection(DcMotor.Direction.REVERSE);

        imu= myOpMode.hardwareMap.get(IMU.class, "imu");

        IMU.Parameters parameters = new IMU.Parameters( new RevHubOrientationOnRobot(
                RevHubOrientationOnRobot.LogoFacingDirection.RIGHT,
                RevHubOrientationOnRobot.UsbFacingDirection.UP));
        imu.initialize(parameters);

        //Define and initialize ALL installed servos.
        //leftHand = myOpMode.hardwareMap.get(Servo.class, "left_hand");
        //rightHand= myOpMode.hardwareMap.get(Servo.class, "right_hand");
        //leftHand.setPosition(MID_SERVO);
        //rightHand.setPosition(MID_SERVO);

        myOpMode.telemetry.addData(">", "Hardware Initialized");
        myOpMode.telemetry.update();
    }

    /**
     * Calculates the left/right motor powers required to achieve the requested robot motion;
     * Drive (axial motion) and Turn (Yaw motion).
     * Then sends these power levels to the motors.
     *
     * @param axial     Fwd/Rev driving power  (-1.0 to 1.0) +ve is forward
     * @param lateral   Left/Right driving power (-1.0 to 1.0) +ve is right
     * @param yaw       Right/Left turning power (-1.o to 1.0) +ve is CW
     */


    public void driveRobotCentric(double axial, double lateral, double yaw) {
        double max;

        // Combine drive and Turn for blended motion.
        double leftFrontPower = axial + lateral + yaw;
        double leftBackPower = axial - lateral -yaw;
        double rightFrontPower = axial - lateral + yaw;
        double rightBackPower = axial + lateral -yaw;

        // Scale the values so neither exceed +/- 1.0
        max = Math.max(Math.abs(leftFrontPower) , Math.abs(rightFrontPower));
        max = Math.max(max, Math.abs(leftBackPower));
        max= Math.max(max, Math.abs(rightBackPower));

        if (max > 1.0){
            leftFrontPower /= max;
            leftBackPower /= max;
            rightFrontPower /= max;
            rightBackPower /= max;
        }

        //Use existing function to drive both wheels.
        setDrivePower (leftFrontPower, leftBackPower, rightFrontPower, rightBackPower);
    }

    /**
     * Calculates the left/right motor powers required to achieve the requested robot motion;
     * Drive (axial motion) and Turn (Yaw motion).
     * Then sends these power levels to the motors.
     *
     * @param axial     Fwd/Rev driving power  (-1.0 to 1.0) +ve is forward
     * @param lateral   Left/Right driving power (-1.0 to 1.0) +ve is right
     * @param yaw       Right/Left turning power (-1.o to 1.0) +ve is CW
     */

    public void driveFieldCentric(double axial, double lateral, double yaw) {
        double botHeading
        double max;

        // Combine drive and Turn for blended motion.
        double leftFrontPower = axial + lateral + yaw;
        double leftBackPower = axial - lateral -yaw;
        double rightFrontPower = axial - lateral + yaw;
        double rightBackPower = axial + lateral -yaw;

        // Scale the values so neither exceed +/- 1.0
        max = Math.max(Math.abs(leftFrontPower) , Math.abs(rightFrontPower));
        max = Math.max(max, Math.abs(leftBackPower));
        max= Math.max(max, Math.abs(rightBackPower));

        if (max > 1.0){
            leftFrontPower /= max;
            leftBackPower /= max;
            rightFrontPower /= max;
            rightBackPower /= max;
        }

        //Use existing function to drive both wheels.
        setDrivePower (leftFrontPower, leftBackPower, rightFrontPower, rightBackPower);
    }

    /**
     * Pass the requested wheel motor powers to the appropriate hardware drive motors.
     *
     * @param leftFrontWheel        Fwd/Rev driving power {-1.0 to 1.0) +ve is forward
     * @param leftBackWheel
     * @param rightFrontWheel
     * @param rightBackWheel       Fwd/Rev driving power (-1.0 to 1.0) +ve is forward
     */

    public void setDrivePower(double leftFrontWheel, double leftBackWheel, double rightFrontWheel, double rightBackWheel) {
        //Output the values of the motor drives.
        leftFrontDrive.setPower(leftFrontWheel);
        leftBackDrive.setPower(leftBackWheel);
        rightFrontDrive.setPower(rightFrontWheel);
        rightBackDrive.setPower(rightBackWheel);
    }

    /**
     * Pass the requested arm power to the appropriate hardware drive motor
     *
     * @param power driving power   (-1.0 to 1.0)
     */

    //public void setArmPower (double power) { arm.setPower(power);}

    /**
     * Send the twon hand-servos to opposing (mirrored) positions, based on the passed offset.
     *
     * @param offset
     */
    //public void setHandPositions(double offset) {
        //offset = Range.clip(offset, -0.5, 0.5);
        //leftHand.setPosition(MID_SERVO + offset);
       // rightHand.setPosition(MID_SERVO - offset);

    //}

    public void encoderDrive(double speed, double leftInches, double rightInches, double timeoutS ){
        int newLeftFrontTarget;
        int newLeftBackTarget;
        int newRightFrontTarget;
        int newRightBackTarget;

        if(myOpMode.opModeIsActive()){
            newLeftFrontTarget= leftFrontDrive.getCurrentPosition()+(int)(leftInches*COUNTS_PER_INCH);
        }


    }


}
