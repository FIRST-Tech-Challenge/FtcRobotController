package org.firstinspires.ftc.teamcode;

/*
This file defines a Java Class that performs all the setup and configuration for our robot's hardware (motors
and sensors). It has five motors (left_front_drive, left_back_drive, right_front_drive, right_back_drive, and arm) and
two servos (left_hand and right_hand)

This on file/class is used by ALL of our OpModes without having to cut and paste the code each time.

Where possible, the actual hardware objects are "abstracted" (or hidden) so the OpMode code just makes calls into the
class rather than accessing the internal hardware directly. This is why the objects are declared "private".
 */

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

public class RobotHardware {

    // Declare OpMode members.
    private LinearOpMode myOpMode = null; // gains access to methods in the calling OpMode.

    // Define Motor and Servo objects (Make them private so they can't be accessed externally)
    public ElapsedTime runtime = new ElapsedTime();
    private DcMotor leftFrontDrive = null;
    private DcMotor leftBackDrive = null;
    private DcMotor rightFrontDrive = null;
    private DcMotor rightBackDrive = null;
    // private DcMotor armDrive = null;
    // private Servo leftHand = null;
    // private Servo rightHand = null;

    // Define Drive constants. (Make them public so they CAN he be used by the calling OpMode)
    public static final double FORWARD_SPEED = 0.6;
    public static final double TURN_SPEED = 0.5;
    public static final double MID_SERVO = 0.5;
    public static final double HAND_SPEED = 0.02;
    public static final double ARM_UP_POWER = 0.45;
    public static final double ARM_DOWN_POWER = -0.45;

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
        // armDrive = myOpMode.hardwareMap.get(DcMotor.class, "arm_drive");

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
        rightFrontDrive.setPower(rightFrontWheel);
        rightBackDrive.setPower(rightBackWheel);

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
        myOpMode.telemetry.update();
    }
}