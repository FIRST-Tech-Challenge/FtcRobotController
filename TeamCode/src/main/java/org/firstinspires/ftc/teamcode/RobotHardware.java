package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;

public class RobotHardware {
    /*
    *This file works in conjunction with the external Hardware Class sample called: ConceptExternalHardwareClass.java
    * Please read the explanations in that Sample about how to use this class definition.
    * 
    * This file defines a Java Class that performs all the setup and configuration for a sample robot's hardware 
    * (motors and sensors) It assumes four motors( leftFrontDrive, leftBackDrive, rightBackDrive, and rightFrontDrive) 
    * and x servos
    * 
    * This one file/class can be used by ALL of your OpModes without having to cut & paste the code each time.
    * 
    * Where possible, the actual hardware objects are "abstracted" (or hidden) so the OpMode code just makes calls into 
    * the class, rather than accessing the internal hardware directly. This is why the objects are declared "private".
    * 
    * Use Android Studio to Copy this class and paste into your team's code folder with *exactly the same name*.
    * 
    * Or... In OnBot Java, add a new file named RobotHardware.java, select this sample, and select Not an OpMode.
    * Also add a new OpMode, select the sample ConceptExternalHardwareClass.java and select TeleOp.
    * 
     */
    
    private LinearOpMode myOpMode = null; // gain access to methods in the calling OpMode.
    private DcMotor leftFrontDrive = null;
    private DcMotor rightFrontDrive = null;
    private DcMotor leftBackDrive = null;
    private DcMotor rightBackDrive = null;
    private DcMotor armDrive = null;

    /**
     *Potential Servos
     */

    private Servo servoOne = null;
    private Servo servoTwo = null;
    private Servo servoThree = null;
    private Servo servoFour = null;
    private Servo servoFive = null;
    private Servo servoSix = null;



    //Define Drive constants. Make them public so they CAN be used by the calling OpMode
    public static final double MID_SERVO = 0.5;
    public static final double HAND_SPEED = 0.02; // sets rate to move servo
    public static final double ARM_UP_POWER = 0.45;
    public static final double ARM_DOWN_POWER = -0.45;

    //Define a constructor that allows the OpMode to pass a reference to itself.
    public RobotHardware (LinearOpMode opmode) { myOpMode = opmode; }

    /**
     * Initalize all the robot's hardware.
     * This method must be called ONCE when the OpMode is initialized.
     * <p>
     * All of the hardware devices are accessed via the hardware map, and initialized.
     */
    public void init() {
        //Define and Initialize Motors (note: need to use reference to actual OpMode).
        leftFrontDrive = myOpMode.hardwareMap.get(DcMotor.class, "left_front_drive");
        leftBackDrive = myOpMode.hardwareMap.get(DcMotor.class, "left_back_drive");
        rightFrontDrive = myOpMode.hardwareMap.get(DcMotor.class, "right_front_drive");
        rightBackDrive = myOpMode.hardwareMap.get(DcMotor.class, "right_back_drive");
        armDrive = myOpMode.hardwareMap.get(DcMotor.class, "arm_drive");
        //To drive forward, most robots need the motor on one side to be reversed, because the axels point in opposite directions.
        //Pushing the left stick forward MUST make robot go forward. So adjust these two lines based on your first test drive.
        //Note: The settings here assume direct drive on left and right wheels. Gear Reduction or 90 Deg drives may require direction flips
        leftFrontDrive.setDirection(DcMotor.Direction.REVERSE);
        rightFrontDrive.setDirection(DcMotor.Direction.FORWARD);
        rightBackDrive.setDirection(DcMotor.Direction.FORWARD);
        leftBackDrive.setDirection(DcMotor.Direction.REVERSE);

        //If there are encoders connected switch to RUN_USING_ENCODER mode for greater accuracy
        //frontLeftDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        //frontRightDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        /**
         * might need to be front and back wheels
         */

        //Define and initialize ALL installed servos.

        servoOne = myOpMode.hardwareMap.get(Servo.class, "servo_one");
        servoTwo = myOpMode.hardwareMap.get(Servo.class, "servo_two");
        servoThree = myOpMode.hardwareMap.get(Servo.class, "servo_three");
        servoFour = myOpMode.hardwareMap.get(Servo.class, "servo_four");
        servoFive = myOpMode.hardwareMap.get(Servo.class, "servo_five");
        servoSix = myOpMode.hardwareMap.get(Servo.class, "servo_six");

        myOpMode.telemetry.addData(">", "Hardware Initialized");
        myOpMode.telemetry.update();
    }
/**
 * Calculates the left/right motor powers required to achieve the requested robot motions: Drive (Axial motion) and
 * Turn (Yaw motion). Then sends these power levels to the motors.
 *
 * @param Drive Fwd/Rev driving power (-1.0 to 1.0) +ve is forward
 * @param Strafe
 * @param Turn  Right/Left turning power (-1.0 to 1.0) +ve is CW
 */

public void driveRobot(double Drive, double Strafe ,double Turn) {
    //Combine drive and turn for blended motion ***drive strafe and turn
    double leftFrontPower = Drive + Strafe + Turn;
    double rightFrontPower = Drive - Strafe - Turn;
    double leftBackPower = Drive - Strafe + Turn;
    double rightBackPower = Drive + Strafe - Turn;

    //Scale the values so neither exceed +/- 1.0
    double max = Math.max(Math.abs(leftFrontPower), Math.abs(rightFrontPower));
    max = Math.max(max, Math.abs(leftBackPower));
    max = Math.max(max, Math.abs(rightBackPower));
    if(max > 1.0)
    {
        leftFrontPower /= max;
        rightFrontPower /= max;
        leftBackPower /= max;
        rightBackPower /= max;
    }

    // Use existing function to drive both wheels.
    setDrivePower(leftFrontPower, rightFrontPower, leftBackPower, rightBackPower);
    }

    /**
     * Pass the requested wheel motor powers to the appropriate hardware drive motors.
     *
     * @param leftWheel      Fwd/Rev driving power (-1.0 to 1.0) +ve is forward
     * @param rightWheel     Fwd/Rev driving power (-1.0 to 1.0) +ve is forward
     * @param leftBackPower
     * @param rightBackPower
     */
    //Since there is 4 wheels the parameters might need to be changed to frontLeftWheel and rightFrontWheel and new
    //parameters called backLeftWheel and rightBackWheel added
    public void setDrivePower(double leftWheel, double rightWheel, double leftBackPower, double rightBackPower) {
        //Output the values to the motor drives.
        leftFrontDrive.setPower(leftFrontDrive.getPower());
        leftBackDrive.setPower(leftBackDrive.getPower());
        rightFrontDrive.setPower(rightFrontDrive.getPower());
        rightBackDrive.setPower(rightBackDrive.getPower());
    }

    /**
     * Pass the requested arm power to the appropriate hardware drive motor
     *
     * @param power driving power (-1.0 to 1.0)
     */
    public void setArmPower(double power) { armDrive.setPower(power); }

    /**
     * Send the two hand-servos to opposing (mirrored) positions, based on the passed offset.
     *
     * @param offset
     */
    public void setHandPositions(double offset) {
        offset = Range.clip(offset, -0.5, 0.5);
        //leftHand.setPosition(MID_SERVO + offset);
        //rightHand.setPosition(MID_SERVO - offset);
    }
}