package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

public class RobotHardware {

    /* Declare OpMode members. */
    private LinearOpMode myOpMode = null;

    private ElapsedTime runtime = null;

    private DcMotor leftFrontDrive = null;
    private DcMotor leftBackDrive = null;
    private DcMotor rightFrontDrive = null;
    private DcMotor rightBackDrive = null;
    private DcMotor armMotor = null;
    private Servo leftHand = null;
    private Servo rightHand = null;

    public static final double MID_SERVO = 0.5 ;
    public static final double HAND_SPEED = 0.02 ;
    public static final double ARM_UP_POWER = 0.45;
    public static final double ARM_DOWN_POWER = -0.45;

    public RobotHardware (LinearOpMode OpMode) {myOpMode = OpMode;}

    /**
     * Initialize all the robot's hardware.
     * This method must be called ONCE when the OpMode is initialized.
     * <p>
     * All of the hardware devices are accessed via the hardware map, and initialized.
     */

    public void init() {
        // Define and Initialize Motors (note: need to use reference to actual OpMode).
        leftFrontDrive = myOpMode.hardwareMap.get(DcMotor.class, "left_front_drive");
        leftBackDrive = myOpMode.hardwareMap.get(DcMotor.class, "left_back_drive");
        rightFrontDrive = myOpMode.hardwareMap.get(DcMotor.class, "right_front_drive");
        rightBackDrive = myOpMode.hardwareMap.get(DcMotor.class, "right_back_drive");
        armMotor = myOpMode.hardwareMap.get(DcMotor.class, "arm");

        // To drive forward, most robots need the motor on one side to be reversed, because the axles point on opposite directions.
        // Pushing the left stick forward MUST make robot go forward. So adjust these two lines based on your first test drive.
        // Note: The settings here assume direct drive on left and right wheels. Gear Reduction or 90 Deg drives mat require direction flips.
        leftFrontDrive.setDirection(DcMotor.Direction.REVERSE);
        leftBackDrive.setDirection(DcMotor.Direction.REVERSE);
        rightFrontDrive.setDirection(DcMotor.Direction.FORWARD);
        rightBackDrive.setDirection(DcMotor.Direction.FORWARD);

        //Define and initialize ALL installed servos.
        leftHand = myOpMode.hardwareMap.get(Servo.class, "left_hand");
        rightHand = myOpMode.hardwareMap.get(Servo.class, "right_hand");
        leftHand.setPosition(MID_SERVO);
        rightHand.setPosition(MID_SERVO);

        myOpMode.telemetry.addData(">", "Hardware Initialized");
        myOpMode.telemetry.update();
    }

    /**
     * Calculates the left/right motor powers required to achieve the requested to achieve the requested
     * robot motions: Drive (Axial motion) and Turn (Yaw motion).
     * Then sends these power levels to the motors.
     *
     * @param Drive     Fwd/Rev driving power(-1.0 to 1.0) +ve is forward
     * @param Turn      Right/Left turning power(-1.0 to 1.0) +ve is CW
     * @param Strafe
     */
    public void driveRobotCentric(double Drive,double Turn, double Strafe){
        //Combine drive and turn for blended motion.
        double leftFront = Drive + Turn + Strafe;
        double leftBack = Drive - Turn + Strafe;
        double rightFront = Drive - Turn - Strafe;
        double rightBack = Drive + Turn - Strafe;

        //Scale the values so neither exceed +/-1.0
        double max = Math.max(Math.abs(leftFront), Math.abs(rightFront));
        max = Math.max(max, Math.abs(leftBack));
        max = Math.max(max, Math.abs(rightBack));

        if (max > 1.0)
        {
            leftFront /= max;
            leftBack /= max;
            rightFront /= max;
            rightBack /= max;
        }

        //Use existing function to drive both wheels.
        setDrivePower(leftFront, leftBack, rightFront, rightBack);
    }

    public void driveFieldCentric(double Drive,double Turn, double Strafe){
        //Combine drive and turn for blended motion.
        double leftFront = Drive + Turn + Strafe;
        double leftBack = Drive - Turn + Strafe;
        double rightFront = Drive - Turn - Strafe;
        double rightBack = Drive + Turn - Strafe;

        //Scale the values so neither exceed +/-1.0
        double max = Math.max(Math.abs(leftFront), Math.abs(rightFront));
        max = Math.max(max, Math.abs(leftBack));
        max = Math.max(max, Math.abs(rightBack));

        if (max > 1.0)
        {
            leftFront /= max;
            leftBack /= max;
            rightFront /= max;
            rightBack /= max;
        }

        //Use existing function to drive both wheels.
        setDrivePower(leftFront, leftBack, rightFront, rightBack);
    }

    /**
     * Pass the requested wheel motor powers to the appropriate hardware drive motors.
     *
     * @param leftFrontWheel     Fwd/Rev driving power (-1.0 to 1.0) +ve is forward
     * @param leftBackWheel     Fwd/Rev driving power (-1.0 to 1.0) +ve is forward
     * @param rightFrontWheel    Fwd/Rev driving power (-1.0 to 1.0) +ve is forward
     * @param rightBackWheel    Fwd/Rev driving power (-1.0 to 1.0) +ve is forward
     */
    public void setDrivePower(double leftFrontWheel, double leftBackWheel, double rightFrontWheel, double rightBackWheel) {
        //Output the values to the motor drives.
        leftFrontDrive.setPower(leftFrontWheel);
        leftBackDrive.setPower(leftBackWheel);
        rightFrontDrive.setPower(rightFrontWheel);
        rightBackDrive.setPower(rightBackWheel);
    }

    /**
     *  Pass the requested arm power to the appropriate hardware drive motor
     *
     * @param power driving power (-1.0 to 1.0)
     */
    public void setArmPower(double power) {armMotor.setPower(power); }

    /**
     * Send the two hand-servos to opposing (mirrored) positions, based on the passed offset.
     *
     * @param offset
     */
    public void setHandPositions(double offset) {
        offset = Range.clip(offset, -0.5, 0.5);
        leftHand.setPosition(MID_SERVO + offset);
        rightHand.setPosition(MID_SERVO - offset);
    }
}


