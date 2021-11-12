/*
Made by Aryan Sinha,
FTC Team 202101101
 */

package org.firstinspires.ftc.teamcode;
//----------------------------------------------------------------------------
import static org.firstinspires.ftc.teamcode.utils.FTCConstants.CAROUSEL_SERVO;
import static org.firstinspires.ftc.teamcode.utils.FTCConstants.CLAW_NAME;
import static org.firstinspires.ftc.teamcode.utils.FTCConstants.CLAW_SERVO;
import static org.firstinspires.ftc.teamcode.utils.FTCConstants.LEFT_MOTOR_NAME;
import static org.firstinspires.ftc.teamcode.utils.FTCConstants.RIGHT_MOTOR_NAME;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.HardwareMap;

/**
 * This class defines the hardware components.
 */
public final class Hardware2
{
    /* Public OpMode members. */
    private DcMotor leftDrive;
    private DcMotor rightDrive;
    private DcMotor claw;
    private Servo carousel;
    private Servo clawServo;
    private final boolean runWithEncoders;
    /* local OpMode members. */
    private HardwareMap hwMap;

    /***
     * Creates an instance of this class that runs without encoders.
     */
    public Hardware2() {
        this(false);
    }

    /**
     * Creates an instance of this class that allows caller to decide whether or not to use encoders.
     * @param runWithEncoders Whether or not to use the encoders.
     */
    public Hardware2(boolean runWithEncoders) {
        this.runWithEncoders = runWithEncoders;
    }

     /** Initialize the drive system variables. The init() method of the hardware class does all the work here
      * REMEMBER that hardwareMap is a reference variable that refers to an object that contains all of the hardware mappings.  It is
      * defined in the OpMode class.
      * @param ahwMap The hardware map.
      */
    public void init(HardwareMap ahwMap) {
        // Save reference to Hardware map
        hwMap = ahwMap;

        // Define and Initialize Motors
        leftDrive  = hwMap.get(DcMotor.class, LEFT_MOTOR_NAME);
        rightDrive = hwMap.get(DcMotor.class, RIGHT_MOTOR_NAME);
        claw = hwMap.get(DcMotor.class, CLAW_NAME);
        carousel = hwMap.get(Servo.class, CAROUSEL_SERVO);
        clawServo = hwMap.get(Servo.class, CLAW_SERVO);

        // Most robots need the motor on one side to be reversed to drive forward
        // Reverse the motor that runs backwards when connected directly to the battery
        leftDrive.setDirection(DcMotor.Direction.REVERSE);
        rightDrive.setDirection(DcMotor.Direction.FORWARD);
        claw.setDirection(DcMotor.Direction.FORWARD);

        // Set all motors to zero power
        leftDrive.setPower(0);
        rightDrive.setPower(0);

        if (runWithEncoders) {
            leftDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            rightDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            rightDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            leftDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODERS);
        } else {
            rightDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            leftDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        }
    }

    /**
     * Returns the left drive motor.
     * @return The left drive motor.
     */
    public DcMotor getLeftDrive() {
        return leftDrive;
    }

    /**
     * Returns the right drive motor.
     * @return The right drive motor.
     */
    public DcMotor getRightDrive() {
        return rightDrive;
    }

    /**
     * Returns the claw.
     * @return The claw.
     */
    public DcMotor getClaw() {
        return claw;
    }

    /**
     * Returns the carousel.
     * @return The carousel.
     */
    public Servo getCarousel() {
        return carousel;
    }

    /**
     * Returns the servo attached to the claw.
     * @return The servo attached to the claw.
     */
    public Servo getClawServo() {
        return clawServo;
    }

    /**
     * Whether or not the code should run with encoders.
     * @return Whether or not the code should run with encoders.
     */
    public boolean isRunWithEncoders() {
        return runWithEncoders;
    }
}