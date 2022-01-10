/*
Made by Aryan Sinha,
FTC Team 202101101
 */

package org.firstinspires.ftc.teamcode.obsoleted.gen1;
//----------------------------------------------------------------------------
import static org.firstinspires.ftc.teamcode.common.utils.FTCConstants.CAROUSEL_SERVO;
import static org.firstinspires.ftc.teamcode.common.utils.FTCConstants.CLAW_NAME;
import static org.firstinspires.ftc.teamcode.common.utils.FTCConstants.CLAW_SERVO;
import static org.firstinspires.ftc.teamcode.common.utils.FTCConstants.LEFT_MOTOR_NAME;
import static org.firstinspires.ftc.teamcode.common.utils.FTCConstants.RIGHT_MOTOR_NAME;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;

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
    private BNO055IMU imu;
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

        imu = hwMap.get(BNO055IMU.class, "imu");
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.mode = BNO055IMU.SensorMode.IMU;
        parameters.angleUnit = BNO055IMU.AngleUnit.DEGREES; //unit for turning
        parameters.accelUnit = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.loggingEnabled = true;
        imu.initialize(parameters);

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

    public void turnRight(double degrees, double power){
        degrees *= -1;
        //Angle before you turn
        double startingAngle = getHeading();
        //Setting motors to turn right
        leftDrive.setPower(power);
        rightDrive.setPower(-power);
        //Waiting until the change in degrees is greater than desired change in degrees

        while ((getHeading()-startingAngle)>=degrees){
        }
        //stop all motors
        leftDrive.setPower(0);
        rightDrive.setPower(0);
    }
    public void turnLeft(double degrees, double power){
        //Angle before you turn
        double startingAngle = getHeading();
        //Setting motors to turn right
        leftDrive.setPower(-power);
        rightDrive.setPower(power);
        //Waiting until the change in degrees is greater than desired change in degrees
        while ((getHeading()-startingAngle)<=degrees){
        }
        //stop all motors
        leftDrive.setPower(0);
        rightDrive.setPower(0);
    }
    public double getHeading() {
        Orientation angles = getImu().getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        double heading = angles.firstAngle;
        return heading;
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

    public BNO055IMU getImu() {return imu;}

    /**
     * Whether or not the code should run with encoders.
     * @return Whether or not the code should run with encoders.
     */
    public boolean isRunWithEncoders() {
        return runWithEncoders;
    }
}