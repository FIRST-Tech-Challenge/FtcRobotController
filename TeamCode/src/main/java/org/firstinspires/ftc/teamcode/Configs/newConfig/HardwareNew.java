/*
Made by Aryan Sinha,
FTC Team 20020

 */

package org.firstinspires.ftc.teamcode.Configs.newConfig;
//----------------------------------------------------------------------------
import static org.firstinspires.ftc.teamcode.Configs.utils.FTCConstants.BLEFT_MOTOR_NAME;
import static org.firstinspires.ftc.teamcode.Configs.utils.FTCConstants.BRIGHT_MOTOR_NAME;
import static org.firstinspires.ftc.teamcode.Configs.utils.FTCConstants.LEFT_MOTOR_NAME;
import static org.firstinspires.ftc.teamcode.Configs.utils.FTCConstants.RIGHT_MOTOR_NAME;


import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;

/**
 * This class defines the hardware components.
 * @author aryansinha
 * @soon-to-be-author karthikperi
 */
public final class HardwareNew
{
    /* Public OpMode members. */
    private DcMotor frontLeftDrive;
    private DcMotor frontRightDrive;
    private DcMotor backLeftDrive;
    private DcMotor backRightDrive;
    private DcMotor Carousel;
    private Servo leftClaw;
    private Servo rightClaw;
    private DcMotor arm;
    //private ColorSensor colorSensor;


    private BNO055IMU imu;
    private final boolean runWithEncoders;
    /* local OpMode members. */
    private HardwareMap hwMap;

    /**
     * Creates an instance of this class that runs without encoders.
     */
    public HardwareNew() {
        this(false);
    }

    /**
     * Creates an instance of this class that allows caller to decide whether or not to use encoders.
     * @param runWithEncoders Whether or not to use the encoders.
     */
    public HardwareNew(boolean runWithEncoders) {
        this.runWithEncoders = runWithEncoders;
    }

    /** Initialize the drive system variables.
     * @author aryansinha
     * @param ahwMap The hardware map *hardwareMap*.
     */
    public void init(HardwareMap ahwMap) {

        // Save reference to Hardware map
        hwMap = ahwMap;

        // Define and Initialize Motors
        frontLeftDrive  = hwMap.get(DcMotor.class, LEFT_MOTOR_NAME);
        frontRightDrive = hwMap.get(DcMotor.class, RIGHT_MOTOR_NAME);
        backLeftDrive  = hwMap.get(DcMotor.class, BLEFT_MOTOR_NAME);
        backRightDrive = hwMap.get(DcMotor.class, BRIGHT_MOTOR_NAME);
        Carousel = hwMap.get(DcMotor.class, "Carousel");
        arm = hwMap.get(DcMotor.class, "arm");
        leftClaw = hwMap.get(Servo.class, "leftClaw");
        rightClaw = hwMap.get(Servo.class, "rightClaw");
        //colorSensor = hwMap.get(ColorSensor.class, "colorSensor");

        // Most robots need the motor on one side to be reversed to drive forward
        // Reverse the motor that runs backwards when connected directly to the battery
        frontLeftDrive.setDirection(DcMotor.Direction.REVERSE);
        frontRightDrive.setDirection(DcMotor.Direction.FORWARD);
        backLeftDrive.setDirection(DcMotor.Direction.REVERSE);
        backRightDrive.setDirection(DcMotor.Direction.FORWARD);
        arm.setDirection(DcMotor.Direction.FORWARD);
        leftClaw.setDirection(Servo.Direction.REVERSE);

        // Set all motors to zero power
        frontRightDrive.setPower(0);
        frontLeftDrive.setPower(0);
        backRightDrive.setPower(0);
        backLeftDrive.setPower(0);
        arm.setPower(0);
        Carousel.setPower(0);

        imu = hwMap.get(BNO055IMU.class, "imu");
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.mode = BNO055IMU.SensorMode.IMU;
        parameters.angleUnit = BNO055IMU.AngleUnit.DEGREES; //unit for turning
        parameters.accelUnit = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.loggingEnabled = true;
        imu.initialize(parameters);

        if (runWithEncoders) {
            frontLeftDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            frontRightDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            backLeftDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            backRightDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            frontRightDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            frontLeftDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODERS);
            backRightDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            backLeftDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODERS);
        } else {
            frontRightDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            frontLeftDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            backRightDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            backLeftDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        }
    }

    /**
     * Turn right
     * @author aryansinha
     * @param degrees rotational degrees
     * @param power Motor power
     */
    public void turnRight(double degrees, double power){
        degrees*=-1;
        //Angle before you turn
        double startingAngle = getHeading();
        //Setting motors to turn right
        frontLeftDrive.setPower(power);
        frontRightDrive.setPower(-power);
        backLeftDrive.setPower(power);
        backRightDrive.setPower(-power);
        //Waiting until the change in degrees is greater than desired change in degrees

        while ((getHeading()-startingAngle)>=degrees);
        //stop all motors
        frontLeftDrive.setPower(0);
        frontRightDrive.setPower(0);
        backLeftDrive.setPower(0);
        backRightDrive.setPower(0);
    }

    /**
     * Turn left
     * @author aryansinha
     * @param degrees degrees
     * @param power Motor power
     */
    public void turnLeft(double degrees, double power){

        //Angle before you turn
        double startingAngle = getHeading();
        //Setting motors to turn right
        frontLeftDrive.setPower(-power);
        frontRightDrive.setPower(power);
        backLeftDrive.setPower(-power);
        backRightDrive.setPower(power);
        //Waiting until the change in degrees is greater than desired change in degrees
        while ((getHeading()-startingAngle)<=degrees){
        }
        //stop all motors
        frontLeftDrive.setPower(0);
        frontRightDrive.setPower(0);
        backLeftDrive.setPower(0);
        backRightDrive.setPower(0);
    }
    public double getHeading() {
        Orientation angles = getImu().getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        double heading = angles.firstAngle;
        return heading;
    }

    /**
     * Set the power of all the motors
     * @author aryansinha
     * @param base
     * @param power
     * @param time
     */
    public void setAllPower(BaseNewOpMode base, double power, long time)
    {
        backLeftDrive.setPower(power);
        backRightDrive.setPower(power);
        frontRightDrive.setPower(power);
        frontLeftDrive.setPower(power);
        base.sleep(time);
        backLeftDrive.setPower(0);
        backRightDrive.setPower(0);
        frontRightDrive.setPower(0);
        frontLeftDrive.setPower(0);
    }

    /**
     * Returns the left drive motor.
     * @return The left drive motor.
     */
    public DcMotor getLeftDrive() {
        return frontLeftDrive;
    }

    /**
     * Returns the right drive motor.
     * @return The right drive motor.
     */
    public DcMotor getRightDrive() {
        return frontRightDrive;
    }

    /**
     * Retruns back right motor
     * @return The back right motor
     */
    public DcMotor getBackRightDrive() {
        return backRightDrive;
    }
    public DcMotor getBackLeftDrive() {
        return backLeftDrive;
    }
    public DcMotor getArm() { return arm; }

    public Servo getLeftClaw() { return leftClaw; }
    public Servo getRightClaw() { return rightClaw; }

    public BNO055IMU getImu() {return imu;}

    /**
     * Whether or not the code should run with encoders.
     * @return Whether or not the code should run with encoders.
     */
    public boolean isRunWithEncoders() {
        return runWithEncoders;
    }

    public DcMotor getCarousel() { return Carousel; }
    //public ColorSensor getColorSensor() { return colorSensor; }
}