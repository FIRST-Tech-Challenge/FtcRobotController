package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;

import static java.lang.Math.abs;
import static java.lang.Math.cos;
import static java.lang.Math.max;
import static java.lang.Math.sin;
import static java.lang.Math.sqrt;
import static java.lang.Math.toRadians;

/**
 *Created by Ethan
 */
public class HardwareOmnibotDriveOnly
{
    /* Public OpMode members. */
    // Shooter speeds
    public final static double MAX_SPIN_RATE = 0.6;
    public final static double MIN_SPIN_RATE = 0.1;
    public final static double MIN_DRIVE_RATE = 0.1;

    // Robot Controller Config Strings
    public final static String IMU = "imu";
    public final static String FRONT_LEFT_MOTOR = "MotorLF";
    public final static String FRONT_RIGHT_MOTOR = "MotorRF";
    public final static String BACK_LEFT_MOTOR = "MotorLR";
    public final static String BACK_RIGHT_MOTOR = "MotorRR";

    protected DcMotor leftMotorFore = null;
    protected DcMotor rightMotorFore = null;
    protected DcMotor leftMotorRear = null;
    protected DcMotor rightMotorRear = null;
    protected BNO055IMU imu = null;

    private static final int encoderClicksPerSecond = 2800;
    private double leftForeMotorPower = 0.0;
    private double leftRearMotorPower = 0.0;
    private double rightForeMotorPower = 0.0;
    private double rightRearMotorPower = 0.0;

    /* local OpMode members. */
    private HardwareMap hwMap  =  null;

    /* Constructor */
    public HardwareOmnibotDriveOnly(){}



    public void initIMU()
    {
        // Init IMU code
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.calibrationDataFile = "BNO055IMUCalibration.json";
        parameters.loggingEnabled = true;
        parameters.loggingTag = "IMU";
        parameters.accelerationIntegrationAlgorithm = new JustLoggingAccelerationIntegrator();

        imu = hwMap.get(BNO055IMU.class, IMU);
        imu.initialize(parameters);
    }

    /*public void calibrateGyro()
    {
        // Calibrate Gyro Code
        gyro.calibrate();
        while(gyro.isCalibrating()) {
            try {
                Thread.sleep(10);
            } catch (InterruptedException e) {
            }
        }
    }*/

    public void resetGyro()
    {
        // Reset Gyro Code
        //gyro.resetZAxisIntegrator();
    }

    public double readIMU()
    {
        // Read IMU Code
        //double heading = (double)gyro.getHeading();
        Orientation angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        double heading = (double)angles.firstAngle;
        //heading = abs(heading - 360.0);
        return heading;
    }

    public void setLeftForeMotorPower(double power)
    {
        if(power != leftForeMotorPower)
        {
            leftForeMotorPower = power;
            leftMotorFore.setPower(power);
        }
    }

    public void setLeftRearMotorPower(double power)
    {
        if(power != leftRearMotorPower)
        {
            leftRearMotorPower = power;
            leftMotorRear.setPower(power);
        }
    }

    public void setRightForeMotorPower(double power)
    {
        if(power != rightForeMotorPower)
        {
            rightForeMotorPower = power;
            rightMotorFore.setPower(power);
        }
    }

    public void setRightRearMotorPower(double power)
    {
        if(power != rightRearMotorPower)
        {
            rightRearMotorPower = power;
            rightMotorRear.setPower(power);
        }
    }

    public void setAllDriveZero()
    {
        setLeftForeMotorPower(0.0);
        setLeftRearMotorPower(0.0);
        setRightForeMotorPower(0.0);
        setRightRearMotorPower(0.0);
    }


    /**
     *
     * @param xPower - -1.0 to 1.0 power in the X axis
     * @param yPower - -1.0 to 1.0 power in the Y axis
     * @param spin - -1.0 to 1.0 power to rotate the robot, reduced to MAX_SPIN_RATE
     * @param angleOffset - The offset from the gyro to run at, such as drive compensation
     */
    public void drive(double xPower, double yPower, double spin, double angleOffset)
    {
        // Read Gyro Angle Here
        double reducedSpin = spin * MAX_SPIN_RATE;
        double gyroAngle = angleOffset + readIMU();
        double leftFrontAngle = toRadians(45.0 + gyroAngle);
        double rightFrontAngle = toRadians(315.0 + gyroAngle);
        double leftRearAngle = toRadians(135.0 + gyroAngle);
        double rightRearAngle = toRadians(225.0 + gyroAngle);

        if(abs(yPower) < MIN_DRIVE_RATE)
        {
            yPower = 0.0;
        }
        if(abs(xPower) < MIN_DRIVE_RATE)
        {
            xPower = 0.0;
        }
        if(abs(spin) < MIN_SPIN_RATE)
        {
            reducedSpin = 0.0;
        }

        double LFpower = (xPower * cos(leftFrontAngle) + yPower * sin(leftFrontAngle))/sqrt(2) + reducedSpin;
        double LRpower = (xPower * cos(leftRearAngle) + yPower * sin(leftRearAngle))/sqrt(2) + reducedSpin;
        double RFpower = (xPower * cos(rightFrontAngle) + yPower * sin(rightFrontAngle))/sqrt(2) + reducedSpin;
        double RRpower = (xPower * cos(rightRearAngle) + yPower * sin(rightRearAngle))/sqrt(2) + reducedSpin;

        double maxPower = max(1.0, max(max(LFpower, LRpower),
                max(RFpower, RRpower)));

        LFpower /= maxPower;
        RFpower /= maxPower;
        RFpower /= maxPower;
        RRpower /= maxPower;

        setLeftForeMotorPower(LFpower);
        setLeftRearMotorPower(LRpower);
        setRightForeMotorPower(RFpower);
        setRightRearMotorPower(RRpower);
    }

    public void resetDriveEncoders()
    {
        int sleepTime = 0;
        int encoderCount = leftMotorFore.getCurrentPosition();

        rightMotorFore.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftMotorRear.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightMotorRear.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftMotorFore.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        while((encoderCount != 0) && (sleepTime < 1000)) {
            try {
                Thread.sleep(10);
            } catch (InterruptedException e) {
            }
            sleepTime += 10;
            encoderCount = leftMotorFore.getCurrentPosition();
        }

        leftMotorFore.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightMotorFore.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        leftMotorRear.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightMotorRear.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    /* Initialize standard Hardware interfaces */
    public void init(HardwareMap ahwMap) {
        // Save reference to Hardware map
        hwMap = ahwMap;

        // Define and Initialize Motors
        leftMotorFore = hwMap.dcMotor.get(FRONT_LEFT_MOTOR);
        rightMotorFore  = hwMap.dcMotor.get(FRONT_RIGHT_MOTOR);
        leftMotorRear = hwMap.dcMotor.get(BACK_LEFT_MOTOR);
        rightMotorRear = hwMap.dcMotor.get(BACK_RIGHT_MOTOR);

        leftMotorFore.setDirection(DcMotor.Direction.FORWARD);
        rightMotorFore.setDirection(DcMotor.Direction.FORWARD);
        leftMotorRear.setDirection(DcMotor.Direction.FORWARD);
        rightMotorRear.setDirection(DcMotor.Direction.FORWARD);

        // Set all motors to zero power
        setAllDriveZero();

        resetDriveEncoders();

// Until we can set the motor parameters in the motor controller, this slows the robot down.
// Need to update the motor controller firmware to do this.
//        leftMotorFore.setMaxSpeed(encoderClicksPerSecond);
//        rightMotorFore.setMaxSpeed(encoderClicksPerSecond);
//        leftMotorRear.setMaxSpeed(encoderClicksPerSecond);
//        rightMotorRear.setMaxSpeed(encoderClicksPerSecond);
//        shootMotor1.setMaxSpeed(encoderClicksPerSecond);
//        shootMotor2.setMaxSpeed(encoderClicksPerSecond);

        initIMU();
    }

    /***
     *
     * waitForTick implements a periodic delay. However, this acts like a metronome with a regular
     * periodic tick.  This is used to compensate for varying processing times for each cycle.
     * The function looks at the elapsed cycle time, and sleeps for the remaining time interval.
     *
     * @param periodMs  Length of wait cycle in mSec.
     * @throws InterruptedException
     */
    /*
    public void waitForTick(long periodMs) throws InterruptedException {

        long  remaining = periodMs - (long)period.milliseconds();

        // sleep for the remaining portion of the regular cycle period.
        if (remaining > 0)
            Thread.sleep(remaining);

        // Reset the cycle clock for the next pass.
        period.reset();
    }
    */
}

