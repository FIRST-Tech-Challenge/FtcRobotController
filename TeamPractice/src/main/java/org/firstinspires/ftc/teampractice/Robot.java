package org.firstinspires.ftc.teampractice;

import android.content.Context;

import com.qualcomm.ftccommon.SoundPlayer;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.ColorRangeSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotor.ZeroPowerBehavior;
import com.qualcomm.robotcore.hardware.DcMotor.RunMode;
import com.qualcomm.robotcore.hardware.DcMotorSimple.Direction;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.TouchSensor;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;

import java.util.Arrays;
import java.util.List;

public class Robot {
    public DcMotorEx leftFront, leftRear, rightRear, rightFront;
    public DcMotorEx armMotor;
    public Servo grabberServo, frontServo;
    private List<DcMotorEx> motors;
    public DigitalChannel limitSwitch;
    public TouchSensor frontSensor;
    public DistanceSensor rearSensor, leftSensor, rightSensor;
    public ColorRangeSensor colorSensor;

    private BNO055IMU imu;
    private Context _appContext;

    private static final double MAX_VELOCITY = 2800d;
    private int[] beepSoundID = new int[2];

    private static final double COUNTS_PER_MOTOR_REV = 560d;    // eg: HD Hex Motor 20:1 560, core hex 288, 40:1 1120
    private static final double DRIVE_GEAR_REDUCTION = 1d;     // This is < 1.0 if geared UP, eg. 26d/10d
    private static final double WHEEL_DIAMETER_INCHES = 2.953d;     // For figuring circumference
    static final double COUNTS_PER_INCH = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) /
            (WHEEL_DIAMETER_INCHES * 3.14159265359d);

    public static volatile double grabberPosition;
    private static volatile boolean frontServoPosition;
    private RunMode armMode;

    public void init(HardwareMap hardwareMap, boolean initIMU) {
        frontSensor = hardwareMap.get(TouchSensor.class, "sensor_front");
        rearSensor = hardwareMap.get(DistanceSensor.class, "sensor_rear");
        colorSensor = hardwareMap.get(ColorRangeSensor.class, "sensor_color");
        leftSensor = hardwareMap.get(DistanceSensor.class, "sensor_left");
        rightSensor = hardwareMap.get(DistanceSensor.class, "sensor_right");

        leftFront = hardwareMap.get(DcMotorEx.class, "left_front");
        leftRear = hardwareMap.get(DcMotorEx.class, "left_rear");
        rightFront = hardwareMap.get(DcMotorEx.class, "right_front");
        rightRear = hardwareMap.get(DcMotorEx.class, "right_rear");
        armMotor = hardwareMap.get(DcMotorEx.class, "arm_motor");
        armMotor.setDirection(Direction.REVERSE);
        armMotor.setCurrentAlert(1d, CurrentUnit.AMPS);
        armMotor.setPositionPIDFCoefficients(5d);
        armMotor.setTargetPositionTolerance(2);
        armMotor.setZeroPowerBehavior(ZeroPowerBehavior.BRAKE);
        setArmMode(RunMode.STOP_AND_RESET_ENCODER);

        grabberServo = hardwareMap.get(Servo.class, "grabber_servo");
        grabberPosition = grabberServo.getPosition();

        frontServo = hardwareMap.get(Servo.class, "front_servo");

        limitSwitch = hardwareMap.get(DigitalChannel.class, "limit_sensor");

        motors = Arrays.asList(leftFront, leftRear, rightRear, rightFront);

        leftFront.setDirection(Direction.FORWARD);
        leftRear.setDirection(Direction.FORWARD);
        rightFront.setDirection(Direction.REVERSE);
        rightRear.setDirection(Direction.REVERSE);

        setDriveZeroPowerBehavior(ZeroPowerBehavior.FLOAT);

        _appContext = hardwareMap.appContext;
        beepSoundID[0] = hardwareMap.appContext.getResources().getIdentifier("beep", "raw", hardwareMap.appContext.getPackageName());
        beepSoundID[1] = hardwareMap.appContext.getResources().getIdentifier("ss_laser", "raw", hardwareMap.appContext.getPackageName());

        if (initIMU) {
            // Set up the parameters with which we will use our IMU. Note that integration
            // algorithm here just reports accelerations to the logcat log; it doesn't actually
            // provide positional information.
            BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
            parameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
            parameters.accelUnit = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
            parameters.calibrationDataFile = "BNO055IMUCalibration.json"; // see the calibration sample opmode
            parameters.loggingEnabled = true;
            parameters.loggingTag = "IMU";
            parameters.accelerationIntegrationAlgorithm = new JustLoggingAccelerationIntegrator();

            // Retrieve and initialize the IMU. We expect the IMU to be attached to an I2C port
            // on a Core Device Interface Module, configured to be a sensor of type "AdaFruit IMU",
            // and named "imu".
            imu = hardwareMap.get(BNO055IMU.class, "imu");
            imu.initialize(parameters);

            while (!imu.isGyroCalibrated()) {
                Thread.yield();
            }
        }
        beep();
    }

    void resetDrive() {
        setDriveMode(RunMode.STOP_AND_RESET_ENCODER);
        setDriveMode(RunMode.RUN_USING_ENCODER);
        setDriveZeroPowerBehavior(ZeroPowerBehavior.BRAKE);
    }

    void setDriveZeroPowerBehavior(ZeroPowerBehavior zeroPowerBehavior) {
        for (DcMotorEx motor : motors) {
            motor.setZeroPowerBehavior(zeroPowerBehavior);
        }
    }

    public void setDrivePower(double pLeftFront, double pLeftRear, double pRightFront, double pRightRear) {
        leftFront.setPower(pLeftFront);
        leftRear.setPower(pLeftRear);
        rightFront.setPower(pRightFront);
        rightRear.setPower(pRightRear);
    }

    void setDriveVelocity(double pLeftFront, double pLeftRear, double pRightFront, double pRightRear) {
        double vLeftFront = pLeftFront * MAX_VELOCITY;
        double vLeftRear = pLeftRear * MAX_VELOCITY;
        double vRightFront = pRightFront * MAX_VELOCITY;
        double vRightRear = pRightRear * MAX_VELOCITY;
        leftFront.setVelocity(vLeftFront);
        leftRear.setVelocity(vLeftRear);
        rightFront.setVelocity(vRightFront);
        rightRear.setVelocity(vRightRear);
    }

    double getHeading() {
        Orientation angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        double degrees = AngleUnit.DEGREES.fromUnit(angles.angleUnit, angles.firstAngle);
        return degrees;
    }

    boolean isDriveBusy() {
        return leftFront.isBusy() && leftRear.isBusy() &&
                rightFront.isBusy() && rightRear.isBusy();
    }

    void setDriveMode(RunMode driveMode) {
        for (DcMotorEx motor : motors) {
            motor.setMode(driveMode);
        }
    }

    public void setDriveTarget(double distance, boolean moveSideway) {
        // Determine new target position, and pass to motor controller
        int moveCounts = (int) (distance * COUNTS_PER_INCH);

        int dirFL = moveSideway ? -1 : 1;
        int dirFR = 1;
        int dirRL = 1;
        int dirRR = moveSideway ? -1 : 1;

        int leftFrontTarget = leftFront.getCurrentPosition() + moveCounts * dirFL;
        int rightFrontTarget = rightFront.getCurrentPosition() + moveCounts * dirFR;
        int leftRearTarget = leftRear.getCurrentPosition() + moveCounts * dirRL;
        int rightRearTarget = rightRear.getCurrentPosition() + moveCounts * dirRR;

        // Set Target and Turn On RUN_TO_POSITION
        leftFront.setTargetPosition(leftFrontTarget);
        rightFront.setTargetPosition(rightFrontTarget);
        leftRear.setTargetPosition(leftRearTarget);
        rightRear.setTargetPosition(rightRearTarget);
    }

    public final void beep() {
        beep(0);
    }

    final void beep(int beepType) {
        new Thread(() ->
                SoundPlayer.getInstance().startPlaying(_appContext, beepSoundID[beepType])
        ).start();
    }

    /** set grabber servo position
     * @param newPosition double 0: open, 1: close
     */
    public void setGrabber(double newPosition) {
        if (grabberPosition != newPosition) {
            grabberServo.setPosition(newPosition);
            grabberPosition = newPosition;
        }
    }

    public void setFrontServo(boolean newPosition) {
        frontServo.setPosition(newPosition ? 0d : 0.6d);
    }

    public void toggleFrontServo() {
        frontServoPosition = !frontServoPosition;
        setFrontServo(frontServoPosition);
    }

    /** set arm motor run mode
     * @param newMode
     */
    public void setArmMode(RunMode newMode) {
        if (armMode != newMode) {
            armMotor.setMode(newMode);
            armMode = newMode;
        }
    }
}
