package org.firstinspires.ftc.teamcode.drive.robotcore.legacy;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.firstinspires.ftc.teamcode.library.functions.FunctionalExtensionsKt;

import static com.qualcomm.robotcore.hardware.DcMotor.RunMode.RUN_TO_POSITION;
import static com.qualcomm.robotcore.hardware.DcMotor.RunMode.RUN_WITHOUT_ENCODER;
import static com.qualcomm.robotcore.hardware.DcMotor.RunMode.STOP_AND_RESET_ENCODER;

@Config
public class HolonomicImpl implements Holonomic {

    private static final double WHEEL_DIAMETER = 4;
    private static final double WHEEL_CIRCUMFERENCE;
//    private static final double TICKS_PER_REVOLUTION = 291.2;
    private static final double TICKS_PER_REVOLUTION = 450;
    private static final double TICKS_PER_INCH /* 134.4*/;
    private static final double DIAGONAL_BETWEEN_WHEELS = 19.25;

    private static final int TARGET_POSITION_TOLERANCE = 90;

    private DcMotorEx frontLeftMotor;
    private DcMotorEx backLeftMotor;
    private DcMotorEx backRightMotor;
    private DcMotorEx frontRightMotor;

    private DcMotorEx[] motors = new DcMotorEx[4];

    static {
        WHEEL_CIRCUMFERENCE = (WHEEL_DIAMETER * Math.PI);
        TICKS_PER_INCH = TICKS_PER_REVOLUTION / WHEEL_CIRCUMFERENCE;
    }

    /**
     * Creates instance with four motors and specified chassis type
     * @param frontLeftMotor front left motor
     * @param backLeftMotor back left motor
     * @param frontRightMotor front right motor
     * @param backRightMotor back right motor
     */
    public HolonomicImpl(DcMotorEx frontLeftMotor, DcMotorEx backLeftMotor, DcMotorEx frontRightMotor, DcMotorEx backRightMotor) {
        this.frontLeftMotor = frontLeftMotor;
        this.frontRightMotor = frontRightMotor;
        this.backRightMotor = backRightMotor;
        this.backLeftMotor = backLeftMotor;

        motors = new DcMotorEx[]{frontLeftMotor, backLeftMotor, backRightMotor, frontRightMotor};

//        this.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        for (DcMotorEx motor : motors) { motor.setTargetPositionTolerance(TARGET_POSITION_TOLERANCE);}
    }

    /**
     * Uses constants in cartesian directions + rotation to set motor power
     * Best for single-direction motion as this can sum the input powers higher than the allowed power range\
     *
     * @param x left-to-right direction
     * @param y back-to-front direction
     * @param z rotation power
     */
    private void run(double x, double y, double z) {
        x = FunctionalExtensionsKt.rangeClip(x, -1, 1);
        y = FunctionalExtensionsKt.rangeClip(y, -1, 1);
        z = FunctionalExtensionsKt.rangeClip(z, -1, 1);

        double leftFrontPower = x + y + z;
        double leftRearPower = -x + y + z;
        double rightFrontPower = x - y + z;
        double rightRearPower = -x - y + z;
        frontLeftMotor.setPower(leftFrontPower);
        backLeftMotor.setPower(leftRearPower);
        frontRightMotor.setPower(rightFrontPower);
        backRightMotor.setPower(rightRearPower);
    }

    @Override
    public void runWithoutEncoder(double x, double y, double z) {
        setMotorsMode(RUN_WITHOUT_ENCODER);
        run(x, y, z);
    }

    /**
     * Sends input powers to {@link HolonomicImpl#run(double, double, double)}
     * See full documentation for {@link Holonomic#runWithoutEncoderVectored(double, double, double, double)}
     */
    @Override
    public void runWithoutEncoderVectored(double x, double y, double z, double offsetTheta) {

        double theta;
        double axisConversionAngle = Math.PI/4;
        double xPrime;
        double yPrime;

        // calculate r
        double r = Math.sqrt(Math.pow(x,2)+ Math.pow(y,2));
        // calculate theta
        if (x == 0) x = 0.00001;
        theta = Math.atan(y / x);
        if (x < 0) theta += Math.PI;
        theta += offsetTheta;
        // calculate x and y prime
        xPrime = r * Math.cos(theta - axisConversionAngle);
        yPrime = r * Math.sin(theta - axisConversionAngle);

        // set motors mode
        setMotorsMode(RUN_WITHOUT_ENCODER);
        // set motor powers
        runWithoutEncoderPrime(xPrime, yPrime, z);
    }

    @Override
    public void runWithoutEncoderPrime(double xPrime, double yPrime, double z) {
        setMotorsMode(RUN_WITHOUT_ENCODER);
        frontLeftMotor.setPower(xPrime + z);
        backLeftMotor.setPower(yPrime + z);
        backRightMotor.setPower(-xPrime + z);
        frontRightMotor.setPower(-yPrime + z);
    }

    @Override
    public void runUsingEncoder(double xTarget, double yTarget, double inputPower) {
        double r;
        double theta;
        double axisConversionAngle = Math.PI/4;
        double xPrime;
        double yPrime;
        double xPower;
        double yPower;
        double LFDistanceIN;
        double LRDistanceIN;
        double RRDistanceIN;
        double RFDistanceIN;
        double LFPower;
        double LRPower;
        double RRPower;
        double RFPower;

        // set motors mode
        frontLeftMotor.setMode(STOP_AND_RESET_ENCODER);
        frontRightMotor.setMode(STOP_AND_RESET_ENCODER);
        backLeftMotor.setMode(STOP_AND_RESET_ENCODER);
        backRightMotor.setMode(STOP_AND_RESET_ENCODER);

        // calculate r
        r = Math.sqrt(Math.pow(xTarget,2)+ Math.pow(yTarget,2));
        // calculate theta
        if (xTarget == 0) xTarget = 0.00001;
        theta = Math.atan(yTarget / xTarget);
        if (xTarget < 0) theta += Math.PI;
        // calculate x and y prime
        xPrime = r * Math.cos(theta - axisConversionAngle);
        yPrime = r * Math.sin(theta - axisConversionAngle);

        // calculate x and y power
        if (Math.abs(yPrime) > Math.abs(xPrime)) {
            yPower = inputPower;
            xPower = inputPower * (xPrime / yPrime);
        } else {
            xPower = inputPower;
            yPower = inputPower * (yPrime / xPrime);
        }

        // set motor distances (inches)
        LFDistanceIN = xPrime;
        LRDistanceIN = yPrime;
        RRDistanceIN = -xPrime;
        RFDistanceIN = -yPrime;

        // set motor powers
        LFPower = xPower;
        LRPower = yPower;
        RRPower = -xPower;
        RFPower = -yPower;

        // program encoder targets
        frontLeftMotor.setTargetPosition(frontLeftMotor.getCurrentPosition() +(int)(LFDistanceIN * TICKS_PER_INCH));
        backLeftMotor.setTargetPosition(frontRightMotor.getCurrentPosition() + (int)(LRDistanceIN * TICKS_PER_INCH));
        backRightMotor.setTargetPosition(backRightMotor.getCurrentPosition() + (int)(RRDistanceIN * TICKS_PER_INCH));
        frontRightMotor.setTargetPosition(frontRightMotor.getCurrentPosition() + (int)(RFDistanceIN * TICKS_PER_INCH));


        // set motors mode
        frontLeftMotor.setMode(RUN_TO_POSITION);
        frontRightMotor.setMode(RUN_TO_POSITION);
        backLeftMotor.setMode(RUN_TO_POSITION);
        backRightMotor.setMode(RUN_TO_POSITION);

        // program motor power targets
        frontLeftMotor.setPower(LFPower);
        backLeftMotor.setPower(LRPower);
        backRightMotor.setPower(RRPower);
        frontRightMotor.setPower(RFPower);
    }

    @Override
    public void turnUsingEncoder(double degrees, double power) {
        int targetPosition = (int) (degrees * TICKS_PER_REVOLUTION * (DIAGONAL_BETWEEN_WHEELS / (360 * WHEEL_DIAMETER)));
        setMotorsMode(STOP_AND_RESET_ENCODER);
        frontLeftMotor.setTargetPosition(targetPosition);
        frontRightMotor.setTargetPosition(targetPosition);
        backLeftMotor.setTargetPosition(targetPosition);
        backRightMotor.setTargetPosition(targetPosition);
        frontLeftMotor.setPower(power);
        frontRightMotor.setPower(power);
        backLeftMotor.setPower(power);
        backRightMotor.setPower(power);
        setMotorsMode(RUN_TO_POSITION);
    }

    @Override
    public boolean motorsAreBusy() {
        int numBusy = 0;
        if (frontLeftMotor.isBusy()) numBusy++;
        if (frontRightMotor.isBusy()) numBusy++;
        if (backLeftMotor.isBusy()) numBusy++;
        if (backRightMotor.isBusy()) numBusy++;
        return (numBusy > 1);
    }

    @Override
    public void setMotorsMode(DcMotor.RunMode runMode) {
        frontLeftMotor.setMode(runMode);
        frontRightMotor.setMode(runMode);
        backLeftMotor.setMode(runMode);
        backRightMotor.setMode(runMode);
    }

    /**
     * Stops all motors
     */
    @Override
    public void stop() {
        setMotorsMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        for (DcMotorEx motor : motors) motor.setPower(0.0);
    }

    @Override
    public void setZeroPowerBehavior(DcMotor.ZeroPowerBehavior zeroPowerBehavior) {
        for (DcMotorEx motor : motors) motor.setZeroPowerBehavior(zeroPowerBehavior);
    }
}