package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.matrices.GeneralMatrixF;
import org.firstinspires.ftc.robotcore.external.matrices.MatrixF;

import java.util.Arrays;
import java.util.List;

public class MecanumDrive {
    private DcMotor frontLeft;
    private DcMotor frontRight;
    private DcMotor backRight;
    private DcMotor backLeft;

    private final static double GEAR_RATIO = 1;
    private final static double WHEEL_RADIUS = 1.5;  // 5 cm
    private final static double TICKS_PER_ROTATION = 537.7;

    private static final double CM_PER_TICK = (2 * Math.PI * GEAR_RATIO * WHEEL_RADIUS) / TICKS_PER_ROTATION;

    private double maxSpeed = 1.0;

    private MatrixF conversion;
    private GeneralMatrixF encoderMatrix = new GeneralMatrixF(3, 1);


    private int frontLeftOffset;
    private int frontRightOffset;
    private int backRightOffset;
    private int backLeftOffset;

    /**
     * constructor for the mecanum drive, it sets up the encoder matrix
     */
    MecanumDrive() {
        float[] data = {1.0f, 1.0f, 1.0f,
                1.0f, -1.0f, -1.0f,
                1.0f, -1.0f, 1.0f};
        conversion = new GeneralMatrixF(3, 3, data);
        conversion = conversion.inverted();
    }

    /**
     * this initializes the mecanum drive
     *
     * @param hwMap comes from the configuration
     */
    void init(HardwareMap hwMap) {
        frontLeft = hwMap.get(DcMotor.class, "leftfront");
        frontLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        frontRight = hwMap.get(DcMotor.class, "rightfront");
        frontRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        backLeft = hwMap.get(DcMotor.class, "leftback");
        backLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        backRight = hwMap.get(DcMotor.class, "rightback");
        backRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        backLeft.setDirection(DcMotorSimple.Direction.REVERSE); // the back left and front left motors are backwards
        frontLeft.setDirection(DcMotorSimple.Direction.REVERSE);
    }

    /**
     * adds tests to test wiring to test our mecanum wheels gearing and wiring
     *
     * @return returns a test for each motor that spins it forwards
     */
    List<QQ_Test> getTests() {
        return Arrays.asList(
                new QQ_TestMotor("Mecanum Wheel -> Front Left", 0.3, frontLeft),
                new QQ_TestMotor("Mecanum Wheel -> Front Left Bkwds", -0.3, frontLeft),
                new QQ_TestMotor("Mecanum Wheel -> Front Right", 0.3, frontRight),
                new QQ_TestMotor("Mecanum Wheel -> Front Right Bkwds", -0.3, frontRight),
                new QQ_TestMotor("Mecanum Wheel -> Back Left", 0.3, backLeft),
                new QQ_TestMotor("Mecanum Wheel -> Back Left Bkwds", -0.3, backLeft),
                new QQ_TestMotor("Mecanum Wheel -> Back Right", 0.3, backRight),
                new QQ_TestMotor("Mecanum Wheel -> Back Right Bkwds", -0.3, backRight));
    }

    /**
     * scales all the speeds so that we travel the way we want to
     * it also makes sure that we don't pass the max speed {@link #setMaxSpeed}
     * then after doing both of those, we set the speed
     *
     * @param flSpeed speed gotten for Front Left Motor
     * @param frSpeed speed gotten for Front Right Motor
     * @param blSpeed speed gotten for Back Left Motor
     * @param brSpeed speed gotten for Back Right Motor
     */
    private void setSpeeds(double flSpeed, double frSpeed, double blSpeed, double brSpeed) {
        double largest = 1.0;
        largest = Math.max(largest, Math.abs(flSpeed));
        largest = Math.max(largest, Math.abs(frSpeed));
        largest = Math.max(largest, Math.abs(blSpeed));
        largest = Math.max(largest, Math.abs(brSpeed));

        frontLeft.setPower(maxSpeed * (flSpeed / largest));
        frontRight.setPower(maxSpeed * (frSpeed / largest));
        backLeft.setPower(maxSpeed * (blSpeed / largest));
        backRight.setPower(maxSpeed * (brSpeed / largest));
    }

    /**
     * this does the math to figure out how fast to move the wheels
     * it then calls {@link #setSpeeds}
     *
     * @param forward this is the speed forward
     * @param strafe  this is the strafe speed
     * @param rotate  this is the rotate speed
     */
    public void driveMecanum(double forward, double strafe, double rotate) {
        double frontLeftSpeed = forward + strafe + rotate;
        double frontRightSpeed = forward - strafe - rotate;
        double backLeftSpeed = forward - strafe + rotate;
        double backRightSpeed = forward + strafe - rotate;

        setSpeeds(frontLeftSpeed, frontRightSpeed, backLeftSpeed, backRightSpeed);
    }

    /**
     * this uses matrix math to figure out how far we've driven (robot relative)
     *
     * @return returns distances in cm from last reset in order forward, strafe
     */
    double[] getDistanceCm() {
        double[] distances = {0.0, 0.0};

        encoderMatrix.put(0, 0, (float) ((frontLeft.getCurrentPosition() - frontLeftOffset) * CM_PER_TICK));
        encoderMatrix.put(1, 0, (float) ((frontRight.getCurrentPosition() - frontRightOffset) * CM_PER_TICK));
        encoderMatrix.put(2, 0, (float) ((backLeft.getCurrentPosition() - backLeftOffset) * CM_PER_TICK));

        MatrixF distanceMatrix = conversion.multiplied(encoderMatrix);
        distances[0] = distanceMatrix.get(0, 0);
        distances[1] = distanceMatrix.get(1, 0);

        return distances;
    }

    /**
     * this allows us to set our max speed
     * it makes sure that we don't try to set the max speed above 1
     *
     * @param speed this is the max speed that is set for the mecanum drive
     */
    void setMaxSpeed(double speed) {
        maxSpeed = Math.min(speed, 1.0);
    }

    /**
     * gets currently set max speed
     *
     * @return the maxSpeed mecanum drive is currently set to
     */
    double getMaxSpeed() {
        return maxSpeed;
    }

    /**
     * Resets the encoder positions using offsets
     */
    void setEncoderOffsets() {
        frontRightOffset = frontRight.getCurrentPosition();
        frontLeftOffset = frontLeft.getCurrentPosition();
        backLeftOffset = backLeft.getCurrentPosition();
        backRightOffset = backRight.getCurrentPosition();
    }
}