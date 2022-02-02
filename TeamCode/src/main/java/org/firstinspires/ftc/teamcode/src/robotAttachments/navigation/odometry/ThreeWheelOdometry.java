package org.firstinspires.ftc.teamcode.src.robotAttachments.navigation.odometry;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ReadWriteFile;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.internal.system.AppUtil;
import org.firstinspires.ftc.teamcode.src.robotAttachments.navigation.odometry.enums.FieldPoints;
import org.firstinspires.ftc.teamcode.src.utills.Executable;
import org.firstinspires.ftc.teamcode.src.utills.ThreadedSubsystemTemplate;

import java.io.File;
import java.util.concurrent.locks.Lock;
import java.util.concurrent.locks.ReentrantLock;


/**
 * A class to do position triangulation based on dead wheel odometry
 *
 * @author Sarthak
 * @since 6/1/2019
 */
public class ThreeWheelOdometry extends ThreadedSubsystemTemplate implements Odometry {
    /**
     * The Vertical Left Odometry Encoder
     */
    private final DcMotor verticalEncoderLeft;
    /**
     * The Vertical Right Odometry Encoder
     */
    private final DcMotor verticalEncoderRight;
    /**
     * The Horizontal Odometry Encoder
     */
    private final DcMotor horizontalEncoder;

    /**
     * The number of encoder ticks per linear inch
     */
    private final double COUNTS_PER_INCH = 1892.3724283364;
    /**
     * An Algorithm constant
     */
    private final double robotEncoderWheelDistance;

    //Algorithm constants
    /**
     * An Algorithm constant
     */
    private final double horizontalEncoderTickPerDegreeOffset;
    /**
     * A lock for synchronous methods
     */
    private final Lock lock = new ReentrantLock();
    /**
     * An Algorithm constant
     */
    private volatile double robotGlobalXCoordinatePosition = 0, robotGlobalYCoordinatePosition = 0, robotOrientationRadians = 0;
    /**
     * An Algorithm constant
     */
    private double previousVerticalRightEncoderWheelPosition = 0, previousVerticalLeftEncoderWheelPosition = 0, prevNormalEncoderWheelPosition = 0;
    /**
     * An Algorithm constant
     */
    private int verticalLeftEncoderPositionMultiplier = 1;
    /**
     * An Algorithm constant
     */
    private int verticalRightEncoderPositionMultiplier = 1;
    /**
     * An Algorithm constant
     */
    private int normalEncoderPositionMultiplier = 1;


    /**
     * Constructor for GlobalCoordinatePosition Thread
     *
     * @param verticalEncoderLeft  left odometry encoder, facing the vertical direction
     * @param verticalEncoderRight right odometry encoder, facing the vertical direction
     * @param horizontalEncoder    horizontal odometry encoder, perpendicular to the other two odometry encoder wheels
     * @param threadSleepDelay     delay in milliseconds for the GlobalPositionUpdate thread (50-75 milliseconds is suggested)
     * @param _opModeIsActive      A Executable object wrapped around {@link LinearOpMode#opModeIsActive()}
     * @param _isStopRequested     A Executable object wrapped around {@link LinearOpMode#isStopRequested()}
     */
    public ThreeWheelOdometry(DcMotor verticalEncoderLeft, DcMotor verticalEncoderRight, DcMotor horizontalEncoder, int threadSleepDelay, Executable<Boolean> _opModeIsActive, Executable<Boolean> _isStopRequested) {
        super(_opModeIsActive, _isStopRequested);
        this.verticalEncoderLeft = verticalEncoderLeft;
        this.verticalEncoderRight = verticalEncoderRight;
        this.horizontalEncoder = horizontalEncoder;
        sleepTime = threadSleepDelay;

        //Files to access the algorithm constants
        File wheelBaseSeparationFile = AppUtil.getInstance().getSettingsFile("wheelBaseSeparation.txt");
        robotEncoderWheelDistance = Double.parseDouble(ReadWriteFile.readFile(wheelBaseSeparationFile).trim()) * COUNTS_PER_INCH;
        File horizontalTickOffsetFile = AppUtil.getInstance().getSettingsFile("horizontalTickOffset.txt");
        this.horizontalEncoderTickPerDegreeOffset = Double.parseDouble(ReadWriteFile.readFile(horizontalTickOffsetFile).trim());

    }


    /**
     * Returns the ports the encoders are plugged into for debug purposes
     *
     * @return the ports in the form Vertical Right, Vertical Left, Horizontal
     */
    public int[] getPorts() {
        int[] ports = new int[3];
        ports[0] = verticalEncoderRight.getPortNumber();
        ports[1] = verticalEncoderLeft.getPortNumber();
        ports[2] = horizontalEncoder.getPortNumber();
        return ports;
    }

    /**
     * Outputs the location to the telemetry
     *
     * @param telemetry The OpMode telemetry
     */
    public void showPosition(Telemetry telemetry) {
        telemetry.addData("X: ", this.getX());
        telemetry.addData("Y: ", this.getY());
        telemetry.addData("Rotation: ", this.getRot());
    }

    /**
     * Getter for the COUNTS_PER_INCH variable
     *
     * @return COUNTS_PER_INCH in ticks per inch
     */
    public double getCOUNTS_PER_INCH() {
        return COUNTS_PER_INCH;
    }

    /**
     * Getter for the right encoder position
     *
     * @return The right encoder position in ticks
     */
    public int returnRightEncoderPosition() {
        return verticalRightEncoderPositionMultiplier * verticalEncoderRight.getCurrentPosition();
    }

    /**
     * Getter for the left encoder position
     *
     * @return The left encoder position in ticks
     */
    public int returnLeftEncoderPosition() {
        return verticalLeftEncoderPositionMultiplier * verticalEncoderLeft.getCurrentPosition();
    }

    /**
     * Getter for the horizontal encoder position
     *
     * @return The horizontal encoder position in ticks
     */
    public int returnHorizontalEncoderPosition() {
        return normalEncoderPositionMultiplier * horizontalEncoder.getCurrentPosition();
    }

    /**
     * Returns the tick values of the encoders
     *
     * @return Returns in this order: Vertical Right, Vertical Left, Horizontal
     */
    public int[] returnRaw() {
        int[] positions = new int[3];
        positions[0] = verticalEncoderRight.getCurrentPosition();
        positions[1] = verticalEncoderLeft.getCurrentPosition();
        positions[2] = horizontalEncoder.getCurrentPosition();
        return positions;
    }

    /**
     * Sets the orientation of the odometry calibration system
     *
     * @param angle the angle to be at
     * @throws InterruptedException Throws if the Thread is interrupted while waiting for lock. If is interrupted, values are not changed
     */
    public void setOrientation(double angle) throws InterruptedException {

        lock.lockInterruptibly();
        try {

            robotOrientationRadians = Math.toRadians(angle);

        } finally {
            lock.unlock();
        }
    }

    /**
     * Sets the position of the robot
     *
     * @param X   The X-position in inches
     * @param Y   The Y-position in inches
     * @param rot The rot in degrees
     */
    public void setPos(double X, double Y, double rot) throws InterruptedException {

        lock.lockInterruptibly();
        try {
            robotGlobalXCoordinatePosition = X * COUNTS_PER_INCH;
            robotGlobalYCoordinatePosition = Y * COUNTS_PER_INCH;
            robotOrientationRadians = Math.toRadians(rot);

        } finally {
            lock.unlock();
        }

    }

    /**
     * Sets the position of the robot using an Enum key from FieldPoints
     *
     * @param initPos the enum key of a three value array of an init position
     * @throws InterruptedException Throws if the Thread is interrupted while waiting for lock. If is interrupted, values are not changed
     */
    public void setPos(FieldPoints initPos) throws InterruptedException {
        double[] tmp = FieldPoints.positionsAndPoints.get(initPos);
        assert (tmp != null);
        this.setPos(tmp[0] * COUNTS_PER_INCH, tmp[1] * COUNTS_PER_INCH, tmp[2]);
    }


    /**
     * Updates the global (x, y, theta) coordinate position of the robot using the odometry encoders
     */
    public void threadMain() throws InterruptedException {
        lock.lockInterruptibly();
        try {

            //Get Current Positions
            double verticalLeftEncoderWheelPosition = (verticalEncoderLeft.getCurrentPosition() * verticalLeftEncoderPositionMultiplier);
            //Position variables used for storage and calculations
            double verticalRightEncoderWheelPosition = (verticalEncoderRight.getCurrentPosition() * verticalRightEncoderPositionMultiplier);

            double leftChange = verticalLeftEncoderWheelPosition - previousVerticalLeftEncoderWheelPosition;
            double rightChange = verticalRightEncoderWheelPosition - previousVerticalRightEncoderWheelPosition;

            //Calculate Angle
            double changeInRobotOrientation = (leftChange - rightChange) / (robotEncoderWheelDistance);
            robotOrientationRadians = ((robotOrientationRadians + changeInRobotOrientation));

            //Get the components of the motion
            double normalEncoderWheelPosition = (horizontalEncoder.getCurrentPosition() * normalEncoderPositionMultiplier);
            double rawHorizontalChange = normalEncoderWheelPosition - prevNormalEncoderWheelPosition;
            double horizontalChange = rawHorizontalChange - (changeInRobotOrientation * horizontalEncoderTickPerDegreeOffset);

            double p = ((rightChange + leftChange) / 2);

            //Calculate and update the position values
            robotGlobalXCoordinatePosition = robotGlobalXCoordinatePosition + (p * Math.sin(robotOrientationRadians) + horizontalChange * Math.cos(robotOrientationRadians));
            robotGlobalYCoordinatePosition = robotGlobalYCoordinatePosition + (p * Math.cos(robotOrientationRadians) - horizontalChange * Math.sin(robotOrientationRadians));

            previousVerticalLeftEncoderWheelPosition = verticalLeftEncoderWheelPosition;
            previousVerticalRightEncoderWheelPosition = verticalRightEncoderWheelPosition;
            prevNormalEncoderWheelPosition = normalEncoderWheelPosition;
        } finally {
            lock.unlock();
        }
    }


    /**
     * Returns the robot's global orientation
     *
     * @return global orientation, in degrees
     */
    public double getRot() {
        return Math.toDegrees(robotOrientationRadians) % 360;
    }

    /**
     * Returns the robot's global X Position in inches
     *
     * @return global X Position in inches
     */
    public double getX() {

        return robotGlobalXCoordinatePosition / COUNTS_PER_INCH;
    }

    /**
     * Returns the robot's global Y Position in inches
     *
     * @return global Y Position in inches
     */
    public double getY() {

        return robotGlobalYCoordinatePosition / COUNTS_PER_INCH;
    }

    public double[] getPos() {
        return new double[]{this.getX(), this.getY(), this.getRot()};
    }


    /**
     * Reverses the left encoder
     */
    public void reverseLeftEncoder() {
        if (verticalLeftEncoderPositionMultiplier == 1) {
            verticalLeftEncoderPositionMultiplier = -1;
        } else {
            verticalLeftEncoderPositionMultiplier = 1;
        }
    }

    /**
     * Reverses the right encoder
     */
    public void reverseRightEncoder() {
        if (verticalRightEncoderPositionMultiplier == 1) {
            verticalRightEncoderPositionMultiplier = -1;
        } else {
            verticalRightEncoderPositionMultiplier = 1;
        }
    }

    /**
     * Reverses the Horizontal encoder
     */
    public void reverseHorizontalEncoder() {
        if (normalEncoderPositionMultiplier == 1) {
            normalEncoderPositionMultiplier = -1;
        } else {
            normalEncoderPositionMultiplier = 1;
        }
    }

    @Override
    protected void onEnd() {
    }
}
