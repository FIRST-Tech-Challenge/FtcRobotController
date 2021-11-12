package org.firstinspires.ftc.teamcode.src.robotAttachments.odometry;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ReadWriteFile;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.internal.system.AppUtil;

import java.io.File;

/**
 * Created by Sarthak on 6/1/2019.
 */
public class OdometryGlobalCoordinatePosition implements Runnable {
    //Odometry wheels
    private BNO055IMU imu = null;
    private final DcMotor verticalEncoderLeft;
    private final DcMotor verticalEncoderRight;
    private final DcMotor horizontalEncoder;

    private final double COUNTS_PER_INCH = 1892.3724283364;

    private boolean isActive = false;

    //Thead run condition
    private boolean isRunning = true;

    //Position variables used for storage and calculations
    double verticalRightEncoderWheelPosition = 0, verticalLeftEncoderWheelPosition = 0, normalEncoderWheelPosition = 0, changeInRobotOrientation = 0;
    private double robotGlobalXCoordinatePosition = 0, robotGlobalYCoordinatePosition = 0, robotOrientationRadians = 0;
    private double previousVerticalRightEncoderWheelPosition = 0, previousVerticalLeftEncoderWheelPosition = 0, prevNormalEncoderWheelPosition = 0;

    //Algorithm constants
    private final double robotEncoderWheelDistance;
    private final double horizontalEncoderTickPerDegreeOffset;

    //Sleep time interval (milliseconds) for the position update thread
    private final int sleepTime;

    //Files to access the algorithm constants
    private final File wheelBaseSeparationFile = AppUtil.getInstance().getSettingsFile("wheelBaseSeparation.txt");
    private final File horizontalTickOffsetFile = AppUtil.getInstance().getSettingsFile("horizontalTickOffset.txt");

    private int verticalLeftEncoderPositionMultiplier = 1;
    private int verticalRightEncoderPositionMultiplier = 1;
    private int normalEncoderPositionMultiplier = 1;
    private double angleOffset = 0;

    /**
     * Constructor for GlobalCoordinatePosition Thread
     *
     * @param verticalEncoderLeft  left odometry encoder, facing the vertical direction
     * @param verticalEncoderRight right odometry encoder, facing the vertical direction
     * @param horizontalEncoder    horizontal odometry encoder, perpendicular to the other two odometry encoder wheels
     * @param threadSleepDelay     delay in milliseconds for the GlobalPositionUpdate thread (50-75 milliseconds is suggested)
     */
    public OdometryGlobalCoordinatePosition(DcMotor verticalEncoderLeft, DcMotor verticalEncoderRight, DcMotor horizontalEncoder, int threadSleepDelay) {
        this.verticalEncoderLeft = verticalEncoderLeft;
        this.verticalEncoderRight = verticalEncoderRight;
        this.horizontalEncoder = horizontalEncoder;
        sleepTime = threadSleepDelay;

        robotEncoderWheelDistance = Double.parseDouble(ReadWriteFile.readFile(wheelBaseSeparationFile).trim()) * COUNTS_PER_INCH;
        this.horizontalEncoderTickPerDegreeOffset = Double.parseDouble(ReadWriteFile.readFile(horizontalTickOffsetFile).trim());

    }

    //Vertical Right, Vertical Left, Horizontal
    public Integer[] getPorts() {
        Integer[] ports = new Integer[3];
        ports[0] = verticalEncoderRight.getPortNumber();
        ports[1] = verticalEncoderLeft.getPortNumber();
        ports[2] = horizontalEncoder.getPortNumber();
        return ports;
    }

    public void showPosition(Telemetry telemetry) {
        telemetry.addData("X: ", this.returnRelativeXPosition());
        telemetry.addData("Y: ", this.returnRelativeYPosition());
        telemetry.addData("Rotation: ", this.returnOrientation());
        telemetry.update();
    }

    public double getCOUNTS_PER_INCH() {
        return COUNTS_PER_INCH;
    }

    public int returnRightEncoderPosition() {
        return verticalRightEncoderPositionMultiplier * verticalEncoderRight.getCurrentPosition();
    }

    public int returnLeftEncoderPosition() {
        return verticalLeftEncoderPositionMultiplier * verticalEncoderLeft.getCurrentPosition();
    }

    public int returnHorizontalEncoderPosition() {
        return normalEncoderPositionMultiplier * horizontalEncoder.getCurrentPosition();
    }

    //Vertical Right, Vertical Left, Horizontal
    public int[] returnRaw() {
        int[] positions = new int[3];
        positions[0] = verticalEncoderRight.getCurrentPosition();
        positions[1] = verticalEncoderLeft.getCurrentPosition();
        positions[2] = horizontalEncoder.getCurrentPosition();
        return positions;
    }

    public boolean setOrientation(double angle) {
        if (isActive) {
            return false;
        }
        robotOrientationRadians = Math.toRadians(angle);
        return true;
    }

    public boolean setPosition(double x, double y, double angle) {

        while (isActive) {
            continue;
        }
        robotGlobalXCoordinatePosition = x * COUNTS_PER_INCH;
        robotGlobalYCoordinatePosition = y * COUNTS_PER_INCH;
        if (imu != null) {
            angleOffset = Math.toRadians(angle);
        } else {
            robotOrientationRadians = Math.toRadians(angle);
        }
        return true;

    }

    public boolean isActive() {
        return isActive;
    }

    public void setImu(BNO055IMU imu) {
        this.imu = imu;
    }


    double getImu() {
        double returnVal = 0;
        if (imu.getAngularOrientation().firstAngle < 0) {
            returnVal = Math.abs(imu.getAngularOrientation().firstAngle);
        } else {
            returnVal = Math.abs(imu.getAngularOrientation().firstAngle - 360);
        }
        return returnVal % 360;

    }

    /**
     * Updates the global (x, y, theta) coordinate position of the robot using the odometry encoders
     */
    private void globalCoordinatePositionUpdate() {
        isActive = true;
        if (imu != null) {
            robotOrientationRadians = Math.toRadians(getImu()) + angleOffset;
            robotOrientationRadians = robotOrientationRadians % 360;
        }
        //Get Current Positions
        verticalLeftEncoderWheelPosition = (verticalEncoderLeft.getCurrentPosition() * verticalLeftEncoderPositionMultiplier);
        verticalRightEncoderWheelPosition = (verticalEncoderRight.getCurrentPosition() * verticalRightEncoderPositionMultiplier);

        double leftChange = verticalLeftEncoderWheelPosition - previousVerticalLeftEncoderWheelPosition;
        double rightChange = verticalRightEncoderWheelPosition - previousVerticalRightEncoderWheelPosition;

        //Calculate Angle
        changeInRobotOrientation = (leftChange - rightChange) / (robotEncoderWheelDistance);
        robotOrientationRadians = ((robotOrientationRadians + changeInRobotOrientation));

        //Get the components of the motion
        normalEncoderWheelPosition = (horizontalEncoder.getCurrentPosition() * normalEncoderPositionMultiplier);
        double rawHorizontalChange = normalEncoderWheelPosition - prevNormalEncoderWheelPosition;
        double horizontalChange = rawHorizontalChange - (changeInRobotOrientation * horizontalEncoderTickPerDegreeOffset);

        double p = ((rightChange + leftChange) / 2);
        double n = horizontalChange;

        //Calculate and update the position values
        robotGlobalXCoordinatePosition = robotGlobalXCoordinatePosition + (p * Math.sin(robotOrientationRadians) + n * Math.cos(robotOrientationRadians));
        robotGlobalYCoordinatePosition = robotGlobalYCoordinatePosition + (p * Math.cos(robotOrientationRadians) - n * Math.sin(robotOrientationRadians));

        previousVerticalLeftEncoderWheelPosition = verticalLeftEncoderWheelPosition;
        previousVerticalRightEncoderWheelPosition = verticalRightEncoderWheelPosition;
        prevNormalEncoderWheelPosition = normalEncoderWheelPosition;
        isActive = false;
    }

    /**
     * Returns the robot's global x coordinate
     *
     * @return global x coordinate
     */
    public double returnRawXCoordinate() {
        return robotGlobalXCoordinatePosition;
    }

    /**
     * Returns the robot's global y coordinate
     *
     * @return global y coordinate
     */
    public double returnRawYCoordinate() {
        return robotGlobalYCoordinatePosition;
    }

    /**
     * Returns the robot's global orientation
     *
     * @return global orientation, in degrees
     */
    public double returnOrientation() {
        return Math.toDegrees(robotOrientationRadians) % 360;
    }

    public double returnRelativeXPosition() {
        return robotGlobalXCoordinatePosition / COUNTS_PER_INCH;
    }

    public double returnRelativeYPosition() {
        return robotGlobalYCoordinatePosition / COUNTS_PER_INCH;
    }

    /**
     * Stops the position update thread
     */
    public void stop() {
        isRunning = false;
    }

    protected String getHorizontalMotorName() {
        return horizontalEncoder.getDeviceName();
    }

    protected String getVerticalRightMotorName() {
        return verticalEncoderRight.getDeviceName();
    }

    protected String getVerticalLeftMotorName() {
        return verticalEncoderLeft.getDeviceName();
    }

    public void reverseLeftEncoder() {
        if (verticalLeftEncoderPositionMultiplier == 1) {
            verticalLeftEncoderPositionMultiplier = -1;
        } else {
            verticalLeftEncoderPositionMultiplier = 1;
        }
    }

    public void reverseRightEncoder() {
        if (verticalRightEncoderPositionMultiplier == 1) {
            verticalRightEncoderPositionMultiplier = -1;
        } else {
            verticalRightEncoderPositionMultiplier = 1;
        }
    }

    public void reverseNormalEncoder() {
        if (normalEncoderPositionMultiplier == 1) {
            normalEncoderPositionMultiplier = -1;
        } else {
            normalEncoderPositionMultiplier = 1;
        }
    }

    /**
     * Runs the thread
     */
    @Override
    public void run() {
        while (isRunning) {
            globalCoordinatePositionUpdate();
            try {
                Thread.sleep(sleepTime);
            } catch (InterruptedException e) {
                e.printStackTrace();
            }
        }
    }
}
