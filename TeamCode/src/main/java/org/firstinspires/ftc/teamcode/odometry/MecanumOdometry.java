package org.firstinspires.ftc.teamcode.odometry;

import android.graphics.Point;

import androidx.annotation.NonNull;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.bots.BotMoveRequest;

/**
 * Odometry class to keep track of a Mecanum wheeled robot using three odometery wheels
 * Odometry requires three non-powered omni wheels connected to encoders: two forward facing wheels
 * and one lateral wheel located at the back of the robot.
 *
 * Math and explanation can be found at https://chsftcrobotics.weebly.com/uploads/1/2/3/6/123696510/odometry.pdf
 */
public class MecanumOdometry implements IBaseOdometry {

    protected Telemetry telemetry;

    private boolean isRunning = true;
    private int sleepTime;

    private double leftRightDistance;
    private double centerBackDistance;

    protected DcMotorEx leftOdo;
    protected DcMotorEx rightOdo;
    protected DcMotorEx horizontalOdo;

    private int currentX;
    private int currentY;
    private int currentHeading;

    private final int COUNTS_PER_INCH_REV = 500;
    private final double ODO_WHEEL_DIAMETER_IN = 2.0;

    private double prevOdoReadingLeft;
    private double prevOdoReadingRight;
    private double prevOdoReadingHoriz;

    /**
     * Create an Odometry object for Mecanum wheel drive that has 3 odometer wheels
     * @param hwMap FTC Hardware Map. It is expected to have the odometer wheels defined as 'leftodo', 'rightodo', and 'horizontal'
     * @param telemetry FTC Telemetry object
     * @param leftRightDistance distance between the left and right odometer wheels is in inches
     * @param centerBackDistance distance between the center of the robot and the back horizontal wheel is also in inches
     */
    public MecanumOdometry(@NonNull HardwareMap hwMap, Telemetry telemetry, double leftRightDistance, double centerBackDistance) {
        this.telemetry = telemetry;
        this.leftRightDistance = leftRightDistance;
        this.centerBackDistance = centerBackDistance;
        this.leftOdo = hwMap.get(DcMotorEx.class, "leftodo");
        this.rightOdo = hwMap.get(DcMotorEx.class, "rightodo");
        this.horizontalOdo = hwMap.get(DcMotorEx.class, "horizontal");
    }

    //region Public Getters and Setters
    public int getSleepTime() {
        return sleepTime;
    }
    public void setSleepTime(int sleepTime) {
        this.sleepTime = sleepTime;
    }
    //endregion


    private void resetEncoders() {
        if (leftOdo != null){
            leftOdo.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        }
        if (rightOdo != null){
            rightOdo.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        }
        if (horizontalOdo != null){
            horizontalOdo.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        }
        prevOdoReadingLeft = 0.0;
        prevOdoReadingRight = 0.0;
        prevOdoReadingHoriz = 0.0;
    }

    @Override
    public void setInitPosition(int startXInches, int startYInches, int startHeadingDegrees) throws Exception {
        this.currentX = startXInches;
        this.currentY = startYInches;
        this.currentHeading = startHeadingDegrees;
        resetEncoders();
    }

    @Override
    public double getAdjustedCurrentHeading() {
        return 0;
    }

    @Override
    public double getXInches() {
        return 0;
    }

    @Override
    public double getYInches() {
        return 0;
    }

    @Override
    public void stop(){ isRunning = false; }

    @Override
    public int getCurrentX() {
        return currentX;
    }

    @Override
    public int getCurrentY() {
        return currentY;
    }

    @Override
    public int getCurrentHeading() {
        return currentHeading;
    }

    @Override
    public void reverseHorEncoder() {

    }

    @Override
    public void setPersistPosition(boolean persistPosition) {

    }

    @Override
    public void init(Point startPos, double initialOrientation) {

    }

    @Override
    public double getInitialOrientation() {
        return 0;
    }

    @Override
    public double getOrientation() {
        return 0;
    }

    @Override
    public int getThreadSleepTime() {
        return 0;
    }

    @Override
    public void setTarget(BotMoveRequest target) {

    }

    @Override
    public double getRealSpeedLeft() {
        return 0;
    }

    @Override
    public double getRealSpeedRight() {
        return 0;
    }

    @Override
    public boolean isLeftLong() {
        return false;
    }

    @Override
    public void run() {
        while(isRunning) {
            updatePosition();
            try {
                Thread.sleep(sleepTime);
            } catch (InterruptedException e) {
                e.printStackTrace();
            }
        }
    }

    private void updatePosition() {

        if (this.leftRightDistance < 1) {
            return;
        }

        double deltaX;
        double deltaY;
        double deltaHeading;

        // Refer to this PDF for calculating the change in X, Y, and Heading
        // https://chsftcrobotics.weebly.com/uploads/1/2/3/6/123696510/odometry.pdf

        double r = this.leftRightDistance / 2;
        double rb = this.centerBackDistance;

        double deltaLeft = (prevOdoReadingLeft - leftOdo.getCurrentPosition()) * COUNTS_PER_INCH_REV;
        double deltaRight = (prevOdoReadingRight - rightOdo.getCurrentPosition()) * COUNTS_PER_INCH_REV;
        double deltaHoriz = (prevOdoReadingHoriz - horizontalOdo.getCurrentPosition()) * COUNTS_PER_INCH_REV;

        double deltaAngle = (deltaRight - deltaLeft) / this.leftRightDistance;

        if (Math.abs(deltaAngle) < 0.001) {
            deltaX = deltaHoriz;
            deltaY = (deltaLeft + deltaRight) / 2;
            deltaHeading = 0;
        } else {
            double rT = r * (deltaLeft + deltaRight) / (deltaRight - deltaLeft);
            double rS = (deltaHoriz / deltaAngle) - rb;

            double cosValue = Math.cos(deltaAngle);
            double sinValue = Math.sin(deltaAngle);

            deltaX = rT * (cosValue - 1) + rS * sinValue;
            deltaY = rT * sinValue + rS * (1 - cosValue);
            deltaHeading = deltaAngle * 180 / Math.PI;
        }

        this.currentX += deltaX;
        this.currentY += deltaY;
        this.currentHeading += deltaHeading;
    }
}
