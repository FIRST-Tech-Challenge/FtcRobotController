package org.firstinspires.ftc.teamcode.odometry;

import android.graphics.Point;
import android.util.Log;

import com.arcrobotics.ftclib.geometry.Pose2d;
import com.arcrobotics.ftclib.geometry.Rotation2d;
import com.arcrobotics.ftclib.geometry.Transform2d;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ReadWriteFile;
import com.spartronics4915.lib.T265Camera;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.internal.system.AppUtil;
import org.firstinspires.ftc.teamcode.bots.BotMoveRequest;

import java.io.File;

/**
 * Odometry class to keep track of a robot using V-SLAM Camera Module
 *
 * Camera: Intel® RealSense™ Tracking Camera T265
 * Product page: https://www.intelrealsense.com/tracking-camera-t265/
 *
 * Sample code can be found at
 * https://github.com/pietroglyph/FtcRobotController/tree/ftc265-example
 * file: TeamCode/src/main/java/org/firstinspires/ftc/teamcode/TestCameraOpMode.java
 *
 * Instructions on adding the required libraries can be found at
 * https://github.com/pietroglyph/ftc265
 */
public class VSlamOdometry implements IBaseOdometry {

    public static final int THREAD_INTERVAL = 100;
    static double INCH_2_METER = 0.0254;

    private HardwareMap hwMap;

    private double encoderMeasurementCovariance;
    private int sleepTime;

    private T265Camera slamra;
    private boolean isRunning = true;

    private int currentX;
    private int currentY;
    private int currentHeading;

    private static VSlamOdometry theInstance;

    private boolean persistPosition = false;

    private static final String TAG = "RobotCoordinatePositionCam";

    private VSlamOdometry(HardwareMap hwMap, int threadDelay) {
        init(hwMap, threadDelay, 0.8);
    }

    public static VSlamOdometry getInstance(HardwareMap hwMap) {
        if (theInstance == null) {
            theInstance = new VSlamOdometry(hwMap, THREAD_INTERVAL);
        }
        return theInstance;
    }

    public static VSlamOdometry getInstance(HardwareMap hwMap, int threadDelay) {
        if (theInstance == null) {
            theInstance = new VSlamOdometry(hwMap, threadDelay);
        }
        return theInstance;
    }

    private void init(HardwareMap hwMap, int threadDelay, double encoderMeasurementCovariance){
        this.hwMap = hwMap;
        this.sleepTime = threadDelay;
        // Increase this value to trust encoder odometry less when fusing encoder measurements with VSLAM
        this.encoderMeasurementCovariance = encoderMeasurementCovariance;
    }

    @Override
    public void setInitPosition(int startXInches, int startYInches, int startHeadingDegrees) throws Exception {

        this.currentX = startXInches;
        this.currentY = startYInches;
        this.currentHeading = startHeadingDegrees;

        double startX = startXInches * INCH_2_METER;
        double startY = startYInches * INCH_2_METER;
        Rotation2d startHeading = Rotation2d.fromDegrees(startHeadingDegrees);
        Pose2d startingPose = new Pose2d(startX, startY, startHeading);
        // This is the transformation between the center of the camera and the center of the robot
        final Transform2d cameraToRobot = new Transform2d();

        slamra = new T265Camera(cameraToRobot, encoderMeasurementCovariance, this.hwMap.appContext);
        slamra.setPose(startingPose);
    }

    @Override
    public double getAdjustedCurrentHeading() {
        double currentHead = this.getCurrentHeading();

        boolean clockwise = currentHead >= 0;
        if (!clockwise){
            currentHead = 360 + currentHead;
        }
        return currentHead;
    }

    @Override
    public double getXInches() {
        return getCurrentX()/INCH_2_METER;
    }

    @Override
    public double getYInches() {
        return getCurrentY()/INCH_2_METER;
    }

    @Override
    public void stop() {
        isRunning = false;
        slamra.stop();
    }

    @Override
    public int getCurrentX() { return currentX; }

    @Override
    public int getCurrentY() { return currentY; }

    @Override
    public int getCurrentHeading() { return currentHeading % 360; }

    @Override
    public void reverseHorEncoder() {
        // does not apply
    }

    @Override
    public void setPersistPosition(boolean persistPosition) {
        this.persistPosition = persistPosition;
    }

    @Override
    public void init(Point startPos, double initialOrientation) {
        try {
            setInitPosition(startPos.x, startPos.y, (int) initialOrientation);
        }
        catch (Exception ex){
            Log.e(TAG, "Failed to start VSLAM camera", ex);
        }
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
        try {
            slamra.start();
            isRunning = true;
            while (isRunning) {
                updatePosition();
                try {
                    Thread.sleep(sleepTime);
                } catch (InterruptedException e) {
                    e.printStackTrace();
                }
            }
        }
        catch (Exception ex){
            Log.e(TAG, "Error starting the camera", ex);
        }
    }

    private void updatePosition(){
        try {
            T265Camera.CameraUpdate up = slamra.getLastReceivedCameraUpdate();

            if (up != null) {
                this.currentX = (int) up.pose.getTranslation().getX();
                this.currentY = (int) up.pose.getTranslation().getY();
                Log.i(TAG, String.format("Cam coordinate: %d : %d", this.currentX, this.currentY));
                this.currentHeading = (int) up.pose.getRotation().getDegrees();
                if (persistPosition) {
                    saveLastPosition();
                }
            }
        }
        catch (Exception ex){
            Log.e(TAG, "Error in update position", ex);
        }
    }

    public void saveLastPosition(){
        BotPosition lastPos = new BotPosition();
        lastPos.setPosX((int)this.getXInches());
        lastPos.setPosY((int)this.getYInches());
        lastPos.setHeading(this.getCurrentHeading());
        File file = AppUtil.getInstance().getSettingsFile(BotPosition.BOT_LAST_POSITION);
        ReadWriteFile.writeFile(file, lastPos.serialize());
    }

    public BotPosition getLastConfig() {
        BotPosition lastPos = null;

        File posFile = AppUtil.getInstance().getSettingsFile(BotPosition.BOT_LAST_POSITION);
        if (posFile.exists()) {
            String data = ReadWriteFile.readFile(posFile);
            lastPos = BotPosition.deserialize(data);
        }

        return lastPos;
    }
}

