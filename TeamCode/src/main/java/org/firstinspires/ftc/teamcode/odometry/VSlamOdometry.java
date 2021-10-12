package org.firstinspires.ftc.teamcode.odometry;

import android.util.Log;

import com.arcrobotics.ftclib.geometry.Pose2d;
import com.arcrobotics.ftclib.geometry.Rotation2d;
import com.arcrobotics.ftclib.geometry.Transform2d;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.spartronics4915.lib.T265Camera;

import org.firstinspires.ftc.robotcore.external.Telemetry;

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

    private static final int DEFAULT_THREAD_SLEEP_TIME = 100;
    static double INCH_2_METER = 0.0254;

    private HardwareMap hwMap;

    private double encoderMeasurementCovariance;
    private int sleepTime;

    private T265Camera slamra;
    private boolean isRunning = true;

    private int currentX;
    private int currentY;
    private int currentHeading;

    private static final String TAG = "RobotCoordinatePositionCam";

    public VSlamOdometry(HardwareMap hwMap) {
        init(hwMap, DEFAULT_THREAD_SLEEP_TIME, 0.8);
    }

    public VSlamOdometry(HardwareMap hwMap, int threadDelay) {
        init(hwMap, threadDelay, 0.8);
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
    public void stop() { isRunning = false; }

    @Override
    public int getCurrentX() { return currentX; }

    @Override
    public int getCurrentY() { return currentY; }

    @Override
    public int getCurrentHeading() { return currentHeading % 360; }

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
                this.currentX = (int) Math.round(up.pose.getTranslation().getX() / INCH_2_METER);
                this.currentY = (int) Math.round(up.pose.getTranslation().getY() / INCH_2_METER);
                this.currentHeading = (int) up.pose.getRotation().getDegrees();
            }
        }
        catch (Exception ex){
            Log.e(TAG, "Error in update position", ex);
        }
    }
}

