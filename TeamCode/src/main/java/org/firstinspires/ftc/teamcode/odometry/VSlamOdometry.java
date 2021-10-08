package org.firstinspires.ftc.teamcode.odometry;

import com.arcrobotics.ftclib.geometry.Pose2d;
import com.arcrobotics.ftclib.geometry.Rotation2d;
import com.arcrobotics.ftclib.geometry.Transform2d;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.spartronics4915.lib.T265Camera;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class VSlamOdometry implements IBaseOdometry {

    private static final int DEFAULT_THREAD_SLEEP_TIME = 100;

    private final HardwareMap hwMap;
    private final Telemetry telemetry;

    private final Transform2d cameraToRobot;
    private final double encoderMeasurementCovariance;
    private final int sleepTime;

    private T265Camera slamra;
    private boolean isRunning = true;

    private int currentX;
    private int currentY;
    private int currentHeading;

    public VSlamOdometry(HardwareMap hwMap, Telemetry telemetry) {
        this.hwMap = hwMap;
        this.telemetry = telemetry;
        this.sleepTime = DEFAULT_THREAD_SLEEP_TIME;

        // This is the transformation between the center of the camera and the center of the robot
        cameraToRobot = new Transform2d();

        // Increase this value to trust encoder odometry less when fusing encoder measurements with VSLAM
        encoderMeasurementCovariance = 0.8;
    }

    @Override
    public void setInitPosition(int startXInches, int startYInches, int startHeadingDegrees) throws Exception {

        this.currentX = startXInches;
        this.currentY = startYInches;
        this.currentHeading = startHeadingDegrees;

        double startX = startXInches * 0.0254;
        double startY = startYInches * 0.0254;
        Rotation2d startHeading = Rotation2d.fromDegrees(startHeadingDegrees);
        Pose2d startingPose = new Pose2d(startX, startY, startHeading);

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
    public int getCurrentHeading() { return currentHeading; }

    @Override
    public void run() {
        slamra.start();
        isRunning = true;
        while(isRunning) {
            T265Camera.CameraUpdate up = slamra.getLastReceivedCameraUpdate();

            if (up != null) {
                this.currentX = (int) Math.round(up.pose.getTranslation().getX() / 0.0254);
                this.currentY = (int) Math.round(up.pose.getTranslation().getY() / 0.0254);
                this.currentHeading = (int) up.pose.getRotation().getDegrees();
            }

            try {
                Thread.sleep(sleepTime);
            } catch (InterruptedException e) {
                e.printStackTrace();
            }
        }
    }
}

