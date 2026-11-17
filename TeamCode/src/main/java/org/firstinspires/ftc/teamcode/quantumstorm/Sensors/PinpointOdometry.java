package org.firstinspires.ftc.teamcode.quantumstorm.Sensors;

import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.teamcode.quantumstorm.Constants;
import org.firstinspires.ftc.teamcode.biobuzz.ftc.SafePinpointPose;

/**
 * goBILDA Pinpoint odometry computer (2 dead-wheel pods + built-in IMU).
 * Tracks the robot's field position. Call update() once per loop.
 */
public class PinpointOdometry {

    private final GoBildaPinpointDriver pinpoint;
    private Pose2D pose;
    private boolean valid;

    public PinpointOdometry(HardwareMap hardwareMap, Pose2D startPose) {

        pinpoint = hardwareMap.get(GoBildaPinpointDriver.class, Constants.PINPOINT);
        SafePinpointPose.checkInterrupted();

        pinpoint.setOffsets(
                Constants.PINPOINT_X_POD_OFFSET_MM,
                Constants.PINPOINT_Y_POD_OFFSET_MM,
                DistanceUnit.MM);
        pinpoint.setEncoderResolution(Constants.PINPOINT_POD_TYPE);
        SafePinpointPose.checkInterrupted();
        pinpoint.setEncoderDirections(
                Constants.PINPOINT_X_DIRECTION,
                Constants.PINPOINT_Y_DIRECTION);

        // Robot must be still here. recalibrateIMU() keeps the position,
        // so the start pose set right after it is not wiped.
        pinpoint.recalibrateIMU();
        SafePinpointPose.checkInterrupted();
        setPose(startPose);
    }

    /** Read the latest position from the Pinpoint. Call once per loop. */
    public void update() {
        SafePinpointPose.checkInterrupted();valid=false;
        pinpoint.update();
        Pose2D sample=SafePinpointPose.read(pinpoint);
        if(sample!=null){pose=sample;valid=true;}
    }

    public boolean isValid() { return valid; }

    /** Tell the Pinpoint where the robot is (start of match, or an AprilTag fix). */
    public void setPose(Pose2D newPose) {
        pinpoint.setPosition(newPose);
        pose = newPose;
    }

    public Pose2D getPose() {
        return pose;
    }

    public double getX() {
        return pose.getX(DistanceUnit.INCH);
    }

    public double getY() {
        return pose.getY(DistanceUnit.INCH);
    }

    public double getHeading() {
        return pose.getHeading(AngleUnit.DEGREES);
    }

    public GoBildaPinpointDriver.DeviceStatus getStatus() {
        return pinpoint.getDeviceStatus();
    }

    public void addTelemetry(Telemetry telemetry) {
        telemetry.addData("Pinpoint", getStatus());
        telemetry.addData("Pinpoint pose valid",valid?"YES":"NO - last known / initialized pose only");
        telemetry.addData("X (in)", "%.1f", getX());
        telemetry.addData("Y (in)", "%.1f", getY());
        telemetry.addData("Heading (deg)", "%.1f", getHeading());
    }
}
