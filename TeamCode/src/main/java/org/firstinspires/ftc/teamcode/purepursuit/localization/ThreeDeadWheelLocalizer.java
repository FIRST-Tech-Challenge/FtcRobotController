package org.firstinspires.ftc.teamcode.purepursuit.localization;

import androidx.annotation.NonNull;

import org.firstinspires.ftc.robotcore.external.Telemetry;

import static org.firstinspires.ftc.teamcode.constants.Constants.LocalizerConstants.*;

import com.qualcomm.hardware.sparkfun.SparkFunOTOS.Pose2D;

public final class ThreeDeadWheelLocalizer implements Localizer {
    private final DeadWheel frontDeadWheel, leftDeadWheel, rightDeadWheel;

    public Pose2D pose;

    /**
     * Reading Order
     */
    private double[] prevTicks;

    public ThreeDeadWheelLocalizer(
            @NonNull DeadWheel frontDeadWheel,
            @NonNull DeadWheel leftDeadWheel,
            @NonNull DeadWheel rightDeadWheel
    ) {
        this.frontDeadWheel = frontDeadWheel;
        this.leftDeadWheel  = leftDeadWheel;
        this.rightDeadWheel = rightDeadWheel;

        pose = new Pose2D(0,0,0);
        prevTicks  = new double[]{0.0, 0.0, 0.0};
    }

    public ThreeDeadWheelLocalizer(
            @NonNull DeadWheel frontDeadWheel,
            @NonNull DeadWheel leftDeadWheel,
            @NonNull DeadWheel rightDeadWheel,
            @NonNull Pose2D pose
    ) {
        this.frontDeadWheel = frontDeadWheel;
        this.leftDeadWheel  = leftDeadWheel;
        this.rightDeadWheel = rightDeadWheel;
        this.pose = pose;

        prevTicks  = new double[]{0.0, 0.0, 0.0};
    }

    @Override public void update() {
        double frontTicks = frontDeadWheel.ticks();
        double leftTicks  = leftDeadWheel.ticks();
        double rightTicks = rightDeadWheel.ticks();

        double deltaFrontTicks = frontTicks - prevTicks[0];
        double deltaLeftTicks  = leftTicks - prevTicks[1];
        double deltaRightTicks = rightTicks - prevTicks[2];

        prevTicks[0] = frontTicks;
        prevTicks[1] = leftTicks;
        prevTicks[2] = rightTicks;

        double deltaXDistanceInches
                = deltaFrontTicks * DEAD_WHEEL_CIRCUMFERENCE_INCHES / ENCODER_TICKS_PER_REVOLUTION;

        double deltaLeftDistanceInches
                = deltaLeftTicks * DEAD_WHEEL_CIRCUMFERENCE_INCHES / ENCODER_TICKS_PER_REVOLUTION;
        double deltaRightDistanceInches
                = deltaRightTicks * DEAD_WHEEL_CIRCUMFERENCE_INCHES / ENCODER_TICKS_PER_REVOLUTION;

        // To get the Y Distance, we take the average between the left and right encoders
        double deltaYDistanceInches
                = 0.5 * (deltaLeftDistanceInches + deltaRightDistanceInches);

        double deltaHeadingRadians
                = (deltaRightDistanceInches + deltaLeftDistanceInches) / TRACK_WIDTH_INCHES;

        double averageHeadingRadians
                = (Math.toRadians(pose.h) + deltaHeadingRadians) / 2.0;

        double cosHeading = Math.cos(averageHeadingRadians);
        double sinHeading = Math.sin(averageHeadingRadians);

        pose.x += deltaXDistanceInches * sinHeading + deltaYDistanceInches * cosHeading;
        pose.y += deltaXDistanceInches * cosHeading + deltaYDistanceInches * sinHeading;
        pose.h = normalizeAngle(pose.h);
    }

    @Override public void reset() {
        frontDeadWheel.reset();
        leftDeadWheel.reset();
        rightDeadWheel.reset();
    }

    @Override public void setPosition(@NonNull Pose2D position) {
        this.pose = position;
    }

    @Override public void debug(@NonNull Telemetry telemetry) {
        telemetry.addData("Global X Position", pose.x);
        telemetry.addData("Global Y Position", pose.y);
        telemetry.addData("Global Heading", pose.h);
    }

    private double normalizeAngle(double angle) {
        double scaledAngle = angle % 360.0;

        if ( scaledAngle < 0.0) {
            scaledAngle += 360.0;
        }

        if (scaledAngle > 180.0) {
            scaledAngle -= 360.0;
        }

        return scaledAngle;
    }
}