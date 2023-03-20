package org.firstinspires.ftc.teamcode.teamUtil.odometry.roadrunner.drive;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.localization.ThreeTrackingWheelLocalizer;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.teamUtil.ConfigNames;
import org.firstinspires.ftc.teamcode.teamUtil.odometry.roadrunner.util.Encoder;

import java.util.Arrays;
import java.util.List;

/*
 * Sample tracking wheel localizer implementation assuming the standard configuration:
 *
 *    /--------------\
 *    |     ____     |
 *    |     ----     |
 *    | ||        || |
 *    | ||        || |
 *    |              |
 *    |              |
 *    \--------------/
 *
 */
@Config
public class StandardTrackingWheelLocalizer extends ThreeTrackingWheelLocalizer {
    public static double TICKS_PER_REV = 8192; //REV Through-Bore Encoder Value
    public static double WHEEL_RADIUS = 0.94; // in; //TODO find this (might need to be updated)
    public static double GEAR_RATIO = 1; // output (wheel) speed / input (encoder) speed

    public static double LATERAL_DISTANCE = 9.400114926813881; // in; distance between the left and right wheels; //TODO find this
    public static double FORWARD_OFFSET = 0; // in; offset of the lateral wheel; //TODO find this

    public static final double X_MULTIPLIER = 120 / 119.25573010536319;
    public static final double Y_MULTIPLIER = 120.0 / 118.47599288833676;
    
    private final Encoder leftEncoder, rightEncoder, frontEncoder;

    public StandardTrackingWheelLocalizer(HardwareMap hardwareMap) {
        super(Arrays.asList(
                new Pose2d(0, LATERAL_DISTANCE / 2, 0), // left
                new Pose2d(0, -LATERAL_DISTANCE / 2, 0), // right
                new Pose2d(FORWARD_OFFSET, 0, Math.toRadians(90)) // front
        ));

        leftEncoder = new Encoder(hardwareMap.get(DcMotorEx.class, ConfigNames.leftDeadWheelEncoder));
        rightEncoder = new Encoder(hardwareMap.get(DcMotorEx.class, ConfigNames.rightDeadWheelEncoder));
        frontEncoder = new Encoder(hardwareMap.get(DcMotorEx.class, ConfigNames.frontDeadWheelEncoder));

        // TODO: reverse any encoders using Encoder.setDirection(Encoder.Direction.REVERSE)
        leftEncoder.setDirection(Encoder.Direction.REVERSE);
    }

    public static double encoderTicksToInches(double ticks) {
        return WHEEL_RADIUS * 2 * Math.PI * GEAR_RATIO * ticks / TICKS_PER_REV;
    }

    @NonNull
    @Override
    public List<Double> getWheelPositions() {
        return Arrays.asList(
                encoderTicksToInches(leftEncoder.getCurrentPosition()) * Y_MULTIPLIER,
                encoderTicksToInches(rightEncoder.getCurrentPosition()) * Y_MULTIPLIER,
                encoderTicksToInches(frontEncoder.getCurrentPosition()) * X_MULTIPLIER
        );
    }

    @NonNull
    @Override
    public List<Double> getWheelVelocities() {
        // TODO: If your encoder velocity can exceed 32767 counts / second (such as the REV Through Bore and other
        //  competing magnetic encoders), change Encoder.getRawVelocity() to Encoder.getCorrectedVelocity() to enable a
        //  compensation method

        return Arrays.asList(
                encoderTicksToInches(leftEncoder.getCorrectedVelocity()) * Y_MULTIPLIER,
                encoderTicksToInches(rightEncoder.getCorrectedVelocity()) * Y_MULTIPLIER,
                encoderTicksToInches(frontEncoder.getCorrectedVelocity()) * X_MULTIPLIER
        );
    }
}
