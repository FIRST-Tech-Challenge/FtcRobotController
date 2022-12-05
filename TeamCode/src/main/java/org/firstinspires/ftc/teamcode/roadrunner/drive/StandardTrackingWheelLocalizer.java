package org.firstinspires.ftc.teamcode.roadrunner.drive;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.localization.ThreeTrackingWheelLocalizer;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.roadrunner.util.Encoder;

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
    public static double X_MULTIPLIER = ((72 / 71.05925) + (72 / 71.31266) + (72 / 71.26539)) / 3;
    public static double Y_MULTIPLIER = -((72 / 70.62737) + (72 / 70.54758) + (72 / 70.00001)) / 3;

    public static double TICKS_PER_REV = 8192;
    public static double WHEEL_RADIUS = .688975; // in
    public static double GEAR_RATIO = 1; // output (wheel) speed / input (encoder) speed

    public static double LATERAL_DISTANCE = 12.37001; // in; distance between the left and right wheels
    public static double FORWARD_OFFSET = 5.25; // in; offset of the lateral wheel

    private final Encoder leftEncoder, rightEncoder, frontEncoder;

    public StandardTrackingWheelLocalizer(HardwareMap hardwareMap) {
        super(Arrays.asList(
            new Pose2d(0, LATERAL_DISTANCE / 2, 0), // left
            new Pose2d(0, -LATERAL_DISTANCE / 2, 0), // right
            new Pose2d(FORWARD_OFFSET, 0, Math.toRadians(90)) // front
        ));

        leftEncoder = new Encoder(hardwareMap.get(DcMotorEx.class, "BR"));
        rightEncoder = new Encoder(hardwareMap.get(DcMotorEx.class, "FL"));
        frontEncoder = new Encoder(hardwareMap.get(DcMotorEx.class, "BL"));
    }

    public static double encoderTicksToInches(double ticks) {
        return WHEEL_RADIUS * 2 * Math.PI * GEAR_RATIO * ticks / TICKS_PER_REV;
    }

    @NonNull
    @Override
    public List<Double> getWheelPositions() {
        return Arrays.asList(
            encoderTicksToInches(leftEncoder.getCurrentPosition()) * X_MULTIPLIER,
            encoderTicksToInches(rightEncoder.getCurrentPosition()) * X_MULTIPLIER,
            encoderTicksToInches(frontEncoder.getCurrentPosition() * Y_MULTIPLIER)
        );
    }

    @NonNull
    @Override
    public List<Double> getWheelVelocities() {
        // TODO: If your encoder velocity can exceed 32767 counts / second (such as the REV Through Bore and other
        //  competing magnetic encoders), change Encoder.getRawVelocity() to Encoder.getCorrectedVelocity() to enable a
        //  compensation method

        return Arrays.asList(
            encoderTicksToInches(leftEncoder.getCorrectedVelocity()) * X_MULTIPLIER,
            encoderTicksToInches(rightEncoder.getCorrectedVelocity()) * X_MULTIPLIER,
            encoderTicksToInches(frontEncoder.getCorrectedVelocity()) * Y_MULTIPLIER
        );
    }
}