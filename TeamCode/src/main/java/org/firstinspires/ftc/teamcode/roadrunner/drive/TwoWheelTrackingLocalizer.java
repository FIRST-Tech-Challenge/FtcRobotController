package org.firstinspires.ftc.teamcode.roadrunner.drive;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.localization.TwoTrackingWheelLocalizer;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.roadrunner.util.Encoder;

import java.util.Arrays;
import java.util.List;

/*
 * Sample tracking wheel localizer implementation assuming the standard configuration:
 *
 *    ^
 *    |
 *    | ( x direction )
 *    |
 *    v
 *    <----( y direction )---->

 *        (forward)
 *    /--------------\
 *    |     ____     |
 *    |     ----     |    <- Perpendicular Wheel
 *    |           || |
 *    |           || |    <- Parallel Wheel
 *    |              |
 *    |              |
 *    \--------------/
 *
 */
@Config
public class TwoWheelTrackingLocalizer extends TwoTrackingWheelLocalizer {
    public static double TICKS_PER_REV = 8192;
    public static double WHEEL_RADIUS = .689; // in 35/2 radius
    public static double GEAR_RATIO = 1; // output (wheel) speed / input (encoder) speed

    public static double PARALLEL_X = -3.58; // X is the up and down direction
    public static double PARALLEL_Y = 2.5523; // Y is the strafe direction

    public static double PERPENDICULAR_X = 5.4677;
    public static double PERPENDICULAR_Y = 1.383;

    public static double X_MULTIPLIER = ((70 / 69.7517) + (70 / 69.6240) + (70 / 69.6582)) / 3;
    public static double Y_MULTIPLIER = ((70 / 69.9348) + (70 / 69.9693) + (70 / 70.0469)) / 3;

    // Parallel/Perpendicular to the forward axis
    // Parallel wheel is parallel to the forward axis
    // Perpendicular is perpendicular to the forward axis
    public final Encoder parallelEncoder, perpendicularEncoder;

    private final SampleMecanumDrive drive;

    public TwoWheelTrackingLocalizer(HardwareMap hardwareMap, SampleMecanumDrive drive) {
        super(Arrays.asList(
            new Pose2d(PARALLEL_X, PARALLEL_Y, 0),
            new Pose2d(PERPENDICULAR_X, PERPENDICULAR_Y, Math.toRadians(90))
        ));

        this.drive = drive;

        parallelEncoder = new Encoder(hardwareMap.get(DcMotorEx.class, "FL"));
        perpendicularEncoder = new Encoder(hardwareMap.get(DcMotorEx.class, "BR"));

        // TODO: reverse any encoders using Encoder.setDirection(Encoder.Direction.REVERSE)
        parallelEncoder.setDirection(Encoder.Direction.REVERSE);
    }

    public static double encoderTicksToInches(double ticks) {
        return WHEEL_RADIUS * 2 * Math.PI * GEAR_RATIO * ticks / TICKS_PER_REV;
    }

    @Override
    public double getHeading() {
        return drive.getRawExternalHeading();
    }

    @Override
    public Double getHeadingVelocity() {
        return drive.getExternalHeadingVelocity();
    }

    @NonNull
    @Override
    public List<Double> getWheelPositions() {
        return Arrays.asList(
            encoderTicksToInches(parallelEncoder.getCurrentPosition()) * X_MULTIPLIER,
            encoderTicksToInches(perpendicularEncoder.getCurrentPosition()) * Y_MULTIPLIER
        );
    }

    @NonNull
    @Override
    public List<Double> getWheelVelocities() {
        // TODO: If your encoder velocity can exceed 32767 counts / second (such as the REV Through Bore and other
        //  competing magnetic encoders), change Encoder.getRawVelocity() to Encoder.getCorrectedVelocity() to enable a
        //  compensation method

        return Arrays.asList(
            encoderTicksToInches(parallelEncoder.getCorrectedVelocity()) * X_MULTIPLIER,
            encoderTicksToInches(perpendicularEncoder.getCorrectedVelocity()) * Y_MULTIPLIER
        );
    }
}
