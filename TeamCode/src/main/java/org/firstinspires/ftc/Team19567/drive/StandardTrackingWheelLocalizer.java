package org.firstinspires.ftc.Team19567.drive;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.localization.ThreeTrackingWheelLocalizer;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import org.firstinspires.ftc.Team19567.util.Encoder;

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
    public static double TICKS_PER_REV = 8192;
    public static double WHEEL_RADIUS = 0.6889764; // in
    public static double GEAR_RATIO = 1; // output (wheel) speed / input (encoder) speed

    public static double LATERAL_DISTANCE = 10; // in; distance between the left and right wheels
    public static double FORWARD_OFFSET = 6.33858; // in; offset of the lateral wheel

    private Encoder leftFrontLeftEnc, leftBackRightEnc, rightFrontBackEnc;

    public static double X_MULTIPLIER = 0.9961804146; // Multiplier in the X direction
    public static double Y_MULTIPLIER = 0.9966733067; // Multiplier in the Y direction

    public StandardTrackingWheelLocalizer(HardwareMap hardwareMap) {
        super(Arrays.asList(
                new Pose2d(0, LATERAL_DISTANCE / 2, 0), // left
                new Pose2d(0, -LATERAL_DISTANCE / 2, 0), // right
                new Pose2d(FORWARD_OFFSET, 0, Math.toRadians(90)) // front
        ));

        leftFrontLeftEnc = new Encoder(hardwareMap.get(DcMotorEx.class, "leftFrontLeftEnc"));
        leftBackRightEnc = new Encoder(hardwareMap.get(DcMotorEx.class, "leftBackRightEnc"));
        rightFrontBackEnc = new Encoder(hardwareMap.get(DcMotorEx.class, "rightFrontBackEnc"));

        // TODO: reverse any encoders using Encoder.setDirection(Encoder.Direction.REVERSE)
        rightFrontBackEnc.setDirection(Encoder.Direction.REVERSE);
        leftBackRightEnc.setDirection(Encoder.Direction.REVERSE);
    }

    public static double encoderTicksToInches(double ticks) {
        return WHEEL_RADIUS * 2 * Math.PI * GEAR_RATIO * ticks / TICKS_PER_REV;
    }

    @NonNull
    @Override
    public List<Double> getWheelPositions() {
        return Arrays.asList(
                encoderTicksToInches(leftFrontLeftEnc.getCurrentPosition()) * X_MULTIPLIER,
                encoderTicksToInches(leftBackRightEnc.getCurrentPosition()) * X_MULTIPLIER,
                encoderTicksToInches(rightFrontBackEnc.getCurrentPosition()) * Y_MULTIPLIER
        );
    }

    @NonNull
    @Override
    public List<Double> getWheelVelocities() {
        // TODO: If your encoder velocity can exceed 32767 counts / second (such as the REV Through Bore and other
        //  competing magnetic encoders), change Encoder.getRawVelocity() to Encoder.getCorrectedVelocity() to enable a
        //  compensation method

        return Arrays.asList(
                encoderTicksToInches(leftFrontLeftEnc.getCorrectedVelocity()) * X_MULTIPLIER,
                encoderTicksToInches(leftBackRightEnc.getCorrectedVelocity()) * X_MULTIPLIER,
                encoderTicksToInches(rightFrontBackEnc.getCorrectedVelocity()) * Y_MULTIPLIER
        );
    }
}
