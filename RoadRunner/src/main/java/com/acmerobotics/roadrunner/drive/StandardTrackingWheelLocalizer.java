package com.acmerobotics.roadrunner.drive;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.localization.ThreeTrackingWheelLocalizer;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

import com.acmerobotics.roadrunner.util.Encoder;

import java.util.Arrays;
import java.util.List;

import androidx.annotation.NonNull;

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

    public static double WHEEL_DIAMETER_MM = 35;
    public static double IN_PER_MM = 0.03937;

    public static double TICKS_PER_REV = 8192;
    public static double WHEEL_RADIUS = (WHEEL_DIAMETER_MM * IN_PER_MM) / 2; // in
    public static double GEAR_RATIO = 1; // output (wheel) speed / input (encoder) speed

    // public static double LATERAL_DISTANCE = 6.415; // in; distance between the left and right wheels
    public static double LATERAL_DISTANCE = 6.402; // in; distance between the left and right wheels
    public static double FORWARD_OFFSET = -5.25; // in; offset of the lateral wheel
    public static int ENCODER_COUNT = 3;

    /*
     * Use these values to tune the dead wheels.
     *
     * Drag the robot in a VERY STRAIGHT line by hand using the LocalizationTest.
     * Observe the telemetry distance vs. the measured distance travelled and use
     * this equation to determine to correction multiplier
     *
     * correction multiplier = measured distance / telemetry distance
     *
     * For tank drives only do this in the X direction.  For mecanum drives perform
     * the operation in the Y, strafing, direction as well.
     */
    public static double X_MULTIPLIER = 1.0;
    public static double Y_MULTIPLIER = 1.0;

    // public static double LEFT_MULTIPLIER = 0.9875;
    // public static double RIGHT_MULTIPLIER = 1.0125;
    public static double LEFT_MULTIPLIER = 1.0;
    public static double RIGHT_MULTIPLIER = 1.0;

    /*
     * Lateral encoder placement
     */
    public enum LateralEncoderLocation {
        FRONT,
        BACK,
    }
    public static LateralEncoderLocation LATERAL_PLACEMENT = LateralEncoderLocation.FRONT;

    /*
     * How the encoders are mounted in the pod.
     */
    public enum EncoderPodOrientation {
        POD_FORWARD,
        POD_BACKWARD,
    }
    public static EncoderPodOrientation ENCODER_POD_ORIENTATION = EncoderPodOrientation.POD_FORWARD;

    public enum EncoderLocation {
        LEFT, RIGHT, LATERAL
    }

    /*
     * Names for your encoders.
     */
    public static String LEFT_ENCODER = "leftEncoder";
    public static String RIGHT_ENCODER = "rightEncoder";
    public static String LATERAL_ENCODER = "lateralEncoder";

    private Encoder leftEncoder, rightEncoder, frontEncoder;

    public StandardTrackingWheelLocalizer(HardwareMap hardwareMap) {
        super(Arrays.asList(
                new Pose2d(1.75, LATERAL_DISTANCE / 2.0, 0), // left
                new Pose2d(1.75, -LATERAL_DISTANCE / 2.0, 0), // right
                new Pose2d(LATERAL_PLACEMENT == LateralEncoderLocation.FRONT ? FORWARD_OFFSET : Math.abs(FORWARD_OFFSET) * -1, 0.25, Math.toRadians(90)) // front
        ));

        leftEncoder = new Encoder(hardwareMap.get(DcMotorEx.class, LEFT_ENCODER));
        rightEncoder = new Encoder(hardwareMap.get(DcMotorEx.class, RIGHT_ENCODER));
        if (DriveConstants.ENCODER_COUNT == 3) {
            frontEncoder = new Encoder(hardwareMap.get(DcMotorEx.class, LATERAL_ENCODER));
        }

        // TODO: reverse any encoders using Encoder.setDirection(Encoder.Direction.REVERSE)
    }

    public static double encoderTicksToInches(double ticks) {
        return WHEEL_RADIUS * 2 * Math.PI * GEAR_RATIO * ticks / TICKS_PER_REV;
    }

    @NonNull
    @Override
    public List<Double> getWheelPositions() {
        double lPos = leftEncoder.getCurrentPosition() * LEFT_MULTIPLIER;
        double rPos = rightEncoder.getCurrentPosition() * RIGHT_MULTIPLIER;
        int fPos = frontEncoder.getCurrentPosition();
        return Arrays.asList(
                encoderTicksToInches(lPos * X_MULTIPLIER),
                encoderTicksToInches(rPos * X_MULTIPLIER),
                encoderTicksToInches(fPos * Y_MULTIPLIER)
        );
    }

    @NonNull
    @Override
    public List<Double> getWheelVelocities() {
        // TODO: If your encoder velocity can exceed 32767 counts / second (such as the REV Through Bore and other
        //  competing magnetic encoders), change Encoder.getRawVelocity() to Encoder.getCorrectedVelocity() to enable a
        //  compensation method

        return Arrays.asList(
                encoderTicksToInches(leftEncoder.getCorrectedVelocity() * X_MULTIPLIER),
                encoderTicksToInches(rightEncoder.getCorrectedVelocity() * X_MULTIPLIER),
                encoderTicksToInches(frontEncoder.getCorrectedVelocity() * Y_MULTIPLIER)
        );
    }

    public double getEncoderInches(EncoderLocation loc) {
        switch (loc) {
            case LATERAL:
                return encoderTicksToInches(frontEncoder.getCurrentPosition());
            case RIGHT:
                return encoderTicksToInches(rightEncoder.getCurrentPosition());
            case LEFT:
                return encoderTicksToInches(leftEncoder.getCurrentPosition());
        }
        return 0;
    }

    public int getEncoderPosition(EncoderLocation loc) {
        switch (loc) {
            case LATERAL:
                return frontEncoder.getCurrentPosition();
            case RIGHT:
                return rightEncoder.getCurrentPosition();
            case LEFT:
                return leftEncoder.getCurrentPosition();
        }
        return 0;
    }
}
