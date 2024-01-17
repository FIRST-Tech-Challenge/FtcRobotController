package org.firstinspires.ftc.teamcode.roadrunner.drive;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.localization.ThreeTrackingWheelLocalizer;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.roadrunner.util.Encoder;

import java.util.Arrays;
import java.util.Collections;
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
    public static double TICKS_PER_REV = 2000; //number of ticks the encoders will count per revolution.
    public static double WHEEL_RADIUS = 0.945; // in
    public static double GEAR_RATIO = 1.0; // output (wheel) speed / input (encoder) speed

<<<<<<< Updated upstream
    public static double LATERAL_DISTANCE = 8.49; // in; distance between the left and right wheels (was 8.71)
    public static double FORWARD_OFFSET = -3.25; // in; offset of the lateral wheel was -4.57 was -4.94
=======
    public static double LATERAL_DISTANCE = 5.25; // in; distance between the left and right wheels (was 8.71)
    public static double FORWARD_OFFSET = -3.75; // in; offset of the lateral wheel was -4.57 was -4.94
>>>>>>> Stashed changes
    //Both the left and right module is offset in x by -1.14in (left), -1.29in (right)
    //The "forward" module is offset in y by ~.55in  (make adjustments to the initializer)

    public static double X_MULTIPLIER = 1.00572959;
    public static double Y_MULTIPLIER = 1.003096363;

<<<<<<< Updated upstream

=======
>>>>>>> Stashed changes
    private Encoder leftEncoder, rightEncoder, frontEncoder;



    public StandardTrackingWheelLocalizer(HardwareMap hardwareMap) {
        super(Arrays.asList(
<<<<<<< Updated upstream
                new Pose2d(-3.75, LATERAL_DISTANCE / 2, 0), // left was 1.14 was -1.125
                new Pose2d(-3.75, -LATERAL_DISTANCE / 2, 0), // right was 1.29   was -1.38
                new Pose2d(FORWARD_OFFSET, 0.6, Math.toRadians(90)) // front
        ));

        leftEncoder = new Encoder(hardwareMap.get(DcMotorEx.class, "pe"));
        rightEncoder = new Encoder(hardwareMap.get(DcMotorEx.class, "flyWheel"));
        frontEncoder = new Encoder(hardwareMap.get(DcMotorEx.class, "FR"));
=======
                new Pose2d(-3.85, LATERAL_DISTANCE / 2, 0), // left was 1.14 was -1.125
                new Pose2d(-3.85, -LATERAL_DISTANCE / 2, 0), // right was 1.29   was -1.38
                new Pose2d(FORWARD_OFFSET, 0.8, Math.toRadians(90)) // front
        ));

        leftEncoder = new Encoder(hardwareMap.get(DcMotorEx.class, "FL"));
        rightEncoder = new Encoder(hardwareMap.get(DcMotorEx.class, "BR"));
        frontEncoder = new Encoder(hardwareMap.get(DcMotorEx.class, "BL"));
>>>>>>> Stashed changes

        frontEncoder.setDirection(Encoder.Direction.FORWARD);
        leftEncoder.setDirection(Encoder.Direction.FORWARD);
        rightEncoder.setDirection(Encoder.Direction.REVERSE);
        // TODO: reverse any encoders using Encoder.setDirection(Encoder.Direction.REVERSE)
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
                encoderTicksToInches(frontEncoder.getCurrentPosition()) * Y_MULTIPLIER
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
