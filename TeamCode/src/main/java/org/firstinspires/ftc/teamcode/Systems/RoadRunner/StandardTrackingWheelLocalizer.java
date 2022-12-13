package org.firstinspires.ftc.teamcode.Systems.RoadRunner;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.localization.ThreeTrackingWheelLocalizer;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import org.firstinspires.ftc.teamcode.Robot;
import org.firstinspires.ftc.teamcode.Systems.RoadRunner.Encoder;

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
public class StandardTrackingWheelLocalizer extends ThreeTrackingWheelLocalizer implements Robot {
    public static double TICKS_PER_REV = ticks_per_revolution;
    public static double WHEEL_RADIUS = wheel_radius; // in
    public static double GEAR_RATIO = gear_ratio; // output (wheel) speed / input (encoder) speed

    public static double LATERAL_DISTANCE = lateral_distance * turning_multiplier; // in; distance between the left and right wheels
    public static double FORWARD_OFFSET = forward_offset; // in; offset of the lateral wheel

    private Encoder leftEncoder, rightEncoder, frontEncoder;

    public StandardTrackingWheelLocalizer(HardwareMap hardwareMap) {
        super(Arrays.asList(
                new Pose2d(0, LATERAL_DISTANCE / 2, 0), // left
                new Pose2d(0, -LATERAL_DISTANCE / 2, 0), // right
                new Pose2d(FORWARD_OFFSET, 0, Math.toRadians(90)) // front
        ));

        leftEncoder = new Encoder(hardwareMap.get(DcMotorEx.class, encoderNames.get(0)));
        rightEncoder = new Encoder(hardwareMap.get(DcMotorEx.class, encoderNames.get(1)));
        frontEncoder = new Encoder(hardwareMap.get(DcMotorEx.class, encoderNames.get(2)));

        for (int i = 0; i < 3; i++) {
            if (invert_encoders[i]) {
                switch (i) {
                    case 0:
                        leftEncoder.setDirection(Encoder.Direction.REVERSE);
                    case 1:
                        rightEncoder.setDirection(Encoder.Direction.REVERSE);
                    case 2:
                        frontEncoder.setDirection(Encoder.Direction.REVERSE);
                }
            }
        }
    }

    public static double encoderTicksToInches(double ticks) {
        return WHEEL_RADIUS * 2 * Math.PI * GEAR_RATIO * ticks / TICKS_PER_REV;
    }

    @NonNull
    @Override
    public List<Double> getWheelPositions() {
        return Arrays.asList(
                encoderTicksToInches(leftEncoder.getCurrentPosition()) * forward_multiplier,
                encoderTicksToInches(rightEncoder.getCurrentPosition()) * forward_multiplier,
                encoderTicksToInches(frontEncoder.getCurrentPosition()) * strafing_multiplier
        );
    }

    @NonNull
    @Override
    public List<Double> getWheelVelocities() {
        // TODO: If your encoder velocity can exceed 32767 counts / second (such as the REV Through Bore and other
        //  competing magnetic encoders), change Encoder.getRawVelocity() to Encoder.getCorrectedVelocity() to enable a
        //  compensation method

        if (integer_overflow) {
            return Arrays.asList(
                    encoderTicksToInches(leftEncoder.getCorrectedVelocity()) * forward_multiplier,
                    encoderTicksToInches(rightEncoder.getCorrectedVelocity()) * forward_multiplier,
                    encoderTicksToInches(frontEncoder.getCorrectedVelocity()) * strafing_multiplier
            );
        }

        return Arrays.asList(
                encoderTicksToInches(leftEncoder.getRawVelocity()) * forward_multiplier,
                encoderTicksToInches(rightEncoder.getRawVelocity()) * forward_multiplier,
                encoderTicksToInches(frontEncoder.getRawVelocity()) * strafing_multiplier
        );
    }
}