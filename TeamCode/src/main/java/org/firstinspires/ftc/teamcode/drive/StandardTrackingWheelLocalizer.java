package org.firstinspires.ftc.teamcode.drive;

import static org.firstinspires.ftc.teamcode.drive.DriveConstants.TRACKER_FORWARD_OFFSET;
import static org.firstinspires.ftc.teamcode.drive.DriveConstants.TRACKER_INCHES_PER_TICK;
import static org.firstinspires.ftc.teamcode.drive.DriveConstants.TRACKER_LATERAL_DISTANCE;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.util.Encoder;

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
public class StandardTrackingWheelLocalizer extends ThreeTrackingWheelLocalizerNoReset {
    public Encoder leftEncoder, rightEncoder, frontEncoder;

    public StandardTrackingWheelLocalizer(HardwareMap hardwareMap) {
        super(Arrays.asList(
                new Pose2d(0, TRACKER_LATERAL_DISTANCE / 2, 0), // left
                new Pose2d(0, -TRACKER_LATERAL_DISTANCE / 2), // right
                new Pose2d(TRACKER_FORWARD_OFFSET, 0, Math.toRadians(90)) // front (direction flipped)
        ));

        leftEncoder  = new Encoder(hardwareMap.get(DcMotorEx.class, "lefte"));
        frontEncoder = new Encoder(hardwareMap.get(DcMotorEx.class, "backe"));
        rightEncoder = new Encoder(hardwareMap.get(DcMotorEx.class, "righte"));

        // TODO: reverse any encoders using Encoder.setDirection(Encoder.Direction.REVERSE)
        leftEncoder.setDirection(Encoder.Direction.FORWARD);
        rightEncoder.setDirection(Encoder.Direction.REVERSE);
    }

    public static double trackerEncoderTicksToInches(double ticks) {
        return ticks * TRACKER_INCHES_PER_TICK;
    }

    public String getRawEncoderText() {
        return String.format("L,R,F = %d, %d, %d",  leftEncoder.getCurrentPosition(),  rightEncoder.getCurrentPosition(),  frontEncoder.getCurrentPosition());
    }

    @NonNull
    @Override
    public List<Double> getWheelPositions() {
        return Arrays.asList(
                trackerEncoderTicksToInches(leftEncoder.getCurrentPosition()),
                trackerEncoderTicksToInches(rightEncoder.getCurrentPosition()),
                trackerEncoderTicksToInches(frontEncoder.getCurrentPosition())
        );
    }

    @NonNull
    @Override
    public List<Double> getWheelVelocities() {
        return Arrays.asList(
                trackerEncoderTicksToInches(leftEncoder.getCorrectedVelocity()),
                trackerEncoderTicksToInches(rightEncoder.getCorrectedVelocity()),
                trackerEncoderTicksToInches(frontEncoder.getCorrectedVelocity())
        );
    }
}
