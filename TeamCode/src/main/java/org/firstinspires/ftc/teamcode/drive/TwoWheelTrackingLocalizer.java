package org.firstinspires.ftc.teamcode.drive;

import static org.firstinspires.ftc.teamcode.drive.DriveConstants.TRACKER_INCHES_PER_TICK;
import static org.firstinspires.ftc.teamcode.drive.DriveConstants.TRACKER_PARALLEL_X;
import static org.firstinspires.ftc.teamcode.drive.DriveConstants.TRACKER_PARALLEL_Y;
import static org.firstinspires.ftc.teamcode.drive.DriveConstants.TRACKER_PERPENDICULAR_X;
import static org.firstinspires.ftc.teamcode.drive.DriveConstants.TRACKER_PERPENDICULAR_Y;

import androidx.annotation.NonNull;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.localization.TwoTrackingWheelLocalizer;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.util.Encoder;

import java.util.Arrays;
import java.util.List;

/*
 * Sample tracking wheel localizer implementation assuming the standard configuration:
 *
 *    ^
 *    |
 *    | ( x direction)
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
public class TwoWheelTrackingLocalizer extends TwoTrackingWheelLocalizer {

    private Encoder parallelEncoder, perpendicularEncoder;
    private GFORCE_KiwiDrive drive;

    public TwoWheelTrackingLocalizer(HardwareMap hardwareMap, GFORCE_KiwiDrive drive) {
        super(Arrays.asList(
            new Pose2d(TRACKER_PARALLEL_X, TRACKER_PARALLEL_Y, 0),
            new Pose2d(TRACKER_PERPENDICULAR_X, TRACKER_PERPENDICULAR_Y, Math.toRadians(90))
        ));

        this.drive = drive;

        parallelEncoder = new Encoder(hardwareMap.get(DcMotorEx.class, "lefte"));
        perpendicularEncoder = new Encoder(hardwareMap.get(DcMotorEx.class, "backe"));
        parallelEncoder.setDirection(Encoder.Direction.FORWARD);
    }

    public static double trackerEncoderTicksToInches(double ticks) {
        return ticks * TRACKER_INCHES_PER_TICK;
    }

    public String getRawEncoderText() {
        return String.format("A,L = %d, %d",  parallelEncoder.getCurrentPosition(),  perpendicularEncoder.getCurrentPosition());
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
                trackerEncoderTicksToInches(parallelEncoder.getCurrentPosition()),
                trackerEncoderTicksToInches(perpendicularEncoder.getCurrentPosition())
        );
    }

    @NonNull
    @Override
    public List<Double> getWheelVelocities() {
        return Arrays.asList(
                trackerEncoderTicksToInches(parallelEncoder.getCorrectedVelocity()),
                trackerEncoderTicksToInches(perpendicularEncoder.getCorrectedVelocity())
        );
    }
}
