package org.firstinspires.ftc.teamcode.drive;

import androidx.annotation.NonNull;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.localization.ThreeTrackingWheelLocalizer;
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
public class StandardTrackingWheelLocalizer /*extends ThreeTrackingWheelLocalizer*/ {
    public static double TICKS_PER_REV = DriveConstants.TICKS_PER_REV;//8192 for rev encoder//roll1Loop:11.8737
    public static double WHEEL_RADIUS = DriveConstants.WHEEL_RADIUS; // in,0.6889764
    public static double GEAR_RATIO = DriveConstants.GEAR_RATIO; // output (wheel) speed / input (encoder) speed

    public static double LATERAL_DISTANCE = DriveConstants.TRACK_WIDTH; // in; distance between the left and right wheels
    public static double FORWARD_OFFSET = DriveConstants.FORWARD_OFFSET; // in; offset of the lateral wheel

    private Encoder leftEncoder, rightEncoder, frontEncoder;
    // x: 50/49.466272502353554 + 50/49.68096645210301 + 50/49.2869229175432 + 50/49.41976823851759 + 50/49.39283574196898
    // y: 50/49.6626437095775 + 50/49.59460500834778 + 50/49.58689479143788 + 50/49.44851181207158 + 50/49.33879937319335
    public static double X_MULTIPLIER = 1.0;
    public static double Y_MULTIPLIER = 1.008415093;


    public StandardTrackingWheelLocalizer(HardwareMap hardwareMap) {
//        super(Arrays.asList(
//                new Pose2d(0, LATERAL_DISTANCE / 2, 0), // left
//                new Pose2d(0, -LATERAL_DISTANCE / 2, 0), // right
//                new Pose2d(FORWARD_OFFSET, 0, Math.toRadians(90.0)) // front
//        ));
        frontEncoder = new Encoder(hardwareMap.get(DcMotorEx.class, "front"));
        leftEncoder = new Encoder(hardwareMap.get(DcMotorEx.class, "left"));
        rightEncoder = new Encoder(hardwareMap.get(DcMotorEx.class, "right"));

        // TODO: reverse any encoders using Encoder.setDirection(Encoder.Direction.REVERSE)
        frontEncoder.setDirection(Encoder.Direction.REVERSE);
        leftEncoder.setDirection(Encoder.Direction.REVERSE);
        rightEncoder.setDirection(Encoder.Direction.REVERSE);
    }

    public static double encoderTicksToInches(double ticks) {
        return WHEEL_RADIUS * 2 * Math.PI * GEAR_RATIO * ticks / TICKS_PER_REV;
    }

    public List<Double> getWheelPos() {
        return Arrays.asList(
                encoderTicksToInches(leftEncoder.getCurrentPosition()) * X_MULTIPLIER,
                encoderTicksToInches(rightEncoder.getCurrentPosition()) * X_MULTIPLIER,
                encoderTicksToInches(frontEncoder.getCurrentPosition()) * Y_MULTIPLIER
        );
    }
//
//    @NonNull
//    @Override
//    public List<Double> getWheelPositions() {
//        return Arrays.asList(
//                encoderTicksToInches(leftEncoder.getCurrentPosition()) * X_MULTIPLIER,
//                encoderTicksToInches(rightEncoder.getCurrentPosition()) * X_MULTIPLIER,
//                encoderTicksToInches(frontEncoder.getCurrentPosition()) * Y_MULTIPLIER
//        );
//    }
//
//    @NonNull
//    @Override
//    public List<Double> getWheelVelocities() {
//        // TODO: If your encoder velocity can exceed 32767 counts / second (such as the REV Through Bore and other
//        //  competing magnetic encoders), change Encoder.getRawVelocity() to Encoder.getCorrectedVelocity() to enable a
//        //  compensation method
//        return Arrays.asList(
//                encoderTicksToInches(leftEncoder.getCorrectedVelocity()) * X_MULTIPLIER,
//                encoderTicksToInches(rightEncoder.getCorrectedVelocity()) * X_MULTIPLIER,
//                encoderTicksToInches(frontEncoder.getCorrectedVelocity()) * Y_MULTIPLIER
//        );
//    }

}