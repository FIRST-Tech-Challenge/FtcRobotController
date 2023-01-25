package org.firstinspires.ftc.masters.drive;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.localization.ThreeTrackingWheelLocalizer;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import org.firstinspires.ftc.masters.util.Encoder;

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
  public static double WHEEL_RADIUS = 0.69; // in
  public static double GEAR_RATIO = 1; // output (wheel) speed / input (encoder) speed

  public static double LATERAL_DISTANCE = 9.53 ; // in; distance between the left and right wheels
  public static double FORWARD_OFFSET = 0.125; // in; offset of the lateral wheel

  public static double X_MULTIPLIER = ((123/119.87)+(124/120.09)+(125.6/122.15))/3 *(60/((60.3+60.7+60.5)/3));
  public static double Y_MULTIPLIER = ((126.9/117.59)+(127/115.97)+(127/117.86))/3 * 60/((63.35+63.4+63.6)/3);

  private final Encoder leftEncoder;
    private final Encoder rightEncoder;
    private final Encoder frontEncoder;

  public StandardTrackingWheelLocalizer(HardwareMap hardwareMap) {
    super(Arrays.asList(
            new Pose2d(0, LATERAL_DISTANCE / 2, 0), // left
            new Pose2d(0, -LATERAL_DISTANCE / 2, 0), // right
            new Pose2d(FORWARD_OFFSET, 0, Math.toRadians(90)) // front
    ));

    leftEncoder = new Encoder(hardwareMap.get(DcMotorEx.class, "frontRight"));
    rightEncoder = new Encoder(hardwareMap.get(DcMotorEx.class, "frontLeft"));
    frontEncoder  = new Encoder(hardwareMap.get(DcMotorEx.class, "backLeft"));


    // TODO: reverse any encoders using Encoder.setDirection(Encoder.Direction.REVERSE)

    //leftEncoder.setDirection(Encoder.Direction.REVERSE);
//    rightEncoder.setDirection(Encoder.Direction.REVERSE);
    frontEncoder.setDirection(Encoder.Direction.REVERSE);
  }

  public static double encoderTicksToInches(double ticks) {
    return WHEEL_RADIUS * 2 * Math.PI * GEAR_RATIO * ticks / TICKS_PER_REV;
  }

  @NonNull
  @Override
  public List<Double> getWheelPositions() {
    return Arrays.asList(
            encoderTicksToInches(leftEncoder.getCurrentPosition())*X_MULTIPLIER,
            encoderTicksToInches(rightEncoder.getCurrentPosition())*X_MULTIPLIER,
            encoderTicksToInches(frontEncoder.getCurrentPosition())*Y_MULTIPLIER
    );
  }

  @NonNull
  @Override
  public List<Double> getWheelVelocities() {
    // TODO: If your encoder velocity can exceed 32767 counts / second (such as the REV Through Bore and other
    //  competing magnetic encoders), change Encoder.getRawVelocity() to Encoder.getCorrectedVelocity() to enable a
    //  compensation method

    return Arrays.asList(
            encoderTicksToInches(leftEncoder.getCorrectedVelocity())*X_MULTIPLIER,
            encoderTicksToInches(rightEncoder.getCorrectedVelocity())*X_MULTIPLIER,
            encoderTicksToInches(frontEncoder.getCorrectedVelocity())*Y_MULTIPLIER
    );
  }
}