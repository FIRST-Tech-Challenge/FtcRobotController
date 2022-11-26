package com.acmerobotics.roadrunner.drive;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.localization.TwoTrackingWheelLocalizer;
import com.acmerobotics.roadrunner.util.Encoder;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

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
@Config
public class TwoWheelTrackingLocalizer extends TwoTrackingWheelLocalizer {

   public static double WHEEL_DIAMETER_MM = 35;
   public static double IN_PER_MM = 0.03937;

   public static double TICKS_PER_REV = 8192;
   public static double WHEEL_RADIUS = (WHEEL_DIAMETER_MM * IN_PER_MM) / 2; // in
   public static double GEAR_RATIO = 1; // output (wheel) speed / input (encoder) speed

   public static double LATERAL_DISTANCE = 6.402; // in; distance between the left and right wheels
   public static double FORWARD_OFFSET = -5.25; // in; offset of the lateral wheel

   public static double PARALLEL_X = 1.75; // X is the up and down direction
   /*
    * Negate this value if using the right encoder
    */
   public static double PARALLEL_Y = LATERAL_DISTANCE / 2; // Y positive is to the left.

   public static double PERPENDICULAR_X = 0.25;
   public static double PERPENDICULAR_Y = FORWARD_OFFSET;

   public static double X_MULTIPLIER = 1.0139;

   // Parallel/Perpendicular to the forward axis
   // Parallel wheel is parallel to the forward axis
   // Perpendicular is perpendicular to the forward axis
   private Encoder parallelEncoder, perpendicularEncoder;

   private SampleMecanumDrive drive;

   public TwoWheelTrackingLocalizer(HardwareMap hardwareMap, SampleMecanumDrive drive) {
      super(Arrays.asList(
              new Pose2d(PARALLEL_X, PARALLEL_Y, 0),
              new Pose2d(PERPENDICULAR_X, PERPENDICULAR_Y, Math.toRadians(90))
      ));

      this.drive = drive;

      parallelEncoder = new Encoder(hardwareMap.get(DcMotorEx.class, "leftEncoder"));
      perpendicularEncoder = new Encoder(hardwareMap.get(DcMotorEx.class, "lateralEncoder"));

      // TODO: reverse any encoders using Encoder.setDirection(Encoder.Direction.REVERSE)
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

   public int getParallelEncoderPosition() {
      return parallelEncoder.getCurrentPosition();
   }

   @NonNull
   @Override
   public List<Double> getWheelPositions() {
      return Arrays.asList(
              encoderTicksToInches(parallelEncoder.getCurrentPosition() * X_MULTIPLIER),
              encoderTicksToInches(perpendicularEncoder.getCurrentPosition())
      );
   }

   @NonNull
   @Override
   public List<Double> getWheelVelocities() {
      // TODO: If your encoder velocity can exceed 32767 counts / second (such as the REV Through Bore and other
      //  competing magnetic encoders), change Encoder.getRawVelocity() to Encoder.getCorrectedVelocity() to enable a
      //  compensation method

      return Arrays.asList(
              encoderTicksToInches(parallelEncoder.getCorrectedVelocity() * X_MULTIPLIER),
              encoderTicksToInches(perpendicularEncoder.getCorrectedVelocity())
      );
   }
}
