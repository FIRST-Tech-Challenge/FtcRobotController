package org.firstinspires.ftc.teamcode.drive;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.localization.ThreeTrackingWheelLocalizer;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.teamcode.GoBildaPinpointDriver;
import com.acmerobotics.roadrunner.localization.TwoTrackingWheelLocalizer;


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
public class PinpointLocalizer extends TwoTrackingWheelLocalizer {


    public static double LATERAL_DISTANCE = 6.5; // in; distance between the left and right wheels
    public static double FORWARD_OFFSET = 2.5; // in; offset of the lateral wheel

    private GoBildaPinpointDriver odo;


    public PinpointLocalizer(HardwareMap hardwareMap) {
        super(Arrays.asList(
                new Pose2d(-LATERAL_DISTANCE, -FORWARD_OFFSET , 0), // X encoder
                new Pose2d(-LATERAL_DISTANCE, FORWARD_OFFSET , Math.toRadians(90)) // Y encoder
        ));


        odo = hardwareMap.get(GoBildaPinpointDriver.class, "odo");
       /*
        Set the odometry pod positions relative to the point that the odometry computer tracks around.
        The X pod offset refers to how far sideways from the tracking point the
        X (forward) odometry pod is. Left of the center is a positive number,
        right of center is a negative number. the Y pod offset refers to how far forwards from
        the tracking point the Y (strafe) odometry pod is. forward of center is a positive number,
        backwards is a negative number.
         */
        odo.setOffsets(-LATERAL_DISTANCE * 25.4, -FORWARD_OFFSET * 25.4); //these are tuned for 3110-0002-0001 Product Insight #1

        /*
        Set the kind of pods used by your robot. If you're using goBILDA odometry pods, select either
        the goBILDA_SWINGARM_POD, or the goBILDA_4_BAR_POD.
        If you're using another kind of odometry pod, uncomment setEncoderResolution and input the
        number of ticks per mm of your odometry pod.
         */
        odo.setEncoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD);
        //odo.setEncoderResolution(13.26291192);


        /*
        Set the direction that each of the two odometry pods count. The X (forward) pod should
        increase when you move the robot forward. And the Y (strafe) pod should increase when
        you move the robot to the left.
         */
        odo.setEncoderDirections(GoBildaPinpointDriver.EncoderDirection.FORWARD, GoBildaPinpointDriver.EncoderDirection.REVERSED);


        /*
        Before running the robot, recalibrate the IMU. This needs to happen when the robot is stationary
        The IMU will automatically calibrate when first powered on, but recalibrating before running
        the robot is a good idea to ensure that the calibration is "good".
        resetPosAndIMU will reset the position to 0,0,0 and also recalibrate the IMU.
        This is recommended before you run your autonomous, as a bad initial calibration can cause
        an incorrect starting value for x, y, and heading.
         */
        odo.recalibrateIMU();
        //odo.resetPosAndIMU();

    }


    @NonNull
    @Override
    public List<Double> getWheelPositions() {
        Pose2D pos = odo.getPosition();
        return Arrays.asList(
                pos.getX(DistanceUnit.INCH),
                pos.getY(DistanceUnit.INCH)
        );
    }

    @NonNull
    @Override
    public void update() {
        odo.update();
    }

    @NonNull
    @Override
    public double getHeading() {
        Pose2D pos = odo.getPosition();

        return pos.getHeading(AngleUnit.RADIANS);
    }
}
