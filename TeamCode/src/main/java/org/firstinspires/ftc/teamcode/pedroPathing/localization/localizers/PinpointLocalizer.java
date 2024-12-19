//package org.firstinspires.ftc.teamcode.pedroPathing.localization.localizers;
//
//
//import com.qualcomm.robotcore.hardware.HardwareMap;
//
//import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
//import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
//import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
//import org.firstinspires.ftc.teamcode.pedroPathing.localization.GoBildaPinpointDriver;
//import org.firstinspires.ftc.teamcode.pedroPathing.localization.Localizer;
//import org.firstinspires.ftc.teamcode.pedroPathing.localization.Pose;
//import org.firstinspires.ftc.teamcode.pedroPathing.pathGeneration.MathFunctions;
//import org.firstinspires.ftc.teamcode.pedroPathing.pathGeneration.Vector;
//
///**
// * This is the Pinpoint class. This class extends the Localizer superclass and is a
// * localizer that uses the two wheel odometry set up with the IMU to have more accurate heading
// * readings. The diagram below, which is modified from Road Runner, shows a typical set up.
// *
// * The view is from the top of the robot looking downwards.
// *
// * left on robot is the y positive direction
// *
// * forward on robot is the x positive direction
// *
// *                    forward (x positive)
// *                                △
// *                                |
// *                                |
// *                         /--------------\
// *                         |              |
// *                         |              |
// *                         |           || |
// *  left (y positive) <--- |           || |
// *                         |     ____     |
// *                         |     ----     |
// *                         \--------------/
// * With the pinpoint your readings will be used in mm
// * to use inches ensure to divide your mm value by 25.4
// * @author Logan Nash
// * @author Havish Sripada 12808 - RevAmped Robotics
// * @author Ethan Doak - Gobilda
// * @version 1.0, 10/2/2024
// */
//public class PinpointLocalizer extends Localizer {
//    private HardwareMap hardwareMap;
//    private GoBildaPinpointDriver odo;
//    private double previousHeading;
//    private double totalHeading;
//
//    /**
//     * This creates a new PinpointLocalizer from a HardwareMap, with a starting Pose at (0,0)
//     * facing 0 heading.
//     *
//     * @param map the HardwareMap
//     */
//    public PinpointLocalizer(HardwareMap map){ this(map, new Pose());}
//
//    /**
//     * This creates a new PinpointLocalizer from a HardwareMap and a Pose, with the Pose
//     * specifying the starting pose of the localizer.
//     *
//     * @param map the HardwareMap
//     * @param setStartPose the Pose to start from
//     */
//    public PinpointLocalizer(HardwareMap map, Pose setStartPose){
//        hardwareMap = map;
//        // TODO: replace this with your Pinpoint port
//        odo = hardwareMap.get(GoBildaPinpointDriver.class,"odo");
//
//        //This uses mm, to use inches divide these numbers by 25.4
//        odo.setOffsets(-84.0, -168.0); //these are tuned for 3110-0002-0001 Product Insight #1
//        //TODO: If you find that the gobilda Yaw Scaling is incorrect you can edit this here
//      //  odo.setYawScalar(1.0);
//        //TODO: Set your encoder resolution here, I have the Gobilda Odometry products already included.
//        //TODO: If you would like to use your own odometry pods input the ticks per mm in the commented part below
//        odo.setEncoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD);
//        //odo.setEncoderResolution(13.26291192);
//        //TODO: Set encoder directions
//        odo.setEncoderDirections(GoBildaPinpointDriver.EncoderDirection.FORWARD, GoBildaPinpointDriver.EncoderDirection.FORWARD);
//
//        resetPinpoint();;

//        setStartPose(setStartPose);
//        totalHeading = 0;
//        previousHeading = setStartPose.getHeading();
//    }
//
//    /**
//     * This returns the current pose estimate.
//     *
//     * @return returns the current pose estimate as a Pose
//     */
//    @Override
//    public Pose getPose() {
//        Pose2D rawPose = odo.getPosition();
//        return new Pose(rawPose.getX(DistanceUnit.INCH), rawPose.getY(DistanceUnit.INCH), rawPose.getHeading(AngleUnit.RADIANS));
//    }
//
//    /**
//     * This returns the current velocity estimate.
//     *
//     * @return returns the current velocity estimate as a Pose
//     */
//    @Override
//    public Pose getVelocity() {
//        Pose2D pose = odo.getVelocity();
//        return new Pose(pose.getX(DistanceUnit.INCH), pose.getY(DistanceUnit.INCH), odo.getHeadingVelocity());
//    }
//
//    /**
//     * This returns the current velocity estimate.
//     *
//     * @return returns the current velocity estimate as a Vector
//     */
//    @Override
//    public Vector getVelocityVector() {
//        Pose2D pose = odo.getVelocity();
//        Vector returnVector = new Vector();
//        returnVector.setOrthogonalComponents(pose.getX(DistanceUnit.INCH), pose.getY(DistanceUnit.INCH));
//        return returnVector;
//    }
//
//    /**
//     * This sets the start pose. Since nobody should be using this after the robot has begun moving,
//     * and due to issues with the PinpointLocalizer, this is functionally the same as setPose(Pose).
//     *
//     * @param setStart the new start pose
//     */
//    @Override
//    public void setStartPose(Pose setStart) {
//        odo.setPosition(new Pose2D(DistanceUnit.INCH, setStart.getX(), setStart.getY(), AngleUnit.RADIANS, setStart.getHeading()));
//    }
//
//    /**
//     * This sets the current pose estimate. Changing this should just change the robot's current
//     * pose estimate, not anything to do with the start pose.
//     *
//     * @param setPose the new current pose estimate
//     */
//    @Override
//    public void setPose(Pose setPose) {
//        odo.setPosition(new Pose2D(DistanceUnit.INCH, setPose.getX(), setPose.getY(), AngleUnit.RADIANS, setPose.getHeading()));
//    }
//
//    /**
//     * This updates the total heading of the robot. The Pinpoint handles all other updates itself.
//     */
//    @Override
//    public void update() {
//        odo.update();
//        totalHeading += MathFunctions.getSmallestAngleDifference(MathFunctions.normalizeAngle(odo.getHeading()), previousHeading);
//        previousHeading = MathFunctions.normalizeAngle(odo.getHeading());
//    }
//
//    /**
//     * This returns how far the robot has turned in radians, in a number not clamped between 0 and
//     * 2 * pi radians. This is used for some tuning things and nothing actually within the following.
//     *
//     * @return returns how far the robot has turned in total, in radians.
//     */
//    @Override
//    public double getTotalHeading() {
//        return totalHeading;
//    }
//
//    /**
//     * This returns the Y encoder value as none of the odometry tuners are required for this localizer
//     * @return returns the Y encoder value
//     */
//    @Override
//    public double getForwardMultiplier() {
//        return odo.getEncoderY();
//    }
//
//    /**
//     * This returns the X encoder value as none of the odometry tuners are required for this localizer
//     * @return returns the X encoder value
//     */
//    @Override
//    public double getLateralMultiplier() {
//        return odo.getEncoderX();
//    }
//
//    /**
//     * This returns either the factory tuned yaw scalar or the yaw scalar tuned by yourself.
//     * @return returns the yaw scalar
//     */
//    @Override
//    public double getTurningMultiplier() {
//        return odo.getYawScalar();
//    }
//
//    /**
//     * This resets the IMU. Note: This does not change the estimated heading orientation.
//     */
//    @Override
//    public void resetIMU() throws InterruptedException {
//        odo.recalibrateIMU();
//
//        try {
//            Thread.sleep(300);
//        } catch (InterruptedException e) {
//            throw new RuntimeException(e);
//        }
//    }
//
//    /**
//     * This resets the pinpoint.
//     */
//    private void resetPinpoint() {
//        odo.resetPosAndIMU();
//
//        try {
//            Thread.sleep(300);
//        } catch (InterruptedException e) {
//            throw new RuntimeException(e);
//        }
//    }
//}
