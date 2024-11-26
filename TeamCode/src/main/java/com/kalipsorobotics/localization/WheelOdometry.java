package com.kalipsorobotics.localization;

import android.os.SystemClock;
import android.util.Log;

import com.kalipsorobotics.modules.IMUModule;
import com.qualcomm.robotcore.hardware.DcMotor;

import com.kalipsorobotics.math.Position;
import com.kalipsorobotics.math.Velocity;

import com.kalipsorobotics.utilities.OpModeUtilities;
import com.kalipsorobotics.modules.DriveTrain;

public class WheelOdometry {
    OpModeUtilities opModeUtilities;
    IMUModule imuModule;
    final static private double TRACK_WIDTH_MM = 297;
    //maybe double check BACK Distance
    static private final double BACK_DISTANCE_TO_MID_MM = 88.5;

    //200-182 offset compare to line between parallel odo pods
    //negative if robot center behind parallel wheels
    final private static double ROBOT_CENTER_OFFSET_MM = -18;
    //positive if backwheel to the right of robot center
    //private final static double BACK_WHEEL_OFFSET_MM = 17;
    private final DcMotor rightEncoder;
    private final DcMotor leftEncoder;
    private final DcMotor backEncoder;
    volatile private Position currentPosition;
    volatile private Velocity currentVelocity;
    private volatile double prevRightTicks = 0;
    private volatile double prevLeftTicks = 0;
    volatile private double prevBackTicks = 0;
    private volatile long prevTime;
    private volatile double currentImuHeading;
    private volatile double prevImuHeading;
//    private final double MM_TO_INCH = 1/25.4;

    public WheelOdometry(DriveTrain driveTrain, OpModeUtilities opModeUtilities, IMUModule imuModule, double xCoordinate,
                         double yCoordinate, double theta) {
        this.opModeUtilities = opModeUtilities;
        this.imuModule = imuModule;
        this.currentPosition = new Position(xCoordinate, yCoordinate, theta);
        Log.d("purepursaction_debug_odo_wheel", "init jimmeh" + currentPosition.toString());
        this.rightEncoder = driveTrain.getRightEncoder();
        this.leftEncoder = driveTrain.getLeftEncoder();
        this.backEncoder = driveTrain.getBackEncoder();
        prevTime = SystemClock.elapsedRealtime();
        prevImuHeading = getIMUHeading();
        currentImuHeading = prevImuHeading;
    }

    private double ticksToMM(double ticks) {
        final double DEAD_WHEEL_RADIUS_MM = 24;
        final double TICKS_PER_REV = 2000;
        final double TICKS_TO_MM = 2.0 * Math.PI * DEAD_WHEEL_RADIUS_MM / TICKS_PER_REV;

//      final double TICKS_TO_MM = 13.2625995;
        return ticks * TICKS_TO_MM;
    }

    public double countRight() {
        return ticksToMM(rightEncoder.getCurrentPosition());
    }
    public double countLeft() {
        return ticksToMM(leftEncoder.getCurrentPosition());}
    public double countBack() {
        return ticksToMM(-backEncoder.getCurrentPosition());
    }

    private Velocity calculateRelativeDelta(double rightTicks, double leftTicks, double backTicks) {
        double deltaRightDistance = rightTicks - prevRightTicks;
        double deltaLeftDistance = leftTicks - prevLeftTicks;
        double deltaMecanumDistance = backTicks - prevBackTicks;

        double encoderDeltaTheta = -(deltaRightDistance - deltaLeftDistance) / (TRACK_WIDTH_MM);
        double imuDeltaTheta = currentImuHeading - prevImuHeading;
        double blendedDeltaTheta = (0.7 * encoderDeltaTheta) + (0.3 * imuDeltaTheta);
        //blended compliment eachother — to reduce drift of imu in big movement and to detect small change
        double deltaTheta = blendedDeltaTheta;
        //double deltaTheta = -(deltaRightDistance - deltaLeftDistance) / (TRACK_WIDTH_MM);

        double deltaMecanumOffsetAdjustment = -(ROBOT_CENTER_OFFSET_MM * deltaTheta);
        //double deltaMecanumBackwheelOffset =  -(BACK_WHEEL_OFFSET_MM * deltaTheta);
        deltaMecanumDistance = deltaMecanumDistance + deltaMecanumOffsetAdjustment;// + deltaMecanumBackwheelOffset;

        double deltaX = (deltaLeftDistance + deltaRightDistance) / 2;
        double deltaXOffsetAdjustment = -(ROBOT_CENTER_OFFSET_MM * (1 - Math.cos(deltaTheta)));
        //double deltaXBackwheelOffset = -(BACK_WHEEL_OFFSET_MM * Math.sin(deltaTheta));
        deltaX = deltaX + deltaXOffsetAdjustment;// + deltaXBackwheelOffset;

        /*double deltaTheta = currentImuHeading - prevImuHeading;
        wrapping to normalize theta -pi to pi
        deltaTheta = Math.atan2(Math.sin(deltaTheta), Math.cos(deltaTheta));*/
        double deltaY = (deltaMecanumDistance - BACK_DISTANCE_TO_MID_MM * deltaTheta);
        double deltaYOffsetAdjustment = -ROBOT_CENTER_OFFSET_MM * (deltaTheta);
        deltaY = deltaY + deltaYOffsetAdjustment;

        Velocity velocity = new Velocity(deltaX, deltaY, deltaTheta);
//        Log.d("purepursaction_debug_odo_wheel delta", velocity.toString());

        return velocity;
    }

    private Velocity linearToArcDelta(Velocity relativeDelta) {
        if (Math.abs(relativeDelta.getTheta()) < 1e-4) {
            return relativeDelta;
        }

        Log.d("odometry", "linearDelta " + relativeDelta);
        double forwardRadius = relativeDelta.getX() / relativeDelta.getTheta();
        double strafeRadius = relativeDelta.getY() / relativeDelta.getTheta();

        double relDeltaX =
                forwardRadius * Math.sin(relativeDelta.getTheta()) + -strafeRadius * (1 - Math.cos(relativeDelta.getTheta()));
        double relDeltaXOffsetAdjustment = -(ROBOT_CENTER_OFFSET_MM * (1 - Math.cos(relativeDelta.getTheta())));
        /*double relDeltaXBackwheelOffset =
                -(strafeRadius + BACK_WHEEL_OFFSET_MM * 1 - Math.cos(relativeDelta.getTheta()));*/
        relDeltaX = relDeltaX + relDeltaXOffsetAdjustment;// + relDeltaXBackwheelOffset;

        double relDeltaY =
                +strafeRadius * Math.sin(relativeDelta.getTheta()) + forwardRadius * (1 - Math.cos(relativeDelta.getTheta()));
        double relDeltaYOffsetAdjustment = -(ROBOT_CENTER_OFFSET_MM * Math.sin(relativeDelta.getTheta()));
        double relDeltaYBackwheelOffset = forwardRadius * (1 - Math.cos(relativeDelta.getTheta()));
        relDeltaY = relDeltaY + relDeltaYOffsetAdjustment + relDeltaYBackwheelOffset;

//        double relDeltaTheta =
//                MathFunctions.angleWrapRad(relativeDelta.getTheta());
        double relDeltaTheta =
                relativeDelta.getTheta();

        Velocity arcDelta = new Velocity(relDeltaX, relDeltaY, relDeltaTheta);
        if (Math.abs(arcDelta.getTheta()) > 0.01) {
            Log.d("odometry new arc delta", "arcDelta4" + arcDelta);
        }
        return arcDelta;
    }

    //converts to global
    private Velocity rotate(Velocity relativeDelta, Position previousGlobalPosition) {
        double sinTheta = Math.sin(previousGlobalPosition.getTheta());
        double cosTheta = Math.cos(previousGlobalPosition.getTheta());

        double adjustedDeltaX = relativeDelta.getX();// -(BACK_WHEEL_OFFSET_MM * relativeDelta.getTheta());
        double adjustedDeltaY = relativeDelta.getY();

        double newX = adjustedDeltaX * cosTheta - adjustedDeltaY * sinTheta;
        double newY = adjustedDeltaY * cosTheta + adjustedDeltaX * sinTheta;


        /*double newX =
                relativeDelta.getX() * Math.cos(previousGlobalPosition.getTheta()) - relativeDelta.getY() * Math.sin(previousGlobalPosition.getTheta());
        double newY =
                relativeDelta.getY() * Math.cos(previousGlobalPosition.getTheta()) + relativeDelta.getX() * Math.sin(previousGlobalPosition.getTheta());*/
//        double newTheta =
//                MathFunctions.angleWrapRad(relativeDelta.getTheta());

        //use blended heading
        double newTheta = relativeDelta.getTheta();

        return new Velocity(newX, newY, newTheta);
    }

    private Position updateGlobal(Velocity relativeDelta, Position previousGlobalPosition) {
        Velocity globalDelta = rotate(relativeDelta, previousGlobalPosition);
        Log.d("global delta", globalDelta.toString());
        Position position = previousGlobalPosition.add(globalDelta);
        Log.d("thetavalue", "theta " + position.getTheta());
        return position;
    }

    public void run() throws InterruptedException{
        while (true) {
            updatePosition();
            if (Thread.interrupted()) {
                throw new InterruptedException();
            }
        }
    }

    public double getIMUHeading() {
        return Math.toRadians(imuModule.getIMU().getRobotYawPitchRollAngles().getYaw());
    }

    public Position updatePosition() {
        double rightTicks = countRight();
        double leftTicks = countLeft();
        double backTicks = countBack();
        currentImuHeading = getIMUHeading();

        Log.d("updatepos", rightTicks + " " + leftTicks + " " + backTicks);

        long currentTime = SystemClock.elapsedRealtime();
        double timeElapsed = (currentTime - prevTime) / 1000.;

        Velocity relativeDelta = calculateRelativeDelta(rightTicks, leftTicks, backTicks);
        Log.d("relativeDelta", relativeDelta.toString());
        relativeDelta = linearToArcDelta(relativeDelta);

        currentVelocity = relativeDelta.divide(timeElapsed);
        prevTime = currentTime;

        currentPosition = updateGlobal(relativeDelta, currentPosition);
        Log.d("currentpos", "current pos " + currentPosition.getTheta());

        prevRightTicks = rightTicks;
        prevLeftTicks = leftTicks;
        prevBackTicks = backTicks;
        prevImuHeading = currentImuHeading;



        //opModeUtilities.getTelemetry().addData("global", currentPosition.toString());
        return currentPosition;
    }

    public Velocity getCurrentVelocity() {
        return currentVelocity;
    }

    public Position getCurrentPosition() {
        return currentPosition;
    }

    public IMUModule getImuModule() {return imuModule;}

    public double getCurrentImuHeading() {
        return this.currentImuHeading;
    }

    public double countX() {
        return (countLeft() + countRight()) / 2;
    }
}