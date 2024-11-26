package com.kalipsorobotics.localization;

import android.os.SystemClock;
import android.util.Log;

import com.kalipsorobotics.math.MathFunctions;
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
    static private final double BACK_DISTANCE_TO_MID_ROBOT_MM = -70;

    //200-182 offset compare to line between parallel odo pods
    //negative if robot center behind parallel wheels
    //final private static double ROBOT_CENTER_OFFSET_MM = -18;
    private final DcMotor rightEncoder;
    private final DcMotor leftEncoder;
    private final DcMotor backEncoder;
    volatile private Position currentPosition;
    volatile private Velocity currentVelocity;
    private volatile double prevRightDistanceMM;
    private volatile double prevLeftDistanceMM;
    volatile private double prevBackDistanceMM;
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
        prevRightDistanceMM = countRight();
        prevLeftDistanceMM = countLeft();
        prevBackDistanceMM = countBack();
    }

    private double ticksToMM(double ticks) {
        final double DEAD_WHEEL_RADIUS_MM = 24;
        final double TICKS_PER_REV = 2000;
        final double TICKS_TO_MM = 2.0 * Math.PI * DEAD_WHEEL_RADIUS_MM / TICKS_PER_REV;

        return ticks * TICKS_TO_MM;
    }

    public double countRight() {
        return ticksToMM(rightEncoder.getCurrentPosition());
    }
    public double countLeft() {
        return ticksToMM(leftEncoder.getCurrentPosition());}
    public double countBack() {
        return ticksToMM(backEncoder.getCurrentPosition());
    }

    private Velocity calculateRelativeDelta(double rightDistanceMM, double leftDistanceMM, double backDistanceMM) {
        double deltaRightDistance = rightDistanceMM - prevRightDistanceMM;
        double deltaLeftDistance = leftDistanceMM - prevLeftDistanceMM;
        double deltaMecanumDistance = backDistanceMM - prevBackDistanceMM;

        double encoderDeltaTheta = -(deltaRightDistance - deltaLeftDistance) / (TRACK_WIDTH_MM);
        double imuDeltaTheta = currentImuHeading - prevImuHeading;

        double arcTanDeltaTheta = Math.atan2(deltaLeftDistance - deltaRightDistance, TRACK_WIDTH_MM);

        //wrapping to normalize theta -pi to pi
        imuDeltaTheta = MathFunctions.angleWrapRad(imuDeltaTheta);



        Log.d("purepursaction_debug_odo_wheel_deltaTheta",
                String.format("encoder = %.4f, imu = %.4f, arcTan = %.4f", encoderDeltaTheta, imuDeltaTheta,
                        arcTanDeltaTheta));

        double blendedDeltaTheta = (0.7 * encoderDeltaTheta) + (0.3 * imuDeltaTheta);
        double deltaTheta = blendedDeltaTheta; //blended compliment eachother — to reduce drift of imu in big movement and to detect small change

        double deltaX = (deltaLeftDistance + deltaRightDistance) / 2;


        double deltaY = (deltaMecanumDistance - BACK_DISTANCE_TO_MID_ROBOT_MM * deltaTheta);


        Velocity velocity = new Velocity(deltaX, deltaY, deltaTheta);

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

        double relDeltaY =
                +strafeRadius * Math.sin(relativeDelta.getTheta()) + forwardRadius * (1 - Math.cos(relativeDelta.getTheta()));

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

        double adjustedDeltaX = relativeDelta.getX();
        double adjustedDeltaY = relativeDelta.getY();

        double newX = adjustedDeltaX * cosTheta - adjustedDeltaY * sinTheta;
        double newY = adjustedDeltaY * cosTheta + adjustedDeltaX * sinTheta;

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
        return -Math.toRadians(imuModule.getIMU().getRobotYawPitchRollAngles().getYaw());
    }

    public Position updatePosition() {
        double rightDistanceMM = countRight();
        double leftDistanceMM = countLeft();
        double backDistanceMM = countBack();
        currentImuHeading = getIMUHeading();

        Log.d("updatepos", rightDistanceMM + " " + leftDistanceMM + " " + backDistanceMM);

        long currentTime = SystemClock.elapsedRealtime();
        double timeElapsed = (currentTime - prevTime) / 1000.;

        Velocity relativeDelta = calculateRelativeDelta(rightDistanceMM, leftDistanceMM, backDistanceMM);
        Log.d("relativeDelta", relativeDelta.toString());
        relativeDelta = linearToArcDelta(relativeDelta);

        currentVelocity = relativeDelta.divide(timeElapsed);
        prevTime = currentTime;

        currentPosition = updateGlobal(relativeDelta, currentPosition);
        Log.d("currentpos", "current pos " + currentPosition.getTheta());

        prevRightDistanceMM = rightDistanceMM;
        prevLeftDistanceMM = leftDistanceMM;
        prevBackDistanceMM = backDistanceMM;

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