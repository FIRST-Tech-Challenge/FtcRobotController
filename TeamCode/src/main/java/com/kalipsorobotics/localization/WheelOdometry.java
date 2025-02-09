package com.kalipsorobotics.localization;

import android.os.SystemClock;
import android.util.Log;

import com.kalipsorobotics.utilities.SharedData;
import com.kalipsorobotics.math.MathFunctions;
import com.kalipsorobotics.modules.IMUModule;
import com.qualcomm.robotcore.hardware.DcMotor;

import com.kalipsorobotics.math.Position;
import com.kalipsorobotics.math.Velocity;

import com.kalipsorobotics.utilities.OpModeUtilities;
import com.kalipsorobotics.modules.DriveTrain;

public class WheelOdometry {
    private static WheelOdometry single_instance = null;
    KalmanFilter kalmanFilter = new KalmanFilter(0.5, 5, 5);
    OpModeUtilities opModeUtilities;
    IMUModule imuModule;

    SensorFusion sensorFusion;

    private double wheelHeadingWeight = 0;
    private double imuHeadingWeight = 1;
    final static private double TRACK_WIDTH_MM = 297;
    //maybe double check BACK Distance
    static private final double BACK_DISTANCE_TO_MID_ROBOT_MM = -70;

    //200-182 offset compare to line between parallel odo pods
    //negative if robot center behind parallel wheels
    //final private static double ROBOT_CENTER_OFFSET_MM = -18;
    private DcMotor rightEncoder;
    private DcMotor leftEncoder;
    private DcMotor backEncoder;
    volatile private Position currentPosition;
    volatile private Velocity currentVelocity;
    private volatile double prevRightDistanceMM;
    private volatile double prevLeftDistanceMM;
    volatile private double prevBackDistanceMM;
    private volatile long prevTime;
    private volatile double currentImuHeading;
    private volatile double prevImuHeading;
//    private final double MM_TO_INCH = 1/25.4;

    private WheelOdometry(OpModeUtilities opModeUtilities, DriveTrain driveTrain, IMUModule imuModule, double xCoordinate, double yCoordinate, double thetaDeg) {
        this.opModeUtilities = opModeUtilities;
        resetHardware(opModeUtilities, driveTrain, imuModule, this);

        this.currentPosition = new Position(xCoordinate, yCoordinate, Math.toRadians(thetaDeg));
        Log.d("purepursaction_debug_odo_wheel", "init jimmeh" + currentPosition.toString());
        prevTime = SystemClock.elapsedRealtime();
        prevImuHeading = getIMUHeading();
        currentImuHeading = prevImuHeading;
        prevRightDistanceMM = countRight();
        prevLeftDistanceMM = countLeft();
        prevBackDistanceMM = countBack();

        sensorFusion = new SensorFusion();
    }

    public static synchronized WheelOdometry getInstance(OpModeUtilities opModeUtilities, DriveTrain driveTrain, IMUModule imuModule, double xCoordinate, double yCoordinate, double thetaDeg) {
        if (single_instance == null) {
            single_instance = new WheelOdometry(opModeUtilities, driveTrain, imuModule, xCoordinate, yCoordinate, thetaDeg);
        } else {
            resetHardware(opModeUtilities, driveTrain, imuModule, single_instance);
        }
        return single_instance;
    }

    private static void resetHardware(OpModeUtilities opModeUtilities, DriveTrain driveTrain, IMUModule imuModule, WheelOdometry wheelOdometry) {
        wheelOdometry.imuModule = imuModule;
        wheelOdometry.rightEncoder = driveTrain.getRightEncoder();
        wheelOdometry.leftEncoder = driveTrain.getLeftEncoder();
        wheelOdometry.backEncoder = driveTrain.getBackEncoder();
    }

    public static void setInstanceNull() {
        single_instance = null;
    }

    private double ticksToMM(double ticks) {
        final double DEAD_WHEEL_RADIUS_MM = 24;
        final double TICKS_PER_REV = 2000;
        final double TICKS_TO_MM = 2.0 * Math.PI * DEAD_WHEEL_RADIUS_MM / TICKS_PER_REV;

        return ticks * TICKS_TO_MM;
    }

    public double countRight() {
        //corresponds to fRight
        //direction FORWARD
        //negative because encoder directions
        return ticksToMM(rightEncoder.getCurrentPosition());
    }
    public double countLeft() {
        //corresponds to fLeft
        //direction FORWARD
        //positive because encoder directions
        return ticksToMM(leftEncoder.getCurrentPosition());}
    public double countBack() {
        //corresponds to bRight
        //direction REVERSE
        //positive because encoder directions
        return ticksToMM(backEncoder.getCurrentPosition());
    }

    private Velocity calculateRelativeDelta(double rightDistanceMM, double leftDistanceMM, double backDistanceMM,
                                            double deltaTimeMS) {
        double deltaRightDistance = rightDistanceMM - prevRightDistanceMM;
        double deltaLeftDistance = leftDistanceMM - prevLeftDistanceMM;
        double deltaMecanumDistance = backDistanceMM - prevBackDistanceMM;

        double encoderDeltaTheta = -(deltaRightDistance - deltaLeftDistance) / (TRACK_WIDTH_MM);
        double imuDeltaTheta = currentImuHeading - prevImuHeading;
        //wrapping to normalize theta -pi to pi
        imuDeltaTheta = MathFunctions.angleWrapRad(imuDeltaTheta);
        double arcTanDeltaTheta = Math.atan2(deltaLeftDistance - deltaRightDistance, TRACK_WIDTH_MM);


//        Position newPosition = kalmanFilter.update(new Position(0, 0, arcTanDeltaTheta));
//        newPosition = kalmanFilter.update(new Position(0, 0, imuDeltaTheta));
//        newPosition = kalmanFilter.update(new Position(0, 0, encoderDeltaTheta));

        Log.d("purepursaction_debug_odo_wheel_deltaTheta",
                String.format("encoder = %.4f, imu = %.4f, arcTan = %.4f", encoderDeltaTheta, imuDeltaTheta,
                        arcTanDeltaTheta));
        //double blendedDeltaTheta = (newPosition.getTheta());

//        double blendedDeltaTheta =
//                (wheelHeadingWeight * encoderDeltaTheta) + (imuHeadingWeight * imuDeltaTheta) + (wheelHeadingWeight * arcTanDeltaTheta);

        double blendedDeltaTheta = sensorFusion.getFilteredAngle(imuDeltaTheta, arcTanDeltaTheta, deltaTimeMS);

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

        double deltaX = relativeDelta.getX();
        double deltaY = relativeDelta.getY();

        double newX = deltaX * cosTheta - deltaY * sinTheta;
        double newY = deltaY * cosTheta + deltaX * sinTheta;

        //use blended heading
        double newTheta = relativeDelta.getTheta();

        return new Velocity(newX, newY, newTheta);
    }

    private Position calculateGlobal(Velocity relativeDelta, Position previousGlobalPosition) {
        Velocity globalDelta = rotate(relativeDelta, previousGlobalPosition);
        Log.d("global delta", globalDelta.toString());
        Position position = previousGlobalPosition.add(globalDelta);
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
        double timeElapsedSeconds = (currentTime - prevTime) / 1000.0;

        Velocity relativeDelta = calculateRelativeDelta(rightDistanceMM, leftDistanceMM,
                backDistanceMM, timeElapsedSeconds * 1000);
        Log.d("relativeDelta", relativeDelta.toString());
        relativeDelta = linearToArcDelta(relativeDelta);

        currentVelocity = relativeDelta.divide(timeElapsedSeconds);
        prevTime = currentTime;

        currentPosition = calculateGlobal(relativeDelta, currentPosition);
        Log.d("currentpos", "current pos " + currentPosition.toString());

        prevRightDistanceMM = rightDistanceMM;
        prevLeftDistanceMM = leftDistanceMM;
        prevBackDistanceMM = backDistanceMM;

        prevImuHeading = currentImuHeading;

        SharedData.setOdometryPosition(currentPosition);

        return currentPosition;
    }

    public double getImuHeadingWeight() {
        return imuHeadingWeight;
    }

    public void setImuHeadingWeight(double imuHeadingWeight) {
        this.imuHeadingWeight = imuHeadingWeight;
    }

    public double getWheelHeadingWeight() {
        return wheelHeadingWeight;
    }

    public void setWheelHeadingWeight(double wheelHeadingWeight) {
        this.wheelHeadingWeight = wheelHeadingWeight;
    }

    public Velocity getCurrentVelocity() {
        return currentVelocity;
    }

    private Position getCurrentPosition() {
        return currentPosition;
    }

    public void setCurrentPosition(double xCoordinate, double yCoordinate, double thetaRad) {
        this.currentPosition = new Position(xCoordinate, yCoordinate, thetaRad);
    }

    public IMUModule getImuModule() {return imuModule;}

    public double getCurrentImuHeading() {
        return this.currentImuHeading;
    }

    public double countX() {
        return (countLeft() + countRight()) / 2;
    }
}