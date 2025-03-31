package com.kalipsorobotics.localization;

import android.os.SystemClock;

import com.kalipsorobotics.math.PositionHistory;
import com.kalipsorobotics.math.MathFunctions;
import com.kalipsorobotics.modules.IMUModule;
import com.kalipsorobotics.utilities.SharedData;
import com.qualcomm.hardware.sparkfun.SparkFunOTOS;
import com.qualcomm.robotcore.hardware.DcMotor;

import com.kalipsorobotics.math.Position;
import com.kalipsorobotics.math.Velocity;

import com.kalipsorobotics.utilities.OpModeUtilities;
import com.kalipsorobotics.modules.DriveTrain;

import java.util.HashMap;


public class WheelOdometry {
    private static WheelOdometry single_instance = null;
    OpModeUtilities opModeUtilities;
    HashMap<Odometry, PositionHistory> odometryPositionHistoryHashMap = new HashMap<>();

    IMUModule imuModule;

    SparkFunOTOS sparkFunOTOS;

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
    final private PositionHistory wheelPositionHistory = new PositionHistory();
    final private PositionHistory wheelIMUPositionHistory = new PositionHistory();
    final private PositionHistory wheelIMUFusePositionHistory = new PositionHistory();
    final private PositionHistory wheelSparkPositionHistory = new PositionHistory();
    final private PositionHistory wheelSparkFusePositionHistory = new PositionHistory();
    final private PositionHistory wheelIMUSparkFusePositionHistory = new PositionHistory();

    private volatile double prevRightDistanceMM;
    private volatile double prevLeftDistanceMM;
    volatile private double prevBackDistanceMM;
    private volatile long prevTime;
    private volatile double currentImuHeading;
    private volatile double prevImuHeading;
//    private final double MM_TO_INCH = 1/25.4;
    private volatile double currentSparkImuHeading;
    private volatile double prevSparkImuHeading;

    private WheelOdometry(OpModeUtilities opModeUtilities, DriveTrain driveTrain, IMUModule imuModule,
                          Position startPosMMRAD) {
        this.opModeUtilities = opModeUtilities;
        resetHardware(opModeUtilities, driveTrain, imuModule, this);

        this.wheelPositionHistory.setCurrentPosition(startPosMMRAD);
        this.wheelIMUPositionHistory.setCurrentPosition(startPosMMRAD);
        this.wheelIMUFusePositionHistory.setCurrentPosition(startPosMMRAD);
        this.wheelSparkPositionHistory.setCurrentPosition(startPosMMRAD);
        this.wheelSparkFusePositionHistory.setCurrentPosition(startPosMMRAD);
        this.wheelIMUSparkFusePositionHistory.setCurrentPosition(startPosMMRAD);
        ////Log.d("purepursaction_debug_odo_wheel", "init jimmeh" + currentPosition.toString());
        prevTime = SystemClock.elapsedRealtime();
        prevImuHeading = getIMUHeading();
        currentImuHeading = prevImuHeading;
        prevSparkImuHeading = getSparkIMUHeading();
        currentSparkImuHeading = prevSparkImuHeading;
        prevRightDistanceMM = countRight();
        prevLeftDistanceMM = countLeft();
        prevBackDistanceMM = countBack();


        sensorFusion = new SensorFusion();
    }

    private WheelOdometry(OpModeUtilities opModeUtilities, DriveTrain driveTrain, IMUModule imuModule,
                          double startX, double startY, double startThetaDeg) {
        this(opModeUtilities, driveTrain, imuModule, new Position(startX, startY, Math.toRadians(startThetaDeg)));
    }

    public static synchronized WheelOdometry getInstance(OpModeUtilities opModeUtilities, DriveTrain driveTrain,
                                                         IMUModule imuModule, Position startPosMMRAD) {
        if (single_instance == null) {
            single_instance = new WheelOdometry(opModeUtilities, driveTrain, imuModule, startPosMMRAD);
        } else {
            resetHardware(opModeUtilities, driveTrain, imuModule, single_instance);
        }
        return single_instance;
    }

    public static synchronized WheelOdometry getInstance(OpModeUtilities opModeUtilities, DriveTrain driveTrain,
                                                         IMUModule imuModule, double startX, double startY, double startThetaDeg) {
        if (single_instance == null) {
            single_instance = new WheelOdometry(opModeUtilities, driveTrain, imuModule, startX, startY, Math.toRadians(startThetaDeg));
        } else {
            resetHardware(opModeUtilities, driveTrain, imuModule, single_instance);
        }
        return single_instance;
    }

    private static void resetHardware(OpModeUtilities opModeUtilities, DriveTrain driveTrain, IMUModule imuModule, WheelOdometry wheelOdometry) {
        wheelOdometry.imuModule = imuModule;
        wheelOdometry.sparkFunOTOS = driveTrain.getOtos();
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


    private Velocity calculateRelativeDeltaWheel(double rightDistanceMM, double leftDistanceMM, double backDistanceMM,
                                                              double deltaTimeMS) {
        double deltaRightDistance = rightDistanceMM - prevRightDistanceMM;
        double deltaLeftDistance = leftDistanceMM - prevLeftDistanceMM;
        double deltaMecanumDistance = backDistanceMM - prevBackDistanceMM;

        double arcTanDeltaTheta = Math.atan2(deltaLeftDistance - deltaRightDistance, TRACK_WIDTH_MM);

        double deltaX = (deltaLeftDistance + deltaRightDistance) / 2;
        double deltaY = (deltaMecanumDistance - BACK_DISTANCE_TO_MID_ROBOT_MM * arcTanDeltaTheta);

        Velocity velocity = new Velocity(deltaX, deltaY, arcTanDeltaTheta);

        return velocity;
    }


    private Velocity calculateRelativeDeltaWheelIMU(double rightDistanceMM, double leftDistanceMM, double backDistanceMM,
                                                    double deltaTimeMS) {
        double deltaRightDistance = rightDistanceMM - prevRightDistanceMM;
        double deltaLeftDistance = leftDistanceMM - prevLeftDistanceMM;
        double deltaMecanumDistance = backDistanceMM - prevBackDistanceMM;

        double imuDeltaTheta = currentImuHeading - prevImuHeading;

        double deltaX = (deltaLeftDistance + deltaRightDistance) / 2;
        double deltaY = (deltaMecanumDistance - BACK_DISTANCE_TO_MID_ROBOT_MM * imuDeltaTheta);

        Velocity velocity = new Velocity(deltaX, deltaY, imuDeltaTheta);

        return velocity;
    }




    private Velocity calculateRelativeDeltaWheelIMUFuse(double rightDistanceMM, double leftDistanceMM,
                                                    double backDistanceMM,
                         double deltaTimeMS) {
        double deltaRightDistance = rightDistanceMM - prevRightDistanceMM;
        double deltaLeftDistance = leftDistanceMM - prevLeftDistanceMM;
        double deltaMecanumDistance = backDistanceMM - prevBackDistanceMM;

        double imuDeltaTheta = currentImuHeading - prevImuHeading;
        double arcTanDeltaTheta = Math.atan2(deltaLeftDistance - deltaRightDistance, TRACK_WIDTH_MM);

        double weightedAverageDeltaTheta = (0.5 * imuDeltaTheta) + (0.5 * arcTanDeltaTheta);

        double deltaX = (deltaLeftDistance + deltaRightDistance) / 2;
        double deltaY = (deltaMecanumDistance - BACK_DISTANCE_TO_MID_ROBOT_MM * weightedAverageDeltaTheta);

        Velocity velocity = new Velocity(deltaX, deltaY, weightedAverageDeltaTheta);

        return velocity;
    }


    private Velocity calculateRelativeDeltaWheelSpark(double rightDistanceMM, double leftDistanceMM,
                                                    double backDistanceMM,
                         double deltaTimeMS) {
        double deltaRightDistance = rightDistanceMM - prevRightDistanceMM;
        double deltaLeftDistance = leftDistanceMM - prevLeftDistanceMM;
        double deltaMecanumDistance = backDistanceMM - prevBackDistanceMM;

        double sparkFunDeltaTheta = currentSparkImuHeading - prevSparkImuHeading;

        double deltaX = (deltaLeftDistance + deltaRightDistance) / 2;
        double deltaY = (deltaMecanumDistance - BACK_DISTANCE_TO_MID_ROBOT_MM * sparkFunDeltaTheta);

        Velocity velocity = new Velocity(deltaX, deltaY, sparkFunDeltaTheta);

        return velocity;
    }


    private Velocity calculateRelativeDeltaWheelSparkFuse(double rightDistanceMM, double leftDistanceMM,
                                                    double backDistanceMM,
                          double deltaTimeMS) {
        double deltaRightDistance = rightDistanceMM - prevRightDistanceMM;
        double deltaLeftDistance = leftDistanceMM - prevLeftDistanceMM;
        double deltaMecanumDistance = backDistanceMM - prevBackDistanceMM;

        double sparkFunDeltaTheta = currentSparkImuHeading - prevSparkImuHeading;
        double arcTanDeltaTheta = Math.atan2(deltaLeftDistance - deltaRightDistance, TRACK_WIDTH_MM);

        double weightedAverageDeltaTheta = (0.5 * sparkFunDeltaTheta) + (0.5 * arcTanDeltaTheta);

        double deltaX = (deltaLeftDistance + deltaRightDistance) / 2;
        double deltaY = (deltaMecanumDistance - BACK_DISTANCE_TO_MID_ROBOT_MM * weightedAverageDeltaTheta);

        Velocity velocity = new Velocity(deltaX, deltaY, weightedAverageDeltaTheta);

        return velocity;
    }

    private Velocity calculateRelativeDeltaWheelIMUSparkFuse(double rightDistanceMM, double leftDistanceMM,
                                                          double backDistanceMM, double deltaTimeMS) {
        double deltaRightDistance = rightDistanceMM - prevRightDistanceMM;
        double deltaLeftDistance = leftDistanceMM - prevLeftDistanceMM;
        double deltaMecanumDistance = backDistanceMM - prevBackDistanceMM;

        double imuDeltaTheta = currentImuHeading - prevImuHeading;
        //wrapping to normalize theta -pi to pi
        imuDeltaTheta = MathFunctions.angleWrapRad(imuDeltaTheta);
        double arcTanDeltaTheta = Math.atan2(deltaLeftDistance - deltaRightDistance, TRACK_WIDTH_MM);
        double sparkFunDeltaTheta = currentSparkImuHeading - prevSparkImuHeading;
        sparkFunDeltaTheta = MathFunctions.angleWrapRad(sparkFunDeltaTheta);
//        Position newPosition = kalmanFilter.update(new Position(0, 0, arcTanDeltaTheta));
//        newPosition = kalmanFilter.update(new Position(0, 0, imuDeltaTheta));
//        newPosition = kalmanFilter.update(new Position(0, 0, encoderDeltaTheta));

        ////Log.d("purepursaction_debug_odo_wheel_deltaTheta",
        //        String.format("encoder = %.4f, imu = %.4f, arcTan = %.4f", encoderDeltaTheta, imuDeltaTheta,
        //                arcTanDeltaTheta));
        //double blendedDeltaTheta = (newPosition.getTheta());

//        double blendedDeltaTheta =
//                (wheelHeadingWeight * encoderDeltaTheta) + (imuHeadingWeight * imuDeltaTheta) + (wheelHeadingWeight * arcTanDeltaTheta);

        //        currentImuHeading + ", " + imuDeltaTheta + ", " + arcTanDeltaTheta + ", " + encoderDeltaTheta +  ",
        //        " + deltaTimeMS + ", " + (imuDeltaTheta / deltaTimeMS));


        double blendedDeltaTheta = sensorFusion.getFilteredAngleDelta(imuDeltaTheta, arcTanDeltaTheta, deltaTimeMS,
                currentImuHeading, currentSparkImuHeading, sparkFunDeltaTheta);

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

        //Log.d("odometry", "linearDelta " + relativeDelta);
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
            //Log.d("odometry new arc delta", "arcDelta4" + arcDelta);
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

        double newTheta = relativeDelta.getTheta();

        return new Velocity(newX, newY, newTheta);
    }

    private Position calculateGlobal(Velocity relativeDelta, Position previousGlobalPosition) {
        Velocity globalDelta = rotate(relativeDelta, previousGlobalPosition);
        //Log.d("global delta", globalDelta.toString());
        Position position = previousGlobalPosition.add(globalDelta);
        return position;
    }

    private void updateWheelPos(double rightDistanceMM, double leftDistanceMM, double backDistanceMM, double timeElapsedSeconds) {
        Velocity wheelRelDelta = calculateRelativeDeltaWheel(rightDistanceMM, leftDistanceMM,
                backDistanceMM, timeElapsedSeconds * 1000);
        wheelRelDelta = linearToArcDelta(wheelRelDelta);
        wheelPositionHistory.setCurrentPosition(calculateGlobal(wheelRelDelta,
                wheelPositionHistory.getCurrentPosition()));
        wheelPositionHistory.setCurrentVelocity(wheelRelDelta.divide(timeElapsedSeconds));
        odometryPositionHistoryHashMap.put(Odometry.WHEEL, wheelPositionHistory);
    }

    private void updateWheelIMUPos(double rightDistanceMM, double leftDistanceMM, double backDistanceMM,
                                   double timeElapsedSeconds) {
        Velocity wheelIMURelDelta = calculateRelativeDeltaWheelIMU(rightDistanceMM, leftDistanceMM, backDistanceMM,
                timeElapsedSeconds * 1000);
        wheelIMURelDelta = linearToArcDelta(wheelIMURelDelta);
        wheelIMUPositionHistory.setCurrentPosition(calculateGlobal(wheelIMURelDelta,
                wheelIMUPositionHistory.getCurrentPosition()));
        wheelIMUPositionHistory.setCurrentVelocity(wheelIMURelDelta.divide(timeElapsedSeconds));
        odometryPositionHistoryHashMap.put(Odometry.WHEEL_IMU, wheelIMUPositionHistory);

    }

    private void updateWheelIMUFusePos(double rightDistanceMM, double leftDistanceMM, double backDistanceMM,
                                       double timeElapsedSeconds) {
        Velocity wheelIMUFuseRelDelta = calculateRelativeDeltaWheelIMUFuse(rightDistanceMM, leftDistanceMM,
                backDistanceMM, timeElapsedSeconds * 1000);
        wheelIMUFuseRelDelta = linearToArcDelta(wheelIMUFuseRelDelta);
        wheelIMUFusePositionHistory.setCurrentPosition(calculateGlobal(wheelIMUFuseRelDelta,
                wheelIMUFusePositionHistory.getCurrentPosition()));
        wheelIMUFusePositionHistory.setCurrentVelocity(wheelIMUFuseRelDelta.divide(timeElapsedSeconds));
        odometryPositionHistoryHashMap.put(Odometry.WHEEL_IMU_FUSE, wheelIMUFusePositionHistory);

    }

    private void updateWheelSparkPos(double rightDistanceMM, double leftDistanceMM, double backDistanceMM,
                                     double timeElapsedSeconds) {
        Velocity wheelSparkRelDelta = calculateRelativeDeltaWheelSpark(rightDistanceMM, leftDistanceMM,
                backDistanceMM, timeElapsedSeconds * 1000);
        wheelSparkRelDelta = linearToArcDelta(wheelSparkRelDelta);
        wheelSparkPositionHistory.setCurrentPosition(calculateGlobal(wheelSparkRelDelta,
                wheelSparkPositionHistory.getCurrentPosition()));
        wheelSparkPositionHistory.setCurrentVelocity(wheelSparkRelDelta.divide(timeElapsedSeconds));
        odometryPositionHistoryHashMap.put(Odometry.WHEEL_SPARK, wheelSparkPositionHistory);

    }

    private void updateWheelSparkFusePos(double rightDistanceMM, double leftDistanceMM, double backDistanceMM,
                                         double timeElapsedSeconds) {
        Velocity wheelSparkFuseRelDelta = calculateRelativeDeltaWheelSparkFuse(rightDistanceMM, leftDistanceMM,
                backDistanceMM, timeElapsedSeconds * 1000);
        wheelSparkFuseRelDelta = linearToArcDelta(wheelSparkFuseRelDelta);
        wheelSparkFusePositionHistory.setCurrentPosition(calculateGlobal(wheelSparkFuseRelDelta,
                wheelSparkFusePositionHistory.getCurrentPosition()));
        wheelSparkFusePositionHistory.setCurrentVelocity(wheelSparkFuseRelDelta.divide(timeElapsedSeconds));
        odometryPositionHistoryHashMap.put(Odometry.WHEEl_SPARK_FUSE, wheelSparkFusePositionHistory);

    }

    private void updateWheelIMUSparkFuse(double rightDistanceMM, double leftDistanceMM, double backDistanceMM,
                                         double timeElapsedSeconds) {

        Velocity wheelIMUSparkFuseRelDelta = calculateRelativeDeltaWheelIMUSparkFuse(rightDistanceMM, leftDistanceMM,
                backDistanceMM, timeElapsedSeconds * 1000);
        wheelIMUSparkFuseRelDelta = linearToArcDelta(wheelIMUSparkFuseRelDelta);
        wheelIMUSparkFusePositionHistory.setCurrentPosition(calculateGlobal(wheelIMUSparkFuseRelDelta,
                wheelIMUSparkFusePositionHistory.getCurrentPosition()));
        wheelIMUSparkFusePositionHistory.setCurrentVelocity(wheelIMUSparkFuseRelDelta.divide(timeElapsedSeconds));
        odometryPositionHistoryHashMap.put(Odometry.WHEEL_IMU_SPARK_FUSE, wheelIMUSparkFusePositionHistory);

    }
    public HashMap<Odometry, PositionHistory> updatePositionAll() {
        double rightDistanceMM = countRight();
        double leftDistanceMM = countLeft();
        double backDistanceMM = countBack();
        currentImuHeading = getIMUHeading();
        currentSparkImuHeading = getSparkIMUHeading();

        //Log.d("updatepos", rightDistanceMM + " " + leftDistanceMM + " " + backDistanceMM);

        long currentTime = SystemClock.elapsedRealtime();
        double timeElapsedSeconds = (currentTime - prevTime) / 1000.0;

        updateWheelPos(rightDistanceMM, leftDistanceMM, backDistanceMM, timeElapsedSeconds);
        updateWheelIMUPos(rightDistanceMM, leftDistanceMM, backDistanceMM, timeElapsedSeconds);
        updateWheelIMUFusePos(rightDistanceMM, leftDistanceMM, backDistanceMM, timeElapsedSeconds);
        updateWheelSparkPos(rightDistanceMM, leftDistanceMM, backDistanceMM, timeElapsedSeconds);
        updateWheelSparkFusePos(rightDistanceMM, leftDistanceMM, backDistanceMM, timeElapsedSeconds);
        updateWheelIMUSparkFuse(rightDistanceMM, leftDistanceMM, backDistanceMM, timeElapsedSeconds);



        //Log.d("currentpos", "current pos " + currentPosition.toString());
        prevTime = currentTime;

        prevRightDistanceMM = rightDistanceMM;
        prevLeftDistanceMM = leftDistanceMM;
        prevBackDistanceMM = backDistanceMM;

        prevImuHeading = currentImuHeading;
        prevSparkImuHeading = currentSparkImuHeading;
        SharedData.setOdometryPosition(odometryPositionHistoryHashMap.get(Odometry.WHEEL_IMU_SPARK_FUSE).getCurrentPosition());
        return odometryPositionHistoryHashMap;
    }

    public Position updatePosition() {
        HashMap<Odometry, PositionHistory> positionHistoryHashMap = updatePositionAll();
        PositionHistory positionHistory = positionHistoryHashMap.get(Odometry.WHEEL_IMU_SPARK_FUSE);
        if (positionHistory == null) {
            throw new RuntimeException("WHEEL_IMU_SPARK_FUSE Position History Null");
        }
        return positionHistory.getCurrentPosition();
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

    public double getSparkIMUHeading() {
        return -sparkFunOTOS.getPosition().h;
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


    public IMUModule getImuModule() {return imuModule;}

    public double getCurrentImuHeading() {
        return this.currentImuHeading;
    }

    public double countX() {
        return (countLeft() + countRight()) / 2;
    }
}