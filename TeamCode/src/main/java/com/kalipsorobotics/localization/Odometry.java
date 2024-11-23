package com.kalipsorobotics.localization;

import android.os.SystemClock;
import android.util.Log;

import com.qualcomm.hardware.sparkfun.SparkFunOTOS;

import com.kalipsorobotics.math.MathFunctions;
import com.kalipsorobotics.math.Position;
import com.kalipsorobotics.math.Velocity;

import com.kalipsorobotics.utilities.OpModeUtilities;
import com.kalipsorobotics.modules.DriveTrain;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

public class Odometry {
    OpModeUtilities opModeUtilities;
    final static private double TRACK_WIDTH = 273.05;
    static private final double BACK_DISTANCE_TO_MID = 69.85;
//    private final DcMotor rightEncoder;
//    private final DcMotor leftEncoder;
//    private final DcMotor backEncoder;
    private final SparkFunOTOS otos;
    volatile private Position currentPosition;
    volatile private Velocity currentVelocity;
//    private volatile double prevRightTicks = 0;
//    private volatile double prevLeftTicks = 0;
//    volatile private double prevBackTicks = 0;
    private volatile double prevX;
    private volatile double prevY;
    private volatile double prevTheta;
    private volatile long prevTime;

    private final double ODO_TO_INCH = 24/1.94;


    public Odometry(DriveTrain driveTrain, OpModeUtilities opModeUtilities, double xCoordinate, double yCoordinate, double theta) {
        this.opModeUtilities = opModeUtilities;
        this.currentPosition = new Position(xCoordinate, yCoordinate, theta);
        this.otos = driveTrain.getOtos();

        sparkResetData(true, Math.toRadians(0));
//        this.rightEncoder = driveTrain.getRightEncoder();
//        this.leftEncoder = driveTrain.getLeftEncoder();
//        this.backEncoder = driveTrain.getBackEncoder();
    }

    private double ticksToMM(double ticks) {
        final double DEAD_WHEEL_RADIUS = 24;
        final double TICKS_PER_REV = 2000;
        final double MM_TO_TICKS = 2.0 * Math.PI * DEAD_WHEEL_RADIUS / TICKS_PER_REV;
        final double TICKS_TO_MM = 1.0 / MM_TO_TICKS;
//      final double TICKS_TO_MM = 13.2625995;
        return ticks / TICKS_TO_MM;
    }

//    public double countRight() {
//        return rightEncoder.getCurrentPosition();
//    }
//    public double countLeft() {
//        return leftEncoder.getCurrentPosition();
//    }
//    public double countBack() {
//        return -backEncoder.getCurrentPosition();
//    }

    public void sparkResetData(Boolean reCalibrate, double heading) {
        otos.resetTracking();
        if (reCalibrate) { otos.calibrateImu(); }
        otos.setOffset(new SparkFunOTOS.Pose2D(-0.75, 0.5, heading));

        otos.setAngularUnit(AngleUnit.RADIANS);

        Log.d("sparkfun", "reset data");
        //7, 8
        //7, 8 1/2
    }


    //1.94
    public double countY() {
        Log.d("odometry", "y is " + -otos.getPosition().x * ODO_TO_INCH);
        return -otos.getPosition().x * ODO_TO_INCH;
    }

    public double countX() {
        Log.d("odometry", "x is " + -otos.getPosition().y * ODO_TO_INCH);
        return -otos.getPosition().y * ODO_TO_INCH;
    }

    public double countTheta() {
        double theta = -otos.getPosition().h;
        theta = Math.round(theta * 100.0) / 100.0;

        Log.d("odometry", "h is " + theta);

        return theta;
    }

    private Velocity calculateRelativeDelta(double x, double y, double theta) {
//        double deltaRightDistance = ticksToMM(rightTicks - prevRightTicks);
//        double deltaLeftDistance = ticksToMM(leftTicks - prevLeftTicks);
        double deltaX = x - prevX;
        double deltaY = y - prevY;
        double deltaTheta = theta - prevTheta;

//        double deltaX = (deltaLeftDistance + deltaRightDistance) / 2;
//        double deltaTheta = -(deltaRightDistance - deltaLeftDistance) / TRACK_WIDTH;
//        double deltaY = -(deltaMecanumDistance - BACK_DISTANCE_TO_MID * deltaTheta);

        return new Velocity(
                deltaX,
                deltaY,
                deltaTheta
        );
    }

    private Velocity linearToArcDelta(Velocity relativeDelta) {
        if (relativeDelta.getTheta() == 0) {
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
                MathFunctions.angleWrapRad(relativeDelta.getTheta());

        Velocity arcDelta = new Velocity(relDeltaX, relDeltaY, relDeltaTheta);
        Log.d("odometry", "arcDelta " + arcDelta);
        return arcDelta;
    }

    //converts to global
    private Velocity rotate(Velocity relativeDelta, Position previousGlobalPosition) {
        double newX =
                relativeDelta.getX() * Math.cos(previousGlobalPosition.getTheta()) - relativeDelta.getY() * Math.sin(previousGlobalPosition.getTheta());
        double newY =
                relativeDelta.getY() * Math.cos(previousGlobalPosition.getTheta()) + relativeDelta.getX() * Math.sin(previousGlobalPosition.getTheta());
        double newTheta =
                MathFunctions.angleWrapRad(relativeDelta.getTheta());
        return new Velocity(newX, newY, newTheta);
    }

    private Position updateGlobal(Velocity relativeDelta, Position previousGlobalPosition) {
        Velocity globalDelta = rotate(relativeDelta, previousGlobalPosition);

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

    public Position updatePosition() {
//        double rightTicks = countRight();
//        double leftTicks = countLeft();
//        double backTicks = countBack();

        double x = countX();
        double y = countY();
        double theta = countTheta();

        //Log.d("updatepos", rightTicks + " " + leftTicks + " " + backTicks);

        long currentTime = SystemClock.elapsedRealtime();
        double timeElapsed = (currentTime - prevTime) / 1000.;

        Velocity relativeDelta = calculateRelativeDelta(x, y, theta);
        relativeDelta = linearToArcDelta(relativeDelta);

        currentVelocity = relativeDelta.divide(timeElapsed);
        prevTime = currentTime;

        currentPosition = updateGlobal(relativeDelta, currentPosition);
        Log.d("currentpos", "current pos " + currentPosition.getTheta());

        prevX = x;
        prevY = y;
        prevTheta = theta;

        //opModeUtilities.getTelemetry().addData("global", currentPosition.toString());
        return currentPosition;
    }

    public Velocity getCurrentVelocity() {
        return currentVelocity;
    }

    public Position getCurrentPosition() {
        return currentPosition;
    }
}
