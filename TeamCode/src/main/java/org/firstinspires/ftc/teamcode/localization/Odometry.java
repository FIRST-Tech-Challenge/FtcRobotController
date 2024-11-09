package org.firstinspires.ftc.teamcode.localization;

import android.os.SystemClock;
import android.util.Log;

import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.math.MathFunctions;
import org.firstinspires.ftc.teamcode.math.Position;
import org.firstinspires.ftc.teamcode.math.Velocity;

import org.firstinspires.ftc.teamcode.utilities.OpModeUtilities;
import org.firstinspires.ftc.teamcode.modules.DriveTrain;

public class Odometry {
    OpModeUtilities opModeUtilities;
    final static private double TRACK_WIDTH = 304.8;
    static private final double BACK_DISTANCE_TO_MID = 69.85;
    private final DcMotor rightEncoder;
    private final DcMotor leftEncoder;
    private final DcMotor backEncoder;
    volatile private Position currentPosition;
    volatile private Velocity currentVelocity;
    private volatile double prevRightTicks = 0;
    private volatile double prevLeftTicks = 0;
    volatile private double prevBackTicks = 0;
    private volatile long prevTime;

    public Odometry(DriveTrain driveTrain, OpModeUtilities opModeUtilities, double xCoordinate, double yCoordinate, double theta) {
        this.opModeUtilities = opModeUtilities;
        this.currentPosition = new Position(xCoordinate, yCoordinate, theta);
        this.rightEncoder = driveTrain.getRightEncoder();
        this.leftEncoder = driveTrain.getLeftEncoder();
        this.backEncoder = driveTrain.getBackEncoder();

    }

    private double ticksToMM(double ticks) {
        final double DEAD_WHEEL_RADIUS = 24;
        final double TICKS_PER_REV = 2000;
        final double MM_TO_TICKS = 2.0 * Math.PI * DEAD_WHEEL_RADIUS / TICKS_PER_REV;
        final double TICKS_TO_MM = 1.0 / MM_TO_TICKS;
//        final double TICKS_TO_MM = 13.2625995;
        return ticks / TICKS_TO_MM;
    }

    public double countRight() {
        return rightEncoder.getCurrentPosition();
    }
    public double countLeft() {
        return leftEncoder.getCurrentPosition();
    }
    public double countBack() {
        return -backEncoder.getCurrentPosition();
    }

    private Velocity calculateRelativeDelta(double rightTicks, double leftTicks, double backTicks) {
        double deltaRightDistance = ticksToMM(rightTicks - prevRightTicks);
        double deltaLeftDistance = ticksToMM(leftTicks - prevLeftTicks);
        double deltaMecanumDistance = ticksToMM(backTicks - prevBackTicks);

        double deltaX = (deltaLeftDistance + deltaRightDistance) / 2;
        double deltaTheta = (deltaRightDistance - deltaLeftDistance) / TRACK_WIDTH;
        double deltaY = -(deltaMecanumDistance - BACK_DISTANCE_TO_MID * deltaTheta);
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

        Log.d("odometry", "linearDelta " + relativeDelta.toString());
        double forwardRadius = relativeDelta.getX() / relativeDelta.getTheta();
        double strafeRadius = relativeDelta.getY() / relativeDelta.getTheta();

        double relDeltaX =
                forwardRadius * Math.sin(relativeDelta.getTheta()) + -strafeRadius * (1 - Math.cos(relativeDelta.getTheta()));

        double relDeltaY =
                +strafeRadius * Math.sin(relativeDelta.getTheta()) + forwardRadius * (1 - Math.cos(relativeDelta.getTheta()));

        double relDeltaTheta =
                MathFunctions.angleWrapRad(relativeDelta.getTheta());

        Velocity arcDelta = new Velocity(relDeltaX, relDeltaY, relDeltaTheta);
        Log.d("odometry", "arcDelta " + arcDelta.toString());
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
        double rightTicks = countRight();
        double leftTicks = countLeft();
        double backTicks = countBack();

        Log.d("updatepos", rightTicks + " " + leftTicks + " " + backTicks);

        long currentTime = SystemClock.elapsedRealtime();
        double timeElapsed = (currentTime - prevTime) / 1000.;

        Velocity relativeDelta = calculateRelativeDelta(rightTicks, leftTicks, backTicks);
        relativeDelta = linearToArcDelta(relativeDelta);

        currentVelocity = relativeDelta.divide(timeElapsed);
        prevTime = currentTime;

        currentPosition = updateGlobal(relativeDelta, currentPosition);
        Log.d("currentpos", "current pos " + currentPosition.getTheta());

        prevRightTicks = rightTicks;
        prevLeftTicks = leftTicks;
        prevBackTicks = backTicks;

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
