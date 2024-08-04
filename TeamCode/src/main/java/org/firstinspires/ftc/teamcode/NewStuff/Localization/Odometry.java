package org.firstinspires.ftc.teamcode.NewStuff.Localization;

import android.util.Log;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.NewStuff.DriveTrain;

public class Odometry {
    final private double TRACK_WIDTH = 304.8;
    private final double BACK_DISTANCE_TO_MID = 69.85;

    private Telemetry jose;
    private DcMotor rightEncoder;
    private DcMotor leftEncoder;
    private DcMotor backEncoder;
    private Position currentPosition;
    private double prevRightTicks = 0;
    private double prevLeftTicks = 0;
    private double prevBackTicks = 0;

    public Odometry(DriveTrain driveTrain, LinearOpMode opMode,
                    Telemetry telemetry, double xCoordinate, double yCoordinate, double theta) {
        this.currentPosition = new Position(xCoordinate, yCoordinate, theta);
        jose = telemetry;
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

    private double countRight() {
        return -rightEncoder.getCurrentPosition();
    }
    private double countLeft() {
        return -leftEncoder.getCurrentPosition();
    }
    private double countBack() {
        return backEncoder.getCurrentPosition();
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

        Velocity arcDelta = new Velocity(relDeltaX, relDeltaY, relativeDelta.getTheta());
        Log.d("odometry", "arcDelta " + arcDelta.toString());
        return arcDelta;
    }
    //converts to global
    private Velocity rotate(Velocity relativeDelta, Position previousGlobalPosition) {
        double newX =
                relativeDelta.getX() * Math.cos(previousGlobalPosition.getTheta()) - relativeDelta.getY() * Math.sin(previousGlobalPosition.getTheta());
        double newY =
                relativeDelta.getY() * Math.cos(previousGlobalPosition.getTheta()) + relativeDelta.getX() * Math.sin(previousGlobalPosition.getTheta());
        return new Velocity(newX,
                            newY,
                            relativeDelta.getTheta());
    }

    private Position updateGlobal(Velocity relativeDelta, Position previousGlobalPosition) {
        Velocity globalDelta = rotate(relativeDelta, previousGlobalPosition);

        return previousGlobalPosition.add(globalDelta);
    }

    public Position updatePosition() {
        double rightTicks = countRight();
        double leftTicks = countLeft();
        double backTicks = countBack();

        Velocity relativeDelta = calculateRelativeDelta(rightTicks, leftTicks, backTicks);

        relativeDelta = linearToArcDelta(relativeDelta);

        currentPosition = updateGlobal(relativeDelta, currentPosition);

        prevRightTicks = rightTicks;
        prevLeftTicks = leftTicks;
        prevBackTicks = backTicks;


        jose.addData("delta ", relativeDelta.toString());

        jose.addData("global", currentPosition.toString());
        jose.update();
        return currentPosition;
    }

    public Position getCurrentPosition() {
        return currentPosition;
    }
}
