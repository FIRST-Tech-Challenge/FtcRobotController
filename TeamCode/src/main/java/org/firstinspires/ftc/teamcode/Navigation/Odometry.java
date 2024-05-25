package org.firstinspires.ftc.teamcode.Navigation;

import android.util.Log;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class Odometry {
    final double TRACK_WIDTH = 304.8;
    private final double BACK_DISTANCE_TO_MID = 69.85;

    Telemetry jose;
    DcMotor rightEncoder;
    DcMotor leftEncoder;
    DcMotor backEncoder;
    Position currentPosition;
    double prevRightTicks = 0;
    double prevLeftTicks = 0;
    double prevBackTicks = 0;

    public Odometry(DcMotor backEncoder, DcMotor rightEncoder, DcMotor leftEncoder, LinearOpMode opMode,
                    Telemetry telemetry, double xCoordinate, double yCoordinate, double theta) {
        this.currentPosition = new Position(xCoordinate, yCoordinate, theta);
        jose = telemetry;
        this.rightEncoder = rightEncoder;
        this.leftEncoder = leftEncoder;
        this.backEncoder = backEncoder;

    }

    //to be deprecated
/*    public void odometryAluminumCobblersProbably() {

        double rightDistance;
        double leftDistance;
        double distanceMiddle;
        double deltaX;
        double deltaY;
        double deltaAngle;
        double initialAngle = 0;
        double angleCoordinate;
        double currentAngle = 0;
        double deltaAngleDegrees = 0;

        rightDistance = rightEncoder.getCurrentPosition() / TICKS_TO_MM;
        leftDistance = leftEncoder.getCurrentPosition() / TICKS_TO_MM;
        distanceMiddle = (leftDistance + rightDistance) / 2;

        theta = (rightDistance - leftDistance) / TRACK_WIDTH;
        deltaAngle = theta / 2;
        deltaAngleDegrees = Math.toDegrees(deltaAngle);

        deltaX = distanceMiddle * Math.cos(initialAngle + deltaAngle);
        deltaY = distanceMiddle * Math.sin(initialAngle + deltaAngle);

        xCoordinate = prevX + deltaX;
        yCoordinate = prevY + deltaY;
        angleCoordinate = currentAngle + deltaAngle;


        jose.addData("theta", theta);
        jose.addData("delta y", deltaY);
        jose.addData("delta x", deltaX);

        jose.addData("x coordinate", xCoordinate);
        jose.addData("y coordinate", yCoordinate);
        jose.addData("angle", angleCoordinate);
    }*/

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

    private Position calculateRelativeDelta(double rightTicks, double leftTicks, double backTicks) {
        double deltaRightDistance = ticksToMM(rightTicks - prevRightTicks);
        double deltaLeftDistance = ticksToMM(leftTicks - prevLeftTicks);

        double deltaDistanceMiddle = (deltaLeftDistance + deltaRightDistance) / 2;
        double deltaTheta = (deltaRightDistance - deltaLeftDistance) / TRACK_WIDTH;
        double deltaBackDistance = ticksToMM((backTicks - prevBackTicks) - BACK_DISTANCE_TO_MID * deltaTheta);
        return new Position(deltaDistanceMiddle, deltaBackDistance, deltaTheta);
    }

    private Position linearToArcDelta(Position relativeDelta) {
        if (relativeDelta.theta == 0) {
            return relativeDelta;
        }

        Log.d("odometry", "linearDelta " + relativeDelta.toString());
        double forwardRadius = relativeDelta.x / relativeDelta.theta;
        double strafeRadius = relativeDelta.y / relativeDelta.theta;

        double relDeltaX =
                forwardRadius * Math.sin(relativeDelta.theta) + -strafeRadius * (1 - Math.cos(relativeDelta.theta));

        double relDeltaY =
                +strafeRadius * Math.sin(relativeDelta.theta) + forwardRadius * (1 - Math.cos(relativeDelta.theta));
      /*  double relDeltaX =
                forwardRadius * Math.sin(relativeDelta.theta) + relativeDelta.y * Math.sin(currentPosition.theta);

        double relDeltaY =
                forwardRadius * (1 - Math.cos(relativeDelta.theta)) + relativeDelta.y * Math.cos(currentPosition.theta);
*/
        jose.addData("strafeRadius", strafeRadius);
        jose.addData("cos strafe theta hamburger", (1 - Math.cos(relativeDelta.theta)));
        jose.addData("sin y axis hamburger", Math.sin(relativeDelta.theta));
        Position arcDelta = new Position(relDeltaX, relDeltaY, relativeDelta.theta);
        Log.d("odometry", "arcDelta " + arcDelta.toString());
        return arcDelta;
    }

    private Position getGlobalPosition(Position relativeDelta, Position previousGlobalPosition) {
        double newX =
                previousGlobalPosition.x + relativeDelta.x * Math.cos(previousGlobalPosition.theta) - relativeDelta.y * Math.sin(previousGlobalPosition.theta);
        double newY =
                previousGlobalPosition.y + relativeDelta.y * Math.cos(previousGlobalPosition.theta) + relativeDelta.x * Math.sin(previousGlobalPosition.theta);
        double newHeading = previousGlobalPosition.theta + relativeDelta.theta;
        return new Position(newX, newY, newHeading);
    }

    public void updatePosition() {
        double rightTicks = countRight();
        double leftTicks = countLeft();
        double backTicks = countBack();

        Position relativeDelta = calculateRelativeDelta(rightTicks, leftTicks, backTicks);

        relativeDelta = linearToArcDelta(relativeDelta);

        currentPosition = getGlobalPosition(relativeDelta, currentPosition);

        prevRightTicks = rightTicks;
        prevLeftTicks = leftTicks;
        prevBackTicks = backTicks;


        jose.addData("delta ", relativeDelta.toString());

        jose.addData("global", currentPosition.toString());
        jose.update();

    }


}
