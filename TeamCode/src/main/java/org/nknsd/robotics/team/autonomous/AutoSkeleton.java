package org.nknsd.robotics.team.autonomous;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.nknsd.robotics.team.components.FlowSensorHandler;
import org.nknsd.robotics.team.components.IMUComponent;
import org.nknsd.robotics.team.components.IntakeServoHandler;
import org.nknsd.robotics.team.components.RotationHandler;
import org.nknsd.robotics.team.components.WheelHandler;

public class AutoSkeleton {
    private final double maxSpeed;                  // Maximum speed the robot can move at
    private final double movementMargin;            // Margin determines how close to the target we have to be before we are there
    private final double turnMargin;
    private WheelHandler wheelHandler;              // Class which handles wheel motions
    private FlowSensorHandler flowSensorHandler;    // Class which gives us our position
    public double[] targetPositions = new double[2];// Array which holds our target position as x, y
    static final double TILE_LENGTH = 24;
    private IMUComponent imuComponent;
    private RotationHandler rotationHandler;
    private double targetRotation = 0;
    private IntakeServoHandler intakeServoHandler;



    public AutoSkeleton(double maxSpeed, double movementMargin, double turnMargin) {
        this.maxSpeed = maxSpeed;
        this.movementMargin = movementMargin;
        this.turnMargin = turnMargin;
    }

    public void link(WheelHandler wheelHandler, RotationHandler rotationHandler, IntakeServoHandler intakeServoHandler, FlowSensorHandler flowSensorHandler, IMUComponent imuComponent) {
        this.wheelHandler = wheelHandler;
        this.flowSensorHandler = flowSensorHandler;
        this.imuComponent = imuComponent;
        this.rotationHandler = rotationHandler;
        this.intakeServoHandler = intakeServoHandler;
    }

    public void setTargetPosition(double x, double y) {
        targetPositions[0] = x; targetPositions[1] = y;
    }

    public void setTargetRotation(double turning) {
        targetRotation = turning;
    }

    public void setTargetArmRotation(RotationHandler.RotationPositions rotationPosition) {
        rotationHandler.setTargetRotationPosition(rotationPosition);
    }

    public boolean isArmRotationAtTarget() {
        return rotationHandler.isAtTargetPosition();
    }

    public boolean runToPosition(Telemetry telemetry) {
        // Get position
        FlowSensorHandler.PoseData pos = flowSensorHandler.getOdometryData().pos;
        double x = pos.x;
        double y = pos.y;
        double yaw = imuComponent.getYaw();
        //double yaw = pos.heading;

        telemetry.addData("Cur X", x);
        telemetry.addData("Cur Y", y);

        // Find difference in position
        x = (targetPositions[0] * TILE_LENGTH - x) * 0.5;
        y = (targetPositions[1] * TILE_LENGTH - y) * 0.5;

        telemetry.addData("Targ X", targetPositions[0] * TILE_LENGTH);
        telemetry.addData("Targ Y", targetPositions[1] * TILE_LENGTH);

        // Check if we're close enough
        double dist = Math.sqrt((x * x) + (y * y));

        // Check if we're at the right heading
        double angleDiff = Math.abs(targetRotation - yaw);

        telemetry.addData("Dist", dist);

        if (dist < movementMargin && angleDiff < turnMargin) {
            wheelHandler.absoluteVectorToMotion(0, 0, 0, 0, telemetry);
            return true;
        } else if (Math.abs(x) < movementMargin) {
            x = 0;
        } else if (Math.abs(y) < movementMargin) {
            y = 0;
        }

        //// Now we convert the x and y to a range of values that the motor can accept
        //double tempNumber = Math.max(Math.abs(x), Math.abs(y));

        //double ratio = Math.max(-maxSpeed, Math.min(maxSpeed, tempNumber)) / tempNumber;
        //x = x * ratio;
        //y = y * ratio;

        //Clamp x & y
        x = Math.min(maxSpeed, Math.max(-maxSpeed, x));
        y = Math.min(maxSpeed, Math.max(-maxSpeed, y));

        telemetry.addData("Speed X", x);
        telemetry.addData("Speed Y", y);

        //Turning
        double turning = targetRotation - yaw;
        turning /= 90;
        turning = Math.min(maxSpeed, Math.max(-maxSpeed, turning));

        wheelHandler.absoluteVectorToMotion(x, y, turning, yaw, telemetry);

        return false;
    }

    public void setServoPower(double power) {
        intakeServoHandler.setServoPower(power);
    }
}
