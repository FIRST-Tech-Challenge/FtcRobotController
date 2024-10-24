package org.nknsd.robotics.team.autonomous;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.nknsd.robotics.team.components.FlowSensorHandler;
import org.nknsd.robotics.team.components.WheelHandler;

public class AutoSkeleton {
    private final double maxSpeed;                  // Maximum speed the robot can move at
    private final double movementMargin;            // Margin determines how close to the target we have to be before we are there
    private WheelHandler wheelHandler;              // Class which handles wheel motions
    private FlowSensorHandler flowSensorHandler;    // Class which gives us our position
    private double[] targetPosition = new double[2];// Array which holds our target position as x, y



    public AutoSkeleton(double maxSpeed, double movementMargin) {
        this.maxSpeed = maxSpeed;
        this.movementMargin = movementMargin;
    }

    public void link(WheelHandler wheelHandler, FlowSensorHandler flowSensorHandler) {
        this.wheelHandler = wheelHandler;
        this.flowSensorHandler = flowSensorHandler;
    }

    public void setTargetPosition(double x, double y) {
        targetPosition[0] = x; targetPosition[1] = y;
    }

    public void setTargetRotation(double turning) {
        // To be implemented, once we have IMU set up
    }

    public boolean runToPosition(Telemetry telemetry) {
        // Get position
        double x = flowSensorHandler.getOdometryData().pos.x;
        double y = flowSensorHandler.getOdometryData().pos.y;

        telemetry.addData("Cur X", x);
        telemetry.addData("Cur Y", y);

        // Find difference in position
        x = targetPosition[0] - x;
        y = targetPosition[1] - y;

        telemetry.addData("Dist X", x);
        telemetry.addData("Dist Y", y);

        // Check if we're close enough
        double dist = Math.sqrt((x * x) + (y * y));

        telemetry.addData("Dist", dist);

        if (dist < movementMargin) {
            wheelHandler.absoluteVectorToMotion(0, 0, 0, 0);
            return true;
        }

        // Now we convert the x and y to a range of values that the motor can accept
        double ratio = Math.max(-maxSpeed, Math.min(maxSpeed, Math.max(x, y))) / Math.max(x, y);
        x = x * ratio;
        y = y * ratio;

        telemetry.addData("Speed X", x);
        telemetry.addData("Speed Y", y);

        //wheelHandler.absoluteVectorToMotion(x, y, 0, 0);

        return false;
    }
}
