package org.nknsd.robotics.team.autonomous;

import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.nknsd.robotics.team.components.ExtensionHandler;
import org.nknsd.robotics.team.components.FlowSensorHandler;
import org.nknsd.robotics.team.components.IMUComponent;
import org.nknsd.robotics.team.components.IntakeSpinnerHandler;
import org.nknsd.robotics.team.components.RotationHandler;
import org.nknsd.robotics.team.components.WheelHandler;
import org.nknsd.robotics.team.helperClasses.PIDModel;

public class AutoSkeleton {
    private final double maxSpeed;                  // Maximum speed the robot can move at
    private final double movementMargin;            // Margin determines how close to the target we have to be before we are there
    private final double turnMargin;
    private WheelHandler wheelHandler;              // Class which handles wheel motions
    private FlowSensorHandler flowSensorHandler;    // Class which gives us our position
    public double[] targetPositions = new double[2];// Array which holds our target position as x, y
    static final double TILE_LENGTH = 20.8;
    private IMUComponent imuComponent;
    private RotationHandler rotationHandler;
    private ExtensionHandler extensionHandler;
    private double targetRotation = 0;
    private IntakeSpinnerHandler intakeSpinnerHandler;

    private PIDModel movementPID = new PIDModel(0, 0, 0);



    public AutoSkeleton(double maxSpeed, double movementMargin, double turnMargin) {
        this.maxSpeed = maxSpeed;
        this.movementMargin = movementMargin;
        this.turnMargin = turnMargin;
    }

    public void link(WheelHandler wheelHandler, RotationHandler rotationHandler, ExtensionHandler extensionHandler, IntakeSpinnerHandler intakeSpinnerHandler, FlowSensorHandler flowSensorHandler, IMUComponent imuComponent) {
        this.wheelHandler = wheelHandler;
        this.flowSensorHandler = flowSensorHandler;
        this.imuComponent = imuComponent;
        this.rotationHandler = rotationHandler;
        this.extensionHandler = extensionHandler;
        this.intakeSpinnerHandler = intakeSpinnerHandler;
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

    public void setTargetArmExtension(ExtensionHandler.ExtensionPositions extensionPosition) {
        extensionHandler.gotoPosition(extensionPosition);
    }

    public boolean isExtensionDone() {
        return extensionHandler.isExtensionDone();
    }

    public boolean runToPosition(Telemetry telemetry, ElapsedTime runtime) {
        // Get position
        FlowSensorHandler.PoseData pos = flowSensorHandler.getOdometryData().pos;
        double x = pos.x;
        double y = pos.y;
        double yaw = imuComponent.getYaw();

        telemetry.addData("Cur X", x);
        telemetry.addData("Cur Y", y);


        // Calculating distance
        double xDist = (targetPositions[0] * TILE_LENGTH - x);
        double yDist = (targetPositions[1] * TILE_LENGTH - y);
        double dist = Math.sqrt((xDist * xDist) + (yDist * yDist));
        double angleDiff = Math.abs(targetRotation - yaw);


        // Check if we're at our target
        if (dist <= movementMargin && angleDiff <= turnMargin) {
            wheelHandler.absoluteVectorToMotion(0, 0, 0, 0, telemetry);
            return true;
        }


        // Calculate force to use
        x = movementPID.calculate(x, targetPositions[0], runtime);
        y = movementPID.calculate(y, targetPositions[1], runtime);
        double turning = targetRotation - yaw;

        telemetry.addData("Speed X", x);
        telemetry.addData("Speed Y", y);
        telemetry.addData("Speed Turning", turning);


        // Run motors
        wheelHandler.absoluteVectorToMotion(x, y, turning, yaw, telemetry);

        return false;
    }

    public void setServoPower(IntakeSpinnerHandler.HandStates handState) {
        intakeSpinnerHandler.setServoPower(handState);
    }


}
