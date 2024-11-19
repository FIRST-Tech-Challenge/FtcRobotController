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
    private PIDModel movementPIDx;
    private PIDModel movementPIDy;
    private boolean xDirPos = true;
    private boolean yDirPos = true;



    public AutoSkeleton(double maxSpeed, double movementMargin, double turnMargin) {
        this.maxSpeed = maxSpeed;
        this.movementMargin = movementMargin;
        this.turnMargin = turnMargin;

        // Creating PID
//        double kP = ( maxSpeed / (TILE_LENGTH * 2) ) * 10;
//        double kI = maxSpeed / (TILE_LENGTH * 2 * 4000); //I is REALLY FREAKING SMALL
//        double kD = 5;
        double kP = 0.5;
        double kI = maxSpeed / (TILE_LENGTH * TILE_LENGTH * 3500); //I is REALLY FREAKING SMALL
        double kD = 5;

        movementPIDx = new PIDModel(kP, kI, kD);
        movementPIDy = new PIDModel(kP, kI, kD);
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
        movementPIDx.resetError(); movementPIDy.resetError();

        FlowSensorHandler.PoseData pos = flowSensorHandler.getOdometryData().pos;
        xDirPos = pos.x < x;
        yDirPos = pos.y < y;
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
        double xTarg = targetPositions[0] * TILE_LENGTH;
        double yTarg = targetPositions[1] * TILE_LENGTH;

        double xDist = (xTarg - x);
        double yDist = (yTarg - y);
        double dist = Math.sqrt((xDist * xDist) + (yDist * yDist));
        double angleDiff = Math.abs(targetRotation - yaw);

        telemetry.addData("Targ X", xTarg);
        telemetry.addData("Targ Y", yTarg);


        // Check if we're at our target
        if (dist <= movementMargin && angleDiff <= turnMargin) {
            wheelHandler.absoluteVectorToMotion(0, 0, 0, 0, telemetry);
            return true;
        }


        // Calculate force to use
        double xSpeed = 0;
        double ySpeed = 0;
        if (Math.abs(xDist) > movementMargin) {
            if (xDist > 0 ^ xDirPos) {
                movementPIDx.resetError();
                //xDirPos = !xDirPos;
            }

            telemetry.addData("PID DATA", "x");
            xSpeed = movementPIDx.calculateWithTelemetry(x, xTarg, runtime, telemetry);
        }
        if (Math.abs(yDist) > movementMargin) {
            if (yDist > 0 ^ yDirPos) {
                movementPIDy.resetError();
                //yDirPos = !yDirPos;
            }

            telemetry.addData("PID DATA", "y");
            ySpeed = movementPIDy.calculateWithTelemetry(y, yTarg, runtime, telemetry);
        }


        double turning = targetRotation - yaw;
        turning /= 90;

        xSpeed = Math.max(Math.min(xSpeed, maxSpeed), -maxSpeed);
        ySpeed = Math.max(Math.min(ySpeed, maxSpeed), -maxSpeed);

        telemetry.addData("Speed X", xSpeed);
        telemetry.addData("Speed Y", ySpeed);
        telemetry.addData("Speed Turning", turning);


        // Run motors
        wheelHandler.absoluteVectorToMotion(xSpeed, ySpeed, turning, yaw, telemetry);

        return false;
    }

    public void runMotorsDirectly(double y, double x, double turning) {
        wheelHandler.relativeVectorToMotion(y, x, turning);
    }

    public void freeze() {
        wheelHandler.relativeVectorToMotion(0, 0, 0);
    }

    public void setServoPower(IntakeSpinnerHandler.HandStates handState) {
        intakeSpinnerHandler.setServoPower(handState);
    }


}
