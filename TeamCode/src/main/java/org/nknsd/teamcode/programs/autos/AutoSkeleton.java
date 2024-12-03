package org.nknsd.teamcode.programs.autos;

import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.nknsd.teamcode.components.handlers.ExtensionHandler;
import org.nknsd.teamcode.components.sensors.FlowSensor;
import org.nknsd.teamcode.components.sensors.IMUSensor;
import org.nknsd.teamcode.components.handlers.IntakeSpinnerHandler;
import org.nknsd.teamcode.components.handlers.RotationHandler;
import org.nknsd.teamcode.components.handlers.WheelHandler;
import org.nknsd.teamcode.helperClasses.PIDModel;

public class AutoSkeleton {
    private double maxSpeed;                  // Maximum speed the robot can move at
    private final double movementMargin;            // Margin determines how close to the target we have to be before we are there
    private final double turnMargin;
    private WheelHandler wheelHandler;              // Class which handles wheel motions
    private FlowSensor flowSensor;    // Class which gives us our position
    public double[] targetPositions = new double[2];// Array which holds our target position as x, y
    static final double TILE_LENGTH = 20.8;
    private IMUSensor imuSensor;
    private RotationHandler rotationHandler;
    private ExtensionHandler extensionHandler;
    private double targetRotation = 0;
    private IntakeSpinnerHandler intakeSpinnerHandler;
    private PIDModel movementPIDx;
    private PIDModel movementPIDy;
    private PIDModel movementPIDturn;
    private boolean xDirPos = true;
    private boolean yDirPos = true;
    private boolean turnDirPos = true;



    public AutoSkeleton(double maxSpeed, double movementMargin, double turnMargin) {
        this.maxSpeed = maxSpeed;
        this.movementMargin = movementMargin;
        this.turnMargin = turnMargin;

        double kP = 0.5;
        double kI = maxSpeed / (TILE_LENGTH * TILE_LENGTH * 3500); //I is REALLY FREAKING SMALL
        double kD = 6;

        movementPIDx = new PIDModel(kP, kI, kD);
        movementPIDy = new PIDModel(kP, kI, kD);
        movementPIDturn = new PIDModel(maxSpeed / 16, maxSpeed / (16000), 0.5);
    }

    public void link(WheelHandler wheelHandler, RotationHandler rotationHandler, ExtensionHandler extensionHandler, IntakeSpinnerHandler intakeSpinnerHandler, FlowSensor flowSensor, IMUSensor imuSensor) {
        this.wheelHandler = wheelHandler;
        this.flowSensor = flowSensor;
        this.imuSensor = imuSensor;
        this.rotationHandler = rotationHandler;
        this.extensionHandler = extensionHandler;
        this.intakeSpinnerHandler = intakeSpinnerHandler;
    }

    public void setTargetPosition(double x, double y) {
        targetPositions[0] = x; targetPositions[1] = y;
        movementPIDx.resetError(); movementPIDy.resetError();

        FlowSensor.PoseData pos = flowSensor.getOdometryData().pos;
        xDirPos = pos.x < x;
        yDirPos = pos.y < y;
    }

    public void setTargetRotation(double turning) {
        targetRotation = turning;

        FlowSensor.PoseData pos = flowSensor.getOdometryData().pos;
        turnDirPos = pos.heading < targetRotation;
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
        FlowSensor.PoseData pos = flowSensor.getOdometryData().pos;
        double x = pos.x;
        double y = pos.y;
        double yaw = imuSensor.getYaw();

        telemetry.addData("Cur X", x);
        telemetry.addData("Cur Y", y);


        // Calculating distance
        double xTarg = targetPositions[0] * TILE_LENGTH;
        double yTarg = targetPositions[1] * TILE_LENGTH;

        double xDist = (xTarg - x);
        double yDist = (yTarg - y);
        double turnDist = targetRotation - yaw;
        double dist = Math.sqrt((xDist * xDist) + (yDist * yDist));

        telemetry.addData("Targ X", xTarg);
        telemetry.addData("Targ Y", yTarg);


        // Check if we're at our target
        if (dist <= movementMargin && Math.abs(turnDist) <= turnMargin) {
            wheelHandler.absoluteVectorToMotion(0, 0, 0, 0, telemetry);
            telemetry.addData("Done?", "Done");
            return true;
        }


        // Calculate force to use
        double xSpeed = 0;
        double ySpeed = 0;
        double turnSpeed = 0;
        if (Math.abs(xDist) > movementMargin / 1.3) { // we need to reduce movement margin to account for the rare scenarios when x & y are both within margin but combined they are not
            if (xDist > 0 ^ xDirPos) {
                movementPIDx.resetError();
                //xDirPos = !xDirPos;
            }

            telemetry.addData("PID DATA", "x");
            xSpeed = movementPIDx.calculateWithTelemetry(x, xTarg, runtime, telemetry);
        } else {
            telemetry.addData("X", "Done");
        }


        if (Math.abs(yDist) > movementMargin / 1.3) {
            if (yDist > 0 ^ yDirPos) {
                movementPIDy.resetError();
                //yDirPos = !yDirPos;
            }

            telemetry.addData("PID DATA", "y");
            ySpeed = movementPIDy.calculateWithTelemetry(y, yTarg, runtime, telemetry);
        } else {
            telemetry.addData("Y", "Done");
        }

        if (Math.abs(turnDist) > turnMargin) {
            if (turnDist > 0 ^ turnDirPos) {
                movementPIDturn.resetError();
                //yDirPos = !yDirPos;
            }

            telemetry.addData("PID DATA", "turn");
            turnSpeed = movementPIDturn.calculateWithTelemetry(yaw, targetRotation, runtime, telemetry);
        } else {
            telemetry.addData("Turn", "Done");
        }


        xSpeed = Math.max(Math.min(xSpeed, maxSpeed), -maxSpeed);
        ySpeed = Math.max(Math.min(ySpeed, maxSpeed), -maxSpeed);
        turnSpeed = Math.max(Math.min(turnSpeed, maxSpeed), -maxSpeed);

        telemetry.addData("Speed X", xSpeed);
        telemetry.addData("Speed Y", ySpeed);
        telemetry.addData("Speed Turning", turnSpeed);


        // Run motors
        wheelHandler.absoluteVectorToMotion(xSpeed, ySpeed, turnSpeed, yaw, telemetry);

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

    public void setMaxSpeed(double maxSpeed) {
        this.maxSpeed = maxSpeed;
    }

    public void relativeRun(double x, double y) {
        wheelHandler.relativeVectorToMotion(y, x, 0);
    }
}
