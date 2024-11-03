package org.firstinspires.ftc.teamcode.opmodes.autonomous.base;

import com.arcrobotics.ftclib.controller.PIDController;
import com.arcrobotics.ftclib.geometry.Pose2d;

import org.firstinspires.ftc.teamcode.opmodes.autonomous.Tunables;

/**
 * This is PID enabled AutoOpMode. @{link {@link #initialize()}} method create instances of PIDController for x, y and rotationSpeed
 * and @{link {@link #updateTargetPose(Pose2d, Pose2d)}} method set the targetX, targetY and targetRotationSpeed for driving
 */
public abstract class PIDControlAutoOpMode extends AutoOpMode {

    protected PIDController xCtrl, yCtrl, rotationCtrl;

    @Override
    public void initialize() {
        super.initialize();

        xCtrl = createXPIDController();
        yCtrl = createYPIDController();
        rotationCtrl = createRotationPIDController();
    }

    @Override
    protected void updateTargetPose(Pose2d currentPose, Pose2d desiredPose) {
        addTelemetryData("curr_X = %f, curr_Y = %f, curr_Rot = %f|des_X = %f, des_Y = %f, des_Rot = %f",
                currentPose.getX(), currentPose.getY(), currentPose.getRotation().getRadians(),
                desiredPose.getX(), desiredPose.getY(), desiredPose.getRotation().getRadians());

        xCtrl.setSetPoint(desiredPose.getX());
        yCtrl.setSetPoint(desiredPose.getY());
        rotationCtrl.setSetPoint(desiredPose.getRotation().getRadians());

        targetX = xCtrl.calculate(-currentPose.getX());
        targetY = yCtrl.calculate(-currentPose.getY());
        targetRotationSpeed = rotateAngleToSpeed(rotationCtrl.calculate(currentPose.getRotation().getRadians()));
    }

    public PIDController createXPIDController() {
        PIDController controller = new PIDController(Tunables.X_KP, Tunables.X_KI, Tunables.X_KD);
        controller.setTolerance(Tunables.X_TOLERANCE);
        return controller;
    }

    public PIDController createYPIDController() {
        PIDController controller = new PIDController(Tunables.Y_KP, Tunables.Y_KI, Tunables.Y_KD);
        controller.setTolerance(Tunables.Y_TOLERANCE);
        return controller;
    }

    public PIDController createRotationPIDController() {
        PIDController controller = new PIDController(Tunables.ROTATION_KP, Tunables.ROTATION_KI, Tunables.ROTATION_KD);
        controller.setTolerance(Tunables.ROTATION_TOLERANCE);
        return controller;
    }
}
