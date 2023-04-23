package org.firstinspires.ftc.teamcode.components;

// TODO: 21/01/2023 Test

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.controller.PIDController;
import com.arcrobotics.ftclib.drivebase.HDrive;
import com.arcrobotics.ftclib.geometry.Vector2d;
import com.arcrobotics.ftclib.hardware.RevIMU;
import com.qualcomm.robotcore.hardware.PIDCoefficients;

/**
 * Team 2's lift which lifts to a certain height
 */
@Config
public class HDriveWrapper {

    // degrees
    private double desiredDirection;
    private final HDrive drive;
    private final RevIMU imu;
    private static PIDController turningController;

    public static PIDCoefficients TURNING_PID = new PIDCoefficients();
    public HDriveWrapper(HDrive drive, RevIMU imu) {
        this.drive = drive;
        this.imu = imu;
        desiredDirection = 0;

        createTurnPID();
    }

    private void createTurnPID() {
        turningController = new PIDController(TURNING_PID.p, TURNING_PID.i, TURNING_PID.d);
        // We want the difference between desired and real heading to be 0
        turningController.setSetPoint(0);
    }

    /**
     * Updates the desired turn based on turning joystick
     * @param x x coord of joystick
     * @param y y coord of joystick
     */
    public void updateTurn(double x, double y){
        // This order and "-" rotate the vector by 90 deg
        Vector2d joystickVector = new Vector2d(y, -x);
        double direction = Math.toDegrees(joystickVector.angle());
        double magnitude = joystickVector.magnitude();
        // only change the turn direction if not in the middle position
        if (magnitude > 0.2) desiredDirection = direction;
    }

    public void fieldOrientedDrive(double strafe, double forward){
        double heading = imu.getAbsoluteHeading();

        // I couldn't find an implementation of a continuous PID, so we need to do this ourselves
        double headingToDesired = desiredDirection - heading;
        // If the difference is greater than PI in magnitude, it means we are turning more than a half-circle
        // To be more efficient, we can add 2PI to the angle and go the other way:
        while (headingToDesired < -Math.PI) headingToDesired += 2 * Math.PI;
        // Same applies to the other direction
        while (headingToDesired > Math.PI) headingToDesired -= 2 * Math.PI;

        // Need to recreate the controller as the PID values could have been changed through the dashboard
        createTurnPID();
        double turn = turningController.calculate(headingToDesired);

        // TODO: Get IMU heading and set turn
        drive.driveFieldCentric(strafe, forward, turn, heading);
    }
}
