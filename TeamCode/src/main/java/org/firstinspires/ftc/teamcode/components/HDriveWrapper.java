package org.firstinspires.ftc.teamcode.components;

// TODO: 21/01/2023 Test

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.controller.PIDController;
import com.arcrobotics.ftclib.drivebase.HDrive;
import com.arcrobotics.ftclib.geometry.Vector2d;
import com.arcrobotics.ftclib.hardware.RevIMU;
import com.qualcomm.robotcore.hardware.PIDCoefficients;

import org.firstinspires.ftc.teamcode.libs.brightonCollege.util.TelemetryContainer;

/**
 * Team 2's lift which lifts to a certain height
 */
@Config
public class HDriveWrapper {

    // degrees
    private double desiredDirection;
    private final HDrive drive;
    private final RevIMU imu;
    public static PIDCoefficients TURNING_PID = new PIDCoefficients(0.012, 0, 0);
    public HDriveWrapper(HDrive drive, RevIMU imu) {
        this.drive = drive;
        this.imu = imu;
        desiredDirection = 0;

        createTurnPID();
    }

    private PIDController createTurnPID() {
        PIDController turningController = new PIDController(TURNING_PID.p, TURNING_PID.i, TURNING_PID.d);
        // We want the difference between desired and real heading to be 0
        turningController.setSetPoint(0);
        return turningController;
    }

    /**
     * Updates the desired turn based on turning joystick
     * @param x x coord of joystick
     * @param y y coord of joystick
     */
    public void updateTurn(double x, double y){
        Vector2d joystickVector = new Vector2d(x, y);
        double direction = Math.toDegrees(joystickVector.angle());
        double magnitude = joystickVector.magnitude();
        // only change the turn direction if not in the middle position
        if (magnitude > 0.2) desiredDirection = direction;
    }

    public void fieldOrientedDrive(double strafe, double forward){
        MultipleTelemetry t = TelemetryContainer.getTelemetry();

        double heading = imu.getHeading();

        t.addData("heading", heading);
        t.addData("desired", desiredDirection);

        // I couldn't find an implementation of a continuous PID, so we need to do this ourselves
        double headingToDesired = heading - desiredDirection;
        // If the difference is greater than 180 deg in magnitude, it means we are turning more than a half-circle
        // To be more efficient, we can add 360 to the angle and go the other way:
        while (headingToDesired < -180) headingToDesired += 360;
        // Same applies to the other direction
        while (headingToDesired > 180) headingToDesired -= 360;

        t.addData("error", headingToDesired);

        // Need to recreate the controller as the PID values could have been changed through the dashboard
        double turn = createTurnPID().calculate(headingToDesired);

        drive.driveFieldCentric(strafe, forward, turn, heading);
    }
}
