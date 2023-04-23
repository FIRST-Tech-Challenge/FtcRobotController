package org.firstinspires.ftc.teamcode.components;

// TODO: 21/01/2023 Test

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.controller.PIDController;
import com.arcrobotics.ftclib.drivebase.HDrive;
import com.arcrobotics.ftclib.geometry.Vector2d;
import com.arcrobotics.ftclib.hardware.RevIMU;
import com.qualcomm.robotcore.hardware.PIDCoefficients;

import org.firstinspires.ftc.teamcode.libs.brightonCollege.util.Maths;
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

    public static PIDCoefficients TURNING_PID = new PIDCoefficients(0.012, 0, 0.0008);
    private static PIDController turningController;
    private boolean isTurnSnapOp;

    public HDriveWrapper(HDrive drive, RevIMU imu) {
        this.drive = drive;
        this.imu = imu;
        desiredDirection = 0;

        isTurnSnapOp = false;

        turningController = new PIDController(0, 0, 0);
        setPIDControllerValues();
    }

    private void setPIDControllerValues() {
        turningController.setPID(TURNING_PID.p, TURNING_PID.i, TURNING_PID.d);
        // We want the difference between desired and real heading to be 0
        turningController.setSetPoint(0);
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

        // Of turn snap is on, determine the closest cardinal direction
        if (isTurnSnapOp) {
            // Transform the angle so that it is between 0 and 360, not -180 and 180
            if (direction < 0) direction += 360;
            // Gets us a double between 0 to 4 inclusive, 0 and 4 correspond to East, 1 North, 2 West, 3 South
            // East gets 0 and 4 because 0 corresponds to angles near 0 and 4 to angles near 360
            double interpolated = Maths.mapClamped(direction, 0, 360, 0, 4);
            interpolated = Math.round(interpolated);
            // 0 and 4 correspond to the same direction
            if (direction == 4) direction = 0;
            // Turn it back into a direction
            direction = Maths.mapClamped(interpolated, 0, 3, 0, 270);
        }

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

        // Need to reset the controller constants as the PID values could have been changed through the dashboard
        setPIDControllerValues();
        double turn = turningController.calculate(headingToDesired);

        drive.driveFieldCentric(strafe, forward, turn, heading);
    }

    public void setTurnSnap(boolean isTurnSnapOn) {
        this.isTurnSnapOp = isTurnSnapOn;
    }
}
