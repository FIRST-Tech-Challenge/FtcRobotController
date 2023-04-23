package org.firstinspires.ftc.teamcode.components;

// TODO: 21/01/2023 Test

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.controller.PIDController;
import com.arcrobotics.ftclib.drivebase.HDrive;
import com.arcrobotics.ftclib.geometry.Vector2d;
import com.arcrobotics.ftclib.hardware.RevIMU;
import com.qualcomm.robotcore.hardware.PIDCoefficients;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.libs.brightonCollege.util.Maths;
import org.firstinspires.ftc.teamcode.libs.brightonCollege.util.TelemetryContainer;

/**
 * Team 2's lift which lifts to a certain height
 */
@Config
public class HDriveWrapper {

    // degrees
    public double desiredDirection;
    private final HDrive drive;
    private final RevIMU imu;

    public static PIDCoefficients TURNING_PID = new PIDCoefficients(0.012, 0, 0.0008);
    private static PIDController turningController;

    public HDriveWrapper(HDrive drive, RevIMU imu) {
        this.drive = drive;
        this.imu = imu;
        desiredDirection = 0;

        turningController = new PIDController(0, 0, 0);
        setPIDControllerValues();
    }

    private void setPIDControllerValues() {
        turningController.setPID(TURNING_PID.p, TURNING_PID.i, TURNING_PID.d);
        // We want the difference between desired and real heading to be 0
        turningController.setSetPoint(0);
    }

    /**
     * Return an equivalent angle in degrees in the range -180 to 180
     * @param angle angle to be normalised
     * @return normalised angle
     */
    public static double normaliseAngle180(double angle) {
        // Go a full rotation until we no longer need to
        while (angle < -180) angle += 360;
        // Same applies to the other direction
        while (angle > 180) angle -= 360;
        return angle;
    }

    /**
     * Return an equivalent angle in degrees in the range 0 to 360
     * @param angle angle to be normalised
     * @return normalised angle
     */
    public static double normaliseAngle360(double angle) {
        // Go a full rotation until we no longer need to
        while (angle < 0) angle += 360;
        // Same applies to the other direction
        while (angle > 360) angle -= 360;
        return angle;
    }

    /**
     * Snaps an angle to the nearest cardinal direction
     * @param direction the angle to snap
     * @return the angle of the closest cardinal direction
     */
    public static double snapAngle(double direction) {
        direction = normaliseAngle360(direction);
        // Gets us a double between 0 to 4 inclusive, 0 and 4 correspond to East, 1 North, 2 West, 3 South
        // East gets 0 and 4 because 0 corresponds to angles near 0 and 4 to angles near 360
        double interpolated = Maths.mapClamped(direction, 0, 360, 0, 4);
        interpolated = Math.round(interpolated);
        // 0 and 4 correspond to the same direction
        if (interpolated == 4) interpolated = 0;
        // Turn it back into a direction
        direction = Maths.mapClamped(interpolated, 0, 3, 0, 270);
        return direction;
    }

    /**
     * Updates the desired rotation based on turning joystick, snapping to the nearest cardinal direction
     * @param x x coord of joystick
     * @param y y coord of joystick
     */
    public void setTurnDirectionSnap(double x, double y){
        // The vector angle is considered starting with +ve x axis, but robot's angle starts with +ve y axis
        // (x, y) -> (y, -x) turns it 90 degrees clockwise
        Vector2d joystickVector = new Vector2d(y, -x);
        double direction = Math.toDegrees(joystickVector.angle());
        double magnitude = joystickVector.magnitude();

        direction = snapAngle(direction);

        // only change the turn direction if not in the middle position
        if (magnitude > 0.2) desiredDirection = direction;
    }

    public void fieldOrientedDriveAbsoluteRotation(double strafe, double forward){
        double heading = imu.getHeading();

        Telemetry t = TelemetryContainer.getTelemetry();
        t.addData("desired", desiredDirection);

        // I couldn't find an implementation of a continuous PID, so we need to do this ourselves
        // This is the difference between desired and actual heading
        double headingToDesired = heading - desiredDirection;
        // This allows us to find the smallest angle between two directions, without going a long way round
        // The largest possible magnitude of the angle change is 180
        headingToDesired = normaliseAngle180(headingToDesired);

        t.addData("error", headingToDesired);

        // Need to reset the controller constants as the PID values could have been changed through the dashboard
        setPIDControllerValues();
        double turn = turningController.calculate(headingToDesired);

        fieldOrientedDriveRelativeRotation(strafe, forward, turn);
    }

    public void fieldOrientedDriveRelativeRotation(double strafe, double forward, double turn){
        double heading = imu.getHeading();
        Telemetry t = TelemetryContainer.getTelemetry();
        t.addData("heading", heading);

        drive.driveFieldCentric(strafe, forward, turn, heading);
    }
}
