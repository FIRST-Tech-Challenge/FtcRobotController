package org.firstinspires.ftc.teamcode.subsystems;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IMU;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;

/**
 * This subsystem is dedicated to interfacing with the Imu.
 * It's main purpose is for returning the angle that the imu reads,
 * but it can also reset the angle
 */


public class ImuSub extends SubsystemBase {
    private Telemetry telemetry;

    private IMU imu;
    private double lastYaw = 0.0;
    private double globalAngle;

    /**
     * Constructor for the imu
     * @param hardwareMapParam The hardware map to be used in imu
     * @param telemetryParam The telemetry to be used for printing things
     */

    public ImuSub(HardwareMap hardwareMapParam, Telemetry telemetryParam) {
        this.telemetry = telemetryParam;

        imu = hardwareMapParam.get(IMU.class, "imu");

        RevHubOrientationOnRobot.LogoFacingDirection logoDirection = RevHubOrientationOnRobot.LogoFacingDirection.UP;
        RevHubOrientationOnRobot.UsbFacingDirection usbDirection = RevHubOrientationOnRobot.UsbFacingDirection.LEFT;

        RevHubOrientationOnRobot orientationOnRobot = new RevHubOrientationOnRobot(logoDirection, usbDirection);


        boolean initSuccessful = imu.initialize(new IMU.Parameters(orientationOnRobot));

        telemetry.addData("IMU Calibrating Successful?", initSuccessful);
    }



    /**
     * Code to run the IMU's loop. Currently prints heading.
     */
    @Override
    public void periodic() {
        telemetry.addData("IMU heading", globalAngle);
    }

    /**
     * Resets the angle for the imu so that it's accurate to whatever position
     */
    public void resetAngle() {

        imu.resetYaw();

        YawPitchRollAngles orientation = imu.getRobotYawPitchRollAngles();
        lastYaw = orientation.getYaw(AngleUnit.DEGREES);
        globalAngle = 0.0;
    }

    /**
     * Get current cumulative angle rotation from last reset.
     *
     * @return Angle in degrees. + = left, - = right.
     */
    public double getAngle() {
        // We experimentally determined the Z axis is the axis we want to use for heading angle.
        // We have to process the angle because the imu works in euler angles so the Z axis is
        // returned as 0 to +180 or 0 to -180 rolling back to -179 or +179 when rotation passes
        // 180 degrees. We detect this transition and track the total cumulative angle of rotation.

        YawPitchRollAngles orientation = imu.getRobotYawPitchRollAngles();

        double deltaAngle = orientation.getYaw(AngleUnit.DEGREES) - lastYaw ;

        if (deltaAngle < -180)
            deltaAngle += 360;
        else if (deltaAngle > 180)
            deltaAngle -= 360;

        globalAngle += deltaAngle;

        lastYaw = orientation.getYaw(AngleUnit.DEGREES);

        return globalAngle;
    }

    /**
     * Gets a degree angle value and converts it to radians
     * @return Final converted angle
     */

    public double getAngleRadians() {
        return (Math.PI * this.getAngle()) / 180;
    }

    /**
     * Gets the correction based on where the robot is told to go versus where it is.
     * @return The amount that the robot needs to be corrected by
     */

    public double checkDirection() {
        // The gain value determines how sensitive the correction is to direction changes.
        // You will have to experiment with your robot to get small smooth direction changes
        // to stay on a straight line.
        double correction, angle, gain = .10;

        angle = getAngle();

        if (angle == 0)
            correction = 0;             // no adjustment.
        else
            correction = -angle;        // reverse sign of angle for correction.

        correction = correction * gain;

        return correction;
    }



}