package org.firstinspires.ftc.teamcode.common;

import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;

// Simple competition bot for teleop control
//  Includes a basic drivetrain with no encoders or odometry
//  plus devices included in the parent class (Bot)
public class TeleopBot extends Bot {
    private TeleopDrivetrain drivetrain = null;

    public TeleopBot(HardwareMap hardwareMap, Telemetry telemetry) {
        super(hardwareMap, telemetry);
        drivetrain = new TeleopDrivetrain(hardwareMap, telemetry);
    }

    public void creepDirection(double axial, double strafe, double yaw) {
        drivetrain.creepDirection(axial, strafe, yaw);
    }

    public void moveDirection(double axial, double strafe, double yaw) {
        drivetrain.moveDirection(axial, strafe, yaw);
    }

    public void stopDrive() {
        drivetrain.stop();
    }
}
