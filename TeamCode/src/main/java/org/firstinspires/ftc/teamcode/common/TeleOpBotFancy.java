package org.firstinspires.ftc.teamcode.common;

import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class TeleOpBotFancy extends BotFancy {
    private TeleOpDrivetrainBasic drivetrain = null;
    public TeleOpBotFancy(HardwareMap hardwareMap, Telemetry telemetry)
    {
        super(hardwareMap, telemetry);
        drivetrain = new TeleOpDrivetrainBasic(hardwareMap, telemetry);
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
