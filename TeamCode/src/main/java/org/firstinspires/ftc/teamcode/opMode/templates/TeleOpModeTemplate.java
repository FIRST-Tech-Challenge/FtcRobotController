package org.firstinspires.ftc.teamcode.opMode.templates;

import com.arcrobotics.ftclib.gamepad.GamepadEx;

import org.firstinspires.ftc.teamcode.RobotController;
import org.firstinspires.ftc.teamcode.subsystems.TankDriveBaseSubsystem;

public class TeleOpModeTemplate extends OpModeTemplate {
    public TeleOpModeTemplate() {
        super(
            TankDriveBaseSubsystem.class
        );
    }

    @Override
    public void createControls(GamepadEx driverController, GamepadEx actionController, RobotController robotController) {

    }
}
