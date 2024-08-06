package org.firstinspires.ftc.teamcode.opMode.templates;

import com.arcrobotics.ftclib.gamepad.GamepadEx;

import org.firstinspires.ftc.teamcode.RobotController;
import org.firstinspires.ftc.teamcode.commands.mechanumDrive.MechanumArcadeDriveCommand;
import org.firstinspires.ftc.teamcode.subsystems.MechanumDriveSubsystem;
import org.firstinspires.ftc.teamcode.util.opModes.OpModeTemplate;

public class TeleOpModeTemplate extends OpModeTemplate {
    public TeleOpModeTemplate() {
        super(
//            TankDriveBaseSubsystem.class
                MechanumDriveSubsystem.class
        );
    }

    @Override
    public void init(RobotController robotController) {
        MechanumDriveSubsystem mechanumDriveSubsystem = robotController.getSubsystem(MechanumDriveSubsystem.class);
        mechanumDriveSubsystem.setDefaultCommand(new MechanumArcadeDriveCommand(mechanumDriveSubsystem, robotController.driverController));
    }

    @Override
    public void createControls(GamepadEx driverController, GamepadEx actionController, RobotController robotController) {

    }
}
