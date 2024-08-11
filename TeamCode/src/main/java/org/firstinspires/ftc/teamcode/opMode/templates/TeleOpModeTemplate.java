package org.firstinspires.ftc.teamcode.opMode.templates;

import com.arcrobotics.ftclib.gamepad.GamepadEx;

import org.firstinspires.ftc.teamcode.RobotController;
import org.firstinspires.ftc.teamcode.commands.driveTrain.mecanumDrive.MecanumArcadeDriveCommand;
import org.firstinspires.ftc.teamcode.subsystems.MecanumDriveSubsystem;
import org.firstinspires.ftc.teamcode.util.opModes.OpModeTemplate;

public class TeleOpModeTemplate extends OpModeTemplate {
    public TeleOpModeTemplate() {
        super(
//            TankDriveBaseSubsystem.class
                MecanumDriveSubsystem.class
        );
    }

    @Override
    public void init(RobotController robotController) {
        MecanumDriveSubsystem mecanumDriveSubsystem = robotController.getSubsystem(MecanumDriveSubsystem.class);
        mecanumDriveSubsystem.setDefaultCommand(new MecanumArcadeDriveCommand(mecanumDriveSubsystem, robotController.driverController));
    }

    @Override
    public void createControls(GamepadEx driverController, GamepadEx actionController, RobotController robotController) {

    }
}
