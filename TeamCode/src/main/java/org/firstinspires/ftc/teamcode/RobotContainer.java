package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.dragonswpilib.GenericHID;
import org.firstinspires.ftc.dragonswpilib.command.button.JoystickButton;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaCurrentGame;
import org.firstinspires.ftc.teamcode.commandGroups.AutonomousCommandGroup;
import org.firstinspires.ftc.teamcode.commands.DriveCommand;
import org.firstinspires.ftc.teamcode.subsystems.DriveSubsystem;
import org.firstinspires.ftc.dragonswpilib.command.Command;

public class RobotContainer {

    private final Gamepad mGamepad1, mGamepad2;
    private final Telemetry mTelemetry;
    private final HardwareMap mHardwareMap;

    private final DriveSubsystem mDriveSubsystem;
    private final DriveCommand mDriveCommand;

    private final AutonomousCommandGroup mAutonomousCommandGroup;

    public RobotContainer(Gamepad gamepad1, Gamepad gamepad2, Telemetry telemetry, HardwareMap hardwareMap){
        mGamepad1 = gamepad1;
        mGamepad2 = gamepad2;
        mTelemetry = telemetry;
        mHardwareMap = hardwareMap;

        mDriveSubsystem = new DriveSubsystem(mHardwareMap, mTelemetry);
        mDriveCommand = new DriveCommand(mTelemetry, mDriveSubsystem, mGamepad1);
        mAutonomousCommandGroup = new AutonomousCommandGroup(mTelemetry, mDriveSubsystem);

        configureButtonBindings();
        configureDefaultCommands();
    }

    @Override
    protected void finalize() {
    }

    private void configureButtonBindings() {
        JoystickButton DPadUp = new JoystickButton(mGamepad1, GenericHID.XboxControllerConstants.kDpadUp);
        DPadUp.onTrue(mAutonomousCommandGroup);
    }

    private void configureDefaultCommands(){
        mDriveSubsystem.setDefaultCommand(mDriveCommand);
    }

    public Command getAutonomousCommand() {
        return mAutonomousCommandGroup;
    }
}
