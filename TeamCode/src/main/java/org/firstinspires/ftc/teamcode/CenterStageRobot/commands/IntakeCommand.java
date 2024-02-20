package org.firstinspires.ftc.teamcode.CenterStageRobot.commands;

import com.arcrobotics.ftclib.command.CommandBase;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.CenterStageRobot.subsystems.IntakeArmSubsystem;
import org.firstinspires.ftc.teamcode.CenterStageRobot.subsystems.IntakeSubsystem;
import org.firstinspires.ftc.teamcode.CenterStageRobot.subsystems.OuttakeSusystem;
import org.firstinspires.ftc.teamcode.CenterStageRobot.subsystems.PixelColorDetectorSubsystem;

public class IntakeCommand extends CommandBase {
    private IntakeSubsystem intakeSubsystem;
    private PixelColorDetectorSubsystem pixelColorDetectorSubsystem;

    private Telemetry telemetry;
    public IntakeCommand(IntakeSubsystem intakeSubsystem, PixelColorDetectorSubsystem pixelColorDetectorSubsystem, Telemetry telemetry) {
        this.intakeSubsystem = intakeSubsystem;
        this.pixelColorDetectorSubsystem = pixelColorDetectorSubsystem;

        this.telemetry = telemetry;


        telemetry.addData("Intake Init", "");

        addRequirements(intakeSubsystem, pixelColorDetectorSubsystem);
    }

    @Override
    public void initialize() {
        intakeSubsystem.run();
    }

    @Override
    public boolean isFinished() {
        return pixelColorDetectorSubsystem.isFrontPixel(); // || intakeSubsystem.isStalled();
    }
}
