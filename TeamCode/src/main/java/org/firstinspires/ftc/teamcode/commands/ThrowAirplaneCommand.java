package org.firstinspires.ftc.teamcode.commands;

import com.arcrobotics.ftclib.command.CommandBase;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.subsystems.LauncherSubsystem;

import java.util.function.BooleanSupplier;

public class ThrowAirplaneCommand extends CommandBase {

    private LauncherSubsystem subsystem;

    public ThrowAirplaneCommand(LauncherSubsystem subsystem) {
        this.subsystem = subsystem;
        addRequirements(subsystem);
    }

    private ElapsedTime elapsedTime;

    @Override
    public void execute() {
        if (elapsedTime == null) {
            elapsedTime = new ElapsedTime();
        }

        if (elapsedTime.milliseconds() < 1000) {
            // First task: launch the airplane
            subsystem.setPosition(LauncherSubsystem.LauncherPosition.ACTIVATE);
            return;
        }

        if (elapsedTime.milliseconds() < 4 * 1000) {
            // Second task: return to zero position
            subsystem.setPosition(LauncherSubsystem.LauncherPosition.ZERO);
            return;
        }
    }

    @Override
    public boolean isFinished() {
        if (elapsedTime == null) {
            return false;
        }
        if (elapsedTime.milliseconds() > 5 * 1000) {
            // Once the launcher has executed for 2 seconds, the command is finished
            elapsedTime = null;
            return true;
        }
        return false;
    }
}
