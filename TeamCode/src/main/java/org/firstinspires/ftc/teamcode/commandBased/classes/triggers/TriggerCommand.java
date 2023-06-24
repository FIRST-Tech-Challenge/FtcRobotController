package org.firstinspires.ftc.teamcode.commandBased.classes.triggers;

import com.arcrobotics.ftclib.command.Command;

import org.firstinspires.ftc.teamcode.commandBased.classes.CommandSchedulerEx;

public interface TriggerCommand extends Command {

    boolean isTriggered();

}
