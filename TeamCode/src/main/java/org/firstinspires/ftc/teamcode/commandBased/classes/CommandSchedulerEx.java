package org.firstinspires.ftc.teamcode.commandBased.classes;

import com.arcrobotics.ftclib.command.Command;
import com.arcrobotics.ftclib.command.CommandScheduler;

import org.firstinspires.ftc.teamcode.commandBased.classes.triggers.TriggerCommand;

import java.util.ArrayList;
import java.util.List;

public class CommandSchedulerEx {
    public static List<TriggerCommand> m_commands= new ArrayList<>();
    public static List<TriggerCommand> m_scheduledCommands= new ArrayList<>();
    public static List<TriggerCommand> m_triggeredCommands= new ArrayList<>();
    public static List<TriggerCommand> m_finishedCommands= new ArrayList<>();

    private static CommandSchedulerEx instance;

    /**
     * Returns the Scheduler instance.
     *
     * @return the instance
     */
    public static synchronized CommandSchedulerEx getInstance() {
        if (instance == null) {
            instance = new CommandSchedulerEx();
        }
        return instance;
    }

    public void run() {
        m_scheduledCommands.clear();
        m_triggeredCommands.clear();
        m_finishedCommands.clear();
        for (TriggerCommand command : m_commands) {
            if (command.isScheduled()) {
                m_scheduledCommands.add(command);
            } else if (command.isTriggered()) {
                m_triggeredCommands.add(command);
            } else if (command.isFinished()) {
                m_finishedCommands.add(command);
            }
        }
    }

    public void add(TriggerCommand command) {
        m_commands.add(command);
    }

    public static List<TriggerCommand> getCommands() {
        return m_commands;
    }

    public List<TriggerCommand> getScheduledCommands() {
        return m_scheduledCommands;
    }

    public List<TriggerCommand> getTriggeredCommands() {
        return m_triggeredCommands;
    }

    public List<TriggerCommand> getFinishedCommands() {
        return m_finishedCommands;
    }
}
