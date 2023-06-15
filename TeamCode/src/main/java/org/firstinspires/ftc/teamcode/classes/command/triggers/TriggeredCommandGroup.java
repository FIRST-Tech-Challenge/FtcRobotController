package org.firstinspires.ftc.teamcode.classes.command.triggers;

import com.arcrobotics.ftclib.command.Command;
import com.arcrobotics.ftclib.command.CommandGroupBase;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;

import java.util.ArrayList;
import java.util.List;

public class TriggeredCommandGroup extends TriggerCommandGroupBase {

    private final List<TriggerCommand> m_commands = new ArrayList<>();
    private final List<TriggerCommand> m_executingCommands = new ArrayList<>();
    private int m_currentCommandIndex = -1;
    private boolean m_runWhenDisabled = true;
    private TriggerCommand currentCommand;

    /**
     * Creates a new SequentialCommandGroup.  The given commands will be run sequentially, with
     * the CommandGroup finishing when the last command finishes.
     *
     * @param commands the commands to include in this group.
     */
    public TriggeredCommandGroup(TriggerCommand... commands) {
        addCommands(commands);
    }


    public void addCommands(TriggerCommand... commands) {
        requireUngrouped(commands);

        if (m_currentCommandIndex != -1) {
            throw new IllegalStateException(
                    "Commands cannot be added to a CommandGroup while the group is running");
        }

        registerGroupedCommands(commands);

        for (TriggerCommand command : commands) {
            m_commands.add(command);
            m_requirements.addAll(command.getRequirements());
            m_runWhenDisabled &= command.runsWhenDisabled();
        }
    }

    @Override
    public void initialize() {
        m_currentCommandIndex = 0;

        if (!m_commands.isEmpty()) {
            currentCommand = m_commands.get(0);
            currentCommand.initialize();
            m_executingCommands.add(currentCommand);
        }
    }

    @Override
    public void execute() {
        if (m_commands.isEmpty()) {
            return;
        }

        for (TriggerCommand triggerCommand: m_executingCommands) {
            triggerCommand.execute();
            if (triggerCommand.isFinished()) {
                triggerCommand.end(false);
                m_executingCommands.remove(triggerCommand);
            }
        }

        if (currentCommand.isTriggered() || currentCommand.isFinished()) {
            m_currentCommandIndex++;
            if (m_currentCommandIndex < m_commands.size()) {
                m_commands.get(m_currentCommandIndex).initialize();
                m_executingCommands.add(currentCommand);
                currentCommand = m_commands.get(m_currentCommandIndex);
            }
        }
    }

    @Override
    public void end(boolean interrupted) {
        if (interrupted && !m_commands.isEmpty()) {
            for (TriggerCommand triggerCommand: m_executingCommands) {
                triggerCommand.end(true);
            }
        }
        m_currentCommandIndex = -1;
    }

    @Override
    public boolean isFinished() {
        return m_executingCommands.isEmpty();
    }

    @Override
    public boolean runsWhenDisabled() {
        return m_runWhenDisabled;
    }

}
