
package org.firstinspires.ftc.teamcode.Commands;

import com.qualcomm.robotcore.util.RobotLog;

import org.firstinspires.ftc.teamcode.Subsystems.Subsystem;
import org.json.JSONException;

import java.util.ArrayList;
import java.util.Collections;

public class SequentialCommandGroup implements Command {
    private ArrayList<Command> commands = new ArrayList<>();
    private int currentCommandIndex = 0;
    private Command currentCommand;
    private CommandScheduler scheduler;
    private int maxExecutionCount = 100;
    private int currentExecutionCount = 0;

    public SequentialCommandGroup(CommandScheduler scheduler, Command... commands) {
        this.scheduler = scheduler;
        Collections.addAll(this.commands, commands);
    }

    public void addCommands(Command... commands) {
        Collections.addAll(this.commands, commands);
    }

    @Override
    public void start() {
        if (!commands.isEmpty()) {
            currentCommandIndex = 0;
            currentCommand = commands.get(currentCommandIndex);
            scheduler.schedule(currentCommand);
            RobotLog.d("Sequential Command Group Started with " + currentCommand.getClass().getSimpleName());
        }
    }

    @Override
    public void execute()  {
        if (currentCommand != null && currentCommand.isFinished()) {
            currentCommand.end();
            currentCommandIndex++;
            currentExecutionCount = 0;
            if (currentCommandIndex < commands.size()) {
                currentCommand = commands.get(currentCommandIndex);
                scheduler.schedule(currentCommand);
                RobotLog.d("Sequential Command Group Next Command Started: " + currentCommand.getClass().getSimpleName());
            } else {
                currentCommand = null;
            }
        } else if (currentExecutionCount > maxExecutionCount) {
            RobotLog.e("Command execution exceeded max count, ending current command");
            if (currentCommand != null) {
                currentCommand.end();
            }
            currentCommand = null;
        } else {
            // currentExecutionCount++;
        }
    }

    @Override
    public void end() {
        if (currentCommand != null) {
            RobotLog.d("Sequential Command Group Ended with " + currentCommand.getClass().getSimpleName());
        }
        for (Command command : commands) {
            command.end();
        }
    }

    @Override
    public boolean isFinished() {
        return currentCommand == null || currentCommandIndex >= commands.size();
    }

    @Override
    public Subsystem getRequiredSubsystem() {
        return null;
    }
}
