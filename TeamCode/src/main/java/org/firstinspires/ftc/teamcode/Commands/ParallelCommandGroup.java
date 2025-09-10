package org.firstinspires.ftc.teamcode.Commands;

import org.firstinspires.ftc.teamcode.Subsystems.Subsystem;
import org.firstinspires.ftc.teamcode.Tools.Parameters;
import org.json.JSONException;
import com.qualcomm.robotcore.util.RobotLog;
import java.util.ArrayList;
import java.util.Arrays;
import java.util.Collections;
import java.util.List;

public class ParallelCommandGroup implements Command {
    private List<Command> commands = new ArrayList<>();
    private CommandScheduler scheduler;
    private final Parameters parameter;
    private Command specificCommand;

    public ParallelCommandGroup(CommandScheduler scheduler, Parameters parameter, Command... commands) {
        this.scheduler = scheduler;
        this.parameter = parameter;
        Collections.addAll(this.commands, commands);
        if (parameter == Parameters.SPECIFIC && commands.length > 0) {
            specificCommand = commands[0];
        }
    }

    public void addCommands(Command... commands) {
        this.commands.addAll(Arrays.asList(commands));
    }

    @Override
    public void start() {
        RobotLog.d("Parallel Command Group Started with parameter: " + parameter);
        for (Command command : commands) {
            command.start();
        }
    }

    @Override
    public void execute() {
        for (Command command : commands) {
            command.execute();
        }
    }

    @Override
    public void end() {
        for (Command command : commands) {
            command.end();
        }
        RobotLog.d("Parallel Command Group Ended");
    }

    @Override
    public boolean isFinished() {
        switch (parameter) {
            case ALL:
                for (Command command : commands) {
                    if (!command.isFinished()) {
                        return false;
                    }
                }
                return true;
            case ANY:
                for (Command command : commands) {
                    if (command.isFinished()) {
                        return true;
                    }
                }
                return false;
            case SPECIFIC:
                return specificCommand != null && specificCommand.isFinished();
            case NEVER:
                return false;
            default:
                return true;
        }
    }

    @Override
    public Subsystem getRequiredSubsystem() {
        return null;
    }
}