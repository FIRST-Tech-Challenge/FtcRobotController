
package org.firstinspires.ftc.teamcode.Commands;

import com.qualcomm.robotcore.util.RobotLog;
import org.firstinspires.ftc.teamcode.Subsystems.Subsystem;
import org.firstinspires.ftc.teamcode.Tools.NewRobot;

import java.util.*;

public class CommandScheduler {
    private static CommandScheduler instance;
    private final List<Command> scheduledCommands = new ArrayList<>();
    private final Map<Subsystem, Command> activeSubsystemCommands = new HashMap<>();
    private NewRobot newRobot;

    public CommandScheduler() {

    }

    public static CommandScheduler getInstance() {
        if (instance == null) {
            instance = new CommandScheduler();
        }
        return instance;
    }


    public void setNewRobot(NewRobot robot) {
        this.newRobot = robot;
    }

    public void schedule(Command command) {
        Subsystem requiredSubsystem = command.getRequiredSubsystem();

        if (requiredSubsystem != null) {
            Command activeCommand = activeSubsystemCommands.get(requiredSubsystem);

            // Prevent duplicate scheduling of the same command
            if (activeCommand == command) {
                RobotLog.d("Command already active, not rescheduling: " + command.getClass().getSimpleName());
                return;
            }

            // Cancel the currently active command if it's not a default command
            if (activeCommand != null && !isDefaultCommand(activeCommand)) {
                cancel(activeCommand);
            }

            // Associate the new command with the subsystem
            activeSubsystemCommands.put(requiredSubsystem, command);
        }

        // Schedule and start the new command if not already in the list
        if (!scheduledCommands.contains(command)) {
            scheduledCommands.add(command);
            command.start();
            RobotLog.d("Command Scheduled: " + command.getClass().getSimpleName());
        }
    }

    public void run() {
        List<Command> finishedCommands = new ArrayList<>();

        // Execute scheduled commands and handle completion
        for (Command command : new ArrayList<>(scheduledCommands)) {
            if (command.isFinished()) {
                command.end();
                finishedCommands.add(command);
                RobotLog.d("Command Finished and Ended: " + command.getClass().getSimpleName());

                Subsystem subsystem = command.getRequiredSubsystem();
                if (subsystem != null) {
                    activeSubsystemCommands.remove(subsystem);

                    // Only reschedule default command if no other commands are active for this subsystem
                    Command defaultCommand = subsystem.getDefaultCommand();
                    if (defaultCommand != null && !activeSubsystemCommands.containsKey(subsystem) && !isCommandScheduled(defaultCommand)) {
                        schedule(defaultCommand);
                    }
                }
            } else {
                command.execute();
            }
        }

        scheduledCommands.removeAll(finishedCommands);

        // Ensure default commands are scheduled when needed
        for (Subsystem subsystem : getAllSubsystems()) {
            if (!activeSubsystemCommands.containsKey(subsystem)) {
                Command defaultCommand = subsystem.getDefaultCommand();
                if (defaultCommand != null && !isCommandScheduled(defaultCommand)) {
                    schedule(defaultCommand);
                }
            }
        }
    }


    public void printCurrentCommands() {
        RobotLog.d("===== Current Commands =====");
        for (Map.Entry<Subsystem, Command> entry : activeSubsystemCommands.entrySet()) {
            RobotLog.d("Subsystem: " + entry.getKey().getClass().getSimpleName() +
                    ", Command: " + entry.getValue().getClass().getSimpleName());
        }
        RobotLog.d("============================");
    }

    private void cancel(Command command) {
        Subsystem requiredSubsystem = command.getRequiredSubsystem();
        if (requiredSubsystem != null) {
            activeSubsystemCommands.remove(requiredSubsystem);
        }

        command.end();
        scheduledCommands.remove(command);
        RobotLog.d("Command Cancelled: " + command.getClass().getSimpleName());
    }

    private Set<Subsystem> getAllSubsystems() {
        Set<Subsystem> subsystems = new HashSet<>();
        if (newRobot != null) {
           // subsystems.add(newRobot.arm); UPDATE
            subsystems.add(newRobot.drive);
           // subsystems.add(newRobot.intakeSubsystem); UPDATE
           // subsystems.add(newRobot.wrist);  UPDATE
        }
        return subsystems;
    }

    public boolean isCommandScheduled(Command command) {
        Subsystem subsystem = command.getRequiredSubsystem();
        return subsystem != null && activeSubsystemCommands.get(subsystem) == command;
    }

    private boolean isDefaultCommand(Command command) {
        Subsystem subsystem = command.getRequiredSubsystem();
        return subsystem != null && subsystem.getDefaultCommand() == command;
    }
    public void removeDuplicateCommands() {
        List<Command> uniqueCommands = new ArrayList<>();

        for (Command command : new ArrayList<>(scheduledCommands)) {
            String name = command.getClass().getSimpleName();
            scheduledCommands.removeIf(c -> c.getClass().getSimpleName().equalsIgnoreCase(name));
            uniqueCommands.add(command);
        }

        scheduledCommands.addAll(uniqueCommands);
    }
}
