package org.firstinspires.ftc.teamcode.dragonswpilib;

import java.util.Collection;
import java.util.Collections;
import java.util.Iterator;
import java.util.LinkedHashMap;
import java.util.LinkedHashSet;
import java.util.Map;
import java.util.Set;

public final class CommandScheduler {
    /** The Singleton Instance. */
    private static CommandScheduler instance;

    /**
     * Returns the Scheduler instance.
     *
     * @return the instance
     */
    public static synchronized CommandScheduler getInstance() {
        if (instance == null) {
            instance = new CommandScheduler();
        }
        return instance;
    }

    // A map from commands to their scheduling state.  Also used as a set of the currently-running
    // commands.
    private final Map<Command, CommandState> m_scheduledCommands = new LinkedHashMap<>();

    // A map from required subsystems to their requiring commands.  Also used as a set of the
    // currently-required subsystems.
    private final Map<Subsystem, Command> m_requirements = new LinkedHashMap<>();

    // A map from subsystems registered with the scheduler to their default commands.  Also used
    // as a list of currently-registered subsystems.
    private final Map<Subsystem, Command> m_subsystems = new LinkedHashMap<>();

    // The set of currently-registered buttons that will be polled every iteration.
    private final Collection<Runnable> m_buttons = new LinkedHashSet<>();

    /**
     * Adds a button binding to the scheduler, which will be polled to schedule commands.
     *
     * @param button The button to add
     */
    public void addButton(Runnable button) {
        m_buttons.add(button);
    }

    /**
     * Initializes a given command, adds its requirements to the list, and performs the init actions.
     *
     * @param command The command to initialize
     * @param interruptible Whether the command is interruptible
     * @param requirements The command requirements
     */
    private void initCommand(Command command, boolean interruptible, Set<Subsystem> requirements) {
        CommandState scheduledCommand = new CommandState(interruptible);
        m_scheduledCommands.put(command, scheduledCommand);
        command.initialize();
        for (Subsystem requirement : requirements) {
            m_requirements.put(requirement, command);
        }
    }

    /**
     * Whether the given commands are running. Note that this only works on commands that are directly
     * scheduled by the scheduler; it will not work on commands inside of CommandGroups, as the
     * scheduler does not see them.
     *
     * @param commands the command to query
     * @return whether the command is currently scheduled
     */
    public boolean isScheduled(Command... commands) {
        for (Command command : commands) {
            if (!m_scheduledCommands.containsKey(command)) {
                return false;
            }
        }
        return true;
    }

    /**
     * Schedules a command for execution. Does nothing if the command is already scheduled. If a
     * command's requirements are not available, it will only be started if all the commands currently
     * using those requirements have been scheduled as interruptible. If this is the case, they will
     * be interrupted and the command will be scheduled.
     *
     * @param interruptible whether this command can be interrupted
     * @param command the command to schedule
     */
    private void schedule(boolean interruptible, Command command) {
        // Do nothing if the command is already scheduled.
        if (m_scheduledCommands.containsKey(command)) {
            return;
        }

        Set<Subsystem> requirements = command.getRequirements();

        // Schedule the command if the requirements are not currently in-use.
        if (Collections.disjoint(m_requirements.keySet(), requirements)) {
            initCommand(command, interruptible, requirements);
        } else {
            // Else check if the requirements that are in use have all have interruptible commands,
            // and if so, interrupt those commands and schedule the new command.
            for (Subsystem requirement : requirements) {
                if (m_requirements.containsKey(requirement)
                        && !m_scheduledCommands.get(m_requirements.get(requirement)).isInterruptible()) {
                    return;
                }
            }
            for (Subsystem requirement : requirements) {
                if (m_requirements.containsKey(requirement)) {
                    cancel(m_requirements.get(requirement));
                }
            }
            initCommand(command, interruptible, requirements);
        }
    }

    /**
     * Schedules multiple commands for execution. Does nothing if the command is already scheduled. If
     * a command's requirements are not available, it will only be started if all the commands
     * currently using those requirements have been scheduled as interruptible. If this is the case,
     * they will be interrupted and the command will be scheduled.
     *
     * @param interruptible whether the commands should be interruptible
     * @param commands the commands to schedule
     */
    public void schedule(boolean interruptible, Command... commands) {
        for (Command command : commands) {
            schedule(interruptible, command);
        }
    }

    /**
     * Schedules multiple commands for execution, with interruptible defaulted to true. Does nothing
     * if the command is already scheduled.
     *
     * @param commands the commands to schedule
     */
    public void schedule(Command... commands) {
        schedule(true, commands);
    }

    /**
     * Cancels commands. The scheduler will only call {@link Command#end(boolean)} method of the
     * canceled command with {@code true}, indicating they were canceled (as opposed to finishing
     * normally).
     *
     * <p>Commands will be canceled even if they are not scheduled as interruptible.
     *
     * @param commands the commands to cancel
     */
    public void cancel(Command... commands) {

        for (Command command : commands) {
            if (!m_scheduledCommands.containsKey(command)) {
                continue;
            }

            command.end(true);
            m_scheduledCommands.remove(command);
            m_requirements.keySet().removeAll(command.getRequirements());
        }
    }

    /** Cancels all commands that are currently scheduled. */
    public void cancelAll() {        
        for (Iterator<Command> iterator = m_scheduledCommands.keySet().iterator();
             iterator.hasNext(); ) {
            Command command = iterator.next();

            cancel(command);
        }
    }

    /**
     * Registers subsystems with the scheduler. This must be called for the subsystem's periodic block
     * to run when the scheduler is run, and for the subsystem's default command to be scheduled. It
     * is recommended to call this from the constructor of your subsystem implementations.
     *
     * @param subsystems the subsystem to register
     */
    public void registerSubsystem(Subsystem... subsystems) {
        for (Subsystem subsystem : subsystems) {
            m_subsystems.put(subsystem, null);
        }
    }

    /**
     * Sets the default command for a subsystem. Registers that subsystem if it is not already
     * registered. Default commands will run whenever there is no other command currently scheduled
     * that requires the subsystem. Default commands should be written to never end (i.e. their {@link
     * Command#isFinished()} method should return false), as they would simply be re-scheduled if they
     * do. Default commands must also require their subsystem.
     *
     * @param subsystem the subsystem whose default command will be set
     * @param defaultCommand the default command to associate with the subsystem
     */
    public void setDefaultCommand(Subsystem subsystem, Command defaultCommand) {
        if (!defaultCommand.getRequirements().contains(subsystem)) {
            throw new IllegalArgumentException("Default commands must require their subsystem!");
        }

        if (defaultCommand.isFinished()) {
            throw new IllegalArgumentException("Default commands should not end!");
        }

        m_subsystems.put(subsystem, defaultCommand);
    }

    public void run() {

        // Dragons : Checker les boutons avant de faire les subsystem.periodic()
        // pour appeler initialize() et PIDSubsystem.enable() avant le subsystem.periodic()
        // Poll buttons for new commands to add.
        for (Runnable button : m_buttons) {
            button.run();
        }

        // Run the periodic method of all registered subsystems.
        for (Subsystem subsystem : m_subsystems.keySet()) {
            subsystem.periodic();
        }

        // Run scheduled commands, remove finished commands.
        for (Iterator<Command> iterator = m_scheduledCommands.keySet().iterator();
             iterator.hasNext(); ) {
            Command command = iterator.next();

            command.execute();
            if (command.isFinished()) {
                command.end(false);
                iterator.remove();
                m_requirements.keySet().removeAll(command.getRequirements());
            }
        }

        // Add default commands for un-required registered subsystems.
        for (Map.Entry<Subsystem, Command> subsystemCommand : m_subsystems.entrySet()) {
            if (!m_requirements.containsKey(subsystemCommand.getKey())
                    && subsystemCommand.getValue() != null) {
                schedule(subsystemCommand.getValue());
            }
        }
    }

    /**
     * Dragons : Fonction pour recommencer à zéro.
     * À appeler si on veut re-créer un RobotContainer.
     */
    public void reset() {
        m_scheduledCommands.clear();
        m_requirements.clear();
        m_subsystems.clear();
        m_buttons.clear();
    }
    
}
