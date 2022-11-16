package org.firstinspires.ftc.teamcode.dragonswpilib;

import java.util.Set;

public interface Command {

    default void initialize() {}

    default void execute() {}

    default void end(boolean interrupted) {}

    default boolean isFinished() {
        return false;
    }

    /**
     * Specifies the set of subsystems used by this command. Two commands cannot use the same
     * subsystem at the same time. If the command is scheduled as interruptible and another command is
     * scheduled that shares a requirement, the command will be interrupted. Else, the command will
     * not be scheduled. If no subsystems are required, return an empty set.
     *
     * <p>Note: it is recommended that user implementations contain the requirements as a field, and
     * return that field here, rather than allocating a new set every time this is called.
     *
     * @return the set of subsystems that are required
     */
    Set<Subsystem> getRequirements();

    /**
     * Decorates this command with a timeout. If the specified timeout is exceeded before the command
     * finishes normally, the command will be interrupted and un-scheduled. Note that the timeout only
     * applies to the command returned by this method; the calling command is not itself changed.
     *
     * <p>Note: This decorator works by composing this command within a CommandGroup. The command
     * cannot be used independently after being decorated, or be re-decorated with a different
     * decorator, unless it is manually cleared from the list of grouped commands with {@link
     * CommandGroupBase#clearGroupedCommand(Command)}. The decorated command can, however, be further
     * decorated without issue.
     *
     * @param seconds the timeout duration
     * @return the command with the timeout added
     */
    default ParallelRaceGroup withTimeout(double seconds) {
        return raceWith(new WaitCommand(seconds));
    }

    /**
     * Decorates this command with a set of commands to run parallel to it, ending when the last
     * command ends. Often more convenient/less-verbose than constructing a new {@link
     * ParallelCommandGroup} explicitly.
     *
     * <p>Note: This decorator works by composing this command within a CommandGroup. The command
     * cannot be used independently after being decorated, or be re-decorated with a different
     * decorator, unless it is manually cleared from the list of grouped commands with {@link
     * CommandGroupBase#clearGroupedCommand(Command)}. The decorated command can, however, be further
     * decorated without issue.
     *
     * @param parallel the commands to run in parallel
     * @return the decorated command
     */
    default ParallelCommandGroup alongWith(Command... parallel) {
        ParallelCommandGroup group = new ParallelCommandGroup(this);
        group.addCommands(parallel);
        return group;
    }

    /**
     * Decorates this command with a set of commands to run parallel to it, ending when the first
     * command ends. Often more convenient/less-verbose than constructing a new {@link
     * ParallelRaceGroup} explicitly.
     *
     * <p>Note: This decorator works by composing this command within a CommandGroup. The command
     * cannot be used independently after being decorated, or be re-decorated with a different
     * decorator, unless it is manually cleared from the list of grouped commands with {@link
     * CommandGroupBase#clearGroupedCommand(Command)}. The decorated command can, however, be further
     * decorated without issue.
     *
     * @param parallel the commands to run in parallel
     * @return the decorated command
     */
    default ParallelRaceGroup raceWith(Command... parallel) {
        ParallelRaceGroup group = new ParallelRaceGroup(this);
        group.addCommands(parallel);
        return group;
    }

    /**
     * Schedules this command.
     *
     * @param interruptible whether this command can be interrupted by another command that shares one
     *     of its requirements
     */
    default void schedule(boolean interruptible) {
        CommandScheduler.getInstance().schedule(interruptible, this);
    }

    /** Schedules this command, defaulting to interruptible. */
    default void schedule() {
        schedule(true);
    }
    default boolean isScheduled() {
        return CommandScheduler.getInstance().isScheduled(this);
    }

    /**
     * Cancels this command. Will call the command's interrupted() method. Commands will be canceled
     * even if they are not marked as interruptible.
     */
    default void cancel() {
        CommandScheduler.getInstance().cancel(this);
    }
}
