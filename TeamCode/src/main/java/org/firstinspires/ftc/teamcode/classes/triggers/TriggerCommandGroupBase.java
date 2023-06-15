package org.firstinspires.ftc.teamcode.classes.triggers;

import com.arcrobotics.ftclib.command.Command;
import com.arcrobotics.ftclib.command.CommandBase;

import java.util.Arrays;
import java.util.Collection;
import java.util.Collections;
import java.util.Set;
import java.util.WeakHashMap;

public abstract class TriggerCommandGroupBase extends CommandBase implements Command {

    private static final Set<Command> m_groupedCommands =
            Collections.newSetFromMap(new WeakHashMap<>());

    static void registerGroupedCommands(Command... commands) {
        m_groupedCommands.addAll(Arrays.asList(commands));
    }

    /**
     * Clears the list of grouped commands, allowing all commands to be freely used again.
     *
     * <p>WARNING: Using this haphazardly can result in unexpected/undesirable behavior.  Do not
     * use this unless you fully understand what you are doing.
     */
    public static void clearGroupedCommands() {
        m_groupedCommands.clear();
    }

    /**
     * Removes a single command from the list of grouped commands, allowing it to be freely used
     * again.
     *
     * <p>WARNING: Using this haphazardly can result in unexpected/undesirable behavior. Do not
     * use this unless you fully understand what you are doing.
     *
     * @param command the command to remove from the list of grouped commands
     */
    public static void clearGroupedCommand(Command command) {
        m_groupedCommands.remove(command);
    }


    /**
     * Requires that the specified commands not have been already allocated to a CommandGroup. Throws
     * an {@link IllegalArgumentException} if commands have been allocated.
     *
     * @param commands The commands to check
     */
    public static void requireUngrouped(Command... commands) {
        requireUngrouped(Arrays.asList(commands));
    }

    /**
     * Requires that the specified commands not have been already allocated to a CommandGroup. Throws
     * an {@link IllegalArgumentException} if commands have been allocated.
     *
     * @param commands The commands to check
     */
    public static void requireUngrouped(Collection<Command> commands) {
        if (!Collections.disjoint(commands, getGroupedCommands())) {
            throw new IllegalArgumentException("Commands cannot be added to more than one CommandGroup");
        }
    }

    static Set<Command> getGroupedCommands() {
        return m_groupedCommands;
    }

    /**
     * Adds the given commands to the command group.
     *
     * @param commands The commands to add.
     */

    public abstract void addCommands(TriggerCommand... commands);
}
