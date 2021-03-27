package com.technototes.library.command;

import java.util.Map;

/** Command group to run commands in parallel until one finished
 * @author Alex Stedman
 */
public class ParallelRaceGroup extends CommandGroup {

    /** Make parallel race group
     *
     * @param commands The commands for the group
     */
    public ParallelRaceGroup(Command... commands) {
        super(commands);
    }

    @Override
    public void schedule(Command c) {
        this.with(c);
    }

    /** Is this finished
     *
     * @return If all of the commands are finished
     */
    @Override
    public boolean isFinished() {
        //if there is a single finished command
        return commandMap.containsValue(true);
    }
}
