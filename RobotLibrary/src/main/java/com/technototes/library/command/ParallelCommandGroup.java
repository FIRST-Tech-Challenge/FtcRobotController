package com.technototes.library.command;

import java.util.Map;

/** Command group to run commands in parallel until all finish
 * @author Alex Stedman
 */
public class ParallelCommandGroup extends CommandGroup {

    /** Make parallel command group
     *
     * @param commands The commands for the group
     */
    public ParallelCommandGroup(Command... commands) {
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
    //if there is no unfinished commands its done
        return !commandMap.containsValue(false);
}
}
