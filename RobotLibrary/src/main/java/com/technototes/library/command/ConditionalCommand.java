package com.technototes.library.command;

import java.util.function.BooleanSupplier;

/** Simple class for commands that require a certain condition to be true to run
 * @author Alex Stedman
 */
public class ConditionalCommand extends Command {
    private BooleanSupplier supplier;

    public ConditionalCommand(BooleanSupplier condition){
        supplier = condition;
    }


    /** Make a conditional command
     *
     * @param condition The condition
     * @param command The command to run when the condition is true.
     */
    public ConditionalCommand(BooleanSupplier condition, Command command) {
        supplier = condition;
        CommandScheduler.getInstance().scheduleWithOther(this, command, condition);

    }

    /** Make a conditional command
     *
     * @param condition The condition
     * @param trueCommand The command to run when the condition is true
     * @param falseCommand The command to run when the condition is false
     */
    public ConditionalCommand(BooleanSupplier condition, Command trueCommand, Command falseCommand) {
        supplier = condition;
        CommandScheduler.getInstance().scheduleWithOther(this, trueCommand, condition);
        CommandScheduler.getInstance().scheduleWithOther(this, falseCommand, ()->!condition.getAsBoolean());

    }

}
