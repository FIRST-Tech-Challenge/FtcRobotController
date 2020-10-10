package com.technototes.library.command;


import com.qualcomm.robotcore.util.ElapsedTime;
import com.technototes.library.subsystem.Subsystem;

import java.util.Arrays;
import java.util.HashSet;
import java.util.Set;
import java.util.function.BooleanSupplier;

public class Command implements Runnable {
    public ElapsedTime commandRuntime;
    public CommandState commandState;
    public Subsystem subsystem;

    public Command() {
        commandState = new CommandState();
        commandRuntime = new ElapsedTime();
        commandRuntime.reset();
    }

    public final Command addRequirements(Subsystem requirements) {
        subsystem = requirements;
        return this;
    }

    public void init() {

    }

    public void execute() {

    }

    public boolean isFinished() {
        return true;
    }

    public void end(boolean cancel) {

    }


    public final Command andThen(Command c) {
        if (c instanceof SequentialCommandGroup) {
            SequentialCommandGroup c2 = new SequentialCommandGroup(this);
            c2.commands.addAll(((SequentialCommandGroup) c).commands);
            return c2;
        }
        return new SequentialCommandGroup(this, c);
    }

    public final Command andThen(BooleanSupplier b, Command c) {
        return andThen(new ConditionalCommand(b, c));
    }

    @Override
    public void run() {
        switch (commandState.state) {
            case RESET:
                init();
                commandState.state = State.INITIALIZED;
                //THERE IS NO RETURN HERE SO IT FALLS THROUGH TO POST-INITIALIZATION
            case INITIALIZED:
                execute();
                commandState.state = isFinished() ? State.EXECUTED : State.INITIALIZED;
                if(!isFinished()){
                    return;
                }
            case EXECUTED:
                end(false);
                commandState.state = State.RESET;
                commandRuntime.reset();
                return;
        }
    }

    public enum State {
        RESET, INITIALIZED, EXECUTED
    }

    @Deprecated
    public enum Type {
        ONE_USE, MULTI_USE
    }

    public static class CommandState {
        public State state;
        public Type type = Type.MULTI_USE;

        public CommandState() {
            state = State.RESET;
        }

        public CommandState(Type t) {
            new CommandState();
            type = t;
        }

    }
}
