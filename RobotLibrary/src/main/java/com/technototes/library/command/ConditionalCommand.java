package com.technototes.library.command;

import java.util.function.BooleanSupplier;

public class ConditionalCommand extends Command {
    private BooleanSupplier supplier;
    private Command ifTrue, ifFalse, currentChoice;

    public ConditionalCommand(BooleanSupplier b, Command c) {
        this(b, c, null);
    }

    public ConditionalCommand(BooleanSupplier b, Command t, Command f) {
        supplier = b;
        ifTrue = t;
        ifFalse = f;
    }

    @Override
    public void run() {
        switch (commandState.state) {
            case RESET:
                currentChoice = supplier.getAsBoolean() ? ifTrue : ifFalse;
                if (currentChoice != null) {
                    currentChoice.init();
                    commandState.state = State.INITIALIZED;
                }
                return;
            case INITIALIZED:
                currentChoice.execute();
                commandState.state = currentChoice.isFinished() ? State.EXECUTED : State.INITIALIZED;
                return;
            case EXECUTED:
                currentChoice.end(false);
                commandState.state = State.RESET;
                return;
        }
    }
}
