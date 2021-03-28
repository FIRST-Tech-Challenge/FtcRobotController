package com.technototes.library.command;

import java.util.function.BooleanSupplier;

//an easy way to make commands
@Deprecated
public class InstantConditionalCommand extends ConditionalCommand {
    public InstantConditionalCommand(BooleanSupplier b, Runnable r) {
        super(b, new InstantCommand(r));
    }

    public InstantConditionalCommand(BooleanSupplier b, Runnable r, Runnable r2) {
        super(b, new InstantCommand(r), new InstantCommand(r2));
    }
}
