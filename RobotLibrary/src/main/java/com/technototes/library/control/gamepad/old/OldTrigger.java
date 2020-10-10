package com.technototes.library.control.gamepad.old;

import com.technototes.library.command.Command;

public abstract class OldTrigger {
    public abstract OldTrigger whenActivated(Command c);

    public abstract OldTrigger whenDeactivated(Command c);

    public abstract OldTrigger whileActivated(Command c);

    public abstract OldTrigger whileDeactivated(Command c);

    public abstract OldTrigger toggleWhenActivated(Command c);

    public abstract OldTrigger toggleWhenDeactivated(Command c);

    public abstract OldTrigger whenActivated(Runnable r);

    public abstract OldTrigger whenDeactivated(Runnable r);

    public abstract OldTrigger whileActivated(Runnable r);

    public abstract OldTrigger whileDeactivated(Runnable r);

    public abstract OldTrigger toggleWhenActivated(Runnable r);

    public abstract OldTrigger toggleWhenDeactivated(Runnable r);


}
