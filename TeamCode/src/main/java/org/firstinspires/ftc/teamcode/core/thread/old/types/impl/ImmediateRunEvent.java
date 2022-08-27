package org.firstinspires.ftc.teamcode.core.thread.old.types.impl;

import org.firstinspires.ftc.teamcode.core.thread.old.types.api.RunListenerOnceEvent;

public class ImmediateRunEvent extends RunListenerOnceEvent {
    public ImmediateRunEvent(Runnable listener) {
        super(listener);
    }

    @Override
    public boolean shouldRun() {
        return true;
    }
}
