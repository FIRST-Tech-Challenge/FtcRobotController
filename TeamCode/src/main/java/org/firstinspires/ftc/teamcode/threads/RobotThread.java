package org.firstinspires.ftc.teamcode.threads;

import java.util.concurrent.atomic.AtomicBoolean;

public class RobotThread extends Thread {
    private final AtomicBoolean cancelled = new AtomicBoolean(false);
    public void cancel() {
        cancelled.set( true );
    }

    public boolean isCancelled() {
        return cancelled.get();
    }


}
