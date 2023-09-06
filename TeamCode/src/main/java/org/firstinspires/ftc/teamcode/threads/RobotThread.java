package org.firstinspires.ftc.teamcode.threads;

import java.util.concurrent.atomic.AtomicBoolean;

public class RobotThread extends Thread {
    private final AtomicBoolean running = new AtomicBoolean(false);
    public void cancel() {
        running.set( true );
    }

    public boolean isCancelled() {
        return running.get();
    }


}
