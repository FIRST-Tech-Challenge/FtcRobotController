package org.firstinspires.ftc.teamcode.support.tasks;

public class SleepTask extends Task {

    private long wakeUp;

    public SleepTask(long ms) {
        wakeUp = System.currentTimeMillis() + ms;
    }

    @Override
    public Progress start() {
        return new Progress() {
            @Override
            public boolean isDone() {
                return System.currentTimeMillis() >= wakeUp;
            }
        };
    }
}
