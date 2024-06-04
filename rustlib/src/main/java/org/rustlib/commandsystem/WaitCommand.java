package org.rustlib.commandsystem;

import com.qualcomm.robotcore.util.ElapsedTime;

public class WaitCommand extends Command {
    private final ElapsedTime timer = new ElapsedTime();
    private final double waitTimeMilliseconds;
    private double startTimestamp;

    public WaitCommand(double waitTimeMilliseconds) {
        this.waitTimeMilliseconds = waitTimeMilliseconds;
    }

    @Override
    public void initialize() {
        startTimestamp = timer.milliseconds();
    }

    @Override
    public boolean isFinished() {
        return timer.milliseconds() > startTimestamp + waitTimeMilliseconds;
    }
}
