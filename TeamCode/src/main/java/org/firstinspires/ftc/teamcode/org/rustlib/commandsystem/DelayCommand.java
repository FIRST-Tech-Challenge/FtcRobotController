package org.firstinspires.ftc.teamcode.org.rustlib.commandsystem;

import java.util.function.BooleanSupplier;

public class DelayCommand extends Command {
    private final BooleanSupplier condition;

    private final double timeout;

    public DelayCommand(BooleanSupplier condition, double timeout) {
        this.condition = condition;
        this.timeout = timeout;
    }

    public DelayCommand(BooleanSupplier condition) {
        this(condition, Double.POSITIVE_INFINITY);
    }

    @Override
    public boolean isFinished() {
        return condition.getAsBoolean() || timeSinceInitialized() > timeout;
    }
}
