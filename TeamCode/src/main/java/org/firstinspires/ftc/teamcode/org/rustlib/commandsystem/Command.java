package org.firstinspires.ftc.teamcode.org.rustlib.commandsystem;

import com.qualcomm.robotcore.util.ElapsedTime;

import java.util.ArrayList;
import java.util.function.BooleanSupplier;

public class Command {
    ArrayList<Trigger> triggers = new ArrayList<>();

    ElapsedTime timer = new ElapsedTime();

    double scheduledTimestamp = 0;
    double initializedTimestamp = 0;

    public Command() {
        CommandScheduler.getInstance().commands.add(this);
    }

    enum State {
        UNSCHEDULED,
        QUEUED,
        SCHEDULED,
        ENDING
    }

    enum Type {
        DEFAULT_COMMAND,
        NORMAL
    }

    Type type = Type.NORMAL;

    final boolean triggered() {
        if (type == Type.DEFAULT_COMMAND) return true;
        boolean triggered = false;
        for (Trigger trigger : triggers) { // The getAsBoolean method must be called for every trigger, because it will set the prior state of each trigger.
            if (trigger.getAsBoolean())
                triggered = true;
        }
        return triggered;
    }

    protected State state = State.UNSCHEDULED;

    public void initialize() {

    }

    public double timeSinceInitialized() {
        return timer.milliseconds() - initializedTimestamp;
    }

    public void execute() {

    }

    public void end(boolean interrupted) {

    }

    public boolean isFinished() {
        return true;
    }

    public void schedule() {
        state = State.QUEUED;
        scheduledTimestamp = CommandScheduler.getInstance().timer.milliseconds();
    }

    public final void cancel() {
        if (state != State.UNSCHEDULED) {
            end(true);
        }
        if (this instanceof CommandGroup) { // Cancel every command in the command group as well as the over-arching command group sequence
            for (Command command : ((CommandGroup) this).commands) {
                command.cancel();
            }
        }
        state = State.UNSCHEDULED;
    }

    public final Command scheduleOn(Trigger trigger) {
        trigger.onTrue(new InstantCommand(this::schedule));
        return this;
    }

    public final Command scheduleOn(BooleanSupplier condition) {
        return scheduleOn(new Trigger(condition));
    }

    public final Command cancelOn(Trigger trigger) {
        trigger.onTrue(new InstantCommand(this::cancel));
        return this;
    }

    public final Command cancelOn(BooleanSupplier condition) {
        cancelOn(new Trigger(condition));
        return this;
    }

    protected final void addRequirements(Subsystem... subsystems) {
        for (Subsystem subsystem : subsystems) {
            subsystem.requirements.add(this);
        }
    }
}
