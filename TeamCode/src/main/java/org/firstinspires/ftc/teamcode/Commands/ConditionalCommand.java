package org.firstinspires.ftc.teamcode.Commands;

import org.firstinspires.ftc.teamcode.Subsystems.Subsystem;
import org.json.JSONException;

import java.util.function.BooleanSupplier;

public class ConditionalCommand implements Command {

    private Command onTrue;
    private Command onFalse;
    private Command commandToRun;
    private BooleanSupplier condition;

    public ConditionalCommand(Command onTrue, Command onFalse, BooleanSupplier condition) {
        this.condition = condition;
        this.onTrue = onTrue;
        this.onFalse = onFalse;
        if (condition.getAsBoolean()) {
            commandToRun = onTrue;
        } else {
            commandToRun = onFalse;
        }
    }

    @Override
    public void start() {
        if (condition.getAsBoolean()) {
            commandToRun = onTrue;
        } else {
            commandToRun = onFalse;
        }

        if (commandToRun != null) {
            commandToRun.start();
        }
    }

    @Override
    public void execute() {
        if (commandToRun != null) {
            commandToRun.execute();
        }
    }

    @Override
    public void end() {
        if (commandToRun != null) {
            commandToRun.end();
        }
    }

    @Override
    public boolean isFinished() {
        return commandToRun != null && commandToRun.isFinished();
    }

    @Override
    public Subsystem getRequiredSubsystem() {
        return null;
    }

    public String getSubsystem() {
        if (commandToRun != null) {
            return "hi";
        }
        return "";
    }
}