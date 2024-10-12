package org.firstinspires.ftc.teamcode.commands;

import com.arcrobotics.ftclib.command.CommandBase;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;

import org.firstinspires.ftc.teamcode.subsystems.IntakeSub;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

/**
 * This command is dedicated to a command that controls the intake for the Tele-op mode
 */

public class IntakeCmd extends CommandBase {

    private final IntakeSub intakeSub;
    double speed;

    /**
     * This command deals with the intake in teleop.
     *
     * @param intakeSubParam The intake sub to be imported
     * @param speedParam The speed to move the intake
     */

    public IntakeCmd(IntakeSub intakeSubParam, double speedParam){
        this.intakeSub = intakeSubParam;
        speed = speedParam;
        addRequirements(this.intakeSub);
    }

    @Override
    public void execute(){
        this.intakeSub.setSpeed(speed);
    }
}
