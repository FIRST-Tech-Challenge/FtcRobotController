package org.firstinspires.ftc.teamcode.commands;

import com.arcrobotics.ftclib.command.CommandBase;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;

import org.firstinspires.ftc.teamcode.subsystems.WristSub;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

/**
 * This command is dedicated to a command that controls the wrist for the Tele-op mode
 */

public class WristCmd extends CommandBase {

    private final WristSub wristSub;
    double speed;

    /**
     * This command deals with the wrist in teleop.
     *
     * @param wristSubParam The wrist sub to be imported
     * @param speedParam The speed to move the wrist
     */

    public WristCmd(WristSub wristSubParam, double speedParam){
        this.wristSub = wristSubParam;
        speed = speedParam;
        addRequirements(this.wristSub);
    }
}
